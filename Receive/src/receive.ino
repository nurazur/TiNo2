// **********************************************************************************
// Copyright nurazur@gmail.com
// **********************************************************************************
// License
// **********************************************************************************
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

// Licence can be viewed at
// http://www.fsf.org/licenses/gpl.txt

// Please maintain this license information along with authorship
// and copyright notices in any redistribution of this code
// *********************************************************************************



// Gateway for the TiNo Wireless Sensor System
//
// Based originally on work from Nathan Chantrell
// modified by meigrafd @2013 - for UART on RaspberryPI
// extended by nurazur nurazur@gmail.com 2014 - 2024
// the RF protocol is completely binary, see https://github.com/nurazur/tino

// TiNo2 Gateway supports on-board measurements of temperature/humidity as well as BME280 and Dallas DS18B20.

#define SKETCHNAME "TiNo2 receive.ino V2.6.0 07/04/2024"
#define BUILD 11

#include <Arduino.h>
#include <avr/sleep.h>

#define SERIAL_SWAP     0
HardwareSerial *mySerial = &Serial;

// basically this sketch supports frequency hopping.
// Working, but It needs to be thoroughly tested though
#define NUM_CHANNELS 1


#include "configuration.h"
#include <datalinklayer.h>
#include "PacketHandler.h" // definition of UseBits struct
#include "print_things.h"
#include <RFM69.h>
#include "calibrate.h"
#include "SHTSensor.h"

#include "key.h"


/*****************************************************************************/
/***                      Class instances                                  ***/
/*****************************************************************************/
RADIO radio;
Configuration Config;
myMAC Mac(radio, Config, (uint8_t*) KEY, mySerial);
Calibration CalMode(Config, mySerial, &Mac, BUILD, (uint8_t*) KEY);
/*****************************************************************************/
/******                   Periodic Interrupt Timer and RTC setup         *****/
/*****************************************************************************/
#include "pitctrl.h"
PITControl PIT;

ISR(RTC_PIT_vect)
{
    PIT.interrupthandler();
}


/*****************************************************************************/
/***                       Actions                                         ***/
/*****************************************************************************/
#include "actions.h"

/* Globals */
extern action *actions;
extern uint8_t num_actions;

/*
if action.node is equal to the node in the received frame, then the flag word in the frame is evaluated.
4 flags can be set (bits 1 to bit 4 in the flag byte). Each flag can be mapped to a pin using the mask byte.
The pin is defined in the pin[i] array (up to 4 different pins for the four flags)
Actions are: ON, OFF, TOGGLE pin, PULSE with length 2^B * 0.125 seconds.

Example: Node 15,  pin 7 shall turn on on flag 02, pin 7 shall turn off on flag 04. pin 8 shall be toggled on flag 0x08, pin 5 shall be pulsed for 2 seconds on flag 0x10

node = 15;
pin = 7;
mask = 0x02;
mode = 0x1; //ON
duration =0; // doesn't matter
default_val=0; (off is default)

node = 15;
pin = 7;
mask = 0x04;
mode = 0x0; //OFF
duration =0; // doesn't matter
default_val=0; (off is default)

node = 15;
pin = 8;
mask = 0x08;
mode = 0x10; // TOGGLE
duration =0; // doesn't matter
default_val=0; (off is default)

node = 15;
pin = 5;
mask = 0x10;
mode = 0x3; //PULSE
duration = 0x2 B=2, 2^B = 4 * 0.125s = 0.5s
default_val=0; (off is default)
*/

/*
*/

// Input sense configuration (ISC)
void disablePinISC(uint8_t pin)
{
  PORT_t *port = digitalPinToPortStruct(pin);
  // Get bit position for getting pin ctrl reg
  uint8_t bit_pos = digitalPinToBitPosition(pin);

  // Calculate where pin control register is
  volatile uint8_t *pin_ctrl_reg = getPINnCTRLregister(port, bit_pos);

  // Disable ISC
  *pin_ctrl_reg = PORT_ISC_INPUT_DISABLE_gc;
}


/*****************************************************************************/
/***              SHT3x and SHTC3  Humidity Sensor                         ***/
/*****************************************************************************/
#include "sht_sensors.h" //TiNo2 wrapper class

SHTSensor *SHT3X=NULL;
SHTSensor *SHTC3=NULL;
SHTSensor *SHT4X=NULL;

HumiditySensor SensorData;


/*****************************************************************************/
/***              HTU21D  Humidity Sensor                                  ***/
/*****************************************************************************/
#include "HTU21D.h"
HTU21D *myHTU21D = NULL;

static bool HTU21D_Init(UseBits &enable)
{
    if (enable.HTU21D)
    {
        myHTU21D = new HTU21D(HTU21D_RES_RH12_TEMP14);
        enable.HTU21D = myHTU21D->begin();
        print_init_result(enable.HTU21D, "HTU21D");

        if (!enable.HTU21D)
        {
            delete myHTU21D;
            myHTU21D = NULL;
        }
    }
    return enable.HTU21D;
}


uint8_t HTU21D_Measure(uint8_t enabled, HTU21D *htu21d, HumiditySensor &Data);
uint8_t HTU21D_Measure(uint8_t enabled, HTU21D *htu21d, HumiditySensor &Data)
{
    uint8_t success = 0;
    if (enabled && htu21d)
    {
        I2C_pullup(Data.PowerPin);

        if (htu21d->begin())
        {
            delay(50);
            Data.temperature = htu21d->readTemperature();
            Data.humidity =    htu21d->readCompensatedHumidity(Data.temperature);
            success = enabled;
        }

        I2C_shutdown(Data.PowerPin);
    }
    return success;
}

/*****************************************************************************/
/***              One-Wire and DS18B20 Temperature Sensors                 ***/
/*****************************************************************************/
#if defined USE_DS18B20
#include "ds18b20.h"
#endif



/*****************************************************************************/
/***              BME280 air pressure and humidity Sensor                  ***/
/*****************************************************************************/
#include <BME280I2C.h>

BME280I2C *BME280;

static bool BME280_Init(UseBits &enable)
{
    if(enable.BME280)
    {
        BME280 = new BME280I2C();
        enable.BME280 = BME280->begin();
        print_init_result(enable.BME280, "BME280");
        if (!enable.BME280)
        {
            delete BME280;
            BME280=NULL;
        }
    }
    return enable.BME280;
}


uint8_t BME280_Measure(uint8_t enabled, BME280I2C *Bme280, HumiditySensor &Data);
uint8_t BME280_Measure(uint8_t enabled, BME280I2C *Bme280, HumiditySensor &Data)
{
    uint8_t success = 0;
    if (enabled && Bme280)
    {
        I2C_pullup(Data.PowerPin);

        delay(2); // start up time according to data sheet
        if (Bme280->begin())
        {
           BME280I2C::TempUnit tempUnit(BME280I2C::TempUnit_Celsius);
           BME280I2C::PresUnit presUnit(BME280I2C::PresUnit_hPa);

           Bme280->read(Data.pressure, Data.temperature, Data.humidity, tempUnit, presUnit);
           success = enabled;
        }
        else
        {
            mySerial->println("cant read BME280");
        }

        I2C_shutdown(Data.PowerPin);
    }
    return success;
}



/*****************************************************************************/
/***                       Pin Change Interrupts                           ***/
/*****************************************************************************/
#include "PinchangeInterrupt.h"
uint8_t event_triggered = 0;

// ISR for the Pin change Interrupt
void wakeUp0() { event_triggered |= 0x1; }
void wakeUp1() { event_triggered |= 0x2; }
void wakeUp2() { event_triggered |= 0x4; }
void wakeUp3() { event_triggered |= 0x8; }


/*****************************************************************************/
/****                         blink                                       ****/
/*****************************************************************************/
void activityLed (unsigned char state, unsigned int time = 0)
{
  if (Config.LedPin > 0)
  {
    pinMode(Config.LedPin, OUTPUT);
    if (time == 0)
    {
      digitalWrite(Config.LedPin, state);
    }
    else
    {
      digitalWrite(Config.LedPin, state);
      delay(time);
      digitalWrite(Config.LedPin, !state);
    }
  }
}




/*****************************************************************************/
/***                   READ VCC                                            ***/
/*****************************************************************************/
#include "analog.h"
long Vcal_x_ADCcal;

/**********************************************************************/
/*    each interrupt occupies 2 bits in serial protocol definition    */
/**********************************************************************/
static void extract_interrupts(byte flags)
{
    uint8_t intpts =0;
    if (flags & 0x2) intpts |= 0x1;
    if (flags & 0x4) intpts |= 0x1<<2;
    if (flags & 0x8) intpts |= 0x1<<4;
    if (flags & 0x10) intpts |= 0x1<<6;
    if (intpts !=0)
    {
        mySerial->print("&int=0x");
        mySerial->print(intpts,HEX);
    }
}

/**********************************************************************/
/*      Bitarray implementation                                       */
/**********************************************************************/

static byte CountSynced[32]; // 256 bits

byte getBit(int index)
{
    return (CountSynced[index/8] >> (7-(index & 0x7))) & 0x1;
}

void setBit(int index, byte value)
{
    CountSynced[index/8] = CountSynced[index/8] | (value & 0x1) << (7-(index & 0x7));  // does not work for 0es
}

/**********************************************************************/
/*      Rolling Code implementation                                   */
/**********************************************************************/
// Rolling Code is not supported due to limited EEPROM space.
// I am looking for a solution.
/*
#define TOLERANCE 10

static bool rolling_code_is_valid(byte nodeid, byte count_new)
{
    bool packet_is_valid = false;
    byte synced;
    byte count_old;

    EEPROM.get(512 + nodeid, count_old);
    synced = getBit(nodeid);

    if (synced)
    {
        int delta = count_new - count_old;
        if (delta < 0) delta += 256;
        if (delta > 0 && delta <= TOLERANCE)  // in sync
        {
            packet_is_valid = true;

            //mySerial->print(",Node in sync. delta=");mySerial->print(delta);

            if (delta >= TOLERANCE/2) // do write too often into EEPROM
            {
            EEPROM.put(512 + nodeid, count_new);
            //mySerial->print(", Update EEPROM");
            }
        }
        else // not in sync
        {
            // do nothing, just ignore this.
            //mySerial->print(",Node not in sync. delta=");mySerial->print(delta);
            packet_is_valid = false;
        }
    }

    else
    {
        EEPROM.put(512 + nodeid, count_new);
        setBit(nodeid,1);
        //mySerial->print("First time. Now synced");
        packet_is_valid = true;
    }
    return packet_is_valid;
}
*/

/*****************************************************************************/
unsigned long MeasurementIntervall_ms;
unsigned long last_measurement_millis=0;

#if NUM_CHANNELS >1
float frec[NUM_CHANNELS];
#endif


/*****************************************************************************/
/****                        SETUP                                        ****/
/*****************************************************************************/
void setup()
{
    // serial port, enable RX pin
    pinConfigure(1, PIN_DIR_INPUT, PIN_PULLUP_ON, PIN_INPUT_ENABLE);
    mySerial->swap(SERIAL_SWAP);
    mySerial->begin(SERIAL_BAUD);
    mySerial->println(SKETCHNAME);


    /***                     ***/
    /***     CALIBRATE?      ***/
    /***                     ***/
    CalMode.configure();


    /*** Print Logo ***/
    mySerial->println("  _______   _   _   _           ___");
    mySerial->println(" |__   __| (_) | \\ | |         |__ \\");
    mySerial->println("    | |     _  |  \\| |   ___      ) |");
    mySerial->println("    | |    | | | . ` |  / _ \\    / / ");
    mySerial->println("    | |    | | | |\\  | | (_) |  / /_");
    mySerial->println("    |_|    |_| |_| \\_|  \\___/  |____|");
    mySerial->println("    by nurazur\r\n");
    print_serial_number(mySerial);

    /***********************************************************/
    // normal initialization starts here
    // Dont load eeprom data here, it is done in the configuration (boot) routine. EEPROM is encrypted.

    MeasurementIntervall_ms = Config.Senddelay *8000L; // senddelay = Intervall in ms to measure Temp and adjust Radio Frequency

    Vcal_x_ADCcal = (long)Config.VccAtCalmV * Config.AdcCalValue;


    // this is an attempt to implement basic frequency hopping.
    // due to synthesizer settle times, some frames may be lost
    #if NUM_CHANNELS >1
    frec[0]= Config.frequency;
    frec[1]= 866.0;
    frec[2]= 867.0;
    frec[3]= 868.0;
    #endif



    /*********************************************************/
    /*************  INITIALIZE ACTOR MODULE ******************/
    /*********************************************************/

    init_actions(mySerial);

    /*********************************************************/
    PIT.init(Config.UseCrystalRtc);
    PIT.disable();

    /***                              ***/
    /***     Pin Change Interrupts    ***/
    /***                              ***/

    /* MegacoreX common.h */


    // #define LOW            0
    // #define HIGH           1

    // #define FALLING        2
    // #define RISING         3
    // #define CHANGE         4

    // #define INPUT          0
    // #define OUTPUT         1
    // #define INPUT_PULLUP   2

    // in this sketch we need:
    // FALLING -> 0
    // RISING -> 1
    // CHANGE -> CHANGE

    // only Pins Px2 and Px6 are fully asynchronuous, these are pins 10,14,18,22 (PC2, PD2, PD6, PF2)

    if (Config.PCI0Pin >=0)
    {
        pinMode(Config.PCI0Pin, Config.PCI0Mode);
        register_pci(0, Config.PCI0Pin, wakeUp0, Config.PCI0Trigger);
        #if DEBUG >= 1
        mySerial->print("PCI0pin:"); mySerial->print((uint8_t)Config.PCI0Pin);mySerial->print(", PCI0Mode: "); mySerial->print((uint8_t)Config.PCI0Mode);mySerial->print(", PCI0Trigger: "); mySerial->println((uint8_t)Config.PCI0Trigger);
        #endif
    }

    if (Config.PCI1Pin >=0)
    {
        pinMode(Config.PCI1Pin, Config.PCI1Mode);
        register_pci(1, Config.PCI1Pin, wakeUp1, Config.PCI1Trigger);
    }

      if (Config.PCI2Pin >=0)
      {
          pinMode(Config.PCI2Pin, Config.PCI2Mode);
          register_pci(2, Config.PCI2Pin, wakeUp2, Config.PCI2Trigger);
      }

      if (Config.PCI3Pin >=0)
      {
          pinMode(Config.PCI3Pin, Config.PCI3Mode);
          register_pci(3, Config.PCI3Pin, wakeUp3, Config.PCI3Trigger);
      }


        // initialize PIR Sensor, if configured
        // it will be still for 3 cycles.
        // PIR shares the event flag with PCI1. PCI1 remains active, if specified.
        //PIR.init();

    sei();


    /* start i2c bus */
    pinMode(Config.I2CPowerPin, OUTPUT);  // set power pin for Sensor to output
    digitalWrite(Config.I2CPowerPin, HIGH);
    delay(1);

    UseBits* sensors;
    sensors = (UseBits*)&Config.SensorConfig;

    Wire.swap(0);
    Wire.begin();
    HTU21D_Init(*sensors);
    if (sensors->SHTC3)
    {
        sensors->SHTC3 = SHT_Init(sensors->SHTC3, SHTC3, SHTSensor::SHTC3);
        print_init_result(sensors->SHTC3, "SHTC3");
    }

    if (sensors->SHT3X)
    {
        sensors->SHT3X = SHT_Init(sensors->SHT3X, SHT3X, SHTSensor::SHT3X);
        print_init_result(sensors->SHT3X, "SHT3X");
    }

    if (sensors->SHT4X)
    {
        sensors->SHT4X = SHT_Init(sensors->SHT4X, SHT4X, SHTSensor::SHT4X);
        print_init_result(sensors->SHT4X, "SHT4X");
    }

    #if defined USE_DS18B20
    if (sensors->DS18B20)
    {
        uint8_t num_ds18b20 = DS18B20_Init (sensors->DS18B20, Config.OneWirePowerPin, Config.OneWireDataPin);
        if (num_ds18b20==0) sensors->DS18B20=0;
    }
    #endif

    BME280_Init(*sensors);

    // Initialize LDR, if enabled
    if (sensors->BRIGHTNESS)
    {
        sensors->BRIGHTNESS = LDR_Init(Config.LdrPin);
        if (sensors->BRIGHTNESS)
            mySerial->println("LDR initialized.");
        else
            mySerial->println("LDR NOT initialized.");
    }


     /***  INITIALIZE RADIO MODULE ***/
    mySerial->print("RF Chip = "); Config.IsRFM69HW ?    mySerial->print("RFM69HCW") : mySerial->print("RFM69CW");  mySerial->println();
    mySerial->print ("FDEV_STEPS: ");mySerial->print(Config.FedvSteps);mySerial->println();

    Mac.radio_begin();  // re-initialize radio

    if (Config.LedPin)
    {
        activityLed(1, 1000); // LED on
    }
}


void loop()
{
    static uint16_t count=0;
    #if NUM_CHANNELS > 1
    static uint8_t frec_counter =0;
    #endif

    Mac.radio_receive(false); // non- blocking
    {
        bool packet_is_valid = true;
        if (Mac.rxpacket.success && Mac.rxpacket.payload[NODEID] != 0) // only show good packets
        {
            // common for all formats
            mySerial->print(Mac.rxpacket.payload[NODEID],DEC); mySerial->print(" ");

            // find out which protocol format is used
            if (!(Mac.rxpacket.payload[FLAGS] & 0x60)) // bit 5 and bit 6 in Flags are 0, flags is x00x xxxx
            {
                // This is the standard protocol for TiNo Sensors / Actors, good for HTU21D, SHT2x, SHT3x, SHTC3, SHT4x1 DS18B20 or BME280 plus an LDR
                Payload *pl = (Payload*) Mac.rxpacket.payload;

                mySerial->print("v=");  mySerial->print(pl->supplyV);
                mySerial->print("&c=");  mySerial->print(pl->count);
                mySerial->print("&t=");  mySerial->print((pl->temp - 1000)*4);
                mySerial->print("&h=");  mySerial->print(int(pl->humidity/2.0*100));
                mySerial->print("&f=");  mySerial->print(pl->flags & 0x1f,HEX);

                if (Mac.rxpacket.datalen >=12)
                {
                    mySerial->print("&p=");  mySerial->print(pl->pressure);
                    mySerial->print("&br=");  mySerial->print(pl->brightness);
                }

                extract_interrupts(pl->flags);

                //bool rolling_code_ok = rolling_code_is_valid(pl->nodeid, pl->count);
                mySerial->flush();
                //if (Mac.rxpacket.errorcode >=0 && rolling_code_ok)
                if (Mac.rxpacket.errorcode >=0)
                {
                    doaction(Mac.rxpacket.payload[NODEID], Mac.rxpacket.payload[FLAGS], actions, num_actions, &PIT);
                }
                //mySerial->print("&sy=");
                //rolling_code_ok ? mySerial->print("1") : mySerial->print("0") ;

            }

            else if ((Mac.rxpacket.payload[FLAGS] >> 5) == 0x2) // TiNo ACK Packet: 010x xxxx
            {
                PayloadAck *pl = (PayloadAck*)Mac.rxpacket.payload;

                mySerial->print("&f=");  mySerial->print(pl->flags & 0x1f,HEX);
                //fei  //can't report at this time  (no tag)
                mySerial->print("&c=");  mySerial->print(pl->count);
                mySerial->print("&t=");  mySerial->print(pl->temp);
                //RSSI;   //can't report at this time.

            }

            else   // it is another packet type.
            {
                switch (Mac.rxpacket.payload[ALT_PACKET_TYPE])
                {
                    case 1:
                        // string packet with length 16 (so we've got 13/5 bytes effective)
                        Config.FecEnable ? Mac.rxpacket.payload[8] = 0 : Mac.rxpacket.payload[16] = 0;
                        mySerial->print((char*)(Mac.rxpacket.payload+ALT_PACKET_TYPE+1)); mySerial->print(";");
                        break;
                    case 2:
                        // string packet with length 24 (so we've got 21 bytes effective)
                        Mac.rxpacket.payload[24] = 0;
                        mySerial->print((char*)(Mac.rxpacket.payload+ALT_PACKET_TYPE+1)); mySerial->print(";");
                        break;
                    case 3:
                        // BME280 with temperature, humidity and pressure data
                        {
                            PacketType3 *pl = (PacketType3*) Mac.rxpacket.payload;
                            mySerial->print("v=");  mySerial->print(pl->supplyV);
                            mySerial->print("&c=");  mySerial->print(pl->count);
                            mySerial->print("&t=");  mySerial->print((pl->temp - 1000)*4);
                            mySerial->print("&h=");  mySerial->print(int(pl->humidity/2.0*100));
                            mySerial->print("&p=");  mySerial->print(pl->pressure);
                            mySerial->print("&f=");  mySerial->print(pl->flags & 0x1f,HEX);
                            extract_interrupts(pl->flags);
                        }
                        break;
                    case 4:
                        {
                            //mySerial->println("Type 4");
                            PacketType4 *pl = (PacketType4*) Mac.rxpacket.payload;
                            mySerial->print("v=");    mySerial->print(pl->supplyV);
                            mySerial->print("&c=");   mySerial->print(pl->count);
                            mySerial->print("&t=");   mySerial->print((pl->temp - 1000)*4);
                            mySerial->print("&t1=");  mySerial->print((pl->temp1 - 1000)*4);
                            mySerial->print("&t2=");  mySerial->print((pl->temp2 - 1000)*4);
                            mySerial->print("&f=");  mySerial->print(pl->flags & 0x1f,HEX);
                            extract_interrupts(pl->flags);
                        }
                        break;
                    case 5:
                        {
                            PacketType5 *pl = (PacketType5*) Mac.rxpacket.payload;
                            mySerial->print("v=");    mySerial->print(pl->supplyV);
                            mySerial->print("&c=");   mySerial->print(pl->count);
                            mySerial->print("&t=");   mySerial->print((pl->temp - 1000)*4);
                            mySerial->print("&h=");   mySerial->print(int(pl->humidity/2.0*100));
                            mySerial->print("&t1=");  mySerial->print((pl->temp1 - 1000)*4);
                            mySerial->print("&br=");  mySerial->print(pl->brightness);
                            mySerial->print("&f=");  mySerial->print(pl->flags & 0x1f,HEX);
                            extract_interrupts(pl->flags);
                        }
                        break;
                    case 6:
                        {
                            PacketType6 *pl = (PacketType6*) Mac.rxpacket.payload;
                            mySerial->print("&c=");   mySerial->print(pl->count);
                            mySerial->print("&a=");   mySerial->print(pl->alarm_type);
                            switch((alarm_t) pl->alarm_type)
                            {
                                case temp:
                                    mySerial->print("&t="); mySerial->print((pl->value - 1000)*4);
                                    break;
                                case humidity:
                                    mySerial->print("&h=");   mySerial->print(int(pl->value/2.0*100));
                                    break;
                                case pressure:
                                    mySerial->print("&p=");  mySerial->print(pl->value);
                                    break;
                                case brightness:
                                    mySerial->print("&br=");  mySerial->print(pl->value);
                                    break;
                                case temp1:
                                    mySerial->print("&t1=");  mySerial->print((pl->value - 1000)*4);
                                    break;
                                case temp2:
                                    mySerial->print("&t2=");  mySerial->print((pl->value - 1000)*4);
                                    break;
                                case supplyV:
                                    mySerial->print("v=");  mySerial->print(pl->value);
                                    break;
                                default:
                                    break;
                            }
                            mySerial->print("&f=");  mySerial->print(pl->flags & 0x1f,HEX);
                            extract_interrupts(pl->flags);
                        }
                        break;
                    default:
                        // packet is invalid
                        packet_is_valid = false;
                        break;
                }

            }
            if (packet_is_valid)
            {
                mySerial->print("&rssi=");    mySerial->print(int(Mac.rxpacket.RSSI*10));
                //mySerial->print("&fo=");    mySerial->print(int16_t(Mac.rxpacket.FEI*radio.FSTEP), DEC); // need to multiply with the resolution of the PLL (in Hz), but I don't need fractions of a Hz
                if (Config.FecEnable) { mySerial->print("&be=");  mySerial->print(Mac.rxpacket.numerrors); }
                mySerial->println("");
                Mac.rxpacket.payload[NODEID] =0;
            }
        }
        else if (Mac.rxpacket.errorcode<0)
        {
             /* Errorcodes:
                 -1:  could not decode FEC data (too many bit errors in codes)
                 -2:  data length does not match
                 -3:  not my message or address corrupted
             */
             /*
            char errors[3];
            errors[0] = "-1: could not decode FEC data (too many bit errors in codes)";
            errors[1] = "-2: data length does not match";
            errors[2] = "-3: not my message or address corrupted";
            int8_t code = -(Mac.rxpacket.errorcode+1);

            mySerial->print("Error Code: "); mySerial->print(errors[code]);
            */
        }
    }

    /***  Frequency hopping ***/
    #if NUM_CHANNELS > 1
    if(radio.noRx())
    {
        frec_counter++;
        if (frec_counter >= NUM_CHANNELS) frec_counter=0;
        radio.switch_frequencyMHz(frec[frec_counter]);
        delayMicroseconds(100); // wait for the synthesizer to settle.
    }
    #endif



    if (MeasurementIntervall_ms > 0) // local measurement
    {
        if ((millis() > last_measurement_millis) || event_triggered)
        {
            last_measurement_millis = millis() + MeasurementIntervall_ms;

            // measure temperature
            float temperature;
            UseBits *u;
            u = (UseBits*)&Config.SensorConfig;


            uint8_t success =0;
            success |= HTU21D_Measure(Config.SensorConfig & HTU21D_bm, myHTU21D, SensorData);
            if (success & HTU21D_bm)
            {
                //print_humidity_sensor_values("HTU21D", SensorData.temperature, SensorData.humidity, mySerial);
            }

            success |= SHT_Measure(Config.SensorConfig & SHT3X_bm, SHT3X, SensorData);
            if (success & SHT3X_bm)
            {
                //print_humidity_sensor_values("SHT3x", SensorData.temperature, SensorData.humidity, mySerial);
            }

            success |= SHT_Measure(Config.SensorConfig & SHTC3_bm, SHTC3, SensorData);
            if (success & SHTC3_bm)
            {
                //print_humidity_sensor_values("SHTC3", SensorData.temperature, SensorData.humidity, mySerial);
            }

            success |= SHT_Measure(Config.SensorConfig & SHT4X_bm, SHT4X, SensorData);
            if (success & SHT4X_bm)
            {
                //print_humidity_sensor_values("SHT4X", SensorData.temperature, SensorData.humidity, mySerial);
            }

            success |= BME280_Measure(Config.SensorConfig & BME280_bm, BME280, SensorData);
            if (success & BME280_bm)
            {
                //print_humidity_sensor_values("BME280", SensorData.temperature, SensorData.humidity, mySerial);
            }

            if(success)
            {
                temperature = SensorData.temperature;
            }
            else
            {
                temperature = radio.readTemperature(0) + Config.radio_temp_offset/10.0;
            }

            // brightness
            uint16_t br;

            #if defined USE_DS18B20
            // Dallas DS18B20 Measurement
            uint8_t num_ds18b20=0;
            float t[3];
            if(u->DS18B20)
            {
                t[0]=t[1]=t[2]=0;
                num_ds18b20 = DS18B20_Measure(u->DS18B20, t, Config.OneWirePowerPin);
            }
            #endif

            mySerial->print(Config.Nodeid); mySerial->print(" ");
            mySerial->print("v=");   mySerial->print((uint16_t)getVcc(Vcal_x_ADCcal));
            mySerial->print("&c=");  mySerial->print(++count);
            mySerial->print("&t=");  mySerial->print(temperature*100,0);

            #if defined USE_DS18B20
            for (uint8_t i=0; i<num_ds18b20; i++)
            {
                mySerial->print("&t"); mySerial->print(i); mySerial->print("="); mySerial->print(t[i]*100,0);
            }
            #endif

            mySerial->print("&h=");  mySerial->print(SensorData.humidity*100,0);
            if (success & BME280_bm)
            {
                mySerial->print("&p=");  mySerial->print((uint32_t)floor(SensorData.pressure*100),0);
            }

            if (LDR_Measure(u->BRIGHTNESS, Config.LdrPin, br))
            {
                mySerial->print("&br="); mySerial->print(br);
            }

            mySerial->print("&f=");
            if (event_triggered)
            {
                mySerial->print(event_triggered <<1);
            }
            else
                mySerial->print("1"); // heartbeat

            // adjust radio frequencuy according to FT table, if applicable
            // in Atmega4808 the FT table is not availabe from eeprom.
            // mySerial->println("Temperature Measurement and Frequency Tuning.");
            /*
            if (Config.UseRadioFrequencyCompensation)
            {
                int fo_steps = Config.FedvSteps - Mac.radio_calc_temp_correction(t);
                radio.setFrequency((Config.frequency * 1000000)/radio.FSTEP + fo_steps );
                mySerial->print("&fo=");  mySerial->print((int)(fo_steps*radio.FSTEP));
            }
            */

            mySerial->println();
        }
    }

    if (event_triggered) // a local Port change interrupt occured.
    {
        doaction(Config.Nodeid, event_triggered, actions, num_actions, &PIT);
        event_triggered =0;
    }
}




