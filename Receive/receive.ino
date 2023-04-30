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
// extended by nurazur nurazur@gmail.com 2014 - 2023
// the RF protocol is completely binary, see https://github.com/nurazur/tino


#define SKETCHNAME "TiNo2 receive.ino Ver 30/04/2023"
#define BUILD 10


#include <avr/sleep.h>
//#include <HardwareSerial.h>
#define SERIAL_BAUD 230400
#define SERIAL_SWAP     0
HardwareSerial *mySerial = &Serial;

#define NUM_CHANNELS 1


#include "configuration.h"
#include <datalinklayer.h>
#include "PacketHandler.h" // definition of UseBits struct

/* For TiNo-HP, TiNo-LC, TiNo-SW1, TiNo-SW  the RFM69 library is included */
//#include <RFM69.h>
#include <RFM69.h>
/* For TiNo-HPCC the CC1101 library is included. comment out the RFM69 library and uncomment the below line. */
//#include <CC1101.h>

#include <EEPROM.h>
#include "calibrate.h"

#define KEY  "TheQuickBrownFox"

/*****************************************************************************/
/***                       Actions Struct                                  ***/
/*****************************************************************************/
typedef struct {
  uint8_t node;      // the node to trigger
  uint8_t port;      // the port to trigger
  uint8_t mask;      // to which bit of the flag byte it is mapped
  uint8_t onoff;    // type of action:  OFF (00) ON(01) TOGGLE (10) PULSE (11); Pulse length = 2^B, where B = bits [23456], B= (action[i] >>2) &0xf, C = default on power up, C = bit[7] 1= on, 0 = off
} action;

/*****************************************************************************/
/***                            I2C Driver                                 ***/
/*****************************************************************************/
// SHT21/HTU21D connects through SoftwareWire, so that SCL and SDA can be any pin
// Add pullup resistors between SDA/VCC and SCL/VCC if not yet provided on Module board

// if I2C bus has no pullups externally, you can use internal Pullups instead.
// Internal pullup resistors are ~30kOHm. Since we are clocking slowly,
// it works. However, they need to be disabled in sleep mode, because SoftwareWire
// keeps them enabled even if i2c is ended with end()
#define USE_I2C_PULLUPS  true // use this for HTU21D in case no Pullups are mounted.
//#define USE_I2C_PULLUPS false // use this in case external Pullups are used.

/*****************************************************************************/
/***                  I2C Bus Tools                                        ***/
/*****************************************************************************/
static void print_humidity_sensor_values(const char* sensorname, float t, float h)
{
    mySerial->print(sensorname);
    mySerial->print(": ");
    mySerial->print(t, 2);
    mySerial->print(" degC, ");
    mySerial->print(h, 2);
    mySerial->println(" %rH");
}


/*****************************************************************************/
/***                      Class instances                                  ***/
/*****************************************************************************/
RADIO radio;
Configuration config;
myMAC Mac(radio, config, (uint8_t*) KEY, mySerial);
Calibration CalMode(config, mySerial, &Mac, BUILD, (uint8_t*) KEY);


/*****************************************************************************/
/***                       Actions                                         ***/
/*****************************************************************************/
#define ADR_NUM_ACTIONS sizeof(Configuration)
#define ADR_ACTIONS   ADR_NUM_ACTIONS + 1
#define MAX_NUM_ACTIONS 40

action *actions;
uint8_t num_actions;
/*
if action.node is equal to this node, then the flag word in the telegram is evaluated.
4 flags can be set (bits 1 to bit 4 in the flag byte). Each flag is mapped to a port using the mask byte.
the port is defined in the port[i] array (up to 4 different ports for the four flags)
Actions are: ON, OFF, TOGGLE Port, PULSE with length 2^B * 0.5 seconds.

Example: Node 15,  Port 7 shall turn on on flag 02, port 7 shall turn off on flag 04. Port 8 shall be toggled on flag 0x08, Port 5 shall be pulsed for 2 seconds on flag 0x10

node = 15;
port = 7;
mask = 0x02;
onoff = 0x1;

node = 15;
port = 7;
mask = 0x04;
onoff = 0x0;

node = 15;
port = 8;
mask = 0x08;
onoff = 0x10;

node = 15;
port = 5;
mask = 0x10;
onoff = 0x3 | (0x02 << 2); // B=2, 2^B = 4 * 0.5s = 2s
*/

unsigned char pulse_port=0;  // the port for which a pulse is defined and active.
unsigned int pulse_duration=0; // counter

void doaction (uint8_t nodeid, uint8_t flags, action actions[], uint8_t num_actions)
{
    for (uint8_t i=0; i< num_actions; i++)
    {
        if(actions[i].node == nodeid) // nodeid matches the actions's nodeId
        {
            if (actions[i].mask & flags ) // the action's trigger bit is set
            {
                uint8_t act = actions[i].onoff &0x3; // Action mode
                switch (act)
                {
                    case 0:     // OFF
                        digitalWrite(actions[i].port, 0);
                        break;
                    case 1:     // ON
                        digitalWrite(actions[i].port, 1);
                        break;
                    case 2:     // Toggle
                        digitalWrite(actions[i].port, !digitalRead(actions[i].port));
                        break;
                    case 3:     //Pulse
                        digitalWrite(actions[i].port, 1);
                        pulse_port = actions[i].port;

                        uint8_t exp = (actions[i].onoff>>2) &0x7F;

                        if (exp <= 4) // less than or equal 8 seconds
                        {
                            pulse_duration  =1;
                            //setup_watchdog(5+exp);
                        }
                        else // more than  8s
                        {
                            pulse_duration = pow(2,exp-4);
                            //setup_watchdog(9); //8s
                        }
                        break;
                }
            }
        }
    }
}

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
        mySerial->print("HTU21D: ");
        enable.HTU21D = myHTU21D->begin();
        if (enable.HTU21D)
        {
            mySerial->print("init(): success\n");
        }
        else 
        {
            mySerial->print("init(): failed\n");
            delete myHTU21D;
            myHTU21D = NULL;
        }
    }
    return enable.HTU21D;
}


uint8_t HTU21D_Measure(bool enabled, float &temperature, float &humidity)
{
    uint8_t success = 0;
    if (enabled && myHTU21D)
    {
        digitalWrite(config.I2CPowerPin, HIGH);
        if (myHTU21D->begin())
        {
            delay(50);
            temperature = myHTU21D->readTemperature();
            humidity =    myHTU21D->readCompensatedHumidity(temperature);
            print_humidity_sensor_values("HTU21D",temperature, humidity);
            success=0x1;
        }
        //digitalWrite(config.I2CPowerPin, LOW);
        //I2C_shutdown();
    }
    return success;
}




/*
void setup_watchdog(int timerPrescaler)
{
    if (timerPrescaler > 9 ) timerPrescaler = 9; //Correct incoming amount if need be
    byte bb = timerPrescaler & 7;
    if (timerPrescaler > 7) bb |= (1<<5); //Set the special 5th bit if necessary
    //This order of commands is important and cannot be combined
    MCUSR &= ~(1<<WDRF); //Clear the watchdog reset
    WDTCSR |= (1<<WDCE) | (1<<WDE); //Set WD_change enable, set WD enable
    WDTCSR = bb; //Set new watchdog timeout value
    WDTCSR |= _BV(WDIE); //Set the interrupt enable, this will keep unit from resetting after each int
}

void stop_watchdog(void)
{
    noInterrupts();
    //This order of commands is important and cannot be combined
    MCUSR &= ~(1<<WDRF); //Clear the watchdog reset
    WDTCSR |= (1<<WDCE) | (1<<WDE); //Set WD_change enable, set WD enable
    WDTCSR = 0; //Set new watchdog timeout value
    interrupts();
}

ISR(WDT_vect)
{
    pulse_duration--;
    if (pulse_duration == 0)
    {
        if (pulse_port >0) digitalWrite(pulse_port, 0);
        pulse_port = 0;
        stop_watchdog();
    }
}
*/





/*****************************************************************************/
/***                       Pin Change Interrupts                           ***/
/*****************************************************************************/
#include "pinchange_interrupt.h"
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
  if (config.LedPin)
  {
    pinMode(config.LedPin, OUTPUT);
    if (time == 0)
    {
      digitalWrite(config.LedPin, state);
    }
    else
    {
      digitalWrite(config.LedPin, state);
      delay(time);
      digitalWrite(config.LedPin, !state);
    }
  }
}


/*****************************************************************************/
/*
     Read VCC by  measuring the 1.5V reference and taking VCC as reference voltage.
     set the reference to Vcc and the measurement to the internal 1.1V reference
*/
/*****************************************************************************/

/*****************************************************************************/
/***                   READ VCC                                            ***/
/*****************************************************************************/

/***   READ VCC ***/

long Vcal_x_ADCcal = 1500L * 1023L;
#define SAMPLE_ACCUMULATION 0x5

// possible modes are: INTERNAL0V55, INTERNAL1V1, INTERNAL2V5, INTERNAL4V34, INTERNAL1V5
int analogReadInternalRef(uint8_t mode)
{
  #if defined(ADC0)
  
  // save registers
  uint8_t vref_ctrla = VREF.CTRLA;
  uint8_t adc0_muxpos = ADC0.MUXPOS;
  uint8_t adc0_ctrlb = ADC0.CTRLB;
  
  // setup DACREF in AC0
  VREF.CTRLA = (VREF.CTRLA & ~(VREF_AC0REFSEL_gm)) | (mode << VREF_AC0REFSEL_gp);
  

  /* Reference should be already set up and should be VDD */
  // ADC0.CTRLC =0b x101 xxxx
  
  
  /* set input to DACREF0 */
  ADC0.MUXPOS = ADC_MUXPOS_DACREF_gc;
  
  /* set sample accumulation*/
  ADC0.CTRLB = SAMPLE_ACCUMULATION;
  
  /* Start conversion */
  ADC0.COMMAND = ADC_STCONV_bm;

  /* Wait for result ready */
  while (!(ADC0.INTFLAGS & ADC_RESRDY_bm))
    ;

  // restore registers;
  VREF.CTRLA = vref_ctrla;
  ADC0.MUXPOS = adc0_muxpos;
  ADC0.CTRLB = adc0_ctrlb;
  /* Combine two bytes */
  return ADC0.RES;

#else /* No ADC, return 0 */
  return 0;
#endif
}

long readVcc()
{
    return analogReadInternalRef(INTERNAL1V5) / (1<<SAMPLE_ACCUMULATION);
}

float getVcc(long vref)
{
    return (float)vref / readVcc();
}

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
/**********************************************************************/

unsigned long MeasurementIntervall_ms;
unsigned long last_measurement_millis=0; 

#if NUM_CHANNELS >1 
float frec[NUM_CHANNELS];
#endif

void setup()
{
    /***  INITIALIZE SERIAL PORT ***/

    #ifdef SOFTSERIAL
        pinMode(RX_PIN, INPUT);
        pinMode(TX_PIN, OUTPUT);
    #endif

    // serial port, enable RX pin
    pinConfigure(1, PIN_DIR_INPUT, PIN_PULLUP_ON, PIN_INPUT_ENABLE);
    mySerial->swap(SERIAL_SWAP);
    mySerial->begin(SERIAL_BAUD);
    
    //mySerial->begin(SERIAL_BAUD);
    mySerial->println(SKETCHNAME);


    /***                     ***/
    /***     CALIBRATE?      ***/
    /***                     ***/
    CalMode.configure();

    MeasurementIntervall_ms = config.Senddelay *8000L; // senddelay = Intervall in ms to measure Temp and adjust Radio Frequency
       
    Vcal_x_ADCcal = (long)config.VccAtCalmV * config.AdcCalValue;
    
    #if NUM_CHANNELS >1 
    frec[0]= config.frequency;
    frec[1]= 866.0;
    frec[2]= 867.0;
    frec[3]= 868.0;
    #endif
    
    /*************  INITIALIZE ACTOR MODULE ******************/
    EEPROM.get(ADR_NUM_ACTIONS, num_actions);


    if (num_actions > 0 && num_actions <= MAX_NUM_ACTIONS)
    {
        mySerial->print("number of actions: "); mySerial->println(num_actions);
        actions = new action[num_actions];
    }
    else
    {
        num_actions = 0;
        actions = NULL;
    }

    for(int i=0; i< num_actions; i++)
    {
        EEPROM.get(ADR_ACTIONS + i*sizeof(action), actions[i]);
    }

    // verify the checksum
    uint16_t cs_from_eeprom;
    EEPROM.get(ADR_ACTIONS + MAX_NUM_ACTIONS * sizeof(action), cs_from_eeprom);
    uint16_t cs_from_data = CalMode.checksum_crc16((uint8_t*) actions, sizeof(action) * num_actions);

    if ((cs_from_eeprom ^ cs_from_data) != 0)
    {
        mySerial->println("Checksum incorrect for Actions.");
        num_actions =0;
    }

    for(int i=0; i< num_actions; i++)
    {
        pinMode(actions[i].port, OUTPUT);
        digitalWrite(actions[i].port, actions[i].onoff>>7);
    }

    
    /************************************/
    /***     Pin Change Interrupts    ***/
    /************************************/

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
    
    if (config.PCI0Pin >=0)
    {
        pinMode(config.PCI0Pin, config.PCI0Mode);
        register_pci(0, config.PCI0Pin, wakeUp0, config.PCI0Trigger);
    }

    if (config.PCI1Pin >=0)  
    {
        pinMode(config.PCI1Pin, config.PCI1Mode);
        register_pci(1, config.PCI1Pin, wakeUp1, config.PCI1Trigger);
    }
  
    if (config.PCI2Pin >=0)  
    {
        pinMode(config.PCI2Pin, config.PCI2Mode);
        register_pci(2, config.PCI2Pin, wakeUp2, config.PCI2Trigger);
    }
  
    if (config.PCI3Pin >=0)  
    {
        pinMode(config.PCI3Pin, config.PCI3Mode);
        register_pci(3, config.PCI3Pin, wakeUp3, config.PCI3Trigger);
    }
    /***********************************************************/
    // normal initialization starts here
    // Dont load eeprom data here, it is done in the configuration (boot) routine. EEPROM is encrypted.

    
    /* start i2c bus */
    pinMode(config.I2CPowerPin, OUTPUT);  // set power pin for Sensor to output
    digitalWrite(config.I2CPowerPin, HIGH);
    delay(1);
    
    UseBits* u;
    u = (UseBits*)&config.SensorConfig;
    
    HTU21D_Init(*u);
    
    /*
    digitalWrite(config.I2CPowerPin, LOW);
    if (USE_I2C_PULLUPS)
    {
        pinMode(config.SDAPin, INPUT);
        pinMode(config.SCLPin, INPUT);
    }
    */
    
     /***  INITIALIZE RADIO MODULE ***/
    mySerial->print("RF Chip = "); config.IsRFM69HW ?    mySerial->print("RFM69HCW") : mySerial->print("RFM69CW");  mySerial->println();
    mySerial->print ("FDEV_STEPS: ");mySerial->print(config.FedvSteps);mySerial->println();

    Mac.radio_begin();  // re-initialize radio

  if (config.LedPin)
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
        if (Mac.rxpacket.success) // only show good packets
        {
            // common for all formats
            mySerial->print(Mac.rxpacket.payload[NODEID],DEC); mySerial->print(" ");

            // find out which protocol format is used
            if (!(Mac.rxpacket.payload[FLAGS] & 0x60)) // bit 5 and bit 6 in Flags are 0, flags is x00x xxxx
            {
                // This is the standard protocol for TiNo Sensors / Actors, good for HTU21D, SHT2x, SHT3x, 1 DS18B20 or BME280 plus an LDR
                Payload *pl = (Payload*) Mac.rxpacket.payload;
                
                mySerial->print("v=");  mySerial->print(pl->supplyV);
                mySerial->print("&c=");  mySerial->print(pl->count);
                mySerial->print("&t=");  mySerial->print((pl->temp - 1000)*4);
                mySerial->print("&h=");  mySerial->print(int(pl->humidity/2.0*100));
                mySerial->print("&f=");  mySerial->print(pl->flags,HEX);
                
                if (Mac.rxpacket.datalen >=12)
                {
                    mySerial->print("&p=");  mySerial->print(pl->pressure);
                    mySerial->print("&br=");  mySerial->print(pl->brightness);
                }
                
                //extract_interrupts(pl->flags);
                /*
                bool rolling_code_ok = rolling_code_is_valid(pl->nodeid, pl->count);

                if (Mac.rxpacket.errorcode >=0 && rolling_code_ok)
                {
                    doaction(Mac.rxpacket.payload[NODEID], Mac.rxpacket.payload[FLAGS], actions, num_actions);
                }
                mySerial->print("&sy=");
                rolling_code_ok ? mySerial->print("1") : mySerial->print("0") ;
                */
            }

            else if ((Mac.rxpacket.payload[FLAGS] >> 5) == 0x2) // TiNo ACK Packet: 010x xxxx
            {
                PayloadAck *pl = (PayloadAck*)Mac.rxpacket.payload;

                mySerial->print("&f=");  mySerial->print(pl->flags,HEX);
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
                        config.FecEnable ? Mac.rxpacket.payload[8] = 0 : Mac.rxpacket.payload[16] = 0;
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

                            extract_interrupts(pl->flags);
                        }
                        break;
                    case 4:
                        {
                            //Serial.println("Type 4");
                            PacketType4 *pl = (PacketType4*) Mac.rxpacket.payload;
                            mySerial->print("v=");    mySerial->print(pl->supplyV);
                            mySerial->print("&c=");   mySerial->print(pl->count);
                            mySerial->print("&t=");   mySerial->print((pl->temp - 1000)*4);
                            mySerial->print("&t1=");  mySerial->print((pl->temp1 - 1000)*4);
                            mySerial->print("&t2=");  mySerial->print((pl->temp2 - 1000)*4);
                            
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
                        }
                        break;
                    default:
                        // packet is invalid
                        break;
                }

            }
            mySerial->print("&rssi=");    mySerial->print(int(Mac.rxpacket.RSSI*10));
            //mySerial->print("&fo=");    mySerial->print(int16_t(Mac.rxpacket.FEI*radio.FSTEP), DEC); // need to multiply with the resolution of the PLL (in Hz), but I don't need fractions of a Hz
            if (config.FecEnable) { mySerial->print("&be=");  mySerial->print(Mac.rxpacket.numerrors); }
            mySerial->println("");
            Mac.rxpacket.payload[NODEID] =0;
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
            if (Mac.rxpacket.errorcode ==-1)
            {
                mySerial->print("Error Code: ");mySerial->print(Mac.rxpacket.errorcode);
                mySerial->print(" NodeId,"); mySerial->print(Mac.rxpacket.payload[NODEID],DEC);
                mySerial->print(",s=");     mySerial->print(Mac.rxpacket.RSSI); mySerial->println();
            }
        
        }
    }
    #if NUM_CHANNELS > 1
    if(radio.noRx())
    {
        frec_counter++;
        if (frec_counter >= NUM_CHANNELS) frec_counter=0;
        //radio.setFrequencyMHz(frec[frec_counter]); 
        radio.switch_frequencyMHz(frec[frec_counter]); 
        delayMicroseconds(100);
    }
    #endif
    
    if (event_triggered) // a local Port change interrupt occured.
    {
        //mySerial->print("Event triggered: "); mySerial->print(event_triggered);
        //mySerial->print(", Nodeid: "); mySerial->print(config.Nodeid);
        //mySerial->println();

        doaction(config.Nodeid, event_triggered, actions, num_actions);
        event_triggered =0;
    }
    
    if (MeasurementIntervall_ms >0) // local measurement 
    {
        if (millis() > last_measurement_millis)
        {
            last_measurement_millis = millis() + MeasurementIntervall_ms;
            
            // measure temperature
            float t, h=0;
            UseBits *u;
            u = (UseBits*)&config.SensorConfig;
            
            if (!HTU21D_Measure(u->HTU21D, t, h))
            {
                t = radio.readTemperature(0) + config.radio_temp_offset/10.0;
                //mySerial->println("it is not possible to measure with HTU21D");
            }
            
            mySerial->print(config.Nodeid); mySerial->print(" ");
            mySerial->print("v=");   mySerial->print((uint16_t)getVcc(Vcal_x_ADCcal));
            mySerial->print("&c=");  mySerial->print(++count);
            mySerial->print("&t=");  mySerial->print(t*100,0);
            mySerial->print("&h=");  mySerial->print(h*100,0);
            mySerial->print("&f=1"); // heartbeat
            
            // adjust radio frequencuy according to FT table, if applicable
            //mySerial->println("Temperature Measurement and Frequency Tuning.");
            if (config.UseRadioFrequencyCompensation)
            {
                int fo_steps = config.FedvSteps - Mac.radio_calc_temp_correction(t);
                radio.setFrequency((config.frequency * 1000000)/radio.FSTEP + fo_steps );
                mySerial->print("&fo=");  mySerial->print((int)(fo_steps*radio.FSTEP));
            }
            mySerial->println();
        }
    }
}




