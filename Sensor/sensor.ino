// RFM69CW Sender for TiNo2 temperature / humidity Sensors with Watchdog.
// Supports Sensors with HTU21D, SHT20, SHT21, SHT25, SHT3C(default on-board chip), SHT30, SHT31, SHT35, BME280, DS18B20
// built for AVR ATMEGA4808


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

/*****************************************************************************/
/***  Third Party libraries used in this project                           ***/
/*****************************************************************************/
// Uses HTU21D from https://github.com/enjoyneering/HTU21D under BSD licence
// Uses a RFM69 Driver originally from LowPowerlab.com However this driver has been changed so much so that it is barely recognizable.
// uses a modified BME280 Driver from https://github.com/finitespace/BME280 under GNU General Public License version 3 or later
// Uses OneWire             // license terms not clearly defined.
// Uses DallasTemperature under GNU LGPL version 2.1 or any later version.
// Uses Sensirion SHT library from https://github.com/Sensirion/arduino-sht under BSD 3-Clause "New" or "Revised" License
// Uses MAX31865 from Ole Wolf <wolf@blazingangles.com> at https://github.com/olewolf/arduino-max31865.git under GNU General Public License version 3 or later
//
/*****************************************************************************/
/***  Revision History                                                     ***/
/*****************************************************************************/
/*
    Build 3:
        Support for EEPROM selected MAC functions (Encryption, FEC, Interleaving)
        Support for external Crystal, compiler option (because of interfering ISR's)
        RFM69CW TX Gauss shaping adjustable
    Build 4:
        Support for RFM69HCW, P1 Mode, P1+2 Mode, Pa Boost mode.
    Build 5:
        Add Serial port enable in Config settings
    Build 6:
        Implement MAC Class
        cleanup code, remove obsolete defines and comments. Support ACK's
    Build 7:
        FEI alignment in calibration
        EEPROM encrypted
    Build 9: various improvements. VCC is now tested during TX Burst. Support DS18B20 Temperature Sensor, SHT30x, BME280

    Build 10:
        one firmware for all sensors
        support for LDR (brightness reading), requires a analog pin (A0 or A1)
        choice of sleep mode can be set in configuration - WDT vs crystal timer
        possible External Heartbeat using TPL5110
        for each PCINT a individual Gateway ID (multi-channel remote control)
        calibration mode: 't' sends a dummy packet
        calibration mode: 'to' starts den OOK Modus (send a CW signal)
        calibration mode: 'tt' retrieves the Tepmerature reading from the RFM69 chip
        calibration mode: Offset for RFM69 temperature in configuration
        Receiver can output locally measured sensor data
    Build 11:
        support SHT4x Temperature/Humidity Sensors
        support AVR64DD32 and AVR64DD28 MCU's using DxCore
*/
// Core:        for 4808: MegaCoreX (https://github.com/MCUdude/MegaCoreX)
// Core:        for AVRxxDDxx DxCore (https://github.com/SpenceKonde/DxCore)

#include "Arduino.h"
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include "SHTSensor.h"
#include "datalinklayer.h"
#include "PacketHandler.h"
#include "SPI.h"
#include "PinchangeInterrupt.h"
#include "print_things.h"
#include "pitctrl.h"

#define FILENAME "TiNo2 V2.4  18/11/2023"
#define BUILD 11
#define USE_RADIO
#define RADIO_SPI_PINSWAP 0
#define SEND_BURST
//#define BATTERYTEST

PacketHandler *TiNo=NULL;

#define P(a) if(Config.SerialEnable) mySerial->a


#define DEBUG 0
/*****************************************************************************/
/***   Radio Driver Instance                                               ***/
/*****************************************************************************/
#include "RFM69.h"

RADIO radio;
/*****************************************************************************/
/***  EEPROM Access  and device calibration / Configuration                ***/
/*****************************************************************************/
#include "configuration.h"
#include "calibrate.h"

/*****************************************************************************/
/***                            Encryption                                 ***/
/*****************************************************************************/
//encryption is OPTIONAL by compilation switch
//to enable encryption you will need to:
// - provide a 16-byte encryption KEY (same on all nodes that talk encrypted)
// - set the varable ENCRYPTION_ENABLE to 1 in the EEPROM, at runtime in Cfg.EncryptionEnable

#define KEY     "TheQuickBrownFox"
/*****************************************************************************/
/***                   Device specific Configuration                       ***/
/*****************************************************************************/
Configuration Config;


/*****************************************************************************/
/***                   Data Link Controller                                ***/
/*****************************************************************************/
myMAC Mac(radio, Config, (uint8_t*) KEY);


/*****************************************************************************/
/***                            Serial Port                                ***/
/*****************************************************************************/
// only Hardwareserial is supported.
// 5 serial ports are defined; Serial, Serial1 and Serial2
// Serial and Serial2 have alternate pin positions, using the swap funktion.
#define SERIAL_BAUD     57600
#define SERIAL_SWAP     0
HardwareSerial *mySerial = &Serial;

/*****************************************************************************/
/***                   Calibration Module                                  ***/
/*****************************************************************************/
Calibration CalMode(Config, mySerial, &Mac, BUILD, (uint8_t*) KEY);


/*****************************************************************************/
/***                   Sleep mode                                          ***/
/*****************************************************************************/

uint16_t watchdog_counter;
//bool watchdog_expired = false;

// interrupt service routine for RTC periodic timer
ISR(RTC_PIT_vect)
{
    RTC.PITINTFLAGS = RTC_PI_bm;              // clear interrupt flag
    watchdog_counter++;
}


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
/******                   Periodic Interrupt Timer and RTC setup         *****/
/*****************************************************************************/

PITControl PIT;


/*****************************************************************************/
/***                  I2C Bus Tools                                        ***/
/*****************************************************************************/
#include "i2c_common.h"


/*****************************************************************************/
/***              SHT3x and SHTC3  Humidity Sensor                         ***/
/*****************************************************************************/
#include "sht_sensors.h"

HumiditySensor SensorData;

SHTSensor *SHT3X=NULL;
SHTSensor *SHTC3=NULL;
SHTSensor *SHT4X=NULL;

static bool SHT4X_Init(UseBits &enable)
{
    if (enable.SHT4X)
    {
        enable.SHT4X = SHT_Init(enable.SHT4X, SHT4X, SHTSensor::SHT4X); // just an example how to use the general SHT initialization
        print_init_result(enable.SHT4X, "SHT4X");
    }
    return enable.SHT4X;
}

static bool SHTC3_Init(UseBits &enable)
{
    if (enable.SHTC3)
    {
        enable.SHTC3 = SHT_Init(enable.SHTC3, SHTC3, SHTSensor::SHTC3);
        print_init_result(enable.SHTC3, "SHTC3");
    }
    return enable.SHTC3;
}

static bool SHT3X_Init(UseBits &enable)
{
    if (enable.SHT3X)
    {
        enable.SHT3X = SHT_Init(enable.SHT3X, SHT3X, SHTSensor::SHT3X);
        print_init_result(enable.SHT3X, "SHT3X");
    }
   return enable.SHT3X;
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
// using DS18B20 is not recommended, the conversion time takes much longer
// (700-900 ms) than with I2C base sensors (<50ms).
// Commenting out the header will remove the entire DS18B20 Module from the Sketch.
#include "ds18b20.h"

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


static void Reset_SPI(uint8_t swap, uint8_t mode = SPI_MODE0)
{
    // when max31865 and RFM have the same SPI, we don't need to stop it, just switch.
    if (swap != RADIO_SPI_PINSWAP)
    {
        SPI.end();
        SPI.swap(swap);
    }

    SPI.begin();
    if (mode != SPI_MODE0) SPI.setDataMode(mode);
}

static void SPI_MISO_Enable(uint8_t swap)
{
    if(swap == 0)
    {
        pinConfigure(PIN_PA5, PIN_DIR_INPUT, PIN_PULLUP_ON, PIN_INPUT_ENABLE);
    }
    else if (swap==1)
    {
        pinConfigure(PIN_PC1, PIN_DIR_INPUT, PIN_PULLUP_ON, PIN_INPUT_ENABLE);
    }
}


static void SPI_MISO_Disable(uint8_t swap)
{
    if(swap == 0)
    {
        pinConfigure(PIN_PA5, PIN_DIR_INPUT, PIN_PULLUP_ON, PIN_INPUT_DISABLE);
    }
    else if (swap==1)
    {
        pinMode(PIN_PC1, INPUT_PULLUP);
        disablePinISC(PIN_PC1);
    }
}


/****************************************************************************/
/**********                    MAX31865                            **********/
/****************************************************************************/
#include "MAX31865.h"

#define PT100
#define MAX31865_SPI_PINSWAP 0

#define FAULT_HIGH_THRESHOLD  0x9304  /* +350C */
#define FAULT_LOW_THRESHOLD   0x2690  /* -100C */

MAX31865_RTD *rtd=NULL;


static void MAX31865_Init(UseBits &enable)
{
    if (enable.MAX31865)
    {
        pinMode(Config.RTDCSPin, INPUT_PULLUP);  // SPI SS

        SPI_MISO_Enable(MAX31865_SPI_PINSWAP);
        P(println("start MAX31865"));

        #ifdef PT1000
        // For PT 1000 (Ref on breakout board = 3900 Ohms 0.1%)
        rtd= new MAX31865_RTD( MAX31865_RTD::RTD_PT1000, Config.RTDCSPin, 3900 );
        #endif

        #ifdef PT100
        // For PT 100  (Ref on breakout board = 430 Ohms 0.1%)
        rtd= new MAX31865_RTD( MAX31865_RTD::RTD_PT100, Config.RTDCSPin, 430 );
        #endif
    }
}

static uint8_t MAX31865_Measure(bool enable, float* pt100_temp)
{
    uint8_t status = 0xFF;
    if(enable && rtd)
    {
        SPI_MISO_Enable(MAX31865_SPI_PINSWAP);
        Reset_SPI(MAX31865_SPI_PINSWAP, SPI_MODE1);
        pinMode(Config.RTDPowerPin,OUTPUT);
        digitalWrite(Config.RTDPowerPin, 1);
        delay(10);

        rtd->configure( VBIAS_ENABLED, CONVERSION_MODE_OFF, ONESHOT_DISABLED, USE_4WIRES, MAX31865_FAULT_DETECTION_NONE, true, true, FAULT_LOW_THRESHOLD, FAULT_HIGH_THRESHOLD );
        //Fault detection cycle.
        rtd->configure( VBIAS_ENABLED, CONVERSION_MODE_OFF, ONESHOT_DISABLED, MAX31865_FAULT_DETECTION_MANUAL_1 ); //0x8
        delay(60);
        rtd->configure( VBIAS_ENABLED, CONVERSION_MODE_OFF, ONESHOT_DISABLED, MAX31865_FAULT_DETECTION_MANUAL_2 );

        status = rtd->fault_status();
        if (status==0 )
        {
            //Serial.println(F("Starting conversion..."));Serial.flush();

            // Start 1 shot measure
            // V_BIAS enabled , No Auto-conversion, 1-shot enabled, No Fault detection
            rtd->configure( VBIAS_ENABLED, CONVERSION_MODE_OFF, ONESHOT_ENABLED, MAX31865_FAULT_DETECTION_NONE );
            delay(70);
            status = rtd->read_all();
            *pt100_temp = rtd->temperature();

            mySerial->print(F( "RTD:")); mySerial->print(rtd->resistance());
            mySerial->print(F( " Ohm, Temp: "));
            mySerial->print( *pt100_temp,2); mySerial->println("º");
        }
        else
        {
            mySerial->println("MAX31865 Failure.");
        }

        digitalWrite(Config.RTDPowerPin, 0);
      }
    return status;
}



/*****************************************************************************/
/***                   READ VCC                                            ***/
/*****************************************************************************/
#include "analog.h"
long Vcal_x_ADCcal;


/*****************************************************************************/
/***                   ENCODERS                                            ***/
/*****************************************************************************/
static uint16_t encode_temp(float t_raw)
{
    return floor(t_raw * 25 + 1000.5);
}

static uint8_t encode_humidity(float h_raw)
{
    return (uint8_t) floor(h_raw *2 +0.5);
}


/*****************************************************************************/
/***                       Pin Change Interrupts                           ***/
/*****************************************************************************/
uint8_t event_triggered = 0;

// ISR's for the Pin change Interrupt

void wakeUp0() { event_triggered |= 0x1; }
void wakeUp1() { event_triggered |= 0x2; }
void wakeUp2() { event_triggered |= 0x4; }
void wakeUp3() { event_triggered |= 0x8; }

/*****************************************************************************/
/***                   PIR                                                 ***/
/*****************************************************************************/

#include "PIR.h"
PIRModule PIR(Config, mySerial, wakeUp1);

/*****************************************************************************/
/****                         blink                                       ****/
/*****************************************************************************/
void activityLed (unsigned char state, unsigned int time = 0)
{
  if (Config.LedPin)
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


// blink led
static void blink (byte pin, byte n = 3)
{
  if (pin)
  {
    pinConfigure(pin, PIN_DIR_OUTPUT, PIN_INPUT_ENABLE);
    for (byte i = 0; i < (2 * n)-1; i++)
    {
      digitalWrite(pin, !digitalRead(pin));
      delay(100);
    }
    //digitalWrite(pin, LOW);
    pinConfigure(pin, PIN_OUT_LOW, PIN_INPUT_DISABLE);
  }
}


/*********************/

void setup() {

    /***                    ***/
    /*** disable all GPIO's ***/
    /***                    ***/
    for (uint8_t i = 0; i < 26; i++)
    {
        pinMode(i, INPUT_PULLUP);
        disablePinISC(i);
    }

    /***                                ***/
    /*** enable Pin 5 or 9, MISO of SPI ***/
    /***                                ***/
    SPI_MISO_Enable(RADIO_SPI_PINSWAP);


    // serial port, enable RX pin
    pinConfigure(1, PIN_DIR_INPUT, PIN_PULLUP_ON, PIN_INPUT_ENABLE);
    mySerial->swap(SERIAL_SWAP);
    mySerial->begin(SERIAL_BAUD);
    mySerial->print(FILENAME); mySerial->print(" Build: "); mySerial->print(BUILD);mySerial->println();

    /***                     ***/
    /***     CALIBRATE?      ***/
    /***                     ***/
    CalMode.configure();

    mySerial->println("  _______   _   _   _           ___");
    mySerial->println(" |__   __| (_) | \\ | |         |__ \\");
    mySerial->println("    | |     _  |  \\| |   ___      ) |");
    mySerial->println("    | |    | | | . ` |  / _ \\    / / ");
    mySerial->println("    | |    | | | |\\  | | (_) |  / /_");
    mySerial->println("    |_|    |_| |_| \\_|  \\___/  |____|");
    mySerial->println("    by nurazur\r\n");

    print_serial_number(mySerial);

    mySerial->print("\r\nNode ID: ");mySerial->println(Config.Nodeid);

    #if DEBUG >= 2
    //print_eeprom(Config, mySerial);
    #endif

    // disable serial RX
    pinMode(1, INPUT_PULLUP);
    disablePinISC(1);

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
    PIR.init();

    sei();

    Vcal_x_ADCcal = (long)Config.VccAtCalmV * Config.AdcCalValue;

    pinMode(Config.LedPin, OUTPUT);

    if (Config.I2CPowerPin > 0)
    {
        pinMode(Config.I2CPowerPin, OUTPUT);
        digitalWrite(Config.I2CPowerPin, HIGH);
        delay(1);
    }

    SensorData.PowerPin = Config.I2CPowerPin;

    // (UseBits) Config.SensorConfig does not work (compiler error), so we need a workaround
    UseBits *sensors = (UseBits*)&Config.SensorConfig;
    //mySerial->print("Sensor Config: "); mySerial->println(Config.SensorConfig, HEX);

    Wire.swap(0); // 0 is default, 1 is identical with 0, 3 has the same pins as UART0
    Wire.begin();
    SHTC3_Init   (*sensors);
    SHT3X_Init   (*sensors);
    SHT4X_Init   (*sensors);
    HTU21D_Init  (*sensors);
    BME280_Init  (*sensors);

    TiNo = new PacketHandler(*sensors);

    #if DEBUG >=1
    mySerial->print("packet type: "); mySerial->print(TiNo->PacketType);
    mySerial->print(", leng: "); mySerial->println(TiNo->PacketLen);
    #endif

    #ifdef DS18B20_H
    if (TiNo->use.DS18B20)
    {
        uint8_t num_ds18b20 = DS18B20_Init (TiNo->use.DS18B20, Config.OneWirePowerPin, Config.OneWireDataPin);
        if (num_ds18b20==0) TiNo->use.DS18B20=0;
        mySerial->print("DS18B20 found: "); mySerial->println(num_ds18b20);
    }
    #else
        #warning no Dallas18b20 Module included in this build
    #endif

    MAX31865_Init(TiNo->use);

    I2C_shutdown(Config.I2CPowerPin);

    // Initialize LDR, if enabled
    if (TiNo->use.BRIGHTNESS)
    {
        TiNo->use.BRIGHTNESS = LDR_Init(Config.LdrPin);
        if (TiNo->use.BRIGHTNESS)
            mySerial->println("LDR initialized.");
        else
            mySerial->println("LDR NOT initialized.");
    }

    TiNo->nodeid(Config.Nodeid);
    TiNo->targetid(Config.Gatewayid);
    TiNo->humidity(0);

    PIT.init(Config.UseCrystalRtc, 0);
    PIT.enable();
    if (Config.UseCrystalRtc) Config.Senddelay *= 8; // so the total delay time  is 8s;


    set_sleep_mode(SLEEP_MODE_PWR_DOWN);      // set sleep mode
    sleep_enable();    // enable sleep control

    #if DEBUG > 0
    #if defined (MEGACOREX)
    int8_t sigrow_offset = SIGROW.TEMPSENSE1; // Read signed value from signature row
    uint8_t sigrow_slope = SIGROW.TEMPSENSE0; // Read unsigned value from signature row
    #elif defined (ARDUINO_avrdd)
    uint16_t sigrow_offset = (SIGROW_TEMPSENSE1H << 8) | SIGROW_TEMPSENSE1L;// Read signed value from signature row
    uint16_t sigrow_slope  = (SIGROW_TEMPSENSE0H << 8) | SIGROW_TEMPSENSE0L;
    #endif
    mySerial->print("Tempsense Offset: "); mySerial->println(sigrow_offset);
    mySerial->print("Tempsense Slope : "); mySerial->println(sigrow_slope);
    #endif

#ifdef USE_RADIO
    Reset_SPI(RADIO_SPI_PINSWAP);
    #if DEBUG > 0
    mySerial->println ("Start Radio.");
    #endif
    Mac.radio_begin(); // puts radio to sleep to save power.

    #if DEBUG > 0
    mySerial->println ("Radio running.");
    // for debug: this makes sure we can read from the radio.
    byte version_raw = radio.readReg(0x10);
    mySerial->print ("Radio chip ver: "); mySerial->print(version_raw>>4, HEX); mySerial->print (" Radio Metal Mask ver: "); mySerial->print(version_raw&0xf, HEX); mySerial->println();
    #endif
#endif

    watchdog_counter = Config.Senddelay+1;     // set to have an initial transmission when starting the sender.
    mySerial->flush();
}



// simple, but messy version
uint8_t calculate_tx_flag()
{
    uint8_t Flags = 0;
    bool watchdog_expired = (watchdog_counter >= Config.Senddelay) && (Config.Senddelay != 0);
    if (watchdog_expired)
    {
        watchdog_counter = 0;
        // set the heartbeat flag when the watchdog counter is expired
        Flags |= 0x01;
    }
    Flags |= (event_triggered << 1);
    return Flags;
}

// depending on events, target id can vary.
uint8_t calculate_tx_targetid()
{
    // default setting
    uint8_t targetid= Config.Gatewayid;

    switch (event_triggered)
    {
        case 0x01:
            targetid = Config.PCI0Gatewayid;
            break;
        case 0x02:
            targetid = Config.PCI1Gatewayid;
            break;
        case 0x04:
            targetid = Config.PCI2Gatewayid;
            break;
        case 0x08:
            targetid = Config.PCI3Gatewayid;
            break;
        default:
            break;
    }
    return targetid;
}


// this function measures Battery voltage, tries all sensors if configured or not and fills the payload accordingly.
// return temperature. (because temp is used by the radio for frequency compensation.)
float Measure(void)
{
    float temperature=0;
    float t_internal =0; // the temperature we return must be a internal sensor.

    // measure Battery Voltage
    mySerial->print("VCC: "); mySerial->print(getVcc(Vcal_x_ADCcal)); mySerial->println("mV");
    #ifdef USE_RADIO
        TiNo->supplyV( Vcal_x_ADCcal / radio.vcc_dac);  // the VCC measured during last TX Burst.
    #else
        TiNo->supplyV( Vcal_x_ADCcal / readVcc() );
    #endif


    // measure Sensors
    TiNo->temp_index = 0;
    uint8_t success =0;
    // TiNo RF protocol allows only for one temperature/humidity sensor.
    // if there are more than one, the last one ist reported.
    // BME280 has priority over SHT4x. SHT4x has priority over SHT3X, and so on.

    uint8_t used_sensors = *((uint8_t*)&TiNo->use);

    if (TiNo->use.HTU21D)
    {
        success |= HTU21D_Measure(Config.SensorConfig & HTU21D_bm, myHTU21D, SensorData);
        if (success & HTU21D_bm) print_humidity_sensor_values("HTU21D", SensorData.temperature, SensorData.humidity, mySerial);
    }

    if (TiNo->use.SHT3X)
    {
        success |= SHT_Measure(used_sensors & SHT3X_bm, SHT3X, SensorData);
        if (success & SHT3X_bm) print_humidity_sensor_values("SHT3x", SensorData.temperature, SensorData.humidity, mySerial);
    }

    if (TiNo->use.SHTC3)
    {
        success |= SHT_Measure(used_sensors & SHTC3_bm, SHTC3, SensorData);
        if (success & SHTC3_bm) print_humidity_sensor_values("SHTC3", SensorData.temperature, SensorData.humidity, mySerial);
    }

    if (TiNo->use.SHT4X)
    {
        success |= SHT_Measure(used_sensors & SHT4X_bm, SHT4X, SensorData);
        if (success & SHT4X_bm) print_humidity_sensor_values("SHT4X", SensorData.temperature, SensorData.humidity, mySerial);
    }

    if (TiNo->use.BME280)
    {
        success |= BME280_Measure(Config.SensorConfig & BME280_bm, BME280, SensorData);
        if (success & BME280_bm)
        {
            print_humidity_sensor_values("BME280", SensorData.temperature, SensorData.humidity, mySerial);
            mySerial->print("Pressure: "); mySerial->print(SensorData.pressure); mySerial->println("hPa");
            TiNo->pressure((uint32_t)floor(SensorData.pressure*100));
        }
    }

    // if a temperature/humidity sensor is present, copy measurement results into Payload
    if(success)
    {
        TiNo->add_temp(encode_temp(SensorData.temperature));
        TiNo->humidity(encode_humidity(SensorData.humidity));

        // all above sensors are usually internal, PT100 and DS18B20 are most lilely external.
        t_internal = SensorData.temperature;
    }

    // PT100 Measurement
    if(TiNo->use.MAX31865)
    {
        uint8_t status = MAX31865_Measure(TiNo->use.MAX31865, &temperature);
        if (status==0)
        {
            success |= MAX31865_bm;
            TiNo->add_temp(encode_temp(temperature));
        }
    }

    #ifdef DS18B20_H
    // Dallas DS18B20 Measurement
    if(TiNo->use.DS18B20)
    {
        float t[3];
        t[0]=t[1]=t[2]=0;
        uint8_t num_ds18b20 = DS18B20_Measure(TiNo->use.DS18B20, t, Config.OneWirePowerPin);
        for (uint8_t i=0; i<num_ds18b20; i++)
        {
            TiNo->add_temp(encode_temp(t[i]));
            mySerial->print("T"); mySerial->print(i); mySerial->print(": "); mySerial->println(t[i]);
        }

        if (num_ds18b20 > 0) success |= DS18B20_bm;

        // if DS18B20 is the only sensor attached:
        if ((success ^ DS18B20_bm)== 0) // or (success == DS18B20_bm)
        {
            temperature = t[0]; t_internal = temperature;
        }
    }
    #endif

    // in case no temperature sensor is detected, we use the temperature from the RF Module
    if (!success)
    {
        mySerial->println("no temp sensor");
        temperature = 12.34;
        #ifdef USE_RADIO
            temperature = radio.readTemperature(0) + Config.radio_temp_offset/10.0;
        #endif
        TiNo->add_temp(encode_temp(temperature));
        t_internal = temperature;
        #if DEBUG >0
            print_humidity_sensor_values("RFM69: ", temperature, humidity, mySerial);
        #endif
    }

    uint16_t br;
    if (LDR_Measure(TiNo->use.BRIGHTNESS, Config.LdrPin, br))
    {
        TiNo->brightness(br);
        mySerial->print("Brightness: "); mySerial->println(br);
    }


    //mySerial->print("MCU Temperature: "); mySerial->println(readMcuTemperaure(),2);
    return t_internal;
}


void loop()
{
    static uint32_t total_count=0;
    float temperature;
    sleep_cpu();
    #if defined MEGACOREX
    // detect false wakeup. this is a pci interrupt from a pin on the wrong edge. ATmega4808 only.
    if (event_triggered & 0x80)
    {
        event_triggered=0;
        return; // start over, go sleep.
    }
    #endif

    // check if PIR dead time has expired and turn it on accordingly
    if (!event_triggered) PIR.check_deadtime();

    // set the flag byte in the payload
    uint8_t Flags = calculate_tx_flag();
    TiNo->flags(Flags);

    // set targetid
    TiNo->targetid(calculate_tx_targetid()); // depends on event_triggerd

    // check if PIR has been triggered, and if yes, disable the interrupt, start timer.
    PIR.triggered(event_triggered);

    //reset the event trigger
    event_triggered = 0;

    //when there is no event and watchdog counter not expired:
    if (!Flags)
    {
        return; // start over, go sleep.
    }

    print_flag(Flags, mySerial);

    // increment frame counter
    total_count++;
    TiNo->increment_count();

    // carry out measurements according to configuration
    temperature = Measure();



    #if defined BATTERYTEST
    if (!(total_count &0x1)) // even counts, we send 2 VCC's and 24 bit count
    {
        TiNo->nodeid(Config.Nodeid | 0x80); // 2 nodeids. Normal node and measurement packets alternating!

        // temp is used for idle voltage, humidity and flags are used as counter bytes.
        TiNo->humidity((uint8_t)&total_count[1]);
        TiNo->flags((uint8_t)&total_count[2]);
        TiNo->temp(Vcal_x_ADCcal / readVcc()); // Idle mode Vcc
        if (Config.SerialEnable)
        {
            #if DEBUG > 0
            //mySerial->print("Node ID: "); mySerial->print(TiNo->nodeid);
            //mySerial->print(", count: "); mySerial->println(TiNo->count | TiNo->humidity <<8 | TiNo->flags <<16);
            //mySerial->print("VCC idle mode:    "); mySerial->print(TiNo->temp); mySerial->println(" mV");
            //mySerial->print("VCC during burst: "); mySerial->print(TiNo->supplyV); mySerial->println(" mV");
            #endif
        }
    }
    else
    {
        TiNo->nodeid(Config.Nodeid);
    }
    #endif


    // send frame
    #ifdef USE_RADIO
        #ifdef SEND_BURST
        temperature<0?  temperature-=0.5 : temperature+=0.5;
        bool ackreceived = false;
            #if DEBUG >0
            mySerial->println("RFM send burst");
            #endif
        Reset_SPI(RADIO_SPI_PINSWAP);
        if(Config.UseRadioFrequencyCompensation)
        {
            ackreceived = Mac.radio_send(TiNo->pData, TiNo->PacketLen, Config.RequestAck, (int)temperature);
        }
        else
        {
            ackreceived = Mac.radio_send(TiNo->pData, TiNo->PacketLen, Config.RequestAck);
        }

        if (Config.RequestAck && Config.SerialEnable)
            (ackreceived) ? mySerial->println("Ack received.") : mySerial->println(" No Ack received.");

        if (ackreceived)
        {
            activityLed (1, 100);
        }
        #else
            mySerial->println("No burst is sent because of #SEND_BURST directive missing");
        #endif

    #endif

    // flush serial TX buffer, preparing sleep
    mySerial->flush();


    // blink
    if (Config.LedPin && Config.LedCount >0)
    {
        if (Config.LedCount != 0xff) Config.LedCount--;
        blink(Config.LedPin,2); // blink LED
    }

    if (MAX31865_SPI_PINSWAP != RADIO_SPI_PINSWAP)
            SPI_MISO_Disable(MAX31865_SPI_PINSWAP);
}
