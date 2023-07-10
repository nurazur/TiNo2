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
// Uses LowPower from  https://github.com/rocketscream/Low-Power under creative commons attribution-sharealike 3.0 unported license. see https://creativecommons.org/licenses/by-sa/3.0/
// Uses HTU21D_SoftI2C modified from https://github.com/enjoyneering/HTU21D under BSD licence, rewritten by nurazur for use of SoftwareWire

// Uses a RFM69 Driver originally from LowPowerlab.com However this driver has been changed so much so that it is barely recognizable.

// uses a modified BME280 Driver from https://github.com/finitespace/BME280 under GNU General Public License version 3 or later
// Uses OneWire             // license terms not clearly defined.
// Uses DallasTemperature under GNU LGPL version 2.1 or any later version.
// Uses Sensirion SHT library from 
// Uses MAX31865 from Ole Wolf <wolf@blazingangles.com> at https://github.com/olewolf/arduino-max31865.git under GNU General Public License version 3 or later
//
/* Revision history:
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
*/
// Core:        MegaCoreX (https://github.com/MCUdude/MegaCoreX)
// Clock:       doesn't matter

#include "Arduino.h"
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <RocketScream_LowPowerAVRZero.h>
#include "SHTSensor.h"
#include "datalinklayer.h"
#include "PacketHandler.h"
#include "SPI.h"
#include "pinchange_interrupt.h"

#define FILENAME "TiNo2 sensor.ino 30/04/2023"
#define BUILD 10
#define USE_RADIO
#define RADIO_SPI_PINSWAP 0
#define SEND_BURST

PacketHandler *TiNo=NULL;


#define DEBUG 0
/*****************************************************************************/
/***   Radio Driver Instance                                               ***/
/*****************************************************************************/
#include "RFM69.h"  // select this radio driver for RFM69HCW and RFM69CW Modules from HopeRF
//#include <CC1101.h>  // select this driver for CC1101 Modules
RADIO radio;

/*****************************************************************************/
/***  EEPROM Access  and device calibration                                ***/
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
//#define SERIAL_BAUD     19200
#define SERIAL_BAUD     57600
//#define SERIAL_BAUD     115200
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
bool watchdog_expired = false;

// interrupt service routine for RTC periodic timer
ISR(RTC_PIT_vect)
{
    RTC.PITINTFLAGS = RTC_PI_bm;              // clear interrupt flag
    watchdog_counter++;
}


/*****************************************************************************/
/***                   PIR                                                 ***/
/*****************************************************************************/
uint16_t pir_counter;
bool pir_is_off;
bool pir_enabled;

static void PIR_init()
{
    pir_counter = Config.PirDeadTime -3;
    pir_is_off = true;
    pir_enabled = (Config.PirDataPin>0) ? true : false;
    if (pir_enabled)
    {
        pinMode(Config.PirDataPin, INPUT);
        detachInterrupt(Config.PirDataPin);
        Serial.println("PIR initialized.");
    }
}

static void PIR_register()
{
    pir_is_off = false;
    pir_counter = 0;
    if (pir_enabled)
    {
        //pinMode(Config.PCI1Pin, INPUT);
        register_pci(1, Config.PirDataPin, wakeUp1, RISING);
    }
}

static void PIR_unregister()
{
    if (pir_enabled)
    {
        detachInterrupt(Config.PirDataPin);
    }
}

static void PIR_triggered(uint8_t event)
{
    if (pir_enabled && !pir_is_off && (event&0x2))
    {
        pir_counter = 0;
        pir_is_off = true;
        PIR_unregister();
        Serial.println("PIR");
    }
}


static void PIR_check_deadtime()
{
    if (pir_enabled && pir_is_off)
    {
        pir_counter++;
        //Serial.printf("PIR counter: %i\n", pir_counter); Serial.flush();
        if (pir_counter > Config.PirDeadTime) // re-enable PIR
        {
            PIR_register(); // resets counter
            Serial.println("PIR active."); Serial.flush();
        }
    }
}
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


static void I2C_shutdown(void)
{
    digitalWrite(Config.I2CPowerPin, LOW);
    
    if ((PORTMUX.TWISPIROUTEA & 0x30) != 0x10) // only default (0) and ALT1 (1) is supported, other values should fall back to default.
    {
        pinMode(PIN_WIRE_SDA,INPUT);
        LowPower.disablePinISC(PIN_WIRE_SDA);

        pinMode(PIN_WIRE_SCL,INPUT);
        LowPower.disablePinISC(PIN_WIRE_SCL); 
    }
    else
    {
        pinMode(PIN_WIRE_SDA_PINSWAP_1,INPUT);
        LowPower.disablePinISC(PIN_WIRE_SDA_PINSWAP_1);

        pinMode(PIN_WIRE_SCL_PINSWAP_1,INPUT);
        LowPower.disablePinISC(PIN_WIRE_SCL_PINSWAP_1); 
    }
}


/*****************************************************************************/
/***              SHT3x and SHTC3  Humidity Sensor                         ***/
/*****************************************************************************/


SHTSensor *SHT3X=NULL;
SHTSensor *SHTC3=NULL;
SHTSensor *SHT4X=NULL;

uint8_t SHT_Measure(bool enabled, SHTSensor *SHT, float &temperature, float &humidity)
{
    uint8_t success=0;
    if (enabled && SHT)
    {   
        //mySerial->print("now read sht\n");
        //pinMode(Config.I2CPowerPin, OUTPUT);
        digitalWrite(Config.I2CPowerPin, HIGH);
        Wire.begin(); // ?? brauchts des?
        
        delay(1);
        SHT->init();
        
        if (SHT->readSample()) 
        {
            temperature = SHT->getTemperature();
            humidity    = SHT->getHumidity();
            print_humidity_sensor_values("SHT", temperature, humidity);
            success =  0x48; // according to SHT3X and SHTC3 bit in UseBits
        }
        else 
        {
            mySerial->print("Error in readSample()\n");
        }
        //digitalWrite(Config.I2CPowerPin, LOW);
        I2C_shutdown();
    }
    return success;
}


static bool SHTC3_Init(UseBits &enable)
{
    if (enable.SHTC3)
    {
        SHTC3 = new SHTSensor(SHTSensor::SHTC3);
        mySerial->print("SHTC3: ");
        enable.SHTC3 = SHTC3->init();
        if (enable.SHTC3) 
        {
            mySerial->print("init(): success\n");
        } 
        else 
        {
            mySerial->print("init(): failed\n");
        }
    }
    return enable.SHTC3;
}

static bool SHT3X_Init(UseBits &enable)
{
    if (enable.SHT3X)
    {      
        SHT3X = new SHTSensor(SHTSensor::SHT3X);
        mySerial->print("SHT3x: ");
        enable.SHT3X = SHT3X->init();
        if (enable.SHT3X) 
        {
            mySerial->print("init(): success\n");
        } 
        else 
        {
            mySerial->print("init(): failed\n");
        }
    }
   return enable.SHT3X;
}

static bool SHT4X_Init(UseBits &enable)
{
    if (enable.SHT4X)
    {      
        SHT4X = new SHTSensor(SHTSensor::SHT4X);
        mySerial->print("SHT4x: ");
        enable.SHT4X = SHT4X->init();
        if (enable.SHT4X) 
        {
            mySerial->print("init(): success\n");
        } 
        else 
        {
            mySerial->print("init(): failed\n");
        }
    }
   return enable.SHT4X;
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
        digitalWrite(Config.I2CPowerPin, HIGH);
        if (myHTU21D->begin())
        {
            delay(50);
            temperature = myHTU21D->readTemperature();
            humidity =    myHTU21D->readCompensatedHumidity(temperature);
            print_humidity_sensor_values("HTU21D",temperature, humidity);
            success=0x1;
        }
        //digitalWrite(Config.I2CPowerPin, LOW);
        I2C_shutdown();
    }
    return success;
}
/**********     One-Wire and DS18B20     **********/

#include <DallasTemperature.h>       // GNU Lesser General Public License v2.1 or later
#include <OneWire.h>                // license terms not clearly defined.

//#define ONE_WIRE_BUS Config.SDAPin
//#define ONE_WIRE_POWER 9

OneWire *oneWire=NULL;
DallasTemperature *ds18b20=NULL;


static uint8_t DS18B20_Start(DallasTemperature *sensor, byte PowerPin);


// One-Wire DS18B20 start-up sequence
static uint8_t DS18B20_Start(DallasTemperature *sensor, byte PowerPin)
{
    if (PowerPin <=25)
    {
        pinMode(PowerPin, OUTPUT); // set power pin for DS18B20 to output
        digitalWrite(PowerPin, HIGH); // turn DS18B20 sensor on
        delay(10); // Allow 10ms for the sensor to be ready
        sensor->begin();
        //sensor->setResolution(10); //Resolutiuon is 0.125 deg, absolutely sufficient!
        delay(10); // Allow 10ms for the sensor to be ready
        return sensor->getDeviceCount();
    }
    else
        return 0;
}

static void DS18B20_Stop(byte PowerPin)
{
    digitalWrite(PowerPin, LOW); // turn Sensor off to save power
}




static uint8_t DS18B20_Init(UseBits &enable)
{
    uint8_t num_devices =0;
    if(enable.DS18B20)
    {
        //--------------------------------------------------------------------------
        // test if 1-wire devices are present
        //--------------------------------------------------------------------------
        pinMode(Config.OneWirePowerPin, OUTPUT); // set power pin for DS18B20 to output
        digitalWrite(Config.OneWirePowerPin, HIGH); // turn DS18B20 sensor on
        delay(10); // Allow 10ms for the sensor to be ready
        
        // enable the data pin as input (at power up, we disable all GPIO's for power saving porposes)
        pinConfigure(Config.OneWireDataPin, PIN_DIR_INPUT, PIN_PULLUP_OFF, PIN_INPUT_ENABLE);
        
        // Start up the library
        oneWire= new OneWire(Config.OneWireDataPin);

        ds18b20 = new DallasTemperature(oneWire);

        num_devices = DS18B20_Start(ds18b20, Config.OneWirePowerPin);
        if (num_devices == 0)
        {
            delete ds18b20;
            ds18b20 = NULL;
            DS18B20_Stop(Config.OneWirePowerPin);
            enable.DS18B20 = 0;
            if (Config.SerialEnable) mySerial->print("no ");

        }
        else
        {
            if (Config.SerialEnable)
            {
                mySerial->print(num_devices, DEC);

            }
        }
        if (Config.SerialEnable)
        {
            mySerial->println(" DS18B20 found.");
            //mySerial->print("power pin: "); mySerial->println(Config.OneWirePowerPin);
            //mySerial->print("data  pin: "); mySerial->println(Config.OneWireDataPin);
            mySerial->flush();
        }
    }
    else
    {
        ;//mySerial->println("DS18B20 disabled!");
    }
    return num_devices; // this allows to adjust packet type at initialization. if only one device, packet type= 0;
}

static uint16_t encode_temp(float t_raw)
{
    return floor(t_raw * 25 + 1000.5);
}

static uint8_t encode_humidity(float h_raw)
{
    return (uint8_t) floor(h_raw *2 +0.5);
}
/*
static uint8_t DS18B20_Measure(bool enable, float *temp, uint16_t *temp1=NULL, uint16_t *temp2=NULL)
{
    float temperature=-40;
    const char *unit= "degC";
    uint8_t num_devices =0;
    if (enable && ds18b20)
    {
        num_devices = DS18B20_Start(ds18b20, Config.OneWirePowerPin);
        //mySerial->print("Num DS18B20: "); mySerial->println(num_devices);

        if (num_devices > 0)
        {
            ds18b20->requestTemperatures();
            switch (num_devices)
            {
                case 3:
                    if (temp2 != NULL)
                    {
                        temperature = ds18b20->getTempCByIndex(2);
                        *temp2 = encode_temp(temperature);
                        mySerial->print("T2: ");mySerial->print(temperature); mySerial->println(unit);
                    }
                     __attribute__ ((fallthrough));
                case 2:
                    if (temp1 != NULL)
                    {
                        temperature = ds18b20->getTempCByIndex(1);
                        *temp1 = encode_temp(temperature);
                        mySerial->print("T1: ");mySerial->print(temperature); mySerial->println(unit);
                    }
                     __attribute__ ((fallthrough));
                case 1:
                    *temp = ds18b20->getTempCByIndex(0);
                    mySerial->print("T0: ");mySerial->print(*temp); mySerial->println(unit);
                    break;
            }

            DS18B20_Stop(Config.OneWirePowerPin); // Turn off power Pin for DS18B20
        }
    }
    return num_devices;
}
*/

static uint8_t DS18B20_Measure(bool enable, uint16_t *temp)
{
    float temperature=-40;
    const char *unit= " degC";
    uint8_t num_devices =0;
    if (enable && ds18b20)
    {
        num_devices = DS18B20_Start(ds18b20, Config.OneWirePowerPin);
        //mySerial->print("Num DS18B20: "); mySerial->println(num_devices);

        if (num_devices > 0)
        {
            ds18b20->requestTemperatures();
            switch (num_devices)
            {
                case 3:
                    temperature = ds18b20->getTempCByIndex(2);
                    temp[2] = encode_temp(temperature);
                    mySerial->print("T2: ");mySerial->print(temperature); mySerial->println(unit);
                     __attribute__ ((fallthrough));
                case 2:
                    temperature = ds18b20->getTempCByIndex(1);
                    temp[1] = encode_temp(temperature);
                    mySerial->print("T1: ");mySerial->print(temperature); mySerial->println(unit);
                     __attribute__ ((fallthrough));
                case 1:
                    temperature = ds18b20->getTempCByIndex(0);
                    temp[0] = encode_temp(temperature);
                    mySerial->print("T0: ");mySerial->print(temperature); mySerial->println(unit);
                    break;
            }

            DS18B20_Stop(Config.OneWirePowerPin); // Turn off power Pin for DS18B20
        }
    }
    return num_devices;
}


#include <BME280I2C.h>

BME280I2C *BME280;

static bool BME280_Init(UseBits &enable)
{
    if(enable.BME280)
    {
        BME280 = new BME280I2C();
        mySerial->print("BME280: ");
        enable.BME280 = BME280->begin();
        if (enable.BME280) 
        {
            mySerial->print("init(): success\n");
        } 
        else 
        {
            mySerial->print("init(): failed\n");
            delete BME280;
            BME280=NULL;
            
        }
    }
    return enable.BME280;
}

uint8_t BME280_Measure(bool enabled, float &temp, float &hum, float &pres)
{
    uint8_t success = 0;
    if (enabled && BME280)
    {
        digitalWrite(Config.I2CPowerPin, HIGH);
        delay(2); // start up time according to data sheet
        if (BME280->begin())
        {
           BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
           BME280::PresUnit presUnit(BME280::PresUnit_hPa);

           BME280->read(pres, temp, hum, tempUnit, presUnit);
           print_humidity_sensor_values("BME280",temp, hum);
           mySerial->print("Pressure: "); mySerial->print(pres); mySerial->println("hPa");
           success = 0x4;
        }
        //digitalWrite(Config.I2CPowerPin, LOW);
        I2C_shutdown();
    }
    return success;
}


/*****************************************************************************/
/***      Brightness  with LDR                                             ***/
/*****************************************************************************/

static bool LDR_Init(UseBits &enable)
{
    if (Config.LdrPin < 0) enable.BRIGHTNESS=0;
    if ( digitalPinToAnalogInput(Config.LdrPin)==NOT_A_PIN) enable.BRIGHTNESS=0;
    if (enable.BRIGHTNESS)
    {
        pinConfigure(Config.LdrPin, PIN_DIR_INPUT, PIN_PULLUP_OFF, PIN_INPUT_ENABLE);
        mySerial->println("LDR inititialised.");
    }
    return enable.BRIGHTNESS;
}


static uint16_t LDR_Measure(bool enabled, int8_t LDRPin) 
{
    uint16_t sensorValue=0;

    if (enabled && LDRPin >= 0)
    {
        pinMode(LDRPin, INPUT_PULLUP);
        delay(20);
        sensorValue = (uint16_t)(1023 - (analogRead(LDRPin)/(1<<ADC0.CTRLB)));
        pinMode(LDRPin, INPUT);
        mySerial->print("Brightness: "); mySerial->println(sensorValue); 
    }
    return sensorValue;
}

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
        //pinMode(PIN_PA5, INPUT_PULLUP);
        //LowPower.disablePinISC(PIN_PA5);
        pinConfigure(PIN_PA5, PIN_DIR_INPUT, PIN_PULLUP_ON, PIN_INPUT_DISABLE);
    }
    else if (swap==1)
    {
        pinMode(PIN_PC1, INPUT_PULLUP);
        LowPower.disablePinISC(PIN_PC1);
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
        //pinMode(RTD_CS_PIN, INPUT_PULLUP);  // SPI SS (avoid becoming SPI Slave) // already done in setup()
        
        SPI_MISO_Enable(MAX31865_SPI_PINSWAP);
        if (Config.SerialEnable) mySerial->println("start MAX31865");

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


// setup external 32.768 kHz crystal for RTC (refer to TB3213)
static void RTC_Xosc32k_Init() {
  uint8_t temp;
    
  // disable oscillator first
  temp = CLKCTRL.XOSC32KCTRLA;
  temp &= ~CLKCTRL_ENABLE_bm;
  CPU_CCP = CCP_IOREG_gc;
  CLKCTRL.XOSC32KCTRLA = temp;

  // wait for status bit to become '0'
  while(CLKCTRL.MCLKSTATUS & CLKCTRL_XOSC32KS_bm);

  // select external crystal for RTC
  temp = CLKCTRL.XOSC32KCTRLA;
  temp &= ~CLKCTRL_SEL_bm;
  CPU_CCP = CCP_IOREG_gc;
  CLKCTRL.XOSC32KCTRLA = temp;

  // enable the oscillator
  temp = CLKCTRL.XOSC32KCTRLA;
  temp |= CLKCTRL_ENABLE_bm | CLKCTRL_RUNSTDBY_bm;
  CPU_CCP = CCP_IOREG_gc;
  CLKCTRL.XOSC32KCTRLA = temp;

  // wait for all registers to be synchronized
  while (RTC.STATUS > 0);

  // select 32.768 kHz external crystal oscillator
  RTC.CLKSEL = RTC_CLKSEL_TOSC32K_gc;
}

static void RTC_ULP32k_init(void)
{
    RTC.CLKSEL = RTC_CLKSEL_INT1K_gc;         // Internal 1024 Hz OSC 
}

static void PIT_init(uint8_t Period)
{
    //C:\Program Files (x86)\Arduino\hardware\tools\avr\avr\include\avr\iom4808.h
    RTC.PITINTCTRL = RTC_PI_bm;  
    RTC.PITCTRLA   = Period  | RTC_PITEN_bm;
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


// should go into configuration class
#if DEBUG == 2
static void print_eeprom(Stream *serial)
{
    /*
    //serial->print("Nodeid = ");        serial->print(Config.Nodeid);          serial->println();
    //serial->print("Networkid = ");     serial->print(Config.Networkid);       serial->println();
    //serial->print("Gatewayid =  ");    serial->print(Config.Gatewayid);       serial->println();
    //serial->print("VccAtCalmV  = ");   serial->print(Config.VccAtCalmV);      serial->println();
    //serial->print("AdcCalValue = ");   serial->print(Config.AdcCalValue);     serial->println();
    //serial->print("Senddelay = ");     serial->print(Config.Senddelay);       serial->println();
    //serial->print("Frequencyband = "); serial->print(Config.Frequencyband);   serial->println();
    //serial->print("frequency = ");     serial->print(Config.frequency);       serial->println();
    //serial->print("TxPower = ");       serial->print(Config.TxPower);         serial->println();
    //serial->print("RequestAck = ");    serial->print(Config.RequestAck);      serial->println();
    //serial->print("LedCount = ");      serial->print(Config.LedCount);        serial->println();
    //serial->print("LedPin = ");        serial->print(Config.LedPin);          serial->println();
    //serial->print("RxPin = ");         serial->print(Config.RxPin);           serial->println();
    //serial->print("TxPin = ");         serial->print(Config.TxPin);           serial->println();
    //serial->print("SDAPin = ");        serial->print(Config.SDAPin);          serial->println();
    //serial->print("SCLpin = ");        serial->print(Config.SCLPin);          serial->println();
    //serial->print("I2CPowerPin = ");   serial->print(Config.I2CPowerPin);     serial->println();
    //serial->print("checksum = ");      serial->print(Config.checksum);        serial->println();
    
    serial->print(offsetof(Configuration,PCI0Pin));serial->print(" Interrupt 0 Pin = ");serial->print(Config.PCI0Pin);        serial->println();
    serial->print(offsetof(Configuration,PCI1Pin));serial->print(" Interrupt 1 Pin = ");serial->print(Config.PCI1Pin);        serial->println();
    serial->print(offsetof(Configuration,PCI2Pin));serial->print(" Interrupt 2 Pin = ");serial->print((int)Config.PCI2Pin);        serial->println();
    serial->print(offsetof(Configuration,PCI3Pin));serial->print(" Interrupt 3 Pin = ");serial->print((int)Config.PCI3Pin);        serial->println();
    //serial->print("Interrupt 0 Type = ");serial->print(show_trigger(Config.PCI0Trigger)); serial->print(", ");  serial->print(show_pinmode(Config.PCI0Trigger)); serial->println();
    //serial->print("Interrupt 1 Type = ");serial->print(show_trigger(Config.PCI1Trigger)); serial->print(", ");  serial->print(show_pinmode(Config.PCI1Trigger)); serial->println();
    //serial->print("Interrupt 2 Type = ");serial->print(show_trigger(Config.PCI2Trigger)); serial->print(", ");  serial->print(show_pinmode(Config.PCI2Trigger)); serial->println();
    //serial->print("Interrupt 3 Type = ");serial->print(show_trigger(Config.PCI3Trigger)); serial->print(", ");  serial->print(show_pinmode(Config.PCI3Trigger)); serial->println();
    //serial->flush();
    serial->print(offsetof(Configuration,UseCrystalRtc));serial->print(" Use Crystal = ");     serial->print(Config.UseCrystalRtc);  serial->println();
    //serial->print("Encryption = ");     serial->print(Config.EncryptionEnable);  serial->println();
    //serial->print("FEC = ");            serial->print(Config.FecEnable);  serial->println();
    //serial->print("Interleave = ");     serial->print(Config.InterleaverEnable);  serial->println();
    serial->print(offsetof(Configuration,EepromVersionNumber)); serial->print(" EEPROM Version= ");  serial->print(Config.EepromVersionNumber);  serial->println();
    serial->print(offsetof(Configuration,SoftwareversionNumber));serial->print(" Software Version = ");  serial->print(Config.SoftwareversionNumber);  serial->println();
    serial->print(offsetof(Configuration,TXGaussShaping));serial->print(" Gauss shaping= ");   serial->print(Config.TXGaussShaping);  serial->println();
    serial->print(offsetof(Configuration,SerialEnable));serial->print(" Serial Port Enable = "); serial->print(Config.SerialEnable);  serial->println();
    serial->print(offsetof(Configuration,IsRFM69HW));serial->print(" RF Chip = "); Config.IsRFM69HW ?    serial->print("RFM69HCW") : serial->print("RFM69CW");  serial->println();
    serial->print(offsetof(Configuration,PaBoost));serial->print(" PA Boost = ");      serial->print(Config.PaBoost);  serial->println();
    serial->print(offsetof(Configuration,FedvSteps));serial->print(" Fdev (Steps) = ");      serial->print(Config.FedvSteps);  serial->println();
    serial->flush();
    */
}
#endif

/*****************************************************************************/
/****                        SIGROW                                       ****/
/*****************************************************************************/
// Serial Number consists of 10 Bytes.
// The first 6 Bytes are printable characters.
// the last 4 Bystes are the actual serial number.

typedef struct 
{
    char prefix[7];
    union
    {
        uint8_t sn_char[4];
        uint32_t sn;
    };
} SerialNumber;


 
void print_serial_number()
{
    SerialNumber SN;
    uint8_t* sernum_addr = (uint8_t*)&SIGROW.SERNUM0;
    int i;
    for (i=0; i<6; i++)
    {
        SN.prefix[i] = (char) *sernum_addr;
        sernum_addr++;
    }
    SN.prefix[6]=0;
   
    for (i=0; i<4; i++)
    {
        SN.sn_char[i] = *sernum_addr;
        sernum_addr++;
    }

    Serial.print("Serial Number: ");
    //Serial.printf(" %0.10li\r\n", SN.sn); // printf kostet mich 1.5 kByte
    Serial.print(SN.prefix); Serial.print(" "); Serial.println(SN.sn);
}
/*****************************************************************************/

 
void setup() {
    
    /***                    ***/
    /*** disable all GPIO's ***/
    /***                    ***/
    for (uint8_t i = 0; i < 26; i++)
    {
        pinMode(i, INPUT_PULLUP);
        LowPower.disablePinISC(i);
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
  
    print_serial_number();
  
    #if DEBUG >= 2
    print_eeprom(mySerial);
    #endif
    // disable serial RX
    pinMode(1, INPUT_PULLUP);
    LowPower.disablePinISC(1);

  

    //UseBits* u;
    //u = (UseBits*)&Config.SensorConfig;
    //TiNo = new PacketHandler(*u);
    TiNo = new PacketHandler(*((UseBits*)&Config.SensorConfig));

    #if DEBUG >=1
    mySerial->print("packet type: "); mySerial->print(TiNo->PacketType);
    mySerial->print(", leng: "); mySerial->println(TiNo->PacketLen);
    #endif 

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
        /*
        attachInterrupt(digitalPinToInterrupt(Config.PCI0Pin), wakeUp0, CHANGE);

        if (Config.PCI0Trigger != CHANGE) // in this case the LSB ist the rising/falling indicator
            Config.PCI0Trigger &= 0x1; 
        */
    }

    if (Config.PCI1Pin >=0)  
    {
        pinMode(Config.PCI1Pin, Config.PCI1Mode);
        register_pci(1, Config.PCI1Pin, wakeUp1, Config.PCI1Trigger);
        /*
        attachInterrupt(digitalPinToInterrupt(Config.PCI1Pin), wakeUp1, CHANGE);

        if (Config.PCI1Trigger != CHANGE) // in this case the LSB ist the rising/falling indicator
        Config.PCI1Trigger &= 0x1; 
        */
    }
  
  if (Config.PCI2Pin >=0)  
  {
      pinMode(Config.PCI2Pin, Config.PCI2Mode);
      register_pci(2, Config.PCI2Pin, wakeUp2, Config.PCI2Trigger);
      /*
      attachInterrupt(digitalPinToInterrupt(Config.PCI2Pin), wakeUp2, CHANGE);
      if (Config.PCI2Trigger != CHANGE) // in this case the LSB ist the rising/falling indicator
            Config.PCI2Trigger &= 0x1; 
      */
  }
  
  if (Config.PCI3Pin >=0)  
  {
      pinMode(Config.PCI3Pin, Config.PCI3Mode);
      register_pci(3, Config.PCI3Pin, wakeUp3, Config.PCI3Trigger);
      /*
      attachInterrupt(digitalPinToInterrupt(Config.PCI3Pin), wakeUp3, CHANGE);
      if (Config.PCI3Trigger != CHANGE) // in this case the LSB ist the rising/falling indicator
            Config.PCI3Trigger &= 0x1; 
      */
      
  }
    // initialize PIR Sensor, if configured
    // it will be still for 3 cycles. 
    // PIR shares the ecent flag with PCI1. PCI1 remains active, if specified.
    PIR_init();
    
    sei();

    
    
    Vcal_x_ADCcal = (long)Config.VccAtCalmV * Config.AdcCalValue;

    pinMode(Config.LedPin, OUTPUT);
  
    pinMode(Config.I2CPowerPin, OUTPUT);
    digitalWrite(Config.I2CPowerPin, HIGH);
    delay(1);
    Wire.swap(0);
    Wire.begin();  
    SHTC3_Init   (TiNo->use);
    SHT3X_Init   (TiNo->use);
    SHT4X_Init   (TiNo->use);
    HTU21D_Init  (TiNo->use);
    BME280_Init  (TiNo->use);
    DS18B20_Init (TiNo->use);
    MAX31865_Init(TiNo->use);

    I2C_shutdown();
    
    //mySerial->print("PORTMUX.TWISPIROUTEA = "); mySerial->println(PORTMUX.TWISPIROUTEA);
    
    LDR_Init(TiNo->use);
    
    TiNo->nodeid(Config.Nodeid);
    TiNo->targetid(Config.Gatewayid);
    TiNo->humidity(0);
  
   
    if (Config.UseCrystalRtc)
    {
        PIT_init(RTC_PERIOD_CYC32768_gc);  // 1s period
        Config.Senddelay *= 8;
        RTC_Xosc32k_Init();
    }
    else
    {
        PIT_init(RTC_PERIOD_CYC8192_gc);  //  8s period
        RTC_ULP32k_init();
    }

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);      // set sleep mode
  sleep_enable();    // enable sleep control
  
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
    #if DEBUG > 0
    mySerial->print("PORTMUX.TWISPIROUTEA = "); mySerial->println(PORTMUX.TWISPIROUTEA);
    #endif
  watchdog_counter = Config.Senddelay+1;     // set to have an initial transmission when starting the sender.
}


void loop()
{
    PIR_check_deadtime(); // check if PIR dead time has expired and turn it on accordingly
    watchdog_expired = ((watchdog_counter >= Config.Senddelay) && (Config.Senddelay != 0)); // when Senddelay equals 0, sleeep forever and wake up only for events.
    if (watchdog_expired || event_triggered)
    {
        uint8_t Flags = 0;
        float temperature=0;
        float humidity =0;
        
        if (watchdog_expired)
        {
            //tinytx.flags |= 0x1; // heartbeat flag
            watchdog_counter = 0;
            mySerial->println("HeartBeat");
            // set the heartbeat flag when the watchdog counter is expired
            Flags |= 0x01;
        }
        if(event_triggered)
        {        
            Flags |= (event_triggered << 1);
            
            mySerial->print("PCI");
            TiNo->targetid(Config.Gatewayid);
            //um Fernbedienungen mit mehreren Kanaelen herzustellen, muss der Node
            //Pakete an verschiedene Nodeid's schicken koennen.
            
            switch (event_triggered)
            {
                case 0x01:
                    TiNo->targetid(Config.PCI0Gatewayid);
                    mySerial->println("0");
                    break;
                case 0x02:
                    TiNo->targetid(Config.PCI1Gatewayid);
                    PIR_triggered(event_triggered);
                    mySerial->println("1");
                    break;
                case 0x04:
                    TiNo->targetid(Config.PCI2Gatewayid);
                    mySerial->println("2");
                    break;
                case 0x08:
                    TiNo->targetid(Config.PCI3Gatewayid);
                    mySerial->println("3");
                    break;
                default:
                    break;
            }
            event_triggered = 0;
        }
        TiNo->flags(Flags);
        
        // measure Battery Voltage
        if(Config.SerialEnable)
            mySerial->print("VCC: "); mySerial->print(getVcc(Vcal_x_ADCcal)); mySerial->println("mV");
        #ifdef USE_RADIO
            TiNo->supplyV( Vcal_x_ADCcal / radio.vcc_dac);  // the VCC measured during last TX Burst.
        #else
            TiNo->supplyV( Vcal_x_ADCcal / readVcc() );
        #endif

        /*** begin of ugly code block***/
        // measure Sensors
        TiNo->temp_index = 0;
        uint8_t success =0;
        success |=  HTU21D_Measure(TiNo->use.HTU21D, temperature, humidity);
        success |= SHT_Measure(TiNo->use.SHT3X, SHT3X, temperature, humidity);
        success |= SHT_Measure(TiNo->use.SHTC3, SHTC3, temperature, humidity);
        success |= SHT_Measure(TiNo->use.SHT4X, SHT4X, temperature, humidity);
        if(TiNo->use.BME280)
        {
            float pressure;
            success |= BME280_Measure(true, temperature, humidity, pressure);
            TiNo->pressure((uint32_t)floor(pressure*100)); 
        }
        if(success) // one of those sensors is present.
        {
            TiNo->add_temp(encode_temp(temperature));
            TiNo->humidity(encode_humidity(humidity));
        }
        
        success = !MAX31865_Measure(TiNo->use.MAX31865, &temperature); // abuse 'success' as status indicator, whereby 0 is success!
        if (TiNo->use.MAX31865 && success)
        {
            TiNo->add_temp(encode_temp(temperature));
        }
        
        uint16_t t[3];
        t[0]=t[1]=t[2]=0;
        uint8_t num_ds18b20 = DS18B20_Measure(TiNo->use.DS18B20, t);
        for (uint8_t i=0; i<num_ds18b20; i++)
            TiNo->add_temp(t[i]);
        /*
        if(num_ds18b20 > 0) TiNo->add_temp(t[0]);
        if(num_ds18b20 > 1) TiNo->add_temp(t[1]);
        if(num_ds18b20 > 2) TiNo->add_temp(t[2]);
        */
        /*** end of ugly code***/
        
        TiNo->brightness( LDR_Measure(TiNo->use.BRIGHTNESS, Config.LdrPin));
        TiNo->increment_count();
        
        

        #ifdef USE_RADIO
            #ifdef SEND_BURST
            bool ackreceived = false;
                #if DEBUG >0
                mySerial->println("RFM send burst");
                #endif
            Reset_SPI(RADIO_SPI_PINSWAP);
            ackreceived = Mac.radio_send(TiNo->pData, TiNo->PacketLen, Config.RequestAck);
            if (Config.RequestAck && Config.SerialEnable)
                (ackreceived) ? mySerial->println("Ack received.") : mySerial->println(" No Ack received.");
            
            if (ackreceived)
            {
                activityLed (1, 100);
            }
            #endif
        #endif
        
        if (Config.SerialEnable) mySerial->flush();
        
        
        if (Config.LedPin && Config.LedCount >0)
        {
            if (Config.LedCount != 0xff) Config.LedCount--;
            blink(Config.LedPin,2); // blink LED
        }
        
        
        
        /*
        digitalWrite(Config.LedPin, HIGH);
        delay(100);
        digitalWrite(Config.LedPin, LOW);
        */
    }
    if (MAX31865_SPI_PINSWAP != RADIO_SPI_PINSWAP)
            SPI_MISO_Disable(MAX31865_SPI_PINSWAP);
    sleep_cpu();
    
}



