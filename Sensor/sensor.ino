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
// Core:        for AVRxxDDyy DxCore (https://github.com/SpenceKonde/DxCore)

#include "Arduino.h"
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include "SHTSensor.h"
#include "datalinklayer.h"
#include "PacketHandler.h"
#include "SPI.h"
#include "PinchangeInterrupt.h"

#define FILENAME "TiNo2 myTiNo2V3.ino 25/10/2023"
#define BUILD 11
#define USE_RADIO
#define RADIO_SPI_PINSWAP 0
#define SEND_BURST

// Battery Test uses a common payload structure to transmit Battery Voltage
// in idle mode and a second measurement under load conditions.
// we add 128 to the nodeid to distinct between the 2 formats.
// count is extended to 3 Bytes by using the flag byte and the humidity byte
// this is for engineering and test purposes only!
//#define BATTERYTEST

PacketHandler *TiNo=NULL;

#define P(a) if(Config.SerialEnable) mySerial->a


#define DEBUG 0
/*****************************************************************************/
/***   Radio Driver Instance                                               ***/
/*****************************************************************************/
#include "RFM69.h"  // select this radio driver for RFM69HCW and RFM69CW Modules from HopeRF
RADIO radio;

// only for the few V1 TiNo2 boards:
//RADIO radio(SS,15,0, digitalPinToInterrupt(15));
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
  /* Get bit position for getting pin ctrl reg */
  uint8_t bit_pos = digitalPinToBitPosition(pin);

  /* Calculate where pin control register is */
  volatile uint8_t *pin_ctrl_reg = getPINnCTRLregister(port, bit_pos);

  /* Disable ISC */
  *pin_ctrl_reg = PORT_ISC_INPUT_DISABLE_gc;
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
    if (Config.I2CPowerPin > 0)
        digitalWrite(Config.I2CPowerPin, LOW);

    #if defined (__AVR_ATmega4808__)
    if ((PORTMUX.TWISPIROUTEA & 0x30) != 0x10) // only default (0) and ALT1 (1) is supported, other values should fall back to default.
    {
        pinMode(PIN_WIRE_SDA,INPUT);
        disablePinISC(PIN_WIRE_SDA);

        pinMode(PIN_WIRE_SCL,INPUT);
        disablePinISC(PIN_WIRE_SCL);
    }
    else
    {
        pinMode(PIN_WIRE_SDA_PINSWAP_1,INPUT);
        disablePinISC(PIN_WIRE_SDA_PINSWAP_1);

        pinMode(PIN_WIRE_SCL_PINSWAP_1,INPUT);
        disablePinISC(PIN_WIRE_SCL_PINSWAP_1);
    }

    #elif defined ARDUINO_avrdd
    pinMode(SDA_NOW,INPUT);
    disablePinISC(SDA_NOW);

    pinMode(SCL_NOW,INPUT);
    disablePinISC(SCL_NOW);
    #endif
}

static void I2C_pullup(void)
{
    #if defined (__AVR_ATmega4808__)
    if ((PORTMUX.TWISPIROUTEA & 0x30) != 0x10) // only default (0) and ALT1 (1) is supported, other values should fall back to default.
    {
        pinMode(PIN_WIRE_SDA,INPUT_PULLUP);
        pinMode(PIN_WIRE_SCL,INPUT_PULLUP);
    }
    else
    {
        pinMode(PIN_WIRE_SDA_PINSWAP_1,INPUT_PULLUP);
        pinMode(PIN_WIRE_SCL_PINSWAP_1,INPUT_PULLUP);
    }

    #elif defined (ARDUINO_avrdd)
    pinMode(SDA_NOW,INPUT_PULLUP);
    pinMode(SCL_NOW,INPUT_PULLUP);
    #endif

    if (Config.I2CPowerPin >0)
    {
        digitalWrite(Config.I2CPowerPin, HIGH);
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
        I2C_pullup();
        delay(1);

        if(SHT->init()) // reads T and rH after initialization / wakeup
        {
            temperature = SHT->getTemperature();
            humidity    = SHT->getHumidity();
            print_humidity_sensor_values("SHT", temperature, humidity);
            success =  0x48; // according to SHT3X and SHTC3 bit in UseBits
        }
        else
        {
            mySerial->print("Error in SHT.init()\n");
        }

        I2C_shutdown();
    }
    return success;
}


static bool SHTC3_Init(UseBits &enable)
{
    if (enable.SHTC3)
    {
        SHTC3 = new SHTSensor(SHTSensor::SHTC3);
        mySerial->print("SHTC3: init(): ");
        enable.SHTC3 = SHTC3->init();
        if (enable.SHTC3)
        {
            mySerial->print("success\n");
        }
        else
        {
            mySerial->print("failed\n");
        }
    }
    return enable.SHTC3;
}

static bool SHT3X_Init(UseBits &enable)
{
    if (enable.SHT3X)
    {
        SHT3X = new SHTSensor(SHTSensor::SHT3X);
        mySerial->print("SHT3x: init(): ");
        enable.SHT3X = SHT3X->init();
        if (enable.SHT3X)
        {
            mySerial->print("success\n");
        }
        else
        {
            mySerial->print("failed\n");
        }
    }
   return enable.SHT3X;
}

static bool SHT4X_Init(UseBits &enable)
{
    if (enable.SHT4X)
    {
        SHT4X = new SHTSensor(SHTSensor::SHT4X);
        mySerial->print("SHT4x: init(): ");
        enable.SHT4X = SHT4X->init();
        if (enable.SHT4X)
        {
            mySerial->print("success\n");
        }
        else
        {
            mySerial->print("failed\n");
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
        mySerial->print("HTU21D: init(): ");
        enable.HTU21D = myHTU21D->begin();
        if (enable.HTU21D)
        {
            mySerial->print("success\n");
        }
        else
        {
            mySerial->print("failed\n");
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
#include <OneWire.h>              // license terms not clearly defined.

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
            mySerial->print("no ");

        }
        else
        {
            mySerial->print(num_devices, DEC);
        }

        mySerial->println(" DS18B20 found.");
        //mySerial->print("power pin: "); mySerial->println(Config.OneWirePowerPin);
        //mySerial->print("data  pin: "); mySerial->println(Config.OneWireDataPin);
        mySerial->flush();
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
        ADC0.CTRLA |= ADC_ENABLE_bm; // enable ADC
        delay(20);
        sensorValue = (uint16_t)(1023 - (analogRead(LDRPin)/(1<<ADC0.CTRLB)));
        pinMode(LDRPin, INPUT);
        ADC0.CTRLA &= ~ADC_ENABLE_bm; // disable ADC
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
        //pinMode(RTD_CS_PIN, INPUT_PULLUP);  // SPI SS (avoid becoming SPI Slave) // already done in setup()

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

/***   READ VCC ***/

long Vcal_x_ADCcal = 1500L * 1023L;
#define SAMPLE_ACCUMULATION 0x5 // 32 samples

// possible modes are: INTERNAL0V55, INTERNAL1V1, INTERNAL2V5, INTERNAL4V34, INTERNAL1V5

#if defined (__AVR_ATmega4808__)
int analogReadInternalRef(uint8_t mode)
{
  #if defined(ADC0)

  // save registers
  uint8_t vref_ctrla = VREF.CTRLA;
  uint8_t adc0_muxpos = ADC0.MUXPOS;
  uint8_t adc0_ctrlb = ADC0.CTRLB;

  // setup DACREF in AC0
  VREF.CTRLA = (VREF.CTRLA & ~(VREF_AC0REFSEL_gm)) | (mode << VREF_AC0REFSEL_gp);


  // Reference should be already set up and should be VDD
  // ADC0.CTRLC =0b x101 xxxx // 4808 SAMPCAP=1, reference = 0b01 = VDD
  // VREF.ADC0REF = VREF_REFSEL_VDD_gc

  // set input to DACREF0
  ADC0.MUXPOS = ADC_MUXPOS_DACREF_gc;

  // set sample accumulation
  ADC0.CTRLB = SAMPLE_ACCUMULATION;

  // Start conversion
  ADC0.COMMAND = ADC_STCONV_bm;

  // Wait for result ready
  while (!(ADC0.INTFLAGS & ADC_RESRDY_bm))
    ;

  // restore registers;
  VREF.CTRLA = vref_ctrla;
  ADC0.MUXPOS = adc0_muxpos;
  ADC0.CTRLB = adc0_ctrlb;
  // Combine two bytes
  return ADC0.RES;

    #else /* No ADC, return 0 */
      return 0;
    #endif
}


#elif defined (ARDUINO_avrdd)
int analogReadInternalRef(uint8_t mode)
{
    (void) mode;

    // disable ADC
    ADC0.CTRLA &= ~ADC_ENABLE_bm;

    // set reference to VDD
    VREF.ADC0REF = VREF_REFSEL_VDD_gc;

    // set DAC0REF to 1.024V
    VREF.DAC0REF = VREF_REFSEL_1V024_gc;

    // set input to DACREF0
    ADC0.MUXPOS = ADC_MUXPOS_DACREF0_gc;

    // set bitb resolution and conversion mode (single ended)
     ADC0.CTRLA = ADC_CONVMODE_SINGLEENDED_gc | ADC_RESSEL_10BIT_gc | ADC_ENABLE_bm;

    // set sample accumulation
    ADC0.CTRLB = SAMPLE_ACCUMULATION;

    // Start conversion
    ADC0.COMMAND = ADC_STCONV_bm;

    // Wait for result ready
    while (!(ADC0.INTFLAGS & ADC_RESRDY_bm));

    ADC0.CTRLA &= ~ADC_ENABLE_bm; // disable ADC
    // Combine two bytes
    return ADC0.RES;
}
#endif


long readVcc()
{
    #if defined (__AVR_ATmega4808__)
        return analogReadInternalRef(INTERNAL1V5) / (1<<SAMPLE_ACCUMULATION);
    #elif defined (ARDUINO_avrdd)
       return analogReadInternalRef(VREF_REFSEL_1V024_gc);
    #endif
}

float getVcc(long vref)
{
    return (float)vref / readVcc();
}


/*****************************************************************************/
/***                   READ MCU TEMPERATURE                                ***/
/*****************************************************************************/

// this is an attempt to use the analog temperature sensor in the MCU
// test have shown that the factory calibration is insufficient.

static float readMcuTemperaure(void)
{
#if defined (MEGACOREX)
    uint16_t temp_raw=0;
    uint32_t temp_l=0;
    float temp;
    int8_t sigrow_offset = SIGROW.TEMPSENSE1; // Read signed value from signature row
    uint8_t sigrow_slope = SIGROW.TEMPSENSE0; // Read unsigned value from signature row

    analogReference(INTERNAL1V1);
    ADC0.MUXPOS = ADC_MUXPOS_TEMPSENSE_gc;

    //uint8_t vref_ctrla = VREF.CTRLA;
    uint8_t adc0_sampctrl = ADC0.SAMPCTRL;
    uint8_t adc0_ctrld = ADC0.CTRLD;

    ADC0.CTRLD &= ~0xE0;
    ADC0.CTRLD |= 0x60;
    ADC0.SAMPCTRL = 0x04;


    // Start conversion
    ADC0.COMMAND = ADC_STCONV_bm;

    // Wait for result ready
    while (!(ADC0.INTFLAGS & ADC_RESRDY_bm));

    temp_raw = ADC0.RES;

    temp_l = temp_raw - sigrow_offset;
    temp_l *= sigrow_slope;
    temp_l += 0x40;
    temp_l >>= 7;
    //temp = (float)temp_l /256.0 -273;
    temp = (float)temp_l /2.0 -273;

    //restore Analog reference to VDD
    analogReference(VDD);

    // restore registers
    ADC0.SAMPCTRL = adc0_sampctrl;
    ADC0.CTRLD = adc0_ctrld;
    #if DEBUG >0
    mySerial->print("ADC TEMPSENS: 0x"); mySerial->println(temp_raw);
    mySerial->print("ADC CTRLC   : 0x"); mySerial->println(ADC0.CTRLC,HEX);
    #endif
    return temp;

#else if (ARDUINO_avrdd)
    uint16_t temp_raw=0;
    uint32_t temp_l=0;
    float temp;

    uint8_t adc0_sampctrl = ADC0.SAMPCTRL;
    uint8_t adc0_ctrld = ADC0.CTRLD;
    uint8_t adc0_ctrlb = ADC0.CTRLB;
    #define NUM_SAMPLES ADC_SAMPNUM_ACC8_gc

    // disable ADC
    ADC0.CTRLA &= ~ADC_ENABLE_bm;

    analogReference(VREF_REFSEL_2V048_gc);

    // Sample accumulation
    ADC0.CTRLB = NUM_SAMPLES;

    ADC0.MUXPOS = ADC_MUXPOS_TEMPSENSE_gc;
    ADC0.CTRLD &= ~0xE0; // clear init delay bits
    ADC0.CTRLD |= ADC_INITDLY_DLY16_gc; // set init delay
    ADC0.SAMPCTRL = 0x10; // increase of sampling time


    // set bitb resolution and conversion mode (single ended)
    ADC0.CTRLA = ADC_CONVMODE_SINGLEENDED_gc | ADC_RESSEL_12BIT_gc | ADC_ENABLE_bm; // enable

    uint16_t sigrow_offset = SIGROW.TEMPSENSE1; // Read offset value from signature row
    uint16_t sigrow_slope = SIGROW.TEMPSENSE0;  // Read slope value from signature row

    // Start conversion
    ADC0.COMMAND = ADC_STCONV_bm;

    // Wait for result ready
    while (!(ADC0.INTFLAGS & ADC_RESRDY_bm));
    temp_raw = ADC0.RES;

    #if DEBUG > 0
    //mySerial->print("\r\nADC RES      : "); mySerial->println(temp_raw);
    #endif

    // apply formula according to Data Sheet
    temp_l = sigrow_offset - (temp_raw>>NUM_SAMPLES);
    temp_l *= sigrow_slope;


    // choose resolution
    //#define SF 4096 // 1 deg resolution
    //#define SF 2048 // 1/2 deg resolution
    //#define SF 1024 // 1/4 deg resolution
    #define SF 512 // 1/8 deg resolution
    #define SCAL (4096/SF)


    temp_l += SF/2;
    temp_l /= SF;
    temp = (float)temp_l / SCAL - 273;

    ADC0.CTRLA &= ~ADC_ENABLE_bm; // disable ADC
    ADC0.SAMPCTRL = adc0_sampctrl;
    ADC0.CTRLD = adc0_ctrld;
    ADC0.CTRLB = adc0_ctrlb;
    return temp;
#endif
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
  #if defined ARDUINO_avrdd
  temp |= CLKCTRL_LPMODE_bm;
  #endif
  CPU_CCP = CCP_IOREG_gc;
  CLKCTRL.XOSC32KCTRLA = temp;

  // wait for all registers to be synchronized
  while (RTC.STATUS > 0);

  // select 32.768 kHz external crystal oscillator
  #if defined (MEGACOREX)
  RTC.CLKSEL = RTC_CLKSEL_TOSC32K_gc;
  #elif defined (ARDUINO_avrdd)
  //RTC.CLKSEL = RTC_CLKSEL_XTAL32K_gc; //compiler error
  RTC.CLKSEL = 0x02;
  #endif
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
    uint8_t* sernum_addr = (uint8_t*)&SIGROW.SERNUM0;
    int i;
    Serial.print("Serial Number ");
    #if defined (ARDUINO_avrdd)

    #if defined __AVR_AVR64DD32__
    Serial.print("AVR64DD32: ");
    #elif defined __AVR_AVR64DD28__
    Serial.print("AVR64DD28: ");
    #else
    Serial.print("AVRxxDDxx: ");
    #endif
    // AVRxxDDxx have 16 Bytes
    for (i=0;  i<16; i++)
    {
        uint8_t sni = *sernum_addr;
        Serial.print(sni,HEX);
        Serial.print(" ");
        sernum_addr++;
    }


    #else
    #if defined (__AVR_ATmega4808__)
    Serial.print("ATMEGA4808: ");
    #endif
    SerialNumber SN;
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

    Serial.print(SN.prefix); Serial.print(" "); Serial.println(SN.sn);
    #endif
}
/*****************************************************************************/


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

    print_serial_number();

    mySerial->print("\r\nNode ID: ");mySerial->println(Config.Nodeid);

    #if DEBUG >= 2
    print_eeprom(mySerial);
    #endif
    // disable serial RX
    pinMode(1, INPUT_PULLUP);
    disablePinISC(1);



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

    Wire.swap(0); // 0 is default, 1 is identical with 0, 3 has the same pins as UART0
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
        PIT_init(RTC_PERIOD_CYC32768_gc);  // results in 1s period (maximum)
        Config.Senddelay *= 2; // so the total delay time  is 1s * 2 * SENDDELAY. i.e. SENDDELAY= 30 results in 1 minute delay
        RTC_Xosc32k_Init();
    }
    else
    {
        PIT_init(RTC_PERIOD_CYC8192_gc);  //  8s period
        RTC_ULP32k_init();
    }

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



// calculate TX Flag depending on events
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


void print_flag(uint8_t flag)
{
    if (flag &0x1)
    {
        mySerial->println("\r\nHeartBeat");
    }

    flag >>=1;
    if (flag)
    {
        mySerial->print("event: PCI");
        for (int i=0; i<4; i++)
        {
            if (flag&0x1)
            {
                mySerial->print((char)('0'+i));
            }
            flag >>= 1;
        }
    }
    mySerial->println();
}




// measure Battery voltage, try all sensors if configured or not and fill the payload accordingly.
// return temperature. (because temp is used by the radio for frequency compensation.)
float Measure(void)
{
    float temperature=0;
    float t_internal =0; // the temperature we return must be a internal sensor.
    float humidity =0;

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
    // BME280 has priority over SHT4x. SHT4x has priority over SHTC3, and so on.
    success |= HTU21D_Measure(TiNo->use.HTU21D, temperature, humidity);
    success |= SHT_Measure(TiNo->use.SHT3X, SHT3X, temperature, humidity);
    success |= SHT_Measure(TiNo->use.SHTC3, SHTC3, temperature, humidity);
    success |= SHT_Measure(TiNo->use.SHT4X, SHT4X, temperature, humidity);

    // BME280 measurement
    if(TiNo->use.BME280)
    {
        float pressure;
        success |= BME280_Measure(true, temperature, humidity, pressure);
        TiNo->pressure((uint32_t)floor(pressure*100));
    }

    // if a temperature/humidity sensor is present, copy measurement results into Payload
    if(success)
    {
        TiNo->add_temp(encode_temp(temperature));
        TiNo->humidity(encode_humidity(humidity));

        // all above sensors are usually internal, PT100 and DS18B20 are most lilely external.
        t_internal = temperature;
    }

    // PT100 Measurement
    if(TiNo->use.MAX31865)
    {
        uint8_t status = MAX31865_Measure(TiNo->use.MAX31865, &temperature);
        if (status==0)
        {
            success |= 0x1;
            TiNo->add_temp(encode_temp(temperature));
        }
    }

    // Dallas DS18B20 Measurement
    if(TiNo->use.DS18B20)
    {
        uint16_t t[3];
        t[0]=t[1]=t[2]=0;
        uint8_t num_ds18b20 = DS18B20_Measure(TiNo->use.DS18B20, t);
        for (uint8_t i=0; i<num_ds18b20; i++)
            TiNo->add_temp(t[i]);

        success |= num_ds18b20;
        // todo: if the DS18B20 is the only sensor, we must set temperature = t[0], and t_internal = temperature;
    }

    // in case no temperature sensor is detected, we use the temperature from the RF Module
    if (!success)
    {
        mySerial->println("no temp sensor");
        temperature = temperature = 12.34;
        #ifdef USE_RADIO
            temperature = radio.readTemperature(0) + Config.radio_temp_offset/10.0;
        #endif
        TiNo->add_temp(encode_temp(temperature));
        t_internal = temperature;
        #if DEBUG >0
            print_humidity_sensor_values("RFM69: ", temperature, humidity);
        #endif
    }

    TiNo->brightness( LDR_Measure(TiNo->use.BRIGHTNESS, Config.LdrPin));

    // for the temperature sensor in the MCU I have currently no use.
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

    print_flag(Flags);

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