// RFM69CW Sender for TiNo2 temperature / humidity Sensors with Watchdog.
// Supports Sensors with HTU21D, SHT20, SHT21, SHT25, SHT3C(default on-board chip), SHT30, SHT31, SHT35,
// SHT40, SHT41, SHT43, SHT45, BME280, DS18B20, MAX31865 (PT100 or PT1000),
// built for AVR ATMEGA4808 and AVR DD devices


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
// Uses OneWire from https://github.com/nurazur/OneWire    // license terms not clearly defined.
// Uses DallasTemperature from https://github.com/milesburton/Arduino-Temperature-Control-Library under GNU LGPL version 2.1 or any later version.
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
        calibration mode: 'to' starts the OOK Modus (send a CW signal)
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
#include <SPI.h>
#include "user_config.h"
#include "tino2.h"

#include "RFM69registers.h"

//#define FILENAME "myTiNo2V7.ino V2.7.1 31/10/2025"

// RTD bug fix
#define FILENAME "myTiNo2V7.ino V2.7.2 01/12/2025"
#define BUILD 11
#define RADIO_SPI_PINSWAP 0

#define P(a) if(Config.SerialEnable) mySerial->a

/*****************************************************************************/
/***                            User Configuration                         ***/
/*****************************************************************************/
// User Configuration has moved to user_config.h

// PIT (Periodic Interrupt Timer) can be used in sender and receiver. However this
// combined code // shares the PIT ISR for both the sleep period control (sender)
// and the action Pulse control (receiver), making the ISR code conflicting in the
// sender case. So using the RTC is the standard for the sender.
// Don't change the following #define!
//
// #define USE_PIT_FOR_SENDER


/*****************************************************************************/
/***              One-Wire and DS18B20 Temperature Sensors                 ***/
/*****************************************************************************/
// using DS18B20 is not recommended, the conversion time takes much longer
// (700-900 ms) than with I2C based sensors (<50ms).
#if defined USE_DS18B20
#include "ds18b20.h"
#warning DS18B20 Temperature Sensor included
#endif

/****************************************************************************/
/**********                    MAX31865   PT100(0)                 **********/
/****************************************************************************/
#if defined USE_MAX31865
#include "tino_max31865.h"
#warning MAX31865 RTD included
#endif

#define MAX31865_INTERRUPT 1

#if (defined TINO_MAX31865_H && MAX31865_INTERRUPT)
#define PIN18INTERRUPTFUNC
void Pin18InteruptFunc(){}
#endif


/****************************************************************************/
/**********                    MAX31855 k-Type Thermocouple        **********/
/****************************************************************************/
#if defined USE_MAX31855
#include "tino_max31855.h"
#endif

/****************************************************************************/
/**********                  MAX31856     Type Thermocouple        **********/
/****************************************************************************/
#if defined USE_MAX31856
    #include "tino_max31856.h"

    // not recommended for use with small batteries. Use above 3.0V must be ensured!
    // Measurement method: Polling or Interrupt driven.
    // the interrupt driven method requires DRDY of MAX31856 to be connected to Pin 18.
    // the interrupt driven method saves power during the conversion period.
    //#define MAX31856_POLLING
    #define MAX31856_INTERRUPT

    #if (defined TINO_MAX31856_H && defined MAX31856_INTERRUPT)
        #ifndef PIN18INTERRUPTFUNC
        void Pin18InteruptFunc(){}
        #define PIN18INTERRUPTFUNC
        #endif
    #endif
#endif

/****************************************************************************/
/**********                    MAX6675 k-Type Thermocouple         **********/
/****************************************************************************/
// obsolete Chip. Use MAX31855 instead
#if defined USE_MAX6675
#include "tino_max6675.h"
#endif

/****************************************************************************/
/**********                    ADS1120 ADC                         **********/
/****************************************************************************/
#if defined USE_ADS1120 || defined USE_ADS1220
#include "tino_ads1220.h"
	#ifdef ADS1x20_RTD
	extern RTD_1120 *Rtd1120;
	PT100_Struct Rtd;
	#endif
#endif


/****************************************************************************/
/**********                    TiNo Packet Manager                 **********/
/****************************************************************************/
PacketHandler *TiNo=NULL;


/*****************************************************************************/
/***   Radio Driver Instance                                               ***/
/*****************************************************************************/
RADIO radio; // standard interrup pi for TiNo2 is 14
//RADIO radio(SS,15,0, digitalPinToInterrupt(15)); // for TiNo2 development boards, series 0 only



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
// Serial and Serial2 have alternate pin positions, using the swap function.
#ifndef SERIAL_BAUD
#warning no baud rate for serial port specified: fall back to 38400 Bd
#define SERIAL_BAUD  38400
#endif

#define SERIAL_SWAP     0
HardwareSerial *mySerial = &Serial;

/*****************************************************************************/
/***                   Calibration Module                                  ***/
/*****************************************************************************/
Calibration CalMode(Config, mySerial, &Mac, BUILD, (uint8_t*) KEY);


/*****************************************************************************/
/***                   Sleep mode                                          ***/
/*****************************************************************************/

uint16_t watchdog_counter=0;


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

// interrupt service routine for RTC periodic timer
ISR(RTC_PIT_vect)
{
    RTC.PITINTFLAGS = RTC_PI_bm;              // clear interrupt flag
    watchdog_counter++;
    #if IS_RECEIVER
    PIT.interrupthandler();
    #endif
}

/*****************************************************************************/
/***              SHT3x and SHTC3  Humidity Sensor                         ***/
/*****************************************************************************/
#include "sht_sensors.h"
HumiditySensor SensorData;

SHTSensor *SHT3X=NULL;
SHTSensor *SHTC3=NULL;
SHTSensor *SHT4X=NULL;


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
/***              BME280 air pressure and humidity Sensor                  ***/
/*****************************************************************************/
#include <BME280I2C.h>
uint8_t BME280_i2c_address = 0x76;
BME280I2C *BME280;

static bool BME280_Init(UseBits &enable, uint8_t i2c_address)
{
    if(enable.BME280)
    {
	   BME280I2C::Settings settings(
	   BME280::OSR_X1,
	   BME280::OSR_X1,
	   BME280::OSR_X1,
	   BME280::Mode_Forced,
	   BME280::StandbyTime_1000ms,
	   BME280::Filter_Off,
	   BME280::SpiEnable_False,
	   (BME280I2C::I2CAddr)i2c_address // 0x76 (default) or 0x77);
		);

        BME280 = new BME280I2C(settings);
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
		   delay(8); // conversion time see data sheet 9.2
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
    SPI.setDataMode(mode);
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

/*****************************************************************************/
/***                   READ VCC                                            ***/
/*****************************************************************************/
//function to read VCC is in analog.h
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

//Module is  PIR.h
PIRModule PIR(Config, mySerial, wakeUp1);


#if IS_RECEIVER
/*****************************************************************************/
/***                       Actions                                         ***/
/*****************************************************************************/
/* Globals */
extern action* actions;
extern uint8_t num_actions;


/**********************************************************************/
/***  each interrupt occupies 2 bits in serial protocol definition  ***/
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
#else
// dummy function in sender
static void extract_interrupts(byte flags)
{
	(void) flags;
}
#endif

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


void initialize_pin_change_interrupts(Configuration Config)
{
    /* MegacoreX common.h */

//  Trigger:
// #define LOW            0
// #define HIGH           1

// #define FALLING        2
// #define RISING         3
// #define CHANGE         4

// Mode:
// #define INPUT          0
// #define OUTPUT         1
// #define INPUT_PULLUP   2

// on ATmega4808 only Pins Px2 and Px6 are fully asynchronuous, these are pins 10,14,18,22 (PC2, PD2, PD6, PF2)
// Pin 14 is reserved for RFM69 Module
// Thermocouple and RTD devices use Pin 18 for interrupts
// on AVR64DD32/28 all GPIOs are fully asynchronuous

    if (Config.PCI0Pin >=0)
    {
        pinMode(Config.PCI0Pin, Config.PCI0Mode);
        register_pci(0, Config.PCI0Pin, wakeUp0, Config.PCI0Trigger);
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
}


/*****************************************************************************/
/***              I2C Bus scanner and device check                         ***/
/*****************************************************************************/


// check I2C bus on address 0x45 or 0x44 if device is a OPT3001
// OPT3001 has a manufacturer-ID at address 0x7E,  must read 0x5449
bool is_opt3001(uint8_t address)
{
    uint16_t deviceID;
    Wire.beginTransmission(address);
    Wire.write(0x7E);
    Wire.endTransmission(true);
    Wire.requestFrom(address, (size_t)2, true);

    deviceID  = Wire.read() << 8;
    deviceID |= Wire.read();
    //Serial.print("deviceID: ");Serial.println(deviceID, HEX);
    if (deviceID == 0x5449)
    {
        return true;
    }
    return false;
}


uint8_t sht_crc8(const uint8_t *data, uint8_t len, uint8_t crcInit)
{
  // adapted from SHT21 sample code from
  // http://www.sensirion.com/en/products/humidity-temperature/download-center/

  uint8_t crc = crcInit;
  uint8_t byteCtr;
  for (byteCtr = 0; byteCtr < len; ++byteCtr) {
    crc ^= data[byteCtr];
    for (uint8_t bit = 8; bit > 0; --bit) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ 0x31;
      } else {
        crc = (crc << 1);
      }
    }
  }
  return crc;
}

// request serial number and check if crc is correct
bool is_sht3x(uint8_t address)
{
    uint8_t snum[6];
    uint8_t crc;
    Wire.beginTransmission(address);
    Wire.write((uint8_t)0x36);
    Wire.write((uint8_t)0x82);
    Wire.endTransmission();
    delay(10);
    Wire.requestFrom(address, (size_t)6, true);

    if (Wire.available() != 6)
    {
        Wire.endTransmission(true);
        return false;
    }

    for (int i=0; i<6; i++)
    {
        snum[i] = Wire.read();
    }


    crc = sht_crc8(snum, 2, 0xFF);
    Wire.endTransmission(true);

    if (crc == snum[2])
    {
        //Serial.println("device is a SHT3x");
        return true;
    }
    return false;
}

// sht4x returns its 32 bit serial number with command 0x89
// if the crc8 of the read number is ok, we assume the chip to be a SHT4x
// SHT3x returns some data with this command, crc8 is ok as well, but the command is not documented
bool is_sht4x(uint8_t address)
{
    uint8_t snum[6];
    //uint16_t serialnum=0;
    //uint16_t serialnum1 =0;
    uint8_t crc;
    uint8_t crc1;
    Wire.beginTransmission(address);
    Wire.write((uint8_t)0x89);

    Wire.endTransmission();
    delay(10);
    Wire.requestFrom(address, (size_t)6, true);

    if (Wire.available() != 6)
    {
        Wire.endTransmission(true);
        return false;
    }

    for (int i=0; i<6; i++)
    {
        snum[i] = Wire.read();
    }

    crc = sht_crc8(snum, 2, 0xFF);
    crc1 = sht_crc8(&snum[3],2,0xFF);
    Wire.endTransmission(true);

    if (crc == snum[2] && crc1== snum[5])
    {
        return true;
    }
    return false;
}

uint8_t OPT3001_i2c_address = 0x45;

void check_i2c_address(byte address, UseBits* devices)
{
    switch(address)
    {
        case 0x40:
            Serial.println("SHT2x / HTU21D");
            devices->HTU21D =1;
            break;
        case 0x44:
            //  3 in this project possibly supported devices have address 44: OPT3001, SHT4x and SHT3x
            // Serial.println("SHT3x or SHT4x or ALT OPT3001");
            // try to identify a OPT3001

            if (is_opt3001(address))
            {
                Serial.println("OPT3001");
                devices->OPT3001 =1;
                OPT3001_i2c_address = address;
            }
            else if (is_sht3x(address))
            {
                Serial.println("SHT3x");
                devices->SHT3X =1;
            }
            else if (is_sht4x(address))
            {
                Serial.println("SHT4x");
                devices->SHT4X = 1;
            }
            break;

        case 0x45:
            //Serial.println("OPT3001 or ALT SHT3x");
            // try to identify a OPT3001
            if (is_opt3001(address))
            {
                Serial.println("OPT3001");
                devices->OPT3001 =1;
                OPT3001_i2c_address = address;
            }
            else if (is_sht3x(address))
            {
                Serial.println("SHT3x (alternate address)");
                devices->SHT3X_ALT = 1;
            }
            break;
        case 0x46: // OPT3001 ADDR Pin tied to SDA
        case 0x47: // OPT3001 ADDR Pin tied to SCL
            if (is_opt3001(address))
            {
                Serial.println("OPT3001");
                devices->OPT3001 =1;
                OPT3001_i2c_address = address;
            }
            else
            {
                Serial.println("identifying OPT3001 failed.");
            }
            break;

        case 0x50:
            Serial.println("EEPROM");
            devices->EEPROM = 1;
            break;
        case 0x70:
            Serial.println("SHTC3");
            devices->SHTC3 = 1;
            break;
        case 0x76:
        case 0x77:
            Serial.println("BME280");
            devices->BME280 = 1;
			BME280_i2c_address = address;
            break;
        default:
			Serial.println("unknown device");
            break;
    }
}


void Scan4I2cDevices(UseBits* devices)
{
    byte error, address;
    int nDevices;

    Serial.println("Scan I2C Bus...");

    nDevices = 0;
    for(address = 1; address < 127; address++)
    {
        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device did acknowledge to the address.

        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0)
        {
          Serial.print("0x");
          if (address < 16)
              Serial.print("0");
          Serial.print(address, HEX);Serial.print(" ");

          check_i2c_address(address, devices);
          nDevices++;
        }
        else if (error == 4)
        {
          Serial.print("Unknown error at address 0x");
          if (address < 16)
            Serial.print("0");
          Serial.print(address, HEX);Serial.print(" ");
        }
    }

    if (nDevices == 0)
        Serial.println("No I2C devices found");
    else
        Serial.println("Scan done");
}


void initialize_i2c_devices(UseBits* sensors);
void initialize_i2c_devices(UseBits* sensors)
{
    if (Config.I2CPowerPin > 0)
    {
        pinMode(Config.I2CPowerPin, OUTPUT);
        digitalWrite(Config.I2CPowerPin, HIGH);
        delay(1);
    }

    Wire.swap(0); // 0 is default, 1 is identical with 0, 3 has the same pins as UART0
    Wire.begin();

	Config.SensorConfig &= 0xFF00; // set all I2C devices to 0, we have now automatic scan

	Scan4I2cDevices(sensors);

    if (sensors->BME280 && F_CPU > 1000000)
        Wire.setClock(50000);

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

	if (sensors->SHT3X_ALT)
    {
        sensors->SHT3X_ALT = SHT_Init(sensors->SHT3X_ALT, SHT3X, SHTSensor::SHT3X_ALT);
        print_init_result(sensors->SHT3X_ALT, "SHT3X (ALT)");
    }

    if (sensors->SHT4X)
    {
        sensors->SHT4X = SHT_Init(sensors->SHT4X, SHT4X, SHTSensor::SHT4X);
        print_init_result(sensors->SHT4X, "SHT4X");
    }

    HTU21D_Init  (*sensors);
    BME280_Init  (*sensors, BME280_i2c_address);
}


void initialize_1wire_devices(UseBits* sensors)
{
    #if defined USE_DS18B20
    if (sensors->DS18B20)
    {
        uint8_t num_ds18b20 = DS18B20_Init (sensors->DS18B20, Config.OneWirePowerPin, Config.OneWireDataPin);
        if (num_ds18b20==0) sensors->DS18B20=0;
        mySerial->print("DS18B20 found: "); mySerial->println(num_ds18b20);
    }
    #else
    (void) sensors ;
    #endif
}


void initialize_analog_devices(UseBits* sensors)
{
    if (sensors->BRIGHTNESS)
    {
        sensors->BRIGHTNESS = LDR_Init(Config.LdrPin);
        if (sensors->BRIGHTNESS)
            mySerial->println("LDR initialized.");
        else
            mySerial->println("LDR NOT initialized.");
    }
}


void initialize_thermocouple_devices(UseBits* sensors)
{
    #ifdef TINO_MAX31865_H
    #if MAX31865_INTERRUPT
	if (sensors->MAX31865)
		attachInterrupt(18, Pin18InteruptFunc, FALLING); // define interrupt on Pin 18, fully asynchronuous Pin (4808 only)
    #endif
    RTD_Init(sensors->MAX31865, Config.RTDPowerPin, Config.RTDCSPin);
	if (sensors->MAX31865)
	{
		mySerial->println("RTD (PT100) device MAX31865 initialized.");
		RTD_Sleep(TiNo->use.MAX31865);
	}
    #if DEBUG > 0
    #warning MAX31865 (RTD) Module included in this build
    #endif
    #endif

    #ifdef TINO_MAX31855_H
    if (sensors->MAX31855)
    {
        ThermoCouple_Init_55(sensors->MAX31855, Config.RTDPowerPin, Config.TCCSPin);
        mySerial->println("Thermocouple device MAX31855 initialized.");
    }
    #if DEBUG > 0
    #warning MAX31855K (K type Thermocouple) Module included in this build
    #endif
    #endif


    #ifdef TINO_MAX31856_H
    if (sensors->MAX31856)
    {
        #ifdef MAX31856_INTERRUPT
        attachInterrupt(18, Pin18InteruptFunc, FALLING); // define interrupt on Pin 18, fullt asynchronuous Pin (4808 only)
        #endif
        ThermoCouple_Init_56(sensors->MAX31856, Config.RTDPowerPin, Config.TCCSPin);
        mySerial->println("Thermocouple device MAX31856 initialized.");
    }
    #if DEBUG > 0
    #warning MAX31856 (Thermocouple) Module included in this build
    #endif
    #endif

    #ifdef TINO_MAX6675_H
    if (sensors->MAX31856)
    {
        ThermoCouple_Init_6675(sensors->MAX6675, Config.RTDPowerPin, Config.TCCSPin);
        mySerial->println("Thermocouple device MAX6675 initialized.");
    }
    #if DEBUG > 0
    #warning MAX6675 (K type Thermocouple) Module included in this build
    #endif
    #endif

    #if defined USE_ADS1120 || defined USE_ADS1220
		#ifdef ADS1x20_THERMOCOUPLE
		if(sensors->ADS1120)
		{
			uint8_t config_0;
			config_0 = ThermoCouple_Init_ADS1120(sensors->ADS1120, 18, Config.TCCSPin, ADS1120_DOSLEEP, ADS1x20_TYPE); // define interrupt on Pin 18, fully asynchronuous Pin
			mySerial->print("Thermocouple device ADS1120 ");
			config_0 &= REG_MASK_GAIN;
			config_0 = 1 << (config_0>>1);
			if (config_0 == 32)
				mySerial->println("initialized.");
			else
			{
				mySerial->println("not found.");
				sensors->ADS1120 =0;
			}
		}
			#if DEBUG > 0
			#warning ADS1x20 (K type Thermocouple) Module included in this build
			#endif

		#elif defined ADS1x20_RTD
		if(sensors->ADS1120)
		{
			#if DEBUG > 0
			Serial.println("initialize ADS1x20 as PT100 ADC...");
			print_parameter(RREF);
			print_parameter(Config.RTDCSPin);
			print_parameter(ADS1120_DOSLEEP);
			print_parameter(ADS1x20_TYPE);
			#endif

			Rtd.enable = sensors->ADS1120;
			if(Rtd.enable)
				Rtd1120 = new RTD_1120(18, Config.RTDCSPin, ADS1120_DOSLEEP, RREF, ADS1x20_TYPE);
			uint8_t rtd_error = 0;

			// todo: for unknown reason, the diagnostic and offset calibration don't work here.
			// rtd_error |= Rtd1120->WireBreakDetection_4WireRtd();
			// rtd_error |= Rtd1120->OffsetCalibration(Rtd);

			if (rtd_error)
			{
				Rtd.enable=0;
				Serial.print("RTD error code: 0x"); Serial.println(rtd_error, HEX);
			}
			else
			{
				Serial.println("ADS1x20 initialized.");
			}

			#if DEBUG > 0
			uint8_t config_reg;
			config_reg = Rtd1120->readRegister(CONFIG_REG0_ADDRESS);
			Serial.print("CONFIG_REG0_ADDRESS: "); Serial.println(config_reg, HEX); // should be 0x05

			config_reg = Rtd1120->readRegister(CONFIG_REG1_ADDRESS);
			Serial.print("CONFIG_REG1_ADDRESS: "); Serial.println(config_reg, HEX); // should be 0x60 (Data rate 175 SPS) or 0x00

			config_reg = Rtd1120->readRegister(CONFIG_REG2_ADDRESS);
			Serial.print("CONFIG_REG2_ADDRESS: "); Serial.println(config_reg, HEX); // should be 0x66

			config_reg = Rtd1120->readRegister(CONFIG_REG3_ADDRESS);
			Serial.print("CONFIG_REG3_ADDRESS: "); Serial.println(config_reg, HEX); // should be 0x80
			#endif
		}
		#else
			sensors->ADS1120=0; // cannot use a ADS1120 because neither RTD nor TC is specified.
			#if DEBUG >0
			Serial.println("/!\\ ADS1120 Measurement not specified. See user_config.h");
			#endif
		#endif
	#else
		#if DEBUG >0
		if (sensors->ADS1120)
		{
			Serial.println("/!\\ No ADS1x20 device included in build. See user_config.h");
			sensors->ADS1120=0; // cannot use a ADS1120, module is not included in build
		}
		#endif


	#endif
}


bool tick = true;


// interrupt service routine for RTC counter overflow
ISR(RTC_CNT_vect)
{
  RTC.INTFLAGS = 0xFF;              // clear interrupt flag
  tick = true;                      // raise tick flag
}


void RTC_init(uint8_t use_crystal, uint16_t intervall_s)
{
  if (!use_crystal)
  {
      RTC.CTRLA = RTC_RUNSTDBY_bm            // enable RTC in standby
              | RTC_PRESCALER_DIV1024_gc;    // prescale for 1 second, we have selected 32 prescaler
  }
  else
  {
      RTC.CTRLA = RTC_RUNSTDBY_bm            // enable RTC in standby
            | RTC_PRESCALER_DIV32768_gc;     // prescale for 1 second
  }
  while (RTC.STATUS & RTC_CTRLABUSY_bm);    // wait for done
  RTC.PER = intervall_s;                    // set RTC period in seconds
  while (RTC.STATUS & RTC_PERBUSY_bm);      // wait for done
  RTC.INTFLAGS = 0xFF;                      // clear interrupt flags
  RTC.INTCTRL = RTC_OVF_bm;                 // enable interrupt on overflow
  RTC.CTRLA |= RTC_RTCEN_bm;                // enable RTC
  while (RTC.STATUS & RTC_CTRLABUSY_bm);    // wait for done
}

/*********************/
void sensor_loop(void);

void setup()
{
	/***                    ***/
    /*** disable all GPIO's ***/
    /***                    ***/
    for (uint8_t i = 6; i < 26; i++)
    {
        pinMode(i, INPUT_PULLUP);
        disablePinISC(i);
    }

    /***                                ***/
    /*** enable Pin 5 or 9, MISO of SPI ***/
    /***                                ***/
    SPI_MISO_Enable(RADIO_SPI_PINSWAP);

    /***     SERIAL PORT     ***/
    pinConfigure(1, PIN_DIR_INPUT, PIN_PULLUP_ON, PIN_INPUT_ENABLE);
    mySerial->swap(SERIAL_SWAP);
    mySerial->begin(SERIAL_BAUD);
    mySerial->print(FILENAME); mySerial->print(" Build: "); mySerial->print(BUILD);mySerial->println();

    /***     CALIBRATE?      ***/
    CalMode.configure();

    /***     PRINT LOGO      ***/
    print_tino2_logo(mySerial);

    /***     PRINT SERIAL NUMBER AND NODE ID ***/
    print_serial_number(mySerial);
    mySerial->print("\r\nNode ID: ");mySerial->println(Config.Nodeid);

    /***     PRINT EEPROM CONFIGURATION cONTENT   ***/
    #if DEBUG >= 2
    //print_eeprom(Config, mySerial);
    #endif

    /***     BATTERY CALIBRATION VALUE    ***/
    Vcal_x_ADCcal = (long)Config.VccAtCalmV * Config.AdcCalValue;

    /***     PIN CHANGE INTERRUPTS    ***/
    initialize_pin_change_interrupts(Config);

    /***     INITIALIZE PIR SENSOR    ***/
    // it will be still for 3 cycles.
    // PIR shares the event flag with PCI1. PCI1 remains active, if specified.
    PIR.init();

    sei();

    pinMode(Config.LedPin, OUTPUT);

    SensorData.PowerPin = Config.I2CPowerPin;
    SensorData.pressure = 0;

    UseBits* sensors;
    sensors = (UseBits*)&Config.SensorConfig;

    /***  INITIALIZE I2C DEVICES ***/
    initialize_i2c_devices(sensors);

    /***   INITIALIZE 1WIRE DEVICES   ***/
    initialize_1wire_devices(sensors);

    /***   INITIALIZE ANALOG DEVICES   ***/
    initialize_analog_devices(sensors);

    /***   INITIALIZE THERMOCOUPLE SENSORS   ***/
    initialize_thermocouple_devices(sensors);

    /***  INITIALIZE RADIO MODULE ***/
    #ifdef USE_RADIO
    Reset_SPI(RADIO_SPI_PINSWAP);
    #if DEBUG >0
        #if DEBUG >1
        mySerial->print("RF Chip = "); Config.IsRFM69HW ?    mySerial->print("RFM69HCW") : mySerial->print("RFM69CW");  mySerial->println();
        mySerial->print ("FDEV_STEPS: ");mySerial->print(Config.FedvSteps);mySerial->println();
        #endif
    mySerial->println ("Start Radio.");
    #endif

    Mac.radio_begin(); // puts radio to sleep to save power.

    // for debug: this makes sure we can read from the radio.
    #if DEBUG > 0
    mySerial->println ("Radio running.");
    byte version_raw = radio.readReg(0x10);
    mySerial->print ("Radio chip ver: "); mySerial->print(version_raw>>4, HEX); mySerial->print (" Radio Metal Mask ver: "); mySerial->print(version_raw&0xf, HEX); mySerial->println();
    #endif
    #endif

    // up to here rx and tx are identical.
    /***********************************************************/

/*** TX specific ***/

    #if (IS_SENSOR_NODE)
    {
        // disable serial RX
        pinMode(1, INPUT_PULLUP);
        disablePinISC(1);

        /* Turn off I2C devices */
        I2C_shutdown(Config.I2CPowerPin);

        /* PACKET CALCULATOR */
        TiNo = new PacketHandler(*sensors);
        TiNo->pressure(0);
        TiNo->brightness(0);
        TiNo->nodeid(Config.Nodeid);
        TiNo->targetid(Config.Gatewayid);
        TiNo->humidity(0);

        #if DEBUG >=1
        mySerial->print("packet type: "); mySerial->print(TiNo->PacketType);
        mySerial->print(", leng: "); mySerial->println(TiNo->PacketLen);
        #endif

        #if defined USE_PIT_FOR_SENDER
        PIT.init(Config.UseCrystalRtc, 0);
        PIT.enable();
        if (Config.UseCrystalRtc) Config.Senddelay *= 8; // wake up from sleep period is 1s, so we need to multiply by 8.
        set_sleep_mode(SLEEP_MODE_PWR_DOWN);
        mySerial->println("USE PIT as wakeup counter");

        #else // Use RTC
        Config.UseCrystalRtc ? PIT.RTC_setCrystal() : PIT.RTC_ULP32k_init(); // set clock to 32k
        RTC_init(Config.UseCrystalRtc, (Config.Senddelay&0x1FFF)*8 - 1); // assume 32.768 kHz clock. Maximum is 18h 12m
        set_sleep_mode(SLEEP_MODE_STANDBY); // cannot use PWR_DOWN mode withn RTC, only possible with PIT
        mySerial->println("USE RTC as wakeup counter");
        #endif

        if (Config.UseCrystalRtc)
        {
            #if defined ARDUINO_avrdd
            CPU_CCP = CCP_IOREG_gc;
            CLKCTRL.OSCHFCTRLA |= 0x1; // Autotune feature on
            #endif
        }

        /*** SLEEP MODE ON ***/
        sleep_enable();    // enable sleep control
        sei();
        // set watchdog_counter to have an initial transmission when starting the sender.
        watchdog_counter = Config.Senddelay+1;

        mySerial->println("\n*******************");
        mySerial->println("*** SENSOR NODE ***");
        mySerial->println("*******************\n");
        mySerial->flush();

        while(1)
        {
            sensor_loop();
            mySerial->flush();
            sleep_cpu();
        }
    }
    #endif
    /*** TX specific end ***/


    #if (IS_RECEIVER)
    {
        #if NUM_CHANNELS >1
        float frec[NUM_CHANNELS];
        #endif

        mySerial->println("\n**************************");
        mySerial->println("* RECEIVER CONFIGURATION *");
        mySerial->println("**************************\n");

        /*** PACKET CALCULATOR ***/
        TiNo = new PacketHandler(*sensors);
        TiNo->pressure(0);
        TiNo->brightness(0);
        TiNo->nodeid(Config.Nodeid);
        TiNo->targetid(Config.Gatewayid);
        TiNo->humidity(0);

        /***    INITIALIZE ACTOR MODULE   ***/
        init_actions(mySerial);

        /*** INITIALIZE PERIODIC INTERRUPT TIMER ***/
        // used to time the pulse of pulsed actions
        PIT.init(Config.UseCrystalRtc); // decide between XO (external crystal) and ULPO
        PIT.disable();

        /*** INITIALIZE RTC TIMER ***/
        // used to generate tick for internal measurements
        RTC_init(Config.UseCrystalRtc, (Config.Senddelay&0x1FFF)*8 - 1); // assume 32.768 kHz clock. Maximum is 18h 12m


        if (Config.LedPin)
        {
            digitalWrite(Config.LedPin, HIGH);
            delay(1000);
            digitalWrite(Config.LedPin, LOW);
        }

        //nexttime = millis() + 20000;
    }
    #endif
}


// simple, but messy version
uint8_t calculate_tx_flag()
{
    uint8_t Flags = 0;
    #if defined USE_PIT_FOR_SENDER
    bool watchdog_expired = (watchdog_counter >= Config.Senddelay) && (Config.Senddelay != 0);
    #else
    bool watchdog_expired = tick && (Config.Senddelay != 0);
    #endif

    if (watchdog_expired)
    {
        watchdog_counter = 0;
        tick= false;
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

	#if (IS_SENSOR_NODE)
		#ifdef USE_RADIO
			TiNo->supplyV( Vcal_x_ADCcal / radio.vcc_dac);  // the VCC measured during last TX Burst.
		#else
			TiNo->supplyV( Vcal_x_ADCcal / readVcc() );
		#endif
	#else
			//RECEIVER
			TiNo->supplyV( Vcal_x_ADCcal / readVcc() );
	#endif


    // measure Sensors
    TiNo->temp_index = 0;
    uint16_t success =0;
    // TiNo RF protocol allows only for one temperature/humidity sensor.
    // if there are more than one, the last one ist reported.
    // BME280 has priority over SHT4x. SHT4x has priority over SHT3X, and so on.

	uint16_t* p_used_i2c_sensors = (uint16_t*) &(TiNo->use);
    uint16_t used_i2c_sensors = *p_used_i2c_sensors;

    if (TiNo->use.HTU21D)
    {
        success |= HTU21D_Measure(used_i2c_sensors & HTU21D_bm, myHTU21D, SensorData);
        if (success & HTU21D_bm) print_humidity_sensor_values("HTU21D", SensorData.temperature, SensorData.humidity, mySerial);
    }

    if (TiNo->use.SHT3X)
    {
        success |= SHT_Measure(used_i2c_sensors & SHT3X_bm, SHT3X, SensorData);
        if (success & SHT3X_bm) print_humidity_sensor_values("SHT3x", SensorData.temperature, SensorData.humidity, mySerial);
    }

    if (TiNo->use.SHTC3)
    {
        success |= SHT_Measure(used_i2c_sensors & SHTC3_bm, SHTC3, SensorData);
        if ((success & SHTC3_bm)) print_humidity_sensor_values("SHTC3", SensorData.temperature, SensorData.humidity, mySerial);
    }

    if (TiNo->use.SHT4X)
    {
        success |= SHT_Measure(used_i2c_sensors & SHT4X_bm, SHT4X, SensorData);
        if (success & SHT4X_bm) print_humidity_sensor_values("SHT4X", SensorData.temperature, SensorData.humidity, mySerial);
    }

    if (TiNo->use.BME280)
    {
        success |= BME280_Measure(used_i2c_sensors & BME280_bm, BME280, SensorData);
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

    #ifdef TINO_MAX31865_H
    // PT100 Measurement
    if(TiNo->use.MAX31865)
    {
        //uint8_t status = MAX31865_Measure(TiNo->use.MAX31865, &temperature);
        uint8_t status = RTD_Measure(TiNo->use.MAX31865, &temperature, MAX31865_INTERRUPT);
        if (status==0)
        {
            mySerial->print(F( "RTD Temp: "));
            mySerial->print(temperature,2); mySerial->println("º");
            success |= MAX31865_bm;
            TiNo->add_temp(encode_temp(temperature));
        }
		else
		{
			 mySerial->println("RTD failure of measurement.");
			 print_parameter(status);
		}
    }
    #endif


    #ifdef TINO_MAX31855_H
    if(TiNo->use.MAX31855)
    {
        float t[2];
        t[0] = t[1] = 0;
        uint8_t status  = Thermocouple_Measure_55(1, t);
        Serial.print("31855 TC stat: ");
        Serial.print(status);

        Serial.print(", CJ_TEMP: ");
        Serial.print(t[COLD_JUNCTION_TEMPERATURE], 3);
        Serial.print(", TC_TEMP: ");
        Serial.println(t[THERMOCOUPLE_TEMPERATURE], 3);

        if (status==0)
        {
            success |= MAX31855_bm;
            TiNo->add_temp(encode_temp(t[THERMOCOUPLE_TEMPERATURE]));
        }
    }
    #endif

    #ifdef TINO_MAX31856_H
    if(TiNo->use.MAX31856)
    {
        float t[2];
        t[0] = t[1] = 0;
        pinMode(MISO, INPUT); // special for this device. It has internal pull-up to VCC

        //***  Measurement by polling ***
        #ifdef MAX31856_POLLING
        uint8_t status  = ThermoCouple_Measure_56(1, t); // polling on bit 6 (1SHOT) in CR0 register
        #endif

        //***   Measurement interrupt driven (current saving 0.4mAs @ 4MHz) **
        #ifdef MAX31856_INTERRUPT
        ThermoCouple_Convert_56(1);
        sleep_cpu();
        uint8_t status = ThermoCouple_GetTemperatures(1, t);
        #endif

        Serial.print("31856 TC stat: ");
        Serial.print(status);
        Serial.print(", CJ_Temp: "); Serial.print(t[COLD_JUNCTION_TEMPERATURE], 3);
        Serial.print(", TC_temp: ");  Serial.println(t[THERMOCOUPLE_TEMPERATURE], 3);
        if (status==0)
        {
            success |= MAX31856_bm;
            TiNo->add_temp(encode_temp(t[THERMOCOUPLE_TEMPERATURE]));
        }
        else
        {

        }
    }
    #endif


    if(TiNo->use.MAX6675)
    {
	#ifdef TINO_MAX6675_H
        float t=0;
        uint8_t status  = Thermocouple_Measure_6675(1, &t);

        Serial.print(" 6675 TC stat: ");
        Serial.print(status);
        Serial.print(", TC_temp: ");  Serial.println(t, 3);
        if (status==0)
        {
            success |= MAX6675_bm;
            TiNo->add_temp(encode_temp(t));
        }
	#else
		TiNo->use.MAX6675 = 0;
	#endif
    }


	#if defined USE_ADS1120 || defined USE_ADS1220
	#ifdef ADS1x20_THERMOCOUPLE
    if(TiNo->use.ADS1120)
    {
        float t[2];
        t[0] = t[1] =0;
        Thermocouple_Measure_ADS1120(TiNo->use.ADS1120, t);
        Serial.print("ADS1120: CJ_TEMP: ");
        Serial.print(t[COLD_JUNCTION_TEMPERATURE], 3);
        Serial.print(", TC_TEMP: ");
        Serial.println(t[THERMOCOUPLE_TEMPERATURE], 3);
        success |= ADS1120_bm;
        TiNo->add_temp(encode_temp(t[THERMOCOUPLE_TEMPERATURE]));
    }
	#elif defined ADS1x20_RTD
	if(TiNo->use.ADS1120 && Rtd.enable)
    {	SPI.begin();
		SPI.setDataMode(SPI_MODE1);
		uint8_t rtd_error =0;
		rtd_error = Rtd1120->WireBreakDetection_4WireRtd();
		rtd_error |= Rtd1120->OffsetCalibration(Rtd);
		rtd_error |= Rtd1120->Measure(Rtd, 1);
		Serial.print("RTD_Temp: "); Serial.println(Rtd.temperature,3);

		if (rtd_error)
		{
			print_parameter(rtd_error);
		}
		else
		{
			success |= ADS1120_bm;
			TiNo->add_temp(encode_temp(Rtd.temperature));
		}
		//success |= ADS1120_bm;
        //TiNo->add_temp(encode_temp(Rtd.temperature));
		#if DEBUG >0
		print_parameter(rtd_error);
		print_parameter(Rtd.ADCvalue);
		print_parameter(Rtd.ADCoffset);
		print_parameter(Rtd.resistance);
		#endif
	}

	#endif
    #endif

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

#if (IS_SENSOR_NODE)
void sensor_loop()
{
    static uint32_t total_count=0;
    float temperature;

    #if defined MEGACOREX
    // detect false wakeup. this is a pci interrupt from a pin on the wrong edge. ATmega4808 only.
    if (event_triggered & 0x80)
    {
		activityLed(1,30);
        event_triggered=0;
        //return; // start over, go sleep. this is logically wrong as we need to check if watchdog expired at the the same event. This is a bug.
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
	print_parameter(total_count);
    TiNo->increment_count();

    /*** carry out measurements according to configuration ***/
    SPI_MISO_Enable(RADIO_SPI_PINSWAP);
	mySerial->print("start measure");mySerial->flush();
	temperature = Measure();

    #if defined BATTERYTEST
    if (!(total_count &0x1)) // even counts, we send 2 VCC's and 24 bit count
    {
        TiNo->nodeid(Config.Nodeid | 0x80); // 2 nodeids. Normal node and measurement packets alternating!

        // temp is used for idle voltage, humidity and flags are used as counter bytes.
        TiNo->humidity(((uint8_t*)&total_count)[1]);
        TiNo->flags(((uint8_t*)&total_count)[2]);
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
        //SPI.end();
        Reset_SPI(RADIO_SPI_PINSWAP);

		//RTD IC needs to be powered, because shared SPI Bus interferes heavily with RF burst
		#if defined USE_RTD_DEVICE
        digitalWrite(Config.RTDPowerPin, 1);
		#endif

        if(Config.UseRadioFrequencyCompensation)
        {
            ackreceived = Mac.radio_send(TiNo->pData, TiNo->PacketLen, Config.RequestAck, (int)temperature);
        }
        else
        {
            ackreceived = Mac.radio_send(TiNo->pData, TiNo->PacketLen, Config.RequestAck);
        }
		#if DEBUG >0
		mySerial->println("RFM burst sent.");
		#endif
        if (Config.RequestAck && Config.SerialEnable)
		{
            if (ackreceived)
			{
				mySerial->println("Ack received.");
				mySerial->print("RSSI: ");    //
				mySerial->println(Mac.rxpacket.RSSI,1);
			}
			else
			{
				mySerial->println(" No Ack received.");
			}
		}
        if (ackreceived)
        {
            activityLed (1, 100);
        }
        #else
            mySerial->println("No burst is sent because of #SEND_BURST directive missing");
        #endif

    #endif


    #ifdef TINO_MAX31865_H
	// power off the MAX31865
    RTD_Sleep(TiNo->use.MAX31865);
    #endif

    #ifdef TINO_MAX31855_H
    // Power needs to be attached as long as we need to use SPI
    // so only here we can turn power of the mAX31855 off
    Thermocouple_Sleep_55(TiNo->use.MAX31855);
    #endif

    #ifdef TINO_MAX31856_H
    ThermoCouple_Sleep_56(TiNo->use.MAX31856);
    #endif

    #ifdef TINO_MAX6675_H
    Thermocouple_Sleep_6675(TiNo->use.MAX6675);
    #endif

    // flush serial TX buffer, preparing sleep
    mySerial->flush();


    // blink
    // Config.LedCount == 0 : never blink
    // Config.LedCount == 255: always blink
    // Config.LedCount = blink x rounds (typical x=3 is good for testing)
    //
    if (Config.LedPin && Config.LedCount >0)
    {
        if (Config.LedCount != 0xff) Config.LedCount--;
        blink(Config.LedPin,2); // blink LED
    }

    SPI_MISO_Disable(RADIO_SPI_PINSWAP);

    #ifdef TINO_MAX31865_H
    pinMode(MISO, INPUT);
    #endif

    #ifdef TINO_MAX31855_H
    pinMode(MISO, INPUT);
    #endif

    #ifdef TINO_MAX31856_H
    pinMode(MISO, INPUT);
    #endif

    #ifdef TINO_MAX6675_H
    pinMode(MISO, INPUT);
    #endif

}
#endif // end IS_SENSOR_NODE

#if IS_SENSOR_NODE
//dummy function, never used in sensor sketch
void loop()
{

}
#endif


uint8_t print_packet0(uint8_t* payload, uint8_t payload_len)
{
	if (payload[NODEID]==0) return 0;

	// find out which protocol format is used
    if (!(payload[FLAGS] & 0x60)) // bit 5 and bit 6 in Flags are 0, flags is x00x xxxx
	{
		// this is the default payload structure
		Payload *pl = (Payload*) payload;
		mySerial->print("v=");  mySerial->print(pl->supplyV);
        mySerial->print("&c=");  mySerial->print(pl->count);
        mySerial->print("&t=");  mySerial->print((pl->temp - 1000)*4);
        mySerial->print("&h=");  mySerial->print(int(pl->humidity/2.0*100));
		mySerial->printf("&f=%x", pl->flags & 0x1f);
        //mySerial->print("&f=");  mySerial->print(pl->flags & 0x1f,HEX);
		if (payload_len >=12)
		{
			mySerial->print("&p=");  mySerial->print(pl->pressure);
			mySerial->print("&br=");  mySerial->print(pl->brightness);
		}

		extract_interrupts(pl->flags); // ??
		mySerial->flush();
	}
	else
		return 0;
	return 1;
}

uint8_t print_ack_packet(uint8_t* payload)
{
	if((payload[FLAGS] >> 5) == 0x2) // TiNo ACK Packet: 010x xxxx
	{
		PayloadAck *pl = (PayloadAck*)payload;
		mySerial->print("&f=");  mySerial->print(pl->flags & 0x1f,HEX);
		mySerial->print("&c=");  mySerial->print(pl->count);
		mySerial->print("&t=");  mySerial->print(pl->temp);
	}
	else
		return 0;
	return 1;
}

uint8_t print_alterate_packets(uint8_t* payload)
{
	uint8_t success = 1;
	if (payload[FLAGS] & 0x20) //alternative packet type, flag is xx1x xxxx
	{
		switch (payload[ALT_PACKET_TYPE])
		{
			case 1:
				// string packet with length 16/8 (so we've got 13/5 bytes effective)
				Config.FecEnable ? payload[8] = 0 : payload[16] = 0;
				mySerial->print((char*)(payload+ALT_PACKET_TYPE+1)); mySerial->print(";");
				break;
			case 2:
				// string packet with length 24 (so we've got 21 bytes effective)
				payload[24] = 0;
				mySerial->print((char*)(payload+ALT_PACKET_TYPE+1)); mySerial->print(";");
				break;
			case 3:
				// BME280 with temperature, humidity and pressure data
				{
					PacketType3 *pl = (PacketType3*) payload;
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
					PacketType4 *pl = (PacketType4*) payload;
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
					PacketType5 *pl = (PacketType5*) payload;
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
					PacketType6 *pl = (PacketType6*) payload;
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
				success = 0;
				break;
        }
	}
	else
	{
		success = 0;
	}

	return success;
}


#if IS_RECEIVER
void loop()
{
    //static uint16_t count=0;
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

			if (print_packet0(Mac.rxpacket.payload, Mac.rxpacket.datalen)) // standard payload, len=8
			{
				//packet_is_valid = true;
			}
			else if (print_ack_packet(Mac.rxpacket.payload))
			{}
            else if (print_alterate_packets(Mac.rxpacket.payload))  // it is another packet type, flag is xx1x xxxx
			{}
			else
				packet_is_valid = false;

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
             // Errorcodes:
                 //-1:  could not decode FEC data (too many bit errors in codes)
                 //-2:  data length does not match
                 //-3:  not my message or address corrupted
             //
            //
            //char errors[3];
            //errors[0] = "-1: could not decode FEC data (too many bit errors in codes)";
            //errors[1] = "-2: data length does not match";
            //errors[2] = "-3: not my message or address corrupted";
            //int8_t code = -(Mac.rxpacket.errorcode+1);

            //mySerial->print("Error Code: "); mySerial->print(errors[code]);
            //
        }
    }

    // ***  Frequency hopping ***
    #if NUM_CHANNELS > 1
    if(radio.noRx())
    {
        frec_counter++;
        if (frec_counter >= NUM_CHANNELS) frec_counter=0;
        radio.switch_frequencyMHz(frec[frec_counter]);
        delayMicroseconds(100); // wait for the synthesizer to settle.
    }
    #endif


    if (tick)
    {
        tick = false;
		Measure();
		TiNo->nodeid(Config.Nodeid);
		TiNo->flags(1);
		TiNo->increment_count();

		/*
		print_parameter(TiNo->pData[NODEID]);
		print_parameter(TiNo->PacketLen);
		print_hexparam (TiNo->pData[FLAGS]);
		print_parameter(TiNo->PacketType);
		*/

		mySerial->print(TiNo->pData[NODEID]); mySerial->print(" ");
		if (print_packet0(TiNo->pData, TiNo->PacketLen)) // standard payload, len=8
		{}
		else if (print_ack_packet(TiNo->pData))
		{}
		else if (print_alterate_packets(TiNo->pData))  // it is another packet type, flag is xx1x xxxx
		{}
		mySerial->println();
    }

    if (event_triggered) // a local Port change interrupt occured.
    {
        doaction(Config.Nodeid, event_triggered, actions, num_actions, &PIT);
        event_triggered =0;
    }
}

#endif