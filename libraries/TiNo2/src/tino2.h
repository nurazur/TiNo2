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

#include "datalinklayer.h"
#include "PacketHandler.h"
#include "PinchangeInterrupt.h"
#include "print_things.h"
#include "pitctrl.h"            // Periodic Interrupt Timer
#include "PIR.h"                // Movement detection
/*****************************************************************************/
/***   Radio Driver                                                        ***/
/*****************************************************************************/
#include "RFM69.h"
/*****************************************************************************/
/***  EEPROM Access  and device calibration / Configuration                ***/
/*****************************************************************************/
#include "configuration.h" // configuration data structures
#include "calibrate.h"     // configuration tool interface
/*****************************************************************************/
/***                  I2C Bus Tools                                        ***/
/*****************************************************************************/
#include "i2c_common.h"
/*****************************************************************************/
/***              SHT3x and SHTC3  Humidity Sensor                         ***/
/*****************************************************************************/
#include "sht_sensors.h"

/*****************************************************************************/
/***                   READ VCC / LDR                                      ***/
/*****************************************************************************/
#include "analog.h"

/*****************************************************************************/
/***                   Reveiver Actions                                    ***/
/*****************************************************************************/
#include "actions.h" // receiver only
