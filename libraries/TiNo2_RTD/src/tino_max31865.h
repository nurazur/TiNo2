// **********************************************************************************
// Arduino driver library for the MAX31865 to be used with TiNo2 
//
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

// This library is based on and requires: 
/***  https://github.com/olewolf/arduino-max31865 */
/***  Copyright (C) 2015 Ole Wolf <wolf@blazingangles.com> */
// under GNU General Public License v3.0
// *********************************************************************************

#ifndef TINO_MAX31865_H
#define TINO_MAX31865_H

#include "Arduino.h"
#include "MAX31865.h"


#define MAX31865_SPI_PINSWAP 0

#define FAULT_HIGH_THRESHOLD  0x9304  /* +350C */
#define FAULT_LOW_THRESHOLD   0x2690  /* -100C */


void RTD_Init(uint8_t enable, uint8_t PowerPin, uint8_t CsPin, MAX31865_RTD::ptd_type type= MAX31865_RTD::RTD_PT100, uint16_t rtd_rref = 430);
uint8_t RTD_Measure(bool enable, float* temp, bool use_interrupt=false);
void RTD_Sleep(bool enable);
class TINO_MAX31865 : public MAX31865_RTD
{
    public:

    uint8_t ReadConversionResult(uint8_t enable, float* temp);
    uint8_t StartConversion(uint8_t enable);
    uint8_t Measure(uint8_t enable, float* temp, bool use_interrupt);

    TINO_MAX31865(uint8_t PowerPin, uint8_t CsPin, ptd_type type, uint16_t rtd_rref) 
    : MAX31865_RTD(type, CsPin, rtd_rref)
    {
        cs_pin = CsPin;
        power_pin = PowerPin;
    }

       
        uint8_t cs_pin;
        uint8_t power_pin;
};


#endif