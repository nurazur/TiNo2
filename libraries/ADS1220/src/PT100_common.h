// **********************************************************************************
// Arduino driver library for the MAX31865, ADS1120 and ADS1220 to be used with TiNo2 
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

#include <Arduino.h>

#ifndef SPI_COMMON_H
#define SPI_COMMON_H

/* RTD data, RTD current, and measurement reference
   voltage. The ITS-90 standard is used; other RTDs
   may have coefficients defined by the DIN 43760 or
   the U.S. Industrial (American) standard. */

#define RTD_A_ITS90         3.9080e-3
#define RTD_A_USINDUSTRIAL  3.9692e-3
#define RTD_A_DIN43760      3.9848e-3
#define RTD_B_ITS90         -5.870e-7
#define RTD_B_USINDUSTRIAL  -5.8495e-7
#define RTD_B_DIN43760      -5.8019e-7
/* RTD coefficient C is required only for temperatures
   below 0 deg. C.  The selected RTD coefficient set
   is specified below. */

#define RTD_A         RTD_A_ITS90
#define RTD_B         RTD_B_ITS90



struct PT100_Struct
{
    uint8_t enable=1;
    int32_t ADCvalue=0;
    int     ADCoffset=0;
    float   resistance=NAN;
    float   temperature=NAN;
    
};

struct PT100_ChipConfig
{
    int8_t CsPin;
    int8_t DrdyPin;
    int8_t PowerPin = -1;
    uint8_t DoSleep =0;
    int PT_type = 0; // 0= PT100, 1= PT1000
    float Rref =430.0;
    uint8_t ChipType =0; // for ADS1x20 0= 1120, 1= 1220
};

class PT100_Class
{
    public:
        virtual void Init(uint8_t enable)=0;
        virtual uint8_t Measure(PT100_Struct &Rtd, uint16_t N)=0;
        virtual uint8_t Diagnostic(void)=0;
};

#endif