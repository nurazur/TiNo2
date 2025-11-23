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
#ifndef TINO_ADS1220_H
#define TINO_ADS1220_H

#include "Arduino.h"
#include "PT100_common.h"
#include "ADS1220.h"

#define ADS1220_FULL_SCALE  (((long int)1<<23)-1)
#define ADS1120_FULL_SCALE  32767

#define USE_ADS1220_ADC 1
#define USE_ADS1120_ADC 0

#define COLD_JUNCTION_TEMPERATURE 0
#define THERMOCOUPLE_TEMPERATURE  1

/* RTD data, RTD current, and measurement reference
   voltage. The ITS-90 standard is used; other RTDs
   may have coefficients defined by the DIN 43760 or
   the U.S. Industrial (American) standard. */
/*
#define RTD_A_ITS90         3.9080e-3
#define RTD_A_USINDUSTRIAL  3.9692e-3
#define RTD_A_DIN43760      3.9848e-3
#define RTD_B_ITS90         -5.870e-7
#define RTD_B_USINDUSTRIAL  -5.8495e-7
#define RTD_B_DIN43760      -5.8019e-7
*/
/* RTD coefficient C is required only for temperatures
   below 0 deg. C.  The selected RTD coefficient set
   is specified below. */
/*
#define RTD_A         RTD_A_ITS90
#define RTD_B         RTD_B_ITS90
*/

#define RTD_GAIN 4
#define MULTIPLEX_A0A1 0
#define MULTIPLEX_A1A0 6
//#define RREF 430.0
#define RREF 1500.0


/******************************************************************************/
/*****  Wrapper funktions to hide TC_1120 Class and to simplify its use  ******/
/******************************************************************************/
uint8_t ThermoCouple_Init_ADS1120(uint8_t enable, uint8_t DrdyPin, uint8_t CsPin, uint8_t DoSleep=0, uint8_t ads_type=USE_ADS1120_ADC);
void Thermocouple_Measure_ADS1120(uint8_t enable, float* temps);
void Thermocouple_Sleep_ADS1120(uint8_t enable);


/******************************************************************************/
/*****  Wrapper funktions to hide RTD1120 Class and to simplify its use  ******/
/******************************************************************************/
/*
struct PT100_Struct
{
    uint8_t enable=1;
    int32_t ADCvalue=0;
    int     ADCoffset=0;
    float   resistance=NAN;
    float   temperature=NAN;
    
};
*/
/*
struct PT100_ChipConfig
{
    int8_t CsPin;
    int8_t DrdyPin;
    int8_t PowerPin = -1;
    uint8_t DoSleep =0;
    int PT_type =0; // 0= PT100, 1= PT1000
    float Rref = 1500.0;
    uint8_t ChipType =USE_ADS1120_ADC; // 0 = for ADS1120, 1= ADS1220
};
*/

/*
void RTD_Init(uint8_t enable, uint8_t DrdyPin, uint8_t CsPin, uint8_t DoSleep=0, float Rref= RREF, uint8_t ads_type=USE_ADS1120_ADC);

void RTD_Init(uint8_t enable, PT100_ChipConfig Conf);

uint8_t RTD_Measure(PT100_Struct &RTD, uint16_t N=1);
uint8_t RTD_Diagnostic(void);
uint8_t PT100_OffsetCalibration(PT100_Struct &RTD);
 */
/*************************************************************************/
/** classes for Thermocouple and for RTD applications                   **/
/*************************************************************************/

class TC_1120 : public ADS1220
{
    public:
    TC_1120(uint8_t DrdyPin, uint8_t CsPin, uint8_t DoSleep=0, uint8_t is_ads1220 = USE_ADS1120_ADC);
    uint8_t do_sleep;
    void Measure(uint8_t enable, float* temperature);
    void Sleep(void);
};



class RTD_1120 : public ADS1220, public PT100_Class
{
    public:
    RTD_1120(uint8_t DrdyPin, uint8_t CsPin, uint8_t DoSleep=0, float R_ref=RREF, uint8_t is_ads1220 = USE_ADS1120_ADC);
    uint8_t do_sleep;
    float Rref;
    
    int32_t VrefMonitor(uint8_t enable);
    uint8_t WireBreakDetection_4WireRtd(void);
    float   Temperature(float resistance);
    uint8_t OffsetCalibration(PT100_Struct &RTD);
    
    void Sleep(void);
    
    void Init(uint8_t enable);
    uint8_t Measure(PT100_Struct &RTD, uint16_t N=1);
    uint8_t Diagnostic(void);
};

#endif