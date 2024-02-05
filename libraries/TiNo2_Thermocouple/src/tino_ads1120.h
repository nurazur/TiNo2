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
#ifndef TINO_ADS1120_H
#define TINO_ADS1120_H

#include "Arduino.h"
#include "ADS1120.h"

#define COLD_JUNCTION_TEMPERATURE 0
#define THERMOCOUPLE_TEMPERATURE  1


void ThermoCouple_Init_ADS1120(uint8_t enable, uint8_t DrdyPin, uint8_t CsPin, uint8_t DoSleep=0);
void Thermocouple_Measure_ADS1120(uint8_t enable, float* temps);
void Thermocouple_Sleep_ADS1120(uint8_t enable);

/*************************************************************************/
/** Wrapper class **/
/*************************************************************************/

class TC_1120 : public ADS1120
{
    public:
    TC_1120(uint8_t DrdyPin, uint8_t CsPin, uint8_t DoSleep=0);
    uint8_t do_sleep;
    void Measure(uint8_t enable, float* temperature);
    void Sleep(void);
};

#endif