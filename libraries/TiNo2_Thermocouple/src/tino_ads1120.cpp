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
#include "tino_ads1120.h"
#include <stdlib.h>
#include <SPI.h>

TC_1120 *tc1120 =NULL;
void DrDyPinInteruptFunc(void){}


TC_1120::TC_1120(uint8_t DrdyPin, uint8_t CsPin, uint8_t DoSleep) : ADS1120()
    {
        do_sleep = DoSleep;
        
        pinConfigure(DrdyPin, PIN_DIR_INPUT, PIN_PULLUP_OFF, PIN_INPUT_ENABLE); // configure Data ready pin (no pullup!)
        
        if (do_sleep)
        {
            attachInterrupt(DrdyPin, DrDyPinInteruptFunc, FALLING);
        }
        
        this->begin(CsPin, DrdyPin);
        this->setDataRate(0x00);        // 20 SPS
        this->setConversionMode(0);     // Single shot (default)
        this->setVoltageRef(0);         // Internal 2.048 V (default)
        this->setOpMode(0x00);          // Normal mode (default)
        this->setMultiplexer(0);        // AIN0 vs AIN1
        this->setGain(32); 
        this->setFIR(2);
        this->setTemperatureMode(1);
        this->powerDown();
    }

void TC_1120::Measure(uint8_t enable, float* temperatures)
{
    if (enable)
    {
        SPI.begin();
        SPI.setDataMode(SPI_MODE1);
        int tc_raw;
        this->setTemperatureMode(1);
        temperatures[COLD_JUNCTION_TEMPERATURE] = this->readADC_SingleTemp(this->do_sleep);
        
        this->setTemperatureMode(0);    // Disable temperature sensor
        tc_raw = (int) this->readADC_Single(this->do_sleep);
        this->powerDown();
        double Vtc  = 2.048 * 1000000 * tc_raw / 32768 / 32; // in microV --- Vref[V] * Code / 2^15 / gain * 10^6 uV
        temperatures[THERMOCOUPLE_TEMPERATURE] = Vtc/41.276 + temperatures[COLD_JUNCTION_TEMPERATURE];
    }
}


void ThermoCouple_Init_ADS1120(uint8_t enable, uint8_t DrdyPin, uint8_t CsPin, uint8_t DoSleep)
{
    if (enable)
    {
        tc1120 = new TC_1120(DrdyPin, CsPin, DoSleep);
    }
}

void Thermocouple_Measure_ADS1120(uint8_t enable, float* temps)
{
    if (enable && tc1120)
        tc1120->Measure(enable, temps);
}