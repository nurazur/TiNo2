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
#include "sht_sensors.h"

uint8_t SHT_Init(uint8_t enable, SHTSensor* &SHT, SHTSensor::SHTSensorType Type)
{
    uint8_t success = 0;
    if (enable)
    {
        SHT = new SHTSensor(Type);
        if (SHT->init())
        {
            success= enable;
        }
        else
        {
            delete SHT;
            SHT=NULL;
        }
    }
    return success;
}

uint8_t SHT_Measure(uint8_t enabled, SHTSensor *SHT, HumiditySensor &Data)
{
    uint8_t success=0;
    if (enabled && SHT)
    {
        I2C_pullup(Data.PowerPin);
        delay(1);

        if(SHT->init()) // reads T and rH after initialization / wakeup
        {
            Data.temperature = SHT->getTemperature();
            Data.humidity    = SHT->getHumidity();
            success =  enabled;
        }

        I2C_shutdown(Data.PowerPin);
    }
    return success;
}


