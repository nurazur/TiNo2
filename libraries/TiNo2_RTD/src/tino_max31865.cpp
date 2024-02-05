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
// under GNU General Public License v3.0

#include <Arduino.h>
#include <SPI.h>

#include <avr/sleep.h>
#include "tino_max31865.h"


TINO_MAX31865 *RTD=NULL;


uint8_t TINO_MAX31865::ReadConversionResult(uint8_t enable, float* temp)
{
    uint8_t status = 0xff;
    
    if (enable)
    {
        status = this->read_all();
        *temp = this->temperature();
    }
    digitalWrite(this->power_pin, 0);
    return status;
}

uint8_t TINO_MAX31865::StartConversion(uint8_t enable)
{
    uint8_t status = 0xff;
    
    if (enable)
    {
        // enable MISO
        pinConfigure(PIN_PA5, PIN_DIR_INPUT, PIN_PULLUP_ON, PIN_INPUT_ENABLE);
        SPI.begin();
        SPI.setDataMode(SPI_MODE1);
        pinMode(this->power_pin,OUTPUT);
        digitalWrite(this->power_pin, 1);
        delay(1);

        this->configure( VBIAS_ENABLED, CONVERSION_MODE_OFF, ONESHOT_DISABLED, USE_4WIRES, MAX31865_FAULT_DETECTION_NONE, true, true, FAULT_LOW_THRESHOLD, FAULT_HIGH_THRESHOLD );
        //Fault detection cycle.
        this->configure( VBIAS_ENABLED, CONVERSION_MODE_OFF, ONESHOT_DISABLED, MAX31865_FAULT_DETECTION_MANUAL_1 ); //0x8
        delay(1);
        this->configure( VBIAS_ENABLED, CONVERSION_MODE_OFF, ONESHOT_DISABLED, MAX31865_FAULT_DETECTION_MANUAL_2 );
        delay(1);
        status = this->fault_status();
        if (status==0 )
        {
            // Start 1 shot measure
            // V_BIAS enabled , No Auto-conversion, 1-shot enabled, No Fault detection
            this->configure( VBIAS_ENABLED, CONVERSION_MODE_OFF, ONESHOT_ENABLED, MAX31865_FAULT_DETECTION_NONE );
        }
    }
    return status;
}

//*
uint8_t TINO_MAX31865::Measure(uint8_t enable, float* temp, bool use_interrupt)
{
    uint8_t status = 0xff;
    status= this->StartConversion(enable);
    if (status == 0)
    {
        if (use_interrupt)
            sleep_cpu();
        else
            delay(70);
        
        status = this->ReadConversionResult(enable, temp);
    }
    return status;
}
//*/


/*
uint8_t TINO_MAX31865::Measure(uint8_t enable, float* temp)
{
    uint8_t status = 0xff;
    
    if (enable)
    {
        // enable MISO
        pinConfigure(PIN_PA5, PIN_DIR_INPUT, PIN_PULLUP_ON, PIN_INPUT_ENABLE);
        SPI.begin();
        SPI.setDataMode(SPI_MODE1);
        pinMode(this->power_pin,OUTPUT);
        digitalWrite(this->power_pin, 1);
        delay(1);

        this->configure( VBIAS_ENABLED, CONVERSION_MODE_OFF, ONESHOT_DISABLED, USE_4WIRES, MAX31865_FAULT_DETECTION_NONE, true, true, FAULT_LOW_THRESHOLD, FAULT_HIGH_THRESHOLD );
        //Fault detection cycle.
        this->configure( VBIAS_ENABLED, CONVERSION_MODE_OFF, ONESHOT_DISABLED, MAX31865_FAULT_DETECTION_MANUAL_1 ); //0x8
        delay(1);
        this->configure( VBIAS_ENABLED, CONVERSION_MODE_OFF, ONESHOT_DISABLED, MAX31865_FAULT_DETECTION_MANUAL_2 );
        delay(1);
        status = this->fault_status();
        if (status==0 )
        {
            // Start 1 shot measure
            // V_BIAS enabled , No Auto-conversion, 1-shot enabled, No Fault detection
            this->configure( VBIAS_ENABLED, CONVERSION_MODE_OFF, ONESHOT_ENABLED, MAX31865_FAULT_DETECTION_NONE );
            delay(70); // 66 ms max as per DS
            status = this->read_all();
            *temp = this->temperature();
        }
        digitalWrite(this->power_pin, 0);

    }
    return status;
}
*/

void RTD_Init(uint8_t enable, uint8_t PowerPin, uint8_t CsPin, MAX31865_RTD::ptd_type type, uint16_t rtd_rref)
{
    if (enable)
    {
        pinMode(CsPin, OUTPUT);  // SPI SS
        digitalWrite(CsPin, 1);
        // enable MISO
        pinConfigure(PIN_PA5, PIN_DIR_INPUT, PIN_PULLUP_ON, PIN_INPUT_ENABLE);
        
        RTD = new TINO_MAX31865 (PowerPin, CsPin, type, rtd_rref);
    }
}

uint8_t RTD_Measure(bool enable, float* temp, bool use_interrupt)
{
    if (enable && RTD)
        return RTD->Measure(enable, temp, use_interrupt);
    else
        return 0xff;

}


void RTD_Sleep(bool enable)
{
    if (enable && RTD)
        digitalWrite(RTD->power_pin, 0);
}