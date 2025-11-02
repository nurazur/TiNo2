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

#include "i2c_common.h"

extern HardwareSerial *mySerial;
extern void disablePinISC(uint8_t pin);
/*****************************************************************************/
/***                  I2C Bus Tools                                        ***/
/*****************************************************************************/

void I2C_shutdown(int8_t PowerPin)
{
    // if Pin <0 : there is no need to shut down the i2C as we assume it is permanently powered.
    if (PowerPin >= 0)
    {
        digitalWrite(PowerPin, LOW);
        #if defined (MEGACOREX)
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
        
        #elif defined (ARDUINO_avrdd) || defined (ARDUINO_avrda)
        pinMode(SDA_NOW,INPUT);
        disablePinISC(SDA_NOW);

        pinMode(SCL_NOW,INPUT);
        disablePinISC(SCL_NOW);
        #endif
    }
}


void I2C_pullup(int8_t PowerPin)
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

    #elif defined (ARDUINO_avrdd)  || defined (ARDUINO_avrda)
    pinMode(SDA_NOW,INPUT_PULLUP);
    pinMode(SCL_NOW,INPUT_PULLUP);
    #endif

    if (PowerPin >0)
    {
        digitalWrite(PowerPin, HIGH);
    }
}

