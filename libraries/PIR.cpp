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

#include <Arduino.h>
#include "configuration.h"
#include "Pinchangeinterrupt.h"
#include "PIR.h"
PIRModule::PIRModule(Configuration& C, Stream* S, void (*U)(void)) :  serial(S), Cfg(C), isr(U)
{
    pir_counter=0;
    pir_is_off=true;
    pir_enabled =false;
}


// call this function in setup()
// the function enables the PIR functionality and turns the PIR off for 3 wake-up periods (usually one period is 8s)
//
void PIRModule::init(void)
{
    pir_counter = Cfg.PirDeadTime -3;
    pir_is_off = true;
    pir_enabled = (Cfg.PirDataPin>0) ? true : false;
    if (pir_enabled)
    {
        pinMode(Cfg.PirDataPin, INPUT_PULLUP); // setup for testing with a switch button
        //pinMode(Cfg.PirDataPin, INPUT); // PIR will pull down the pin when armed
        detachInterrupt(Cfg.PirDataPin);
        serial->println("PIR initialized.");
    }
}

// activate the PIR
void PIRModule::registerDevice(void)
{
    pir_is_off = false;
    pir_counter = 0;
    if (pir_enabled)
    {
        register_pci(1, Cfg.PirDataPin, isr, RISING);

    }
}

// deactivate the PIR
void PIRModule::unregister(void)
{
    if (pir_enabled)
    {
        detachInterrupt(Cfg.PirDataPin);
    }
}


// test if the PIR has been triggered.
// PIR funtionality must be activated.
// in case the PIR has been triggered, it is turned off for PirDeadTime wakeup cycles
void PIRModule::triggered(uint8_t event)
{
    if (pir_enabled && !pir_is_off && (event&0x2))
    {
        pir_counter = 0;
        pir_is_off = true;
        this->unregister();
        serial->println("PIR");
    }
}


// check if the dead time has expired.
// in case it has expired, activate the PIR
void PIRModule::check_deadtime(void)
{
    if (pir_enabled && pir_is_off)
    {
        pir_counter++;
        if (pir_counter > Cfg.PirDeadTime) // re-enable PIR
        {
            this->registerDevice(); // resets counter
            serial->println("PIR active."); serial->flush();
        }
    }
}
