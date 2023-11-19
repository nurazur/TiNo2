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

#include "Arduino.h"
#include "pitctrl.h"
void PITControl::init(uint8_t ClockGen, uint8_t receiver)
{
    //(void) receiver;
    //C:\Program Files (x86)\Arduino\hardware\tools\avr\avr\include\avr\iom4808.h
    //RTC.PITINTCTRL = RTC_PI_bm | RTC_PITEN_bm;

    RTC.PITINTCTRL = RTC_PI_bm;
    if (ClockGen == USE_ULP32K) // internal 1024Hz Clock
    {
        if (receiver) RTC.PITCTRLA = RTC_PERIOD_CYC128_gc; // 1024 Hz / 128 = 8Hz (period 125ms)
        else          RTC.PITCTRLA = RTC_PERIOD_CYC8192_gc; // 1024 Hz/ 8192 = 1/8hz (period 8s)
        while(RTC.PITSTATUS);
        this->RTC_ULP32k_init();
    }
    else
    {   // crystal runs on 32768 Hz
        if (receiver)
            RTC.PITCTRLA = RTC_PERIOD_CYC4096_gc; // 32768 / 4096 = 8 Hz (period 1/8 s  = 125 ms)
        else
            RTC.PITCTRLA = RTC_PERIOD_CYC32768_gc; // 32768 / 32768 = 1Hz (period 1s, maximum)

        while(RTC.PITSTATUS);
        this->RTC_setCrystal();
    }
}

    void PITControl::enable(void)
    {
        RTC.PITCTRLA   |= RTC_PITEN_bm;
        while(RTC.PITSTATUS);
    }


    void PITControl::disable()
    {
        RTC.PITCTRLA   &= ~RTC_PITEN_bm;
        while(RTC.PITSTATUS);
    }

    void PITControl::interrupthandler(void)
    {
        RTC.PITINTFLAGS = RTC_PI_bm;              // clear interrupt flag
        this->pulse_duration--;
        if (this->pulse_duration == 0)
        {
            this->disable();
            if(this->pulse_port >0)
            {
                //pulse_port =0; // brauchts des?
                digitalWrite(this->pulse_port, this->pulse_port_default);
                //Serial.println(millis()-t_start); // for debug only
            }
        }
    }


    void PITControl::RTC_ULP32k_init(void)
    {
        RTC.CLKSEL = RTC_CLKSEL_INT1K_gc;         // Internal 1024 Hz OSC
    }

    void PITControl::RTC_setCrystal()
    {
        uint8_t temp;

        // disable oscillator first
        temp = CLKCTRL.XOSC32KCTRLA;
        temp &= ~CLKCTRL_ENABLE_bm;
        CPU_CCP = CCP_IOREG_gc;
        CLKCTRL.XOSC32KCTRLA = temp;

        // wait for status bit to become '0'
        while(CLKCTRL.MCLKSTATUS & CLKCTRL_XOSC32KS_bm);

        // select external crystal for RTC
        temp = CLKCTRL.XOSC32KCTRLA;
        temp &= ~CLKCTRL_SEL_bm;
        CPU_CCP = CCP_IOREG_gc;
        CLKCTRL.XOSC32KCTRLA = temp;

        // enable the oscillator
        temp = CLKCTRL.XOSC32KCTRLA;
        temp |= CLKCTRL_ENABLE_bm | CLKCTRL_RUNSTDBY_bm;
        #if defined ARDUINO_avrdd
        temp |= CLKCTRL_LPMODE_bm;
        #endif
        CPU_CCP = CCP_IOREG_gc;
        CLKCTRL.XOSC32KCTRLA = temp;


        // wait for all registers to be synchronized
        while (RTC.STATUS > 0);

        // select 32.768 kHz external crystal oscillator
        #if defined (MEGACOREX)
        RTC.CLKSEL = RTC_CLKSEL_TOSC32K_gc;
        #elif defined (ARDUINO_avrdd)
        //RTC.CLKSEL = RTC_CLKSEL_XTAL32K_gc; // strange compiler error
        RTC.CLKSEL = 0x02;
        #endif
    }


    void PITControl::start(uint8_t pin, uint8_t default_val, uint16_t duration)
    {
        this->pulse_duration = duration; // count number
        this->pulse_port_default = default_val;
        this->t_start= millis(); // debug only
        this->pulse_port = pin;
        this->enable();
    }