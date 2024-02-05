/**********************************************************************/
/**********************         PIT class       ***********************/
/**********************************************************************/
/* this class is intended to be used by the receiver.
** When actions are handled by the receiver, and a action requests a Pulse to be run, we enable the PIT.
** The ISR counts until the programmed time is over and then disables the PIT. During the time the PIT is run, the receiver works as normal.
***/

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

#ifndef PITCTRL_H
#define PITCTRL_H


// select external 32768Hz clock or internal 1024 Hz clock
#define USE_OSC32K 1
#define USE_ULP32K 0
#define PITCLOCKGEN USE_ULP32K


class PITControl
{
    public:
        PITControl(void){}

        void init(uint8_t ClockGen = PITCLOCKGEN, uint8_t receiver =1); // setup the clock generator, enable interrupt, but do not start the PIT
        void enable(); // starts the PIT
        void disable(); // stops the pIT
        void interrupthandler(); // handles PIT interrupts
        void start(uint8_t pin, uint8_t default_val, uint16_t duration);


        uint8_t  pulse_port=0;
        uint16_t pulse_duration=2;
        uint8_t  pulse_port_default=0; // normally on or off
        uint32_t t_start; // for debug purposes only!

    //protected:

    void RTC_ULP32k_init(void);
    void RTC_setCrystal();
}; // end of PITControl

#endif // PITCTRL_H