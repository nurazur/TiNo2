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

#ifndef ANALOG_H
#define ANALOG_H
/*
Issue #497 in DxCore
When using the bootloader, the MVIO Fuse is evaluated as:
#define IS_MVIO_ENABLED() ((FUSE.SYSCFG1 & 0x01) == 0)
However the bits defining MVIO in FUSE.SYSCFG1 are bits 3 and 4, where bit 4 is 0 in case MVIO is set for dual supply configuration, and 1 when set to single supply configuration.
so the correct MVIO evaluation macro is this:
#define IS_MVIO_ENABLED() ((FUSE.SYSCFG1 & 0x10) == 0)

Tested for AVR64DD28 and AVR64DD32. The error is also in AVR64DD20 and AVR64DD14 pin definitions, but I can't test it because I do not have such parts available.
*/

// Workaround for error in pins_arduino.h
// Error: #define IS_MVIO_ENABLED() ((FUSE.SYSCFG1 & 0x10) == 0)
// FUSE_MVSYSCFG_gm == 0x18
// MVSYSCFG_DUAL_gc == (0x01<<3)  == 0x10
// see C:\Program Files (x86)\Arduino\hardware\tools\avr\avr\include\avr
//
#if defined (ARDUINO_avrdd)

#ifdef  IS_MVIO_ENABLED
#undef  IS_MVIO_ENABLED
#define IS_MVIO_ENABLED() ((FUSE.SYSCFG1 & FUSE_MVSYSCFG_gm) == MVSYSCFG_DUAL_gc)
#endif

#endif

// Workaround for error in pins_arduino.h
// MegacoreX calculates correctly from a Arduino Pin to a valid Analog input.
// However many Arduino pins that are not actually analog pins output some Analog Inputs which is an error.
// In these cases NOT_A_PIN should be returned.
#if defined MEGACOREX_DEFAULT_32PIN_PINOUT
    #if defined digitalPinToAnalogInput
    #undef digitalPinToAnalogInput
    #define digitalPinToAnalogInput(p) (p<12) ? (NOT_A_PIN) : (p<20) ? (p-12) : (p<26 && p>21) ? (p-10) : (NOT_A_PIN)
    #endif
#endif


int analogReadInternalRef(uint8_t mode);
long readVcc();
float getVcc(long vref);
float readMcuTemperaure(void);
bool LDR_Init(int8_t pin);
uint8_t LDR_Measure(bool enabled, int8_t LDRPin, uint16_t& brightness);

#endif // ANALOG_H