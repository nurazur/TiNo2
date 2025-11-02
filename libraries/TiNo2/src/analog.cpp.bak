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
#include "analog.h"


/*****************************************************************************/
/*
     Read VCC by  measuring the 1.5V reference and taking VCC as reference voltage.
     set the reference to Vcc and the measurement to the internal 1.1V reference
*/
/*****************************************************************************/

/*****************************************************************************/
/***                   READ VCC                                            ***/
/*****************************************************************************/

#define SAMPLE_ACCUMULATION 0x5

// possible modes for ATMega4808 are: INTERNAL0V55, INTERNAL1V1, INTERNAL2V5, INTERNAL4V34, INTERNAL1V5
// possible modes for ATMega4808 are: INTERNAL0V55, INTERNAL1V1, INTERNAL2V5, INTERNAL4V34, INTERNAL1V5

int analogReadInternalRef(uint8_t mode)
{
    #if defined (MEGACOREX) && defined ADC0

    // save registers
    uint8_t vref_ctrla = VREF.CTRLA;
    uint8_t adc0_muxpos = ADC0.MUXPOS;
    uint8_t adc0_ctrlb = ADC0.CTRLB;

    // setup DACREF in AC0
    VREF.CTRLA = (VREF.CTRLA & ~(VREF_AC0REFSEL_gm)) | (mode << VREF_AC0REFSEL_gp);


    // Reference should be already set up and should be VDD
    // ADC0.CTRLC =0b x101 xxxx // 4808
    // VREF.ADC0REF = VREF_REFSEL_VDD_gc

    // set input to DACREF0
    ADC0.MUXPOS = ADC_MUXPOS_DACREF_gc;

    // set sample accumulation
    ADC0.CTRLB = SAMPLE_ACCUMULATION;

    // Start conversion
    ADC0.COMMAND = ADC_STCONV_bm;

    // Wait for result ready
    while (!(ADC0.INTFLAGS & ADC_RESRDY_bm))
    ;

    // restore registers;
    VREF.CTRLA = vref_ctrla;
    ADC0.MUXPOS = adc0_muxpos;
    ADC0.CTRLB = adc0_ctrlb;
    // Combine two bytes
    return ADC0.RES;

    #elif defined (ARDUINO_avrdd) || defined (ARDUINO_avrda)
    (void) mode;
    // set reference to VDD
    VREF.ADC0REF = VREF_REFSEL_VDD_gc;

    // set DAC0REF to 1.024V
    VREF.DAC0REF = VREF_REFSEL_1V024_gc;

    // set input to DACREF0
    ADC0.MUXPOS = ADC_MUXPOS_DACREF0_gc;

    // set bitb resolution and conversion mode (single ended)
     ADC0.CTRLA = ADC_CONVMODE_SINGLEENDED_gc | ADC_RESSEL_10BIT_gc | ADC_ENABLE_bm; //Freerun and enable
    // set sample accumulation
    ADC0.CTRLB = SAMPLE_ACCUMULATION;

    // Start conversion
    ADC0.COMMAND = ADC_STCONV_bm;

    // Wait for result ready
    while (!(ADC0.INTFLAGS & ADC_RESRDY_bm))
        ;

    // Combine two bytes
    return ADC0.RES;

    #else // no ADC or not a AVR DD or AVR 0 series MCU
        return 0;
    #endif
}


/*
     Read VCC by  measuring the internal 1.5V (4808) / 1.024V (AVR DD) reference and using VCC as ADC reference voltage.
*/
long readVcc()
{
    #if defined (__AVR_ATmega4808__)
        return (long) analogReadInternalRef(INTERNAL1V5) / (1<<SAMPLE_ACCUMULATION); // value is divided by 32
    #elif defined (ARDUINO_avrdd) || defined (ARDUINO_avrda)
       return (long) analogReadInternalRef(VREF_REFSEL_1V024_gc); // raw value, not divided by 32,
    #endif
}

float getVcc(long vref)
{
    return (float)vref / readVcc();
}


/*****************************************************************************/
/***                   READ MCU TEMPERATURE                                ***/
/*****************************************************************************/
float readMcuTemperaure(void)
{
#if defined (MEGACOREX)
    uint16_t temp_raw=0;
    uint32_t temp_l=0;
    float temp;
    int8_t sigrow_offset = SIGROW.TEMPSENSE1; // Read signed value from signature row
    uint8_t sigrow_slope = SIGROW.TEMPSENSE0; // Read unsigned value from signature row

    analogReference(INTERNAL1V1);
    ADC0.MUXPOS = ADC_MUXPOS_TEMPSENSE_gc;

    //uint8_t vref_ctrla = VREF.CTRLA;
    uint8_t adc0_sampctrl = ADC0.SAMPCTRL;
    uint8_t adc0_ctrld = ADC0.CTRLD;

    ADC0.CTRLD &= ~0xE0;
    ADC0.CTRLD |= 0x60;
    ADC0.SAMPCTRL = 0x04;


    // Start conversion
    ADC0.COMMAND = ADC_STCONV_bm;

    // Wait for result ready
    while (!(ADC0.INTFLAGS & ADC_RESRDY_bm));

    temp_raw = ADC0.RES;

    temp_l = temp_raw - sigrow_offset;
    temp_l *= sigrow_slope;
    temp_l += 0x40;
    temp_l >>= 7;
    //temp = (float)temp_l /256.0 -273;
    temp = (float)temp_l /2.0 -273;

    //restore Analog reference to VDD
    analogReference(VDD);

    // restore registers
    ADC0.SAMPCTRL = adc0_sampctrl;
    ADC0.CTRLD = adc0_ctrld;
    //#if DEBUG >0
    //mySerial->print("ADC TEMPSENS: 0x"); mySerial->println(temp_raw);
    //mySerial->print("ADC CTRLC   : 0x"); mySerial->println(ADC0.CTRLC,HEX);
    //#endif
    return temp;

#elif defined (ARDUINO_avrdd) || defined (ARDUINO_avrda)
    uint16_t temp_raw=0;
    uint32_t temp_l=0;
    float temp;

    uint8_t adc0_sampctrl = ADC0.SAMPCTRL;
    uint8_t adc0_ctrld = ADC0.CTRLD;
    uint8_t adc0_ctrlb = ADC0.CTRLB;
    #define NUM_SAMPLES ADC_SAMPNUM_ACC8_gc

    // disable ADC
    ADC0.CTRLA &= ~ADC_ENABLE_bm;

    analogReference(VREF_REFSEL_2V048_gc);

    // Sample accumulation
    ADC0.CTRLB = NUM_SAMPLES;

    ADC0.MUXPOS = ADC_MUXPOS_TEMPSENSE_gc;
    ADC0.CTRLD &= ~0xE0;
    ADC0.CTRLD |= ADC_INITDLY_DLY16_gc;
    ADC0.SAMPCTRL = 0x10; // increase of sampling time


    // set bitb resolution and conversion mode (single ended)
    ADC0.CTRLA = ADC_CONVMODE_SINGLEENDED_gc | ADC_RESSEL_12BIT_gc | ADC_ENABLE_bm; // enable

    uint16_t sigrow_offset = SIGROW.TEMPSENSE1; // Read signed value from signature row
    uint16_t sigrow_slope = SIGROW.TEMPSENSE0; // Read unsigned value from signature row

    // Start conversion
    ADC0.COMMAND = ADC_STCONV_bm;

    // Wait for result ready
    while (!(ADC0.INTFLAGS & ADC_RESRDY_bm));
    temp_raw = ADC0.RES;

    #if DEBUG > 0
    //mySerial->print("\r\nADC RES      : "); mySerial->println(temp_raw);
    #endif
    temp_l = sigrow_offset - (temp_raw>>NUM_SAMPLES);
    temp_l *= sigrow_slope;

    // resolution of the measurement is limited.
    // resolution of the measurement is slope / 4096 = approx. 900/4096 = 0.22 deg

    //#define SF 4096 // 1 deg resolution
    //#define SF 2048 // 1/2 deg resolution
    //#define SF 1024 // 1/4 deg resolution
    #define SF 512 // 1/8 deg resolution
    #define SCAL (4096/SF)


    temp_l += SF/2;
    temp_l /= SF;
    temp = (float)temp_l / SCAL - 273;

    ADC0.CTRLA &= ~ADC_ENABLE_bm; // disable ADC
    ADC0.SAMPCTRL = adc0_sampctrl;
    ADC0.CTRLD = adc0_ctrld;
    ADC0.CTRLB = adc0_ctrlb;
    return temp;
#endif
}

/*****************************************************************************/
/***      Brightness  with LDR                                             ***/
/*****************************************************************************/
// The LDR is placed between LdrPin and GND. The Pull-up of the pin is used to form a voltage divider
// the brighter the light, the smaller the LDR resistance.
// Therefore, we output (1023-ADCRES) as result, so that high values represent bright light.
// this is only a rough estimate of brightness, like dark / dawn-dusk /bright because korrelation is
// usually difficult.


bool LDR_Init(int8_t pin)
{
    // configure the pin as input (no pullup yet) in case it is a valid analog pin.
    bool enabled= false;
    if ((pin >= 0) && (digitalPinToAnalogInput(pin) != NOT_A_PIN))
    {
        pinConfigure(pin, PIN_DIR_INPUT, PIN_PULLUP_OFF, PIN_INPUT_ENABLE);
        enabled=true;
    }
    return enabled;
}


uint8_t LDR_Measure(bool enabled, int8_t LDRPin, uint16_t& brightness)
{
    uint8_t success = 0;
    brightness=0;
#if defined (MEGACOREX)
    // the pins available on 4808 (32-Pin) are 12, (13 an14 are reserved for RF Module), 15-18 (19 is LED), 22-25
    if (enabled && LDRPin >= 0)
    {
        pinMode(LDRPin, INPUT_PULLUP);
        ADC0.CTRLA |= ADC_ENABLE_bm; // enable ADC
        delay(20);
        brightness = (uint16_t)(1023 - (analogRead(LDRPin)/(1<<ADC0.CTRLB)));
        pinMode(LDRPin, INPUT);
        //ADC0.CTRLA &= ~ADC_ENABLE_bm; // disable ADC
        success++;
    }
#elif defined (ARDUINO_avrdd) || defined (ARDUINO_avrda)
    // the pins available on DD devices and TiNo2 are:
    // 8-11 if MVIO disabled, 13-18, 22-24 (DD32 devices only)
    if (enabled && LDRPin >= 2)
    {
        uint8_t adc_channel = digitalPinToAnalogInput(LDRPin);
        //mySerial->print("Brightness ADC channel: "); mySerial->println(adc_channel);

        // disable ADC (important on DD series)
        ADC0.CTRLA &= ~ADC_ENABLE_bm;

        pinMode(LDRPin, INPUT_PULLUP);

        // set reference to VDD
        analogReference(VREF_REFSEL_VDD_gc);

        /* Select channel */
        ADC0.MUXPOS = ((adc_channel & 0x7F) << ADC_MUXPOS_gp);

        // set bitb resolution and conversion mode (single ended)
        ADC0.CTRLA = ADC_CONVMODE_SINGLEENDED_gc | ADC_RESSEL_10BIT_gc | ADC_ENABLE_bm; // enable

        // LDR settling time
        delay(20);

        // Start conversion
        ADC0.COMMAND = ADC_STCONV_bm;

        // Wait for result ready
        while (!(ADC0.INTFLAGS & ADC_RESRDY_bm));

        // get result (16 bit accumulated)
        brightness = ADC0.RES / (1<<ADC0.CTRLB);
        brightness = (uint16_t) 1023 - brightness;

        pinMode(LDRPin, INPUT);
        ADC0.CTRLA &= ~ADC_ENABLE_bm; // disable ADC

        success++;
    }
#endif
    //mySerial->print("Brightness: "); mySerial->println(sensorValue);
    return success;
}

