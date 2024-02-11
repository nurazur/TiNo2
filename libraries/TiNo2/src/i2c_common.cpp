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

