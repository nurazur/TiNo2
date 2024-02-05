#ifndef TINO_MAX31855_H
#define TINO_MAX31855_H

#include "Arduino.h"
#include "MAX31855.h"

#define COLD_JUNCTION_TEMPERATURE 0
#define THERMOCOUPLE_TEMPERATURE  1


void ThermoCouple_Init_55(uint8_t enable, uint8_t PowerPin, uint8_t CsPin);
uint8_t Thermocouple_Measure_55(uint8_t enable, float* temps);
void Thermocouple_Sleep_55(uint8_t enable);


class TC : public MAX31855
{
    public:

        uint8_t Measure(uint8_t enable, float* temps);
        void Sleep(void);

        TC(uint8_t PowerPin, uint8_t CsPin, __SPI_CLASS__ * mySPI) : MAX31855(CsPin, mySPI)
        {
            cs_pin = CsPin;
            power_pin = PowerPin;
        }


        TC(uint8_t PowerPin, uint8_t CsPin) : MAX31855(CsPin, &SPI)
        {
            cs_pin = CsPin;
            power_pin = PowerPin;
        }
        
        uint8_t cs_pin;
        uint8_t power_pin;
};

#endif