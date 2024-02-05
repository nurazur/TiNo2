#ifndef TINO_MAX6675_H
#define TINO_MAX6675_H

#include "Arduino.h"
#include "MAX6675.h"

void ThermoCouple_Init_6675(uint8_t enable, uint8_t PowerPin, uint8_t CsPin);
uint8_t Thermocouple_Measure_6675(uint8_t enable, float* temps);
void Thermocouple_Sleep_6675(uint8_t enable);
void Thermocouple_StartMeasure6675(uint8_t enable);
uint8_t Thermocouple_Read6675(uint8_t enable, float* temp);

/*************************************************************************/
/** Wrapper class to accommodate Power Pin **/
/*************************************************************************/
class TC_6675 : public MAX6675
{
    public:
    TC_6675(uint8_t PowerPin, uint8_t CsPin) : MAX6675(CsPin, &SPI)
    {
        cs_pin = CsPin;
        power_pin = PowerPin;
    }
    
    uint8_t Measure(uint8_t enable, float* temperature);
    uint8_t Read(uint8_t enable, float* temperature);
    void StartMeasure(uint8_t enable);
    void Sleep(void);
    
    uint8_t cs_pin;
    uint8_t power_pin;
};



#endif