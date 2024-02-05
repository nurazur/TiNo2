#include "tino_max31855.h"
#include <stdlib.h>
#include <SPI.h>


TC *TC55 =NULL; 

uint8_t TC::Measure(uint8_t enable, float* temps)
{
    uint8_t status=0xff;
    if (enable)
    {   
        pinMode(this->power_pin, OUTPUT);
        digitalWrite(this->power_pin, 1);
        delay(100); // must be 200ms in real application! see DS page 4 note 6
        this->begin();
        status = this->read();

        temps[COLD_JUNCTION_TEMPERATURE] = this->getInternal();
        temps[THERMOCOUPLE_TEMPERATURE]  = this->getTemperature();
        pinMode(this->cs_pin, INPUT_PULLUP);
        //digitalWrite(this->power_pin, 0);
        //this->Sleep();
    }
    return status;
}

void TC::Sleep(void)
{
    digitalWrite(this->power_pin, 0);
    pinMode(this->cs_pin, INPUT);
}

void ThermoCouple_Init_55(uint8_t enable, uint8_t PowerPin, uint8_t CsPin)
{
    if (enable)
    {
        pinMode(CsPin, OUTPUT);  // SPI SS
        digitalWrite(CsPin, 1);
        TC55 = new TC (PowerPin, CsPin);
    }
}


uint8_t Thermocouple_Measure_55(uint8_t enable, float* temps)
{
    if(TC55 && enable) 
        return TC55->Measure(enable, temps);
    else
        return 0xff;
}

void Thermocouple_Sleep_55(uint8_t enable)
{
    if (TC55 && enable)
        TC55->Sleep();
}