#include "tino_max6675.h"
#include <stdlib.h>
#include <SPI.h>


TC_6675 *TC6675=NULL;


uint8_t TC_6675::Measure(uint8_t enable, float* temperature)
{
    uint8_t status=0xff;
    if (enable)
    {
        digitalWrite(this->power_pin, 1);
        this->begin();
        delay(220);
        status = this->read();
        pinMode(this->cs_pin, INPUT_PULLUP);
        *temperature = this->getTemperature();
    }
    return status;
}


/*
uint8_t TC_6675::Measure(uint8_t enable, float* temperature)
{
    this->StartMeasure(enable);
    delay(220);
    return this->Read(enable, temperature);
}
*/

void TC_6675::StartMeasure(uint8_t enable)
{
    if (enable)
    {
        digitalWrite(this->power_pin, 1);
        this->begin();
    }
}

uint8_t TC_6675::Read(uint8_t enable, float* temperature)
{
    uint8_t status=0xff;
    if (enable)
    {
        //digitalWrite(this->power_pin, 1);
        //this->begin();
        //delay(220);
        status = this->read();
        pinMode(this->cs_pin, INPUT_PULLUP);
        digitalWrite(this->power_pin, 1);
        *temperature = this->getTemperature();
    }
    return status;
}



void TC_6675::Sleep(void)
{
    digitalWrite(this->power_pin, 0);
    pinMode(this->cs_pin, INPUT);
}



void ThermoCouple_Init_6675(uint8_t enable, uint8_t PowerPin, uint8_t CsPin)
{
    if (enable)
    {
        pinMode(CsPin, OUTPUT);  // SPI SS
        digitalWrite(CsPin, 1);
        TC6675 = new TC_6675(PowerPin, CsPin);
        
    }
}

void Thermocouple_StartMeasure6675(uint8_t enable)
{
    TC6675->StartMeasure(enable);
}


uint8_t Thermocouple_Read6675(uint8_t enable, float* temp)
{
    return TC6675->Read(enable, temp);
}


uint8_t Thermocouple_Measure_6675(uint8_t enable, float* temps)
{
    if(TC6675 && enable) 
        return TC6675->Measure(enable, temps);
    else
        return 0xff;
}

void Thermocouple_Sleep_6675(uint8_t enable)
{
    if (TC6675 && enable)
    {
        TC6675->Sleep();
    }
}
