#ifndef DS18B20_H
#define DS18B20_H


/**********     One-Wire and DS18B20     **********/
#include <DallasTemperature.h>       // GNU Lesser General Public License v2.1 or later
#include <OneWire.h>  // license terms not clearly defined.


uint8_t DS18B20_Start(DallasTemperature *sensor, byte PowerPin);
void DS18B20_Stop(byte PowerPin);
uint8_t DS18B20_Init(uint8_t enable, uint8_t PowerPin, uint8_t DataPin);
uint8_t DS18B20_Measure(bool enable, float *temp, uint8_t PowerPin);

#endif