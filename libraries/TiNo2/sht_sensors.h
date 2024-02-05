#include <Arduino.h>
#include "i2c_common.h"
#include "SHTSensor.h"

#ifndef SHT_SENSORS_H
#define SHT_SENSORS_H
// SHTSensor* &SHT : this is a reference to a pointer, otherwise we would have to use a pointer to pointer.
uint8_t SHT_Init   (uint8_t enable, SHTSensor* &SHT, SHTSensor::SHTSensorType Type);

uint8_t SHT_Measure(uint8_t enabled, SHTSensor *SHT, HumiditySensor &Data);

#endif