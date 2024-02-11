#include <Arduino.h>
#include "SHTSensor.h"

#ifndef I2C_COMMON_H
#define I2C_COMMON_H
typedef struct {
    int8_t PowerPin;
    float temperature;
    float humidity;
    float pressure;
}
HumiditySensor;


void I2C_shutdown(int8_t PowerPin);
void I2C_pullup  (int8_t PowerPin);
#endif