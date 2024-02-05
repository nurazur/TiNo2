#include "sht_sensors.h"

uint8_t SHT_Init(uint8_t enable, SHTSensor* &SHT, SHTSensor::SHTSensorType Type)
{
    uint8_t success = 0;
    if (enable)
    {
        SHT = new SHTSensor(Type);
        if (SHT->init())
        {
            success= enable;
        }
        else
        {
            delete SHT;
            SHT=NULL;
        }
    }
    return success;
}

uint8_t SHT_Measure(uint8_t enabled, SHTSensor *SHT, HumiditySensor &Data)
{
    uint8_t success=0;
    if (enabled && SHT)
    {
        I2C_pullup(Data.PowerPin);
        delay(1);

        if(SHT->init()) // reads T and rH after initialization / wakeup
        {
            Data.temperature = SHT->getTemperature();
            Data.humidity    = SHT->getHumidity();
            success =  enabled;
        }

        I2C_shutdown(Data.PowerPin);
    }
    return success;
}


