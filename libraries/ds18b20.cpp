#include "Arduino.h"
#include "ds18b20.h"



OneWire *oneWire=NULL;
DallasTemperature *ds18b20=NULL;


// One-Wire DS18B20 start-up sequence
uint8_t DS18B20_Start(DallasTemperature *sensor, byte PowerPin)
{
    if (PowerPin <=25)
    {
        pinMode(PowerPin, OUTPUT); // set power pin for DS18B20 to output
        digitalWrite(PowerPin, HIGH); // turn DS18B20 sensor on
        delay(10); // Allow 10ms for the sensor to be ready
        sensor->begin();
        //sensor->setResolution(10); //Resolutiuon is 0.125 deg, absolutely sufficient!
        delay(10); // Allow 10ms for the sensor to be ready
        return sensor->getDeviceCount();
    }
    else
        return 0;
}


void DS18B20_Stop(byte PowerPin)
{
    digitalWrite(PowerPin, LOW); // turn Sensor off to save power
}


uint8_t DS18B20_Init(uint8_t enable, uint8_t PowerPin, uint8_t DataPin)
{
    uint8_t num_devices =0;
    if(enable)
    {
        //--------------------------------------------------------------------------
        // test if 1-wire devices are present
        //--------------------------------------------------------------------------
        pinMode(PowerPin, OUTPUT); // set power pin for DS18B20 to output
        digitalWrite(PowerPin, HIGH); // turn DS18B20 sensor on
        delay(10); // Allow 10ms for the sensor to be ready

        // enable the data pin as input (at power up, we disable all GPIO's for power saving porposes)
        pinConfigure(DataPin, PIN_DIR_INPUT, PIN_PULLUP_OFF, PIN_INPUT_ENABLE);

        // Start up the library
        oneWire= new OneWire(DataPin);

        ds18b20 = new DallasTemperature(oneWire);

        num_devices = DS18B20_Start(ds18b20, PowerPin);
        if (num_devices == 0)
        {
            delete ds18b20;
            ds18b20 = NULL;
            DS18B20_Stop(PowerPin);
        }
    }
    return num_devices; // this allows to adjust packet type at initialization. if only one device, packet type= 0;
}


uint8_t DS18B20_Measure(bool enable, float *temp, uint8_t PowerPin)
{
    //float temperature=-40;
    uint8_t num_devices =0;
    if (enable && ds18b20)
    {
        num_devices = DS18B20_Start(ds18b20, PowerPin);

        if (num_devices > 0)
        {
            ds18b20->requestTemperatures();
            switch (num_devices)
            {
                case 3:
                    temp[2] = ds18b20->getTempCByIndex(2);
                     __attribute__ ((fallthrough));
                case 2:
                    temp[1] = ds18b20->getTempCByIndex(1);
                     __attribute__ ((fallthrough));
                case 1:
                    temp[0] = ds18b20->getTempCByIndex(0);
                    break;
            }

            DS18B20_Stop(PowerPin); // Turn off power Pin for DS18B20
        }
    }
    return num_devices;
}
