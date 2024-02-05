

#include "PacketHandler.h"



int8_t PacketHandler::calc_packettype(void)
{
    // prefer SHTC3 over SHT3x and HTU21D in case both are present.
    // currently there is no Packet type for 2 humidity sensors.

    if(use.SHTC3)
    {
        use.HTU21D=0;
        use.SHT3X=0;
        use.SHT4X=0;
    }
    else if (use.SHT4X)
    {
        use.HTU21D=0;
        use.SHT3X=0;
    }
    else if (use.SHT3X)
    {
        use.HTU21D=0;
    }

    /*
    // no packet type available for 2 humidity sensors. BME280 preferred.
    if (use.BME280 && (use.HTU21D || use.SHT3X || use.SHTC3 || use.SHT4X))
    {
        PacketType = 0;
        use.HTU21D =0;
        use.SHTC3 =0;
        use.SHT3X =0; // don't use the humidity sensors in this case!
        use.SHT4X =0;
    }
    */
    this->extra_temperature_sensor = use.DS18B20 || use.MAX31865;
    this->is_humidity_sensor = use.HTU21D | use.BME280 | use.SHT3X | use.SHTC3 | use.SHT4X;

    PacketType = -1;
    if (use.BME280) // has pressure, only Packet type 0 supports it.
    {
        PacketType = 0;
        if (use.HTU21D || use.SHT3X || use.SHTC3 || use.SHT4X)
        {
            use.HTU21D =0;
            use.SHTC3 =0;
            use.SHT3X =0; // don't use the humidity sensors in this case!
            use.SHT4X =0;
        }
    }
    else //no pressure measurement -> packet 0,4, or 5
    {
        if (is_humidity_sensor)
        {
            if (extra_temperature_sensor)  // no pressure, has humidity, has extra temp sensor
            {
                PacketType = 5;
            }
            else                           // no pressure, has humidity, no additional temp sensor
            {
                PacketType = 0;
            }
        }
        else // no pressure, no humidity
        {
            if (extra_temperature_sensor)   //can be DS18B20 and/or MAX31865
            {
                if (use.DS18B20)
                {
                    PacketType = 4;
                    use.BRIGHTNESS = 0; // packet type 4 cannot have brightness
                }
                else //MAX31865 only
                {
                    if (use.BRIGHTNESS)
                        PacketType = 5; //has brightness
                    else
                        PacketType = 0; // no brightness, only one temp sensor.
                }
            }
            else               // no explicit temperature sensor on board, i.e for Interrupts only like a remote control -> measure RFM temp sensor
                PacketType = 0;
        }
    }



    switch (PacketType)
    {
        case 0:
            pData = (uint8_t*) &t0;
            if (!use.BME280  && !use.BRIGHTNESS){
                PacketLen = 8;
            }
            else{
                PacketLen = sizeof(Payload);
            }
            break;
        case 4:
            pData  = (uint8_t*) &t4;
            PacketLen = sizeof(PacketType4);
            t4.packet_type=4;
            use.BRIGHTNESS=0; // brightness not possible with packet type 4
            break;
        case 5:
            pData= (uint8_t*) &t5;
            t5.packet_type=5;
            PacketLen = sizeof(PacketType5);
            break;
        default:
            break;
    }

    return PacketType;
}

void PacketHandler::humidity(uint8_t h)
{
    switch(this->PacketType)
    {
        case 0:
            t0.humidity = h;
            break;
        case 5:
            t5.humidity = h;
            break;
        default:
            break;
    }
}

void PacketHandler::pressure(uint32_t p)
{
    if (this->PacketType == 0)
    {
        t0.pressure = p;
    }
}


void PacketHandler::flags   (uint8_t f)
{
    pData[2] = f;
    if (this->PacketType !=0)
        pData[2] |= 0x20;
}

uint8_t PacketHandler::flags(void)
{
    return pData[2];
}

void PacketHandler::count(uint8_t c)
{
    switch(this->PacketType)
    {
        case 0:
            t0.count = c;
            break;
        case 4:
        case 5:
            pData[3] = c;
            break;
        default:
            break;
    }
}

uint8_t PacketHandler::increment_count(void)
{
    uint8_t c =0;
    switch(this->PacketType)
    {
        case 0:
            t0.count++;
            c = t0.count;
            break;
        case 4:
            t4.count++;
            if (t4.count == 0) t4.count_msb++;
            c = t4.count;
            break;
        case 5:
            t5.count++;
            c = t5.count;
            break;
        default:
            break;
    }
    return c;
}

void PacketHandler::supplyV(uint16_t v)
{
    switch(this->PacketType)
    {
        case 0:
            t0.supplyV = v;
            break;
        case 4:
            t4.supplyV = v;
            break;
        case 5:
            t5.supplyV = v;
            break;
        default:
            break;
    }
}

void PacketHandler::temp(uint16_t t)
{
    switch(this->PacketType)
    {
        case 0:
            t0.temp = t;
            break;
        case 4:
            t4.temp = t;
            break;
        case 5:
            t5.temp = t;
            break;
        default:
            break;
    }
}

void PacketHandler::temp1(uint16_t t)
{
    switch(this->PacketType)
    {
        case 4:
            t4.temp1 = t;
            break;
        case 5:
            t5.temp1 = t;
            break;
        default:
            break;
    }
}

void PacketHandler::temp2(uint16_t t)
{
    if (this->PacketType == 4)
    {
        t4.temp2 = t;
    }
}


uint8_t PacketHandler::add_temp(uint16_t t)
{
    switch (this->temp_index)
    {
        case 0:
            this->temp(t);
            this->temp_index++;
            break;
        case 1:
            this->temp1(t);
            this->temp_index++;
            break;
        case 2:
            this->temp2(t);
            break;
        default:
            break;
    }
    return this->temp_index;
}

void PacketHandler::brightness(uint16_t b)
{
    switch(this->PacketType)
    {
        case 0:
            t0.brightness = b;
            break;
        case 5:
            t5.brightness = b;
            break;
        default:
            break;
    }
}



