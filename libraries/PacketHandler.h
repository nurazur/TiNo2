// **********************************************************************************
// Copyright nurazur@gmail.com
// **********************************************************************************
// License
// **********************************************************************************
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

// Licence can be viewed at
// http://www.fsf.org/licenses/gpl.txt

// Please maintain this license information along with authorship
// and copyright notices in any redistribution of this code
// *********************************************************************************

#ifndef PACKETHANDLER_H
#define PACKETHANDLER_H

#include "datalinklayer.h"

#define HTU21D_bm   (1<<0)
#define DS18B20_bm  (1<<1)
#define BME280_bm   (1<<2)
#define SHT3X_bm    (1<<3)
#define MAX31865_bm (1<<4)
#define BRIGHTNESS_bm (1<<5)
#define SHTC3_bm    (1<<6)
#define SHT4X_bm    (1<<7)

typedef struct
{
    byte HTU21D:1;
    byte DS18B20:1;
    byte BME280:1;
    byte SHT3X:1;
    byte MAX31865:1;
    byte BRIGHTNESS:1;
    byte SHTC3:1;
    byte SHT4X:1;

} UseBits;

typedef struct
{
    byte is_temperature_sensor:1;
    byte is_humidity_sensor:1;
    byte is_i2c_sensor:1;
    byte is_spi_sensor:1;
    byte is_onewirebus:1;
    byte reserved:3;
} SensorType;

class PacketHandler
{
    public:
        UseBits use;


        PacketHandler (UseBits sensor_cfg)
        {
            this->use = sensor_cfg;
            PacketLen =  0;
            this->calc_packettype();
            this->count(0);
            this->temp1(0);
            this->temp2(0);
            this->temp_index=0;
        }

        int8_t calc_packettype(void);
        void humidity(uint8_t h);
        void pressure(uint32_t p);


        void targetid(uint8_t t) {pData[0] = t;}
        void nodeid  (uint8_t n) {pData[1] = n;}
        void flags   (uint8_t f);
        uint8_t flags(void);
        void count(uint8_t c);
        uint8_t increment_count(void);
        void supplyV(uint16_t v);
        void temp(uint16_t t);
        void temp1(uint16_t t);
        void temp2(uint16_t t);
        uint8_t add_temp(uint16_t t);
        void brightness(uint16_t b);

        int8_t PacketType;
        uint8_t PacketLen;
        uint8_t extra_temperature_sensor;
        uint8_t is_humidity_sensor;
        uint8_t is_i2c_sensor;
        uint8_t temp_index;
        union
        {
            Payload t0;
            PacketType4 t4;
            PacketType5 t5;
        };

        uint8_t *pData;
};

#endif