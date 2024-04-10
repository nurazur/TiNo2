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
#include "Arduino.h"
#include "configuration.h"

// Serial Number consists of 10 Bytes.
// The first 6 Bytes are printable characters.
// the last 4 Bystes are the actual serial number.
typedef struct
{
    char prefix[7];
    union
    {
        uint8_t sn_char[4];
        uint32_t sn;
    };
} SerialNumber;
void print_serial_number(HardwareSerial* S);
void print_humidity_sensor_values(const char* sensorname, float t, float h, HardwareSerial* S = &Serial);
void print_flag(uint8_t flag, Stream* S);
void print_init_result(bool result_pass, const char* sensor_str, HardwareSerial* S = &Serial);
void print_eeprom(Configuration& Config, Stream *serial);

