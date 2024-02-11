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

