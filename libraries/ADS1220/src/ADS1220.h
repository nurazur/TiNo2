/***
MIT License

Copyright (c) 2017 John Leeman
Copyright (c) 2020 Ryan Wagoner <rswagoner@gmail.com>
Copyright (c) 2025 nurazur <nurazur@gmail.com>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
***/


#ifndef ADS1220_h
#define ADS1220_h

#include <Arduino.h>
#include <SPI.h>

#define SPI_MASTER_DUMMY   0xFF
// Commands for the ADC
#define CMD_RESET 0x07
#define CMD_START_SYNC 0x08
#define CMD_PWRDWN 0x03
#define CMD_RDATA 0x1f
#define CMD_RREG 0x20
#define CMD_WREG 0x40

// Configuration registers
#define CONFIG_REG0_ADDRESS 0x00
#define CONFIG_REG1_ADDRESS 0x01
#define CONFIG_REG2_ADDRESS 0x02
#define CONFIG_REG3_ADDRESS 0x03

// Register masks for settings
// Register 0
#define REG_MASK_MUX 0xF0
#define REG_MASK_GAIN 0x0E
#define REG_MASK_PGA_BYPASS 0x01

// Register 1
#define REG_MASK_DATARATE 0xE0
#define REG_MASK_OP_MODE 0x18
#define REG_MASK_CONV_MODE 0x04
#define REG_MASK_TEMP_MODE 0x02
#define REG_MASK_BURNOUT_SOURCES 0x01

// Register 2
#define REG_MASK_VOLTAGE_REF 0xC0
#define REG_MASK_FIR_CONF 0x30
#define REG_MASK_PWR_SWITCH 0x08
#define REG_MASK_IDAC_CURRENT 0x07

// Register 3
#define REG_MASK_IDAC1_ROUTING 0xE0
#define REG_MASK_IDAC2_ROUTING 0x1C
#define REG_MASK_DRDY_MODE 0x02
#define REG_MASK_RESERVED 0x01
/*
multiplexer

  | Value | AINp | AINn |
  | ----- | ---- | ---- |
  | 0x00  | AIN0 | AIN1 |
  | 0X01  | AIN0 | AIN2 |
  | 0X02  | AIN0 | AIN3 |
  | 0X03  | AIN1 | AIN2 |
  | 0X04  | AIN1 | AIN3 |
  | 0X05  | AIN2 | AIN3 |
  | 0X06  | AIN1 | AIN0 |
  | 0X07  | AIN3 | AIN2 |
  | 0X08  | AIN0 | AVSS |
  | 0X09  | AIN1 | AVSS |
  | 0X0A  | AIN2 | AVSS |
  | 0X0B  | AIN3 | AVSS |
  | 0X0C  |  REF/4 MON  |
  | 0X0D  | APWR/4 MON  |
  | 0X0E  |   SHORTED   |
  
  
*/
#define MUX_AIN0_AIN1 0x00
#define MUX_AIN0_AIN2 0x01
#define MUX_AIN0_AIN3 0x02
#define MUX_AIN1_AIN2 0x03
#define MUX_AIN1_AIN3 0x04
#define MUX_AIN2_AIN3 0x05
#define MUX_AIN1_AIN0 0x06
#define MUX_AIN3_AIN2 0x07
#define MUX_AIN0_AVSS 0x08
#define MUX_AIN1_AVSS 0x09
#define MUX_AIN2_AVSS 0x0A
#define MUX_AIN3_AVSS 0x0B
#define MUX_REF_DIV4  0x0C
#define MUX_APWR_DIV4 0x0D
#define MUX_SHORTED   0x0E

/* Selects where IDACx is routed to.
     0 - Disabled
     1 - AIN0/REFP1
     2 - AIN1
     3 - AIN2
     4 - AIN3/REFN1
     5 - REFP0
     6 - REFN0
  */

#define IDAC_DISABLED 0
#define IDAC_AIN0	1
#define IDAC_AIN1	2
#define IDAC_AIN2	3
#define IDAC_AIN3	4
#define IDAC_REFP0	5
#define IDAC_REFP1	6

class ADS1220 {
  public:
    ADS1220(uint8_t type=0);
    uint8_t cs_pin;
    uint8_t drdy_pin;
    uint8_t IS_ADS1220; // if zero, it is a 16 bit 1120
    void writeRegister(uint8_t address, uint8_t value);
    uint8_t readRegister(uint8_t address);
    void begin(uint8_t CSpin, uint8_t DRDYpin);
    bool isDataReady(void);

    void startADC_Single(void); // trigger single shot conversion
    int32_t getADC_Single(void); // get result from single shot conversion
    int32_t readADC_Single(uint8_t sleep=0); // trigger, wait, get result
    double readADC_SingleTemp(uint8_t sleep=0);
    double convertToTemp(int32_t data);
    void sendCommand(uint8_t command);
    void reset(void);
    void startSync(void);
    void powerDown(void);
    void rdata(void);
    void writeRegisterMasked(uint8_t value, uint8_t mask, uint8_t address);
    void setMultiplexer(uint8_t value);
    void setGain(uint8_t gain);
    void setPGAbypass(bool value);
    void setDataRate(uint8_t value);
    void setOpMode(uint8_t value);
    void setConversionMode(uint8_t value);
    void setTemperatureMode(uint8_t value);
    void setBurnoutCurrentSources(bool value);
    void setVoltageRef(uint8_t value);
    void setFIR(uint8_t value);
    void setPowerSwitch(uint8_t value);
    void setIDACcurrent(uint8_t value);
    void setIDAC1routing(uint8_t value);
    void setIDAC2routing(uint8_t value);
    void setDRDYmode(uint8_t value);
  };
#endif
