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

/*
Version = 5
Fits Build 7 and later
*/
#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <Arduino.h>

#define EEPROMVERSIONNUMBER 10
#if (EEPROMVERSIONNUMBER == 7)
typedef struct
{
    byte Nodeid =           1;
    byte Networkid =        210;
    byte Gatewayid =        22;
    uint16_t VccAtCalmV  =  1100;
    uint16_t AdcCalValue =  1023;
    uint16_t Senddelay =    2;
    byte SensorConfig = 0x01;
    float frequency = 866.0;
    byte TxPower = 25;
    char radio_temp_offset=0; // the calibration value of the temp sensor in the radio in degC*10
    byte UseRadioFrequencyCompensation=0;
    byte RequestAck =0;
    byte LedCount = 3;
    byte LedPin = 19;
    char LdrPin = 12;
    char PirPowerPin =9;
    uint16_t PirDeadTime=3;
    byte RxPin = 1;     // unused with 4808
    byte TxPin = 0;     // unused with 4808
    byte SDAPin = 2;    // unused with 4808
    byte SCLPin = 3;    // unused with 4808
    byte OneWireDataPin=10;
    byte I2CPowerPin = 8;

    char PCI0Pin=13;// 30
    byte PCI0Trigger:3; // Change/Falling/Rising = 2/0/1
    byte PCI0Mode:5; // 0= INPUT, 2 = INPUT_PULLUP
    byte PCI0Gatewayid =22;

    char PCI1Pin=12; //33
    byte PCI1Trigger:3;
    byte PCI1Mode:5; // 0= INPUT, 2 = INPUT_PULLUP
    byte PCI1Gatewayid =22;

    char PCI2Pin=-1;
    byte PCI2Trigger:3;
    byte PCI2Mode:5; // 0= INPUT, 2 = INPUT_PULLUP
    byte PCI2Gatewayid =22;

    char PCI3Pin=-1;  //39
    byte PCI3Trigger:3;
    byte PCI3Mode:5; // 0= INPUT, 2 = INPUT_PULLUP
    byte PCI3Gatewayid =22;

    byte UseCrystalRtc=0; //42
    byte EncryptionEnable=1;
    byte FecEnable =1;
    byte InterleaverEnable =1;
    byte EepromVersionNumber=EEPROMVERSIONNUMBER;
    uint16_t SoftwareversionNumber=0;
    byte TXGaussShaping =0;
    byte SerialEnable =1;
    byte IsRFM69HW = 1; // standard on 4808
    byte PaBoost = 0;
    int16_t FedvSteps= 0;
    uint16_t checksum=0xffff;
}
Configuration;

#elif (EEPROMVERSIONNUMBER == 8)
    typedef struct
{
    byte Nodeid =           1;
    byte Networkid =        210;
    byte Gatewayid =        22;
    uint16_t VccAtCalmV  =  1100;
    uint16_t AdcCalValue =  1023;
    uint16_t Senddelay =    2;
    byte SensorConfig = 0x01;
    float frequency = 866.0;
    byte TxPower = 25;
    char radio_temp_offset=0; // the calibration value of the temp sensor in the radio in degC*10
    byte UseRadioFrequencyCompensation=0;
    byte RequestAck =0;
    byte LedCount = 3;
    byte LedPin = 19;
    char LdrPin = 12;
    char PirPowerPin =9;
    uint16_t PirDeadTime=3;
    byte RxPin = 1;     // unused with 4808
    byte RTDPowerPin = 22;
    byte RTDCSPin = 11;
    byte OneWirePowerPin = 9;
    byte OneWireDataPin=10;
    byte I2CPowerPin = 8;

    char PCI0Pin=13;// 30
    byte PCI0Trigger:3; // Change/Falling/Rising = 2/0/1
    byte PCI0Mode:5; // 0= INPUT, 2 = INPUT_PULLUP
    byte PCI0Gatewayid =22;

    char PCI1Pin=12; //33
    byte PCI1Trigger:3;
    byte PCI1Mode:5; // 0= INPUT, 2 = INPUT_PULLUP
    byte PCI1Gatewayid =22;

    char PCI2Pin=-1;
    byte PCI2Trigger:3;
    byte PCI2Mode:5; // 0= INPUT, 2 = INPUT_PULLUP
    byte PCI2Gatewayid =22;

    char PCI3Pin=-1;  //39
    byte PCI3Trigger:3;
    byte PCI3Mode:5; // 0= INPUT, 2 = INPUT_PULLUP
    byte PCI3Gatewayid =22;

    byte UseCrystalRtc=0; //42
    byte EncryptionEnable=1;
    byte FecEnable =1;
    byte InterleaverEnable =1;
    byte EepromVersionNumber=EEPROMVERSIONNUMBER;
    uint16_t SoftwareversionNumber=0;
    byte TXGaussShaping =0;
    byte SerialEnable =1;
    byte IsRFM69HW = 1; // standard on 4808
    byte PaBoost = 0;
    int16_t FedvSteps= 0;
    uint16_t checksum=0xffff;
}
Configuration;

#elif (EEPROMVERSIONNUMBER == 9)
    typedef struct
{
    byte Nodeid =           1;
    byte Networkid =        210;
    byte Gatewayid =        22;
    uint16_t VccAtCalmV  =  1100;
    uint16_t AdcCalValue =  1023;
    uint16_t Senddelay =    2;
    byte SensorConfig = 0x01;
    float frequency = 866.0;
    byte TxPower = 25;
    char radio_temp_offset=0; // the calibration value of the temp sensor in the radio in degC*10
    byte UseRadioFrequencyCompensation=0;
    byte RequestAck =0;
    byte LedCount = 3;
    byte LedPin = 19;
    char LdrPin = 12;
    char PirDataPin =9;
    uint16_t PirDeadTime=3;
    byte RxPin = 1;     // unused with 4808
    byte RTDPowerPin = 22;
    byte RTDCSPin = 11;
    byte OneWirePowerPin = 9;
    byte OneWireDataPin=10;
    char I2CPowerPin = 8;


    char PCI0Pin=13;// 30
    byte PCI0Trigger:3; // Change/Falling/Rising = 4/2/3
    byte PCI0Mode:5; // 0= INPUT, 2 = INPUT_PULLUP
    byte PCI0Gatewayid =22;

    char PCI1Pin=12; //33
    byte PCI1Trigger:3;
    byte PCI1Mode:5; // 0= INPUT, 2 = INPUT_PULLUP
    byte PCI1Gatewayid =22;

    char PCI2Pin=-1;
    byte PCI2Trigger:3;
    byte PCI2Mode:5; // 0= INPUT, 2 = INPUT_PULLUP
    byte PCI2Gatewayid =22;

    char PCI3Pin=-1;  //39
    byte PCI3Trigger:3;
    byte PCI3Mode:5; // 0= INPUT, 2 = INPUT_PULLUP
    byte PCI3Gatewayid =22;

    /*
    char PCIPin[4];
    byte PCI0Trigger:3
    byte PCI0Mode:5;
    byte PCI1Trigger:3
    byte PCI1Mode:5;
    byte PCI2Trigger:3
    byte PCI2Mode:5;
    byte PCI3Trigger:3
    byte PCI3Mode:5;
    byte PCIGatewayid[4];
    */

    byte UseCrystalRtc=0; //42
    byte EncryptionEnable=1;
    byte FecEnable =1;
    byte InterleaverEnable =1;
    byte EepromVersionNumber=EEPROMVERSIONNUMBER;
    uint16_t SoftwareversionNumber=0;
    byte TXGaussShaping =0;
    byte SerialEnable =1;
    byte IsRFM69HW = 1; // standard on 4808
    byte PaBoost = 0;
    int16_t FedvSteps= 0;
    uint16_t checksum=0xffff;
}
Configuration;

#elif (EEPROMVERSIONNUMBER == 10)
    typedef struct
{
    byte Nodeid =           1;
    byte Networkid =        210;
    byte Gatewayid =        22;
    uint16_t VccAtCalmV  =  1100;
    uint16_t AdcCalValue =  1023;
    uint16_t Senddelay =    2;
    //byte SensorConfig = 0x01;
    uint16_t SensorConfig = 0x01;
    float frequency = 866.0;
    byte TxPower = 25;
    char radio_temp_offset=0; // the calibration value of the temp sensor in the radio in degC*10
    byte UseRadioFrequencyCompensation=0;
    byte RequestAck =0; // bit 1: 1 = do not ACK even if requested. 0 = ACK if requested
                        // bit 2: gateway mode. Listen to any node, regardless which target address. ACK to nodeid==targetid only.
    byte LedCount = 3;
    byte LedPin = 19;
    char LdrPin = 12;
    char PirDataPin =9;
    uint16_t PirDeadTime=3;
    char TCCSPin;     // Thermocouple CS Pin
    byte RTDPowerPin = 22;  // this pin is shared with Thermocouple
    byte RTDCSPin = 11;
    byte OneWirePowerPin = 9;
    byte OneWireDataPin=10;
    char I2CPowerPin = 8;

    char PCI0Pin=13;
    byte PCI0Trigger:3; // Change/Falling/Rising = 4/2/3
    byte PCI0Mode:5; // 0= INPUT, 2 = INPUT_PULLUP
    byte PCI0Gatewayid =22;

    char PCI1Pin=12; //33  // shared with PIR power
    byte PCI1Trigger:3;
    byte PCI1Mode:5; // 0= INPUT, 2 = INPUT_PULLUP
    byte PCI1Gatewayid =22;

    char PCI2Pin=-1;
    byte PCI2Trigger:3;
    byte PCI2Mode:5; // 0= INPUT, 2 = INPUT_PULLUP
    byte PCI2Gatewayid =22;

    char PCI3Pin=-1;
    byte PCI3Trigger:3;
    byte PCI3Mode:5; // 0= INPUT, 2 = INPUT_PULLUP
    byte PCI3Gatewayid =22;

    byte UseCrystalRtc=0;
    byte EncryptionEnable=1;
    byte FecEnable =1;
    byte InterleaverEnable =1; // don't need as FEC and Interleaver go together.
    byte EepromVersionNumber=EEPROMVERSIONNUMBER;
    uint16_t SoftwareversionNumber=0;
    byte TXGaussShaping =0;
    byte SerialEnable =1;
    byte IsRFM69HW = 1; // standard on 4808
    byte PaBoost = 0;
    int16_t FedvSteps= 0;
    uint16_t checksum=0xffff;
}
Configuration;
#endif

#endif // CONFIGURATION_H