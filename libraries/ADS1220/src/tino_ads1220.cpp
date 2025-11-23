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
#include "tino_ads1220.h"
#include <stdlib.h>

#ifdef STM32WL
#include "STM32LowPower.h"
#endif

#include <SPI.h>

TC_1120 *tc1120 = NULL;
RTD_1120 *Rtd1120 = NULL;


TC_1120::TC_1120(uint8_t DrdyPin, uint8_t CsPin, uint8_t DoSleep, uint8_t is_ads1220) : ADS1220(is_ads1220)
{
    do_sleep = DoSleep;
    //ads_type = is_ads1220;
    //pinConfigure(DrdyPin, PIN_DIR_INPUT, PIN_PULLUP_OFF, PIN_INPUT_ENABLE); // configure Data ready pin (no pullup!)
    pinMode(DrdyPin, INPUT);

    if (do_sleep)
    {
        //attachInterrupt(DrdyPin, DrDyPinInteruptFunc, FALLING);
    }

    this->begin(CsPin, DrdyPin);
    this->setDataRate(0x00);        // 20 SPS
    this->setConversionMode(0);     // Single shot (default)
    this->setVoltageRef(0);         // Internal 2.048 V (default)
    this->setOpMode(0x00);          // Normal mode (default)
    this->setMultiplexer(MUX_AIN0_AIN1);        // AIN0 vs AIN1
    this->setGain(32);
    this->setFIR(2);
    this->setTemperatureMode(1);
    this->powerDown();
}

void TC_1120::Measure(uint8_t enable, float* temperatures)
{
    if (enable)
    {
        SPI.begin();
        SPI.setDataMode(SPI_MODE1);
        int32_t tc_raw;
        this->setTemperatureMode(1);
        temperatures[COLD_JUNCTION_TEMPERATURE] = this->readADC_SingleTemp(this->do_sleep);

        this->setTemperatureMode(0);    // Disable temperature sensor
        tc_raw = this->readADC_Single(this->do_sleep);
        this->powerDown();

        double Vtc=0;
        if(this->IS_ADS1220)
            Vtc = 2.048 * 1000000 * tc_raw / ADS1220_FULL_SCALE / 32; // in microV --- Vref[V] * Code / 2^15 / gain * 10^6 uV
        else
            Vtc = 2.048 * 1000000 * tc_raw / ADS1120_FULL_SCALE / 32;

        temperatures[THERMOCOUPLE_TEMPERATURE] = Vtc/41.276 + temperatures[COLD_JUNCTION_TEMPERATURE];
    }
}


uint8_t ThermoCouple_Init_ADS1120(uint8_t enable, uint8_t DrdyPin, uint8_t CsPin, uint8_t DoSleep, uint8_t ads_type)
{
    if (enable)
    {
        tc1120 = new TC_1120(DrdyPin, CsPin, DoSleep, ads_type);
        if (tc1120->IS_ADS1220)
            Serial.println("ADS1220 configuration");
        else
            Serial.println("ADS1120 configuration");
    }
	return tc1120->readRegister(CONFIG_REG0_ADDRESS);
}

void Thermocouple_Measure_ADS1120(uint8_t enable, float* temps)
{
    if (enable && tc1120)
        tc1120->Measure(enable, temps);
}



RTD_1120::RTD_1120(uint8_t DrdyPin, uint8_t CsPin, uint8_t DoSleep, float R_ref, uint8_t is_ads1220) : ADS1220(is_ads1220)
    {
        do_sleep = DoSleep;
        Rref = R_ref;
        // this class relies on the DRDY pin to be read out in order to detect a coversion is complete. 
        pinMode(DrdyPin, INPUT);
        
        this->begin(CsPin, DrdyPin);
        
        uint8_t confreg =0;
        
        // conf register 1
        
        //this->setDataRate(0x00);        // 0x00 = 20 SPS, 0x03 is 175 SPS
        //this->setOpMode(0x00);          // Normal mode (default)
        //this->setConversionMode(0);     // Single shot (default)
        //this->setTemperatureMode(0);    // default
        
        
        confreg |= (0x03) << 5;         // Datarate, 3 bits         00 = 20 SPS, 0x03 is 175 SPS
        /*
        //block commented out because all zeros
        confreg |= (0x00) << 3;         // operation mode, 2 bits   00 = Normal mode (default)
        confreg |= (0x00) << 2;         // Converion mode            0 = single shot
        confreg |= (0x00) << 1;         // temperature mode          0 = off
        confreg |= (0x00) << 0;         // burn-out current sources  0 = off
        */
        writeRegister(CONFIG_REG1_ADDRESS, confreg);   //write 0x00
        
        // conf register 0
        confreg = 0;
        uint8_t gain = RTD_GAIN;
        uint8_t gain_setting;
        for(gain_setting=0; gain>>=1;gain_setting++);
        confreg |= MUX_AIN0_AIN1<<4;   // Multiplexer, 4 bits
        confreg |= gain_setting << 1;   // PGA gain, 3 bits, 0=1x, 1=2x, 2=4x, 3=8x
        
        confreg |= 0x1;                 // PGA Bypass, 1 bit
        writeRegister(CONFIG_REG0_ADDRESS, confreg); // write 0x05
        
        //this->setMultiplexer(MUX_AIN0_AIN1);  // AIN0 vs AIN1
        //this->setGain(RTD_GAIN);        // can possibly be 2, up to 300 degC
        //this->setPGAbypass(1);
        
        
        // conf register 2
        confreg = 0;
        confreg |= 1 << 6;           // Voltage reference (1 = RefP0 / RefN0)
        confreg |= 2 << 4;            // FIR setting, 0= off, 2 = 50Hz only
        confreg |= 6 << 0;          // Idac current setting, 0 = off, 7 = 1.5mA, 6 = 1mA, 5 = 500uA
        writeRegister(CONFIG_REG2_ADDRESS, confreg);
        
        //this->setVoltageRef(1);         // 1 - External on REFP0 and REFN0 inputs
        //this->setFIR(2);
        //this->setIDACcurrent(6);        // 0 = off, 7 = 1.5mA, 6 = 1mA, 5 = 500uA
        
        
        // conf register 3
        confreg = 0;
        confreg |= 4 << 5;           // 3 bits, Idac1 routing    0 = Disabled, 4 = AIN3/REFN1
        //confreg |= 0 << 2;           // 3 bits, Idac1 routing    0 = disabled
        //confreg |= 0 << 1;           // 1 bit, DRDY mode, always 0
        
        //this->setIDAC1routing(4);       // 0 = Disabled, 4 = AIN3/REFN1
        writeRegister(CONFIG_REG3_ADDRESS, confreg);
        
        this->powerDown();
    }

    
void RTD_1120::Init(uint8_t enable)
{
    (void)enable;
    Serial.println("ADS1x20 PT100 Device initialized.");
}    

uint8_t RTD_1120::Measure(PT100_Struct &RTD, uint16_t N)
{
    RTD.ADCvalue =0;
    uint8_t error = 0x20;
    if(RTD.enable)
    {
        error =0;
        SPI.begin();
        SPI.setDataMode(SPI_MODE1);

        this->startADC_Single();
        // empirical tests show that results degrade for delays < 5. To be safe, I increased delay to 10.
        delay(10);

        for (uint16_t i= 0; i<N; i++)
        {
            int32_t raw = this->readADC_Single(this->do_sleep);
            RTD.ADCvalue += raw;
        }
        this->powerDown();

        if (this->IS_ADS1220)
            RTD.resistance =  ((float)RTD.ADCvalue / N - RTD.ADCoffset/100.0) * this->Rref / ADS1220_FULL_SCALE / RTD_GAIN;
        else
            RTD.resistance =  ((float)RTD.ADCvalue / N - RTD.ADCoffset/100.0) * this->Rref / ADS1120_FULL_SCALE / RTD_GAIN;
        // resistance must be > 0 !
        if (RTD.resistance <0) error |= 0x10;
        
        RTD.temperature = this->Temperature(RTD.resistance);
    }
    return error;
}

uint8_t RTD_1120::OffsetCalibration(PT100_Struct &RTD)
{
    uint8_t error = 0x20;
    if(RTD.enable)
    {
        error=0;
        uint8_t configreg0 = readRegister(CONFIG_REG0_ADDRESS);
        // generate local PT100 struct so we don't mess with the given struct RTD
        PT100_Struct rtd_local; 
        setMultiplexer(0xE);
        error |= Measure(rtd_local, 100);
        RTD.ADCoffset = rtd_local.ADCvalue;
        writeRegister(CONFIG_REG0_ADDRESS, configreg0);
    }
    return error;
}


int32_t RTD_1120::VrefMonitor(uint8_t enable)
{
    //float Vref_mV=0;
    int32_t vrefmonitor = 0;
    if (enable)
    {
        SPI.begin();
        SPI.setDataMode(SPI_MODE1);
        uint8_t configreg0 = this->readRegister(CONFIG_REG0_ADDRESS);
        this->setMultiplexer(0xC);
        
        vrefmonitor = this->readADC_Single(this->do_sleep);
        this->writeRegister(CONFIG_REG0_ADDRESS, configreg0);
        //this->powerDown();
    
        //Vref_mV = vrefmonitor * 2048 * 4 / (IS_ADS1220 ? ADS1220_FULL_SCALE : ADS1120_FULL_SCALE);
    }
    return vrefmonitor;
}


/* return vale:
bit 0: Lead 1 broken
bit 1: Lead 2 broken
bit 2: Lead 3 broken
bit 3: Lead 4 broken
bit 4: RTD is negative
bit 5: RTD chip is disabled
*/
uint8_t RTD_1120::WireBreakDetection_4WireRtd(void)
{
    uint8_t errorflags = 0;
    int32_t vref_monitor;
    vref_monitor = this->VrefMonitor(1);
    
    if (vref_monitor < 100) // vref_monitor is 0, but are there stray currents?
    {
        // Lead 1 and/or Lead 4 broken
        errorflags= errorflags | 0x9;
    }

    uint8_t configreg2 = this->readRegister(CONFIG_REG2_ADDRESS);
    uint8_t configreg3 = this->readRegister(CONFIG_REG3_ADDRESS);
    
    this->setIDACcurrent(4); // 250uA
    this->setIDAC1routing(1); // idac1 to AIN0
    this->setIDAC2routing(2); // idac2 to AIN1
    
    //float res;
    //int32_t adcval;
    
    PT100_Struct rtd;
    rtd.enable=1;
    rtd.ADCoffset=0;
    this->Measure(rtd);

    this->writeRegister(CONFIG_REG2_ADDRESS, configreg2);
    this->writeRegister(CONFIG_REG3_ADDRESS, configreg3);
    this->powerDown();

    if (rtd.resistance < 0)
        errorflags |= 0x10;
     
    int32_t fs = Rtd1120->IS_ADS1220 ? ADS1220_FULL_SCALE : ADS1120_FULL_SCALE;
    if (rtd.ADCvalue == -fs-1)
    {
        // Lead 3 broken
        errorflags |= 0x4;
    }
    else if (rtd.ADCvalue == fs)
    {
        // Lead 2 broken
        errorflags |=2;
    }
    else if(errorflags & 0x9) // error in lead 1/4 has been signalled
    {
        // lead 2 and 3 test ok, but this works only if lead 4 is intact.
        // so Lead 1 must be broken.
        errorflags &= 0x1; // clear error lead 4, 3 and 2
    }
    else
    {
        // no error
    }

    return errorflags;
}


float RTD_1120::Temperature(float res)
{
  static const double a2   = 2.0 * RTD_B;
  static const double b_sq = RTD_A * RTD_A;
  const double rtd_resistance = 100;

  double c = 1.0 - res / rtd_resistance;
  double D = b_sq - 2.0 * a2 * c;
  double temperature_deg_C = ( -RTD_A + sqrt( D ) ) / a2;

  return( temperature_deg_C );
}

uint8_t RTD_1120::Diagnostic(void)
{
    return WireBreakDetection_4WireRtd();
}
/******************************************************************************/
/*****  Wrapper funktions to hide RTD1120 Class and to simplify its use  ******/
/******************************************************************************/
/*
uint8_t RTD_Diagnostic(void)
{
    uint8_t errorflags= 0;
    if (Rtd1120)
    {
        errorflags = Rtd1120->WireBreakDetection_4WireRtd();
    }
    else
    {
        errorflags |= 0x20;
    }

    return errorflags;
}


void RTD_Init(uint8_t enable, uint8_t DrdyPin, uint8_t CsPin, uint8_t DoSleep, float Rref, uint8_t ads_type)
{
    if (enable)
    {
        Rtd1120 = new RTD_1120(DrdyPin, CsPin, DoSleep, Rref, ads_type);
    }
}


void RTD_Init(uint8_t enable, PT100_ChipConfig Conf)
{
    if (enable)
    {
        Rtd1120 = new RTD_1120(Conf.DrdyPin, Conf.CsPin, Conf.DoSleep, Conf.Rref, Conf.ChipType);
    }
}


uint8_t RTD_Measure(PT100_Struct &RTD, uint16_t N)
{
    if (RTD.enable && Rtd1120)
    {
        return Rtd1120->Measure(RTD, N);
    }
    return 0x20; // error: chip is disabled or not initialized
}


uint8_t PT100_OffsetCalibration(PT100_Struct &RTD)
{
    if (RTD.enable && Rtd1120)
    {
        return Rtd1120->OffsetCalibration(RTD);
    }
    return 0x20; // error: chip is disabled or not initialized
}
*/
