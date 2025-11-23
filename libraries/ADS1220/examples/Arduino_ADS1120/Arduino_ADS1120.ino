// ADS1120 Demo for PT100 and/or K-type Thermocouple

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
#include <SPI.h>
#include <avr/interrupt.h>
#include "ADS1220.h"

/****  Definition of the pins to be used ***/
#
/* Tino2 *
#define CS       7
#define DRDY    18
#define DO_SLEEP 0
#define SPI_SWAP 0 // 0= default SPI pins, 1  = Alternative SPI pins on Arduino Pins 8,9,10,11
*/

/* Arduino Boards */
#define CS      10
#define DRDY     9
#define DO_SLEEP 0
#define SPI_SWAP 0 

/*****************************************************************************/

#define USE_ADS1220 1
#define USE_ADS1120 0

// this sketch uses ADS1120
#define ADS1x20_TYPE USE_ADS1120

#define ADS1220_FULL_SCALE  (((long int)1<<23)-1)
#define ADS1120_FULL_SCALE  32767

ADS1220 ads1x20(ADS1x20_TYPE);


/*** choose either TC or RTD Measurement - hardware dependent ***/
//#define TC_MEASUREMENT
#define RTD_MEASUREMENT

/*****************************************************************************/
#ifdef RTD_MEASUREMENT
// set THREE_WIRE_MEASUREMENT to 0 if a 2-Wire or 4 Wire PT100 is used.
#define THREE_WIRE_MEASUREMENT 0
#define RTD_GAIN 4
#define RREF 1500.0

/* RTD data, RTD current, and measurement reference
   voltage. The ITS-90 standard is used; other RTDs
   may have coefficients defined by the DIN 43760 or
   the U.S. Industrial (American) standard. */

#define RTD_A_ITS90         3.9080e-3
#define RTD_A_USINDUSTRIAL  3.9692e-3
#define RTD_A_DIN43760      3.9848e-3
#define RTD_B_ITS90         -5.870e-7
#define RTD_B_USINDUSTRIAL  -5.8495e-7
#define RTD_B_DIN43760      -5.8019e-7
/* RTD coefficient C is required only for temperatures
   below 0 deg. C.  The selected RTD coefficient set
   is specified below. */

#define RTD_A         RTD_A_ITS90
#define RTD_B         RTD_B_ITS90


struct PT100_Struct
{
    uint8_t enable=1;
    int32_t ADCvalue=0;
    int     ADCoffset=0;
    float   resistance=NAN;
    float   temperature=NAN;
    
};



PT100_Struct Rtd;

float R2Temperature(float res)
{
  static const double a2   = 2.0 * RTD_B;
  static const double b_sq = RTD_A * RTD_A;
  const double rtd_resistance = 100;

  double c = 1.0 - res / rtd_resistance;
  double D = b_sq - 2.0 * a2 * c;
  double temperature_deg_C = ( -RTD_A + sqrt( D ) ) / a2;

  return( temperature_deg_C );
}


uint8_t Measure_RTD(PT100_Struct &RTD, uint16_t N)
{
	RTD.ADCvalue =0;
    uint8_t error = 0x20;

	error =0;

	ads1x20.startADC_Single();
	// empirical tests show that results degrade for delays < 5. To be safe, I increased delay to 10.
	delay(10);

	for (uint16_t i= 0; i<N; i++)
	{
		int32_t raw = ads1x20.readADC_Single(DO_SLEEP);
		RTD.ADCvalue += raw;
	}
	ads1x20.powerDown();

	if (ADS1x20_TYPE == USE_ADS1220)
		RTD.resistance =  ((float)RTD.ADCvalue / N - RTD.ADCoffset/100.0) * RREF / ADS1220_FULL_SCALE / RTD_GAIN;
	else
		RTD.resistance =  ((float)RTD.ADCvalue / N - RTD.ADCoffset/100.0) * RREF / ADS1120_FULL_SCALE / RTD_GAIN;
	// resistance must be > 0 !
	if (RTD.resistance <0) error |= 0x10;
	
	#if THREE_WIRE_MEASUREMENT
	RTD.resistance *= 2;
	#endif
	RTD.temperature = R2Temperature(RTD.resistance);
    return error;
}
#endif
/*****************************************************************************/

static uint8_t spi_swap = SPI_SWAP; 



//                  -40     -30     -20     -10   0    10     20     30     40      50     60   70      80      90
float K_T2mV[] = {-1.527, -1.156, -0.778, -0.392, 0, 0.397, 0.798, 1.203, 1.612, 2.023, 2.436, 2.851, 3.267, 3.682};


float calc_mV_from_T (float temp)
{
    int i = floor(temp/10);
    int x0 = i *10;
    int x1 = x0+10;
    
    i+=4;
    float y0 = K_T2mV[i];
    float y1 = K_T2mV[++i];
    
    float mv = y0 + ((y1-y0)/(x1-x0)) * (temp - x0);
    return mv;
}


float calc_T_from_mV (float mv)
{
    int i=0;
    for (; i<14; i++)
    {
        if (K_T2mV[i] >= mv)
            break; // found upper value
    }
    
    //Serial.println(i);
    
    float x1 = K_T2mV[i];
    float x0 = K_T2mV[i-1];
    
    int y1 = (i-4) *10;
    int y0 = y1-10;
    
    /*
    Serial.println(x0,3);
    Serial.println(x1,3);
    Serial.println(y0);
    Serial.println(y1);
    */
    
    float t = y0 + ((y1-y0)/(x1-x0)) * (mv - x0);
    return t;
}



void setup()
{
    Serial.begin(57600);
        
    // alternative SPI Port. On 4808, DA and DD devices we select the port that fits to pins 8,9,10,11
    if(spi_swap)
    {
        #if defined (MEGACOREX)
            spi_swap = SPI_SWAP;
			SPI.swap(spi_swap);
        #elif defined (ARDUINO_avrdd)
            spi_swap = SPI_MUX_PINSWAP_5; // DD devices have lots of alternate pin positions, but only 1 SPI
			SPI.swap(spi_swap);
        #elif defined (ARDUINO_avrda)
            spi_swap = SPI1_SWAP0; // DA devices have 2 SPI's  but Arduino IDE supports only one instance of SPI class
			SPI.swap(spi_swap);
        #endif
    }
   
    ads1x20.begin(CS, DRDY);
	
	
    /*
    Using a 1120 with ADS1220 protocol: for RTD and Thermocouple it works ok, because the third byte (least significant byte)
    loaded reads always 0. We then assume a 24 bit ADC reading with the 8 LSBs being 0, which is fine because we continue calculating with 
    24 bit FS reading.
    
    Using a 1220 with ADS1120 protocol: it works for TC and RTD measurements because a 16 bit resolution is sufficient.
    The third byte (LSB) is just not read and not taken into account.
    */
 

	/*****************************************************************************/
	/******                 K-Type Thermocouple Measurement                  *****/
	/*****************************************************************************/
    #ifdef TC_MEASUREMENT
	ads1x20.setDataRate(0x00);        // 20 SPS
    ads1x20.setConversionMode(0);     // Single shot (default)
    ads1x20.setVoltageRef(0);         // Internal 2.048 V (default)
    ads1x20.setOpMode(0x00);          // Normal mode (default)
    ads1x20.setMultiplexer(MUX_AIN0_AIN1);        // AIN0 vs AIN1
    ads1x20.setGain(32);
    ads1x20.setFIR(2);
    ads1x20.setTemperatureMode(1);
    ads1x20.powerDown();
	/*****************************************************************************/
	/******                 2/4 Wire PT100 Measurement                       *****/
	/*****************************************************************************/
    #elif defined RTD_MEASUREMENT
	ads1x20.setDataRate(0x03); //00 = 20 SPS, 0x03 is 175 SPS
	ads1x20.setGain(RTD_GAIN);
	ads1x20.setPGAbypass(1);
	
	// conf register 2
	
	//ads1x20.setVoltageRef(1);         // 1 - External on REFP0 and REFN0 inputs
    //ads1x20.setFIR(2);
	//ads1x20.setIDACcurrent(6);        // 0 = off, 7 = 1.5mA, 6 = 1mA, 5 = 500uA
	
	uint8_t confreg =0;
	confreg |= 1 << 6;     // Voltage reference (1 = RefP0 / RefN0)
	confreg |= 2 << 4;     // FIR setting, 0= off, 2 = 50Hz only
    
	// Idac current setting, 0 = off, 7 = 1.5mA, 6 = 1mA, 5 = 500uA  6 for 4-Wire, 5 for 3 Wire
	#if THREE_WIRE_MEASUREMENT
	confreg |= 5 << 0; //500uA for 3-Wire PT100
	#else
	confreg |= 6 << 0; // 1mA for 2/4 Wire PT100
	#endif
	ads1x20.writeRegister(CONFIG_REG2_ADDRESS, confreg);
	
	// conf register 3
	
	//ads1x20.setIDAC1routing(IDAC_AIN3);       // 0 = Disabled, 4 = AIN3/REFN1
	confreg = 0;
	confreg |= IDAC_AIN3 << 5;           // 3 bits, Idac1 routing    0 = Disabled, 4 = AIN3/REFN1
	
	#if THREE_WIRE_MEASUREMENT
	confreg |= IDAC_AIN2 << 2;         // set IDAC to AIN2 for 3-Wire Measurement
	#endif
	//confreg |= 0 << 1;         // 1 bit, DRDY mode, always 0
	ads1x20.writeRegister(CONFIG_REG3_ADDRESS, confreg);

	ads1x20.powerDown();
    #endif
    /*****************************************************************************/

    Serial.println("ADS1120 initialized");
    Serial.flush();
    

};


// THERMOCOUPLE ADS1120
#ifdef TC_MEASUREMENT
void loop()
{
	float cold_junction_temperature, thermocouple_temperature;
	int32_t tc_raw;
	
	ads1x20.setTemperatureMode(1);
    cold_junction_temperature = ads1x20.readADC_SingleTemp(DO_SLEEP);

    ads1x20.setTemperatureMode(0);    // Disable temperature sensor
	tc_raw = ads1x20.readADC_Single(DO_SLEEP);
    ads1x20.powerDown();

	double Vtc=0;
	if(ADS1x20_TYPE == USE_ADS1220)
		Vtc = 2.048 * 1000000 * tc_raw / ADS1220_FULL_SCALE / 32; // in microV --- Vref[V] * Code / 2^15 / gain * 10^6 uV
	else
		Vtc = 2.048 * 1000000 * tc_raw / ADS1120_FULL_SCALE / 32;

	thermocouple_temperature = Vtc/41.276 + cold_junction_temperature;

    Serial.print("CJ_Temp:");  Serial.print(cold_junction_temperature);
    Serial.print(",TC_Temp:"); Serial.println(thermocouple_temperature); 
    Serial.flush();
    
    //sleep_cpu();
	
	delay(2000);
}





#elif defined RTD_MEASUREMENT
// PT100 ADS1120
void loop()
{
    uint8_t rtd_error =0;

	//rtd_error = Rtd1120->WireBreakDetection_4WireRtd();
    //rtd_error |= Rtd1120->OffsetCalibration(Rtd);
	rtd_error |= Measure_RTD(Rtd, 1);

    
    if (rtd_error)
    {
        Serial.print("RTD error code: 0x"); Serial.println(rtd_error, HEX); Serial.flush();
        
        if (rtd_error & 0x01)
            Serial.println("Error: Lead 1 open");
        if (rtd_error & 0x02)
            Serial.println("Error: Lead 2 open");
        if (rtd_error & 0x04)
            Serial.println("Error: Lead 3 open");
        if (rtd_error & 0x08)
            Serial.println("Error: Lead 4 open");
        if (rtd_error & 0x10)
            Serial.println("Error: false connection or open circuit.");
        if (rtd_error & 0x20)
            Serial.println("Error: sensor disabled.");
    }
    else
    {
        Serial.print("RTD_Resistance: ");  Serial.print(Rtd.resistance,3);
        Serial.print(", RTD_Temp: "); Serial.print(Rtd.temperature,3); //Serial.print(" degC");
        Serial.print(", RTD ADC: ");Serial.print((float)Rtd.ADCvalue - Rtd.ADCoffset/100.0,3);
        Serial.println("\n\r");
        
    }
    
    Serial.flush();
    //sleep_cpu();
	
	delay(2000);
}

#endif
