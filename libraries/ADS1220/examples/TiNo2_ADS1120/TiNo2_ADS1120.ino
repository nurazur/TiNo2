/**
* Example for TiNo2 devices demonstrating 2/4-wire PT100 or K-type Thermocouple usage of ADS1120 / ADS1220
* This sketch requires the TiNo2 Boards to be installed on Arduino IDE.
* see https://github.com/nurazur/TiNo2?tab=readme-ov-file#install-tino2-package for instructions how to install the TiNo2 board package
*/
#include "Arduino.h"
#include <SPI.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include "pitctrl.h"
#include "tino_ads1220.h"


// Definition of the pins to be used
#define CS      10
#define DRDY    18
#define DO_SLEEP 1
#define SPI_SWAP 0 // 0= default SPI pins, 1  = Alternative SPI pins on Arduino Pins 8,9,10,11

//#define TC_MEASUREMENT
#define RTD_MEASUREMENT

#ifdef RTD_MEASUREMENT
extern RTD_1120 *Rtd1120;
static uint8_t rtd_enable =1;
PT100_Struct Rtd;
#endif

#define USE_ADS1220 0


static uint8_t spi_swap = SPI_SWAP; 

/*****************************************************************************/
/***                   Sleep mode                                          ***/
/*****************************************************************************/

uint16_t watchdog_counter;
//bool watchdog_expired = false;

// interrupt service routine for RTC periodic timer
ISR(RTC_PIT_vect)
{
    RTC.PITINTFLAGS = RTC_PI_bm;              // clear interrupt flag
    watchdog_counter++;
}

// Input sense configuration (ISC)
void disablePinISC(uint8_t pin)
{
  PORT_t *port = digitalPinToPortStruct(pin);
  // Get bit position for getting pin ctrl reg
  uint8_t bit_pos = digitalPinToBitPosition(pin);

  // Calculate where pin control register is
  volatile uint8_t *pin_ctrl_reg = getPINnCTRLregister(port, bit_pos);

  // Disable ISC
  *pin_ctrl_reg = PORT_ISC_INPUT_DISABLE_gc;
}


/*****************************************************************************/
/******                   Periodic Interrupt Timer and RTC setup         *****/
/*****************************************************************************/

PITControl PIT;

/*****************************************************************************/

static void SPI_MISO_Enable(uint8_t swap)
{
    if(swap == 0)
    {
        pinConfigure(MISO, PIN_DIR_INPUT, PIN_PULLUP_ON, PIN_INPUT_ENABLE);
    }
    #if defined (MEGACOREX)
    else if (swap==SPI_MUX_PINSWAP_1)
    {
        pinConfigure(PIN_SPI_MISO_PINSWAP_1, PIN_DIR_INPUT, PIN_PULLUP_ON, PIN_INPUT_ENABLE);// PIN_PC1, Arduino Pin 9
    }
    #endif
    #if defined (ARDUINO_avrdd)
    else if (swap==SPI_MUX_PINSWAP_5)
    {
        pinConfigure(PIN_SPI_MISO_PINSWAP_5, PIN_DIR_INPUT, PIN_PULLUP_ON, PIN_INPUT_ENABLE);// PIN_PC1, Arduino Pin 9
    }
    #endif
    #if defined (ARDUINO_avrda)
    else if (swap==SPI1_SWAP0)
    {
        pinConfigure(PIN_SPI1_MISO, PIN_DIR_INPUT, PIN_PULLUP_ON, PIN_INPUT_ENABLE);// PIN_PC1, Arduino Pin 9
        //Serial.print("Miso enabled: "); Serial.println(PIN_SPI1_MISO);
    }
    #endif
}


static void SPI_MISO_Disable(uint8_t swap)
{
    if(swap == 0)
    {
        //pinConfigure(PIN_PA5, PIN_DIR_INPUT, PIN_PULLUP_ON, PIN_INPUT_DISABLE);
        pinConfigure(MISO, PIN_DIR_INPUT, PIN_PULLUP_OFF, PIN_INPUT_DISABLE);
    }
    #if defined (MEGACOREX)
    else if (swap==SPI_MUX_PINSWAP_1)
    {
        pinMode(PIN_SPI_MISO_PINSWAP_1, INPUT_PULLUP);
        //pinConfigure(PIN_SPI_MISO_PINSWAP_1, PIN_DIR_INPUT, PIN_PULLUP_OFF, PIN_INPUT_DISABLE);
    }
    #endif
    #if defined (ARDUINO_avrdd)
    else if (swap==SPI_MUX_PINSWAP_5)
    {
        pinMode(PIN_SPI_MISO_PINSWAP_5, INPUT_PULLUP);
        //pinConfigure(PIN_SPI_MISO_PINSWAP_5, PIN_DIR_INPUT, PIN_PULLUP_OFF, PIN_INPUT_DISABLE);
    }
    #endif
    #if defined (ARDUINO_avrda)
    else if (swap==SPI1_SWAP0)
    {
        pinMode(PIN_SPI1_MISO, INPUT_PULLUP);
        //pinConfigure(PIN_SPI1_MISO, PIN_DIR_INPUT, PIN_PULLUP_OFF, PIN_INPUT_DISABLE);
    }
    #endif

}

//                  -40     -30     -20     -10   0    10     20     30     40      50     60   70      80      90
float K_T2mV[] = {-1.527, -1.156, -0.778, -0.392, 0, 0.397, 0.798, 1.203, 1.612, 2.023, 2.436, 2.851, 3.267, 3.682};

/*
uint8_t find_low_index_from _mV(float mv)
{
    int low_index;

}
*/

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
    /***                    ***/
    /*** disable all GPIO's ***/
    /***                    ***/
    
    for (uint8_t i = 0; i < 26; i++)
    {
        pinMode(i, INPUT_PULLUP);
        disablePinISC(i);
    }
    
    Serial.begin(57600);
    
    
    // alternative SPI Port. On 4808, DA and DD devices we select the port that fits to pins 8,9,10,11
    if(spi_swap)
    {
        #if defined (MEGACOREX)
            spi_swap = SPI_SWAP;
        #elif defined (ARDUINO_avrdd)
            spi_swap = SPI_MUX_PINSWAP_5; // DD devices have lots of alternate pin positions, but only 1 SPI
        #elif defined (ARDUINO_avrda)
            spi_swap = SPI1_SWAP0; // DA devices have 2 SPI's  but Arduino IDE supports only one instance of SPI class
        #endif
    }
    //Serial.print("SPI Channel: "); Serial.println(spi_swap);
    SPI.swap(spi_swap);
    //Serial.println("SPI Channel selected");
    
    // Initialize the ADS1120
    SPI_MISO_Enable(spi_swap);
    Serial.println("SPI MISO enabled");
      
    // set periodic interrupt timer
    PIT.init(0, 0); // ULPO 8s
    PIT.enable();
    
    // set sleep mode
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);      
    sleep_enable();    // enable sleep control
    
    
    /*
    Using a 1120 with ADS1220 protocol: for RTD and Thermocouple it works ok, because the third byte (least significant byte)
    loaded reads always 0. We then assume a 24 bit ADC reading with the 8 LSBs being 0, which is fine because we continue calculating with 
    24 bit FS reading.
    
    Using a 1220 with ADS1120 protocol: it works for TC and RTD measurements because a 16 bit resolution is sufficient.
    The third byte (LSB) is just not read and not taken into account.
    */
    
    #ifdef TC_MEASUREMENT
        ThermoCouple_Init_ADS1120(1, DRDY, CS, DO_SLEEP, USE_ADS1220); 
    #elif defined RTD_MEASUREMENT
        Rtd.enable = rtd_enable;
		if(Rtd.enable)
			Rtd1120 = new RTD_1120(DRDY, CS, DO_SLEEP, RREF, USE_ADS1220);

		uint8_t rtd_error = Rtd1120->WireBreakDetection_4WireRtd();
		rtd_error |= Rtd1120->OffsetCalibration(Rtd);
        if (rtd_error)
        {
            rtd_enable = 0;
            Rtd.enable=0;
            Serial.print("RTD error code: 0x"); Serial.println(rtd_error, HEX);
        }
    #else
        Rtd.enable = rtd_enable;
	    if(Rtd.enable)
		    Rtd1120 = new RTD_1120(DRDY, CS, DO_SLEEP, RREF, USE_ADS1120);
        uint8_t rtd_error=0;
        rtd_error |= Rtd1120->OffsetCalibration(Rtd);
        if (rtd_error)
        {
            rtd_enable = 0;
            Rtd.enable=0;
            Serial.print("RTD error code: 0x"); Serial.println(rtd_error, HEX);
        }
    #endif
    
	#ifndef TC_MEASUREMENT
    uint8_t config_reg;
    config_reg = Rtd1120->readRegister(CONFIG_REG0_ADDRESS);
    Serial.print("CONFIG_REG0_ADDRESS: "); Serial.println(config_reg, HEX); // should be 0x05
    
    config_reg = Rtd1120->readRegister(CONFIG_REG1_ADDRESS);
    Serial.print("CONFIG_REG1_ADDRESS: "); Serial.println(config_reg, HEX); // should be 0x60 (Data rate 175 SPS) or 0x00
    
    config_reg = Rtd1120->readRegister(CONFIG_REG2_ADDRESS);
    Serial.print("CONFIG_REG2_ADDRESS: "); Serial.println(config_reg, HEX); // should be 0x66
    
    config_reg = Rtd1120->readRegister(CONFIG_REG3_ADDRESS);
    Serial.print("CONFIG_REG3_ADDRESS: "); Serial.println(config_reg, HEX); // should be 0x80
    #endif
    /*
    uint8_t gain = 8;
    uint8_t i;
    for(i=0; gain>>=1;i++);
    Serial.println(i);
    
    gain = 4;
    for(i=0; gain>>=1;i++);
    Serial.println(i);
    
    gain=2;
    for(i=0; gain>>=1;i++);
    Serial.println(i);
    
    gain = 1;
    for(i=0; gain>>=1;i++);
    Serial.println(i);
    */
    
    //Serial.println("ADS1120 initialized");
    Serial.flush();
    
    
    //Serial.print("Test T 2 mV: 25 "); Serial.println(calc_mV_from_T(25), 3);
    /*
    Serial.print("R= : 84.27 "); Serial.print(PT100_Temperature(84.27),3); Serial.println(" expected: -40.0");
    Serial.print("R= : 96.09 "); Serial.print(PT100_Temperature(96.09),3); Serial.println(" expected: -10.0");
    Serial.print("R= : 103.90 "); Serial.print(PT100_Temperature(103.90),3); Serial.println(" expected: 10.0");
    Serial.print("R= 107.79: "); Serial.print(PT100_Temperature(107.79),3); Serial.println(" expected: 20.0");
    Serial.print("R= 111.67: "); Serial.print(PT100_Temperature(111.67),3); Serial.println(" expected: 30.0");
    Serial.print("R= 119.4: "); Serial.print(PT100_Temperature(119.4),3); Serial.println(" expected: 50.0");
    Serial.print("R= 138.51: "); Serial.print(PT100_Temperature(138.51),3); Serial.println(" expected: 100.0");
    Serial.print("R= 175.86: "); Serial.print(PT100_Temperature(175.86),3); Serial.println(" expected: 200.0");
    Serial.print("R= 214.9: "); Serial.print(PT100_Temperature(214.9),3); Serial.println(" expected: 308");
    */
};



#ifdef TC_MEASUREMENT
// THERMOCOUPLE ADS1120
void loop()
{
    float temps[2];
    temps[0] = temps[1] =0;
    Thermocouple_Measure_ADS1120(1, temps);

    Serial.print("CJ_Temp:");  Serial.print(temps[COLD_JUNCTION_TEMPERATURE]);
    Serial.print(",TC_Temp:"); Serial.println(temps[THERMOCOUPLE_TEMPERATURE]); // poor man's conversion to temperature
    Serial.flush();
    
    sleep_cpu();
}





#elif defined RTD_MEASUREMENT
// PT100 ADS1120
void loop()
{
    uint8_t rtd_error =0;

	rtd_error = Rtd1120->WireBreakDetection_4WireRtd();
    rtd_error |= Rtd1120->OffsetCalibration(Rtd);
	rtd_error |= Rtd1120->Measure(Rtd, 1);

    
    if (rtd_error)
    {
        Serial.print("RTD error code: 0x"); Serial.println(rtd_error, HEX);Serial.flush();
        
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
    sleep_cpu();
}

#else
    // Offset calibration and temperature measurement test
void loop()
{
    uint8_t rtd_error =0;

    //rtd_error = PT100_Diagnostic();
    rtd_error = Rtd1120->OffsetCalibration(Rtd);
    //rtd_error |= PT100_Measure_ADS1120(Rtd);

    Serial.print("ADC Offset x100: ");Serial.print(Rtd.ADCoffset);
    Serial.print(", 0x");Serial.println(Rtd.ADCoffset, HEX);
    Rtd1120->setTemperatureMode(1);
    
    int32_t adcraw;
    double T;
    T = Rtd1120->readADC_SingleTemp(DO_SLEEP);
    Rtd1120->setTemperatureMode(0);
    Serial.print("Temperature: "); Serial.println(T,3);
    
    adcraw -= Rtd.ADCoffset/100
        
    Serial.flush();
    sleep_cpu();
}



#endif
