/***
TiNo2 Example for using the Texas Instruments ADS11120
It is intended for noise tests of PT100 devices.
***/

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

#define PRINT_RESISTANCE
#define PRINT_TEMP
#define PRINT_ADC

#define PRINT_FOR_PLOTTER

uint8_t enable_ads1120=1;

#include <math.h>
class RunningStatsCalculator
{

public:
    uint32_t count;
    float mean;
    float dSquared;

    RunningStatsCalculator()
    {
        this->count = 0;
        this->mean = 0;
        this->dSquared = 0;
    }
    
    void update(float newValue)
    {
        this->count++;
        //float meanDifferential = (newValue - this->mean) / this->count;
        //float newMean = this->mean + meanDifferential;
        float newMean;
        if (count > 1)
        {
            newMean = this->mean + 0.2 * (newValue - this->mean);
        }
        else
        {
            newMean = newValue;
        }
        float dSquaredIncrement = (newValue - newMean) * (newValue - this->mean);
        float newDSquared = this->dSquared + dSquaredIncrement;
        
        this->mean = newMean;

        this->dSquared = newDSquared;
    }
    
    float get_mean(void)
    {
        return this->mean;
    }
    
    float get_dSquared(void) 
    {
        return this->dSquared;
    }
    
    float get_sampleVariance(void) 
    {
        return this->count > 1 ? this->dSquared / (this->count - 1) : 0;
    }

    float get_Stdev()
    {
        return sqrt(this->get_sampleVariance());
    }

};


RunningStatsCalculator Statistics;



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

//extern RTD_1120 *rtd1120;

//float rtd_res=0;
//int32_t rtd_raw=0;
float avg_adcvalue;

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
    
    // alternative SPI Port. On 4808, DA and DD devices we can select the port that fits to pins 8,9,10,11
    if(spi_swap)
    {
        #if defined (MEGACOREX)
            spi_swap = 1;
        #elif defined (ARDUINO_avrdd)
            spi_swap = SPI_MUX_PINSWAP_5; // DD devices have lots of alternate pin positions, but only 1 SPI
        #elif defined (ARDUINO_avrda)
            spi_swap = SPI1_SWAP0; // DA devices have 2 SPI's  but Arduino IDE supports only one instance of SPI class
        #endif
    }
    //Serial.print("SPI Channel: "); Serial.println(spi_swap);
    SPI.swap(spi_swap);
    
    // Initialize the ADS1120
    SPI_MISO_Enable(spi_swap);
    
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
		rtd_error |= Rtd1120->Measure(Rtd, 1);
        if (rtd_error)
        {
            rtd_enable = 0;
            Rtd.enable=0;
            Serial.print("RTD error code: 0x"); Serial.println(rtd_error, HEX);
        }
        //Serial.println(Rtd.ADCvalue);Serial.flush();
        avg_adcvalue = Rtd.ADCvalue;
    #endif
    Serial.flush();
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
void loop() {
    uint16_t N =1;
    uint8_t rtd_error=0;
    //rtd_error = PT100_Diagnostic();
	rtd_error |= Rtd1120->Measure(Rtd, N);
    if (rtd_error)
    {
        Serial.print("Error # 0x");Serial.println(rtd_error, HEX);
    }
    Statistics.update(Rtd.ADCvalue/N);
    
    //  0.2 is low pass filter
    avg_adcvalue += 0.2 * ((float)Rtd.ADCvalue/N - avg_adcvalue);
    
    float avg_res;
    avg_res =  RREF * avg_adcvalue / 32767 / RTD_GAIN;
    
    #ifdef PRINT_ADC
        #ifdef PRINT_FOR_PLOTTER
        Serial.print("ADC_reading:");Serial.print((float)Rtd.ADCvalue/N,2); // for serial plotter
        Serial.print(",ADC_average:");
        //Serial.print(avg_adcvalue);
        Serial.print(Statistics.get_mean());
        #else
        // for Excel
        //Serial.print("");Serial.print((float)Rtd.ADCvalue/N,2); // for Excel
        //Serial.print(",");Serial.print(avg_adcvalue);
        Serial.print("");Serial.print((float)Rtd.ADCvalue/N,2); 
        Serial.print(",");Serial.print(Statistics.get_mean());
        Serial.print(",");Serial.print(Statistics.get_Stdev());
        //Serial.print(",");Serial.print(vref_monitor_mV);
        
        #endif
    #endif
    
    
    #ifdef PRINT_RESISTANCE
        //for serial plotter
        #ifdef PRINT_FOR_PLOTTER
        Serial.print(",RTD_Resistance:");  Serial.print(Rtd.resistance,3); 
        Serial.print(",RTD_Res_average:");  Serial.print(avg_res,3);
        #else
        // for EXCEL
        Serial.print(",");  Serial.print(Rtd.resistance,3);
        Serial.print(",");  Serial.print(avg_res,3);
        #endif
    #endif

    #ifdef PRINT_TEMP
        #ifdef PRINT_FOR_PLOTTER
        Serial.print(",RTD_Temp:"); Serial.print(Rtd1120->Temperature(Rtd.resistance),2);
        Serial.print(",RTD_Temp_average:"); Serial.print(Rtd1120->Temperature(avg_res),3);
        #else
        Serial.print(","); Serial.print(Rtd1120->Temperature(Rtd.resistance),2);
        Serial.print(","); Serial.print(Rtd1120->Temperature(avg_res),3);
        #endif
    #endif
    Serial.println();
    
    //SPI.end();
    //SPI_MISO_Disable();

    
    //Serial.println("go sleep.");
    Serial.flush();
    sleep_cpu();

}



#else
    #error //no measurement defined!
#endif


/*
void loop()
{
    
}
*/