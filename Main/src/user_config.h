/***
Basic Compiler directives for TiNo2 Build
Additionally, depending on hardware, externally used modules can be integrated
***/

/*****************************************************************************/
/***                            Encryption                                 ***/
/*****************************************************************************/
// encryption is OPTIONAL by compilation switch
// encryption will encrypt the RF packages and the EEPROM content.
// to enable encryption you will need to:
//  - provide a 16-byte encryption KEY (same on all nodes that talk encrypted)
//  - set the variable ENCRYPTION_ENABLE to 1 in the EEPROM, at runtime in Cfg.EncryptionEnable

// comment the following line if you don't want to encrypt RF message traffic
// it is STRONGLY recommended to change de Key below to your own 16-Byte Password
//#define KEY   "TheQuickBrownFox"
#define KEY "WiNW_AzurdelaMer"
#ifndef KEY
#warning NO ENCRYPTION KEY DEFINED
#define KEY NULL
#endif

/*****************************************************************************/
/***                           TYPE OF NODE                                ***/
/*****************************************************************************/
// note, platform.ini defines these compile switches already. So there is no need to edit.
// Arduino IDE Users need to set IS_SENSOR_NODE to true or false, depending what type of TiNo2 is wanted.
#if !defined IS_RECEIVER && !defined IS_SENSOR_NODE
// this block is for Arduino IDE only.
/*** set to false if you want to compile a receiver sketch    ***/
/*** set to true  if you want to compile a sensor node sketch ***/
	#define IS_SENSOR_NODE true


	// don't edit the block below. 
	#if IS_SENSOR_NODE
		#define IS_RECEIVER false
		#if F_CPU >= 8000000L
			#error "Processor Speed too fast for Sensor Node. Consider F_CPU <= 4 MHz"
		#endif
	#else
		#define IS_RECEIVER true
		#if F_CPU < 8000000L
			#error Processor Speed too slow for Receiver Node. Consider F_CPU >= 8 MHz
		#endif
	#endif
#else
	// PlatformIO
	#if IS_SENSOR_NODE
		#define IS_RECEIVER false
	#elif IS_RECEIVER
		#define IS_SENSOR_NODE false
	#endif
#endif


/*****************************************************************************/
/***                           Debug Modes                                 ***/
/*****************************************************************************/
// comment the following line for test purposes. If commented, the radio won't be used and 
// a alternative loop function is compiled
#define USE_RADIO

// comment the follwing line if you want to use the radio, but just 
// skip sending a pulse (useful for debugging). Compiler warnings may come up, just ignore.
#define SEND_BURST

// Battery test enables a test message on even counts. the standard message protocol is then
// abused to transmit a 24 bit count in the fileds count, humidity, flags. The temperature field is used to transmit 
// idle voltage, the vcc field is used to transmit idle voltage. 
//#define BATTERYTEST

// enable debugging messages : there are two verbose levels: 1 and 2
#define DEBUG 0

/*****************************************************************************/
/***                           Serial Port                                 ***/
/*****************************************************************************/
// Baud rate of serial port
#if IS_SENSOR_NODE
#define SERIAL_BAUD  57600
#elif IS_RECEIVER
#define SERIAL_BAUD  230400
#endif

/*****************************************************************************/
/***              One-Wire and DS18B20 Temperature Sensors                 ***/
/*****************************************************************************/
// using DS18B20 is not recommended, the conversion time takes much longer
// (700-900 ms) than with I2C based sensors (<50ms).

#define USE_DS18B20


/****************************************************************************/
/**********                    MAX31865   PT100(0)                 **********/
/****************************************************************************/
// comment out the following line for use of a MAX31865 ADC with a PT100 or PT1000

#define USE_MAX31865


/****************************************************************************/
/**********                    MAX6675 k-Type Thermocouple        **********/
/****************************************************************************/
// comment out the following line for use of a k-type Thermocouple with MAX6675
// using this device is not recommended, use MAX31855 instead.
//#define USE_MAX6675



/****************************************************************************/
/**********                    MAX31855 k-Type Thermocouple        **********/
/****************************************************************************/
// comment out the following line for use of a k-type Thermocouple with MAX31855  
//#define USE_MAX31855


/****************************************************************************/
/**********                    MAX31856 Thermocouple               **********/
/****************************************************************************/
// comment out the following line for use of a k-type Thermocouple with MAX31856  
//#define USE_MAX31856


/****************************************************************************/
/**********                    ADS1x20 ADC                         **********/
/****************************************************************************/
// comment out the following line for connecting a thermocouple to the ADS1x20
//#define ADS1x20_THERMOCOUPLE

// comment out the following line for connecting a PT100 to the ADS1x20
//#define ADS1x20_RTD


// comment out the following line for use of ADS1120 (16-bit ADC)
#define USE_ADS1120

// comment out the following line for use of ADS1220 (24-bit ADC)
// #define USE_ADS1220

#if defined USE_ADS1120
// specify whether you want to use the interrupt method (recommended) or the polling method (saves one GPIO)
#define ADS1120_DOSLEEP 1  // 1= use interrupt method, 0 = use polling method
#define ADS1x20_TYPE 0
#elif defined USE_ADS1220
// specify whether you want to use the interrupt method (recommended) or the polling method (saves one GPIO)
#define ADS1120_DOSLEEP 1  // 1= use interrupt method, 0 = use polling method
#define ADS1x20_TYPE 1
#endif

/****************************************************************************/
#if defined USE_MAX31855 || defined USE_MAX31856 || defined USE_MAX6675 || defined ADS1x20_THERMOCOUPLE
# define USE_THERMOCOUPLE_DEVICE
#endif

#if defined USE_MAX31865 || defined ADS1x20_RTD
#define USE_RTD_DEVICE
#endif 
/****************************************************************************/
/**********            FREQUECY HOPPING ON RECEIVER                **********/
/****************************************************************************/
// basically this sketch supports frequency hopping.
// Working, but It needs to be thoroughly tested.
#define NUM_CHANNELS 1