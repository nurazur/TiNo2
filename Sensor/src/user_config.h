/*****************************************************************************/
/***                            Encryption                                 ***/
/*****************************************************************************/
// encryption is OPTIONAL by compilation switch
// encryption will encrypt the RF packages and the EEPROM content.
// to enable encryption you will need to:
//  - provide a 16-byte encryption KEY (same on all nodes that talk encrypted)
//  - set the varable ENCRYPTION_ENABLE to 1 in the EEPROM, at runtime in Cfg.EncryptionEnable

// comment the following line if you don't want to encrypt RF message traffic
// it is STRONGLY recommended to change de Key below to your own 16-Byte Password
#define KEY   "TheQuickBrownFox"
#ifndef KEY
#warning NO ENCRYPTION KEY DEFINED
#define KEY NULL
#endif

/*****************************************************************************/
/***                           Debug Modes                                 ***/
/*****************************************************************************/
// comment the following line for test purposes. If commented, the radio won't be used and 
// a alternative loop function is compiled
#define USE_RADIO

// comment the follwing line if you want to use the radio, but just 
// skip sending a pulse (useful for debugging)
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
#define SERIAL_BAUD  57600


/*****************************************************************************/
/***              One-Wire and DS18B20 Temperature Sensors                 ***/
/*****************************************************************************/
// using DS18B20 is not recommended, the conversion time takes much longer
// (700-900 ms) than with I2C based sensors (<50ms).

//#define USE_DS18B20


/****************************************************************************/
/**********                    MAX31865   PT100(0)                 **********/
/****************************************************************************/
// comment out the following line for use of a MAX31865 ADC with a PT100 or PT1000

//#define USE_MAX31865


/****************************************************************************/
/**********                    MAX31855 k-Type Thermocouple        **********/
/****************************************************************************/
// comment out the following line for use of a k-type Thermocouple with MAX31855  

//#define USE_MAX31855


/****************************************************************************/
/**********                    ADS1120 ADC                         **********/
/****************************************************************************/
// comment out the following line for use of a k-type Thermocouple with ADS1120 
#define USE_ADS1120

// specify whether you want to use the interrupt method (recommended) or the polling method (saves one GPIO)
#if defined USE_ADS1120
#define ADS1120_DOSLEEP 1  // 1= use sleep method, 0 = use polling method
#endif