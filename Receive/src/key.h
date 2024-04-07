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
//#define KEY   "TheQuickBrownFox"
#define KEY   "WiNW_AzurdelaMer"
#ifndef KEY
#warning NO ENCRYPTION KEY DEFINED
#define KEY NULL
#endif


/*****************************************************************************/
/***                           Debug Modes                                 ***/
/*****************************************************************************/
// enable debugging messages : there are two verbose levels: 1 and 2
#define DEBUG 0


/*****************************************************************************/
/***                           Serial Port                                 ***/
/*****************************************************************************/
// Baud rate of serial port
#define SERIAL_BAUD 230400

/*****************************************************************************/
/***              One-Wire and DS18B20 Temperature Sensors                 ***/
/*****************************************************************************/

#define USE_DS18B20