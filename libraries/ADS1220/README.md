# ADS1x20
C++ Arduiono library for the Texas Instruments ADS1120 and ADS1220 ADC. Special extentions for thermocouple and PT100 measurements are implemented. The library favours a interrupt mode that allows to sleep the CPU during conversion time. This produces considerable power savings for battery operated applications like the TiNo2.
# Library
**ADS1220.h and ADS1220.cpp** provide the basic functions required to control the ADS1120 or ADS1220. The architecture of these two devices is identical except for the ADC section. The ADS1120 is a 16-bit ADC, while the ADS1220 offers a 24-bit ADC. Therefore, they are software-compatible, except when reading samples: the ADS1120 outputs 16-bit samples, whereas the ADS1220 outputs 24-bit samples. 

**tino_ads1220.h, tino_ads1220.cpp and PT100_common.h** provide wrapper functions and classes that simplify the implementation of temperature measurements using PT100 sensors or thermocouples. 

# Examples
**Arduino_ADS1120.ino** runs on Arduino boards such as the Uno, Nano or Pro Mini. It demonstrates the use of the ADS1220 class for precise PT100 and K-type thermocouple temperature measurements.

**TiNo2_ADS1120.ino** runs in the TiNo2 ecosystem (MEGACOREX) and demonstrates the use of the wrapper classes. This example includes specific low power modes targeting extreme low power consumption during idle time.


