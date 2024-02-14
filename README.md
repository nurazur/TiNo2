# TiNo2
[German Version](https://github.com/nurazur/TiNo2/blob/master/LIESMICH.md)

![](https://github.com/nurazur/TiNo2/blob/master/Tino2_logo.png)

"**Ti**ny **No**de": battery powered wireless sensor or wireless actor.
This is the 2nd generation of TiNo. There are numerous technical improvements, mainly more flash memory and further reduction of power consumption. The antenna is now accommodated on the PCB. The project is based on Microchip Avr0 series processors (here the Atmega3208 or atmega4808).

The key points are still the same:

- simplicity
- low cost
- very small size (smaller than a matchbox)
- ultra low sleep current
- long battery life time: 5 years and more on a CR2032 cell
- long range (100m +)
- (even) simple(r) to build up
- communication security
- Plug&Play Firmware

Sensors can be almost any, like temperature, relative humidity, air pressure, altitude meter, light intensity, UV Index, movement detectors (PIR), Reed switches, etc. However sensors have to be specified to work down to 2.2V. Otherwise the charge of the battery can not be fully used.

PCBs fit into low cost PVC boxes with the size of a matchbox that are readily available on the market.  

# Features
## new Features
- internal ceramic antenna
- support for Sensirion humidity sensors SHT4x series
- Microchip CPU Atmega4808 with configurable custom logic, event system and many technical improvements compared to Atmega328P
- 48kB Flash memory (instead of 32kB) allowing one single firmware build for all supported sensors.
- UPDI programming interface (instead of ISP)

## General
- Voltage from ca. 1.8V to 3.6V
- Operates with a CR2032 cell up to 5 years
- various PCBs fitting into selected PVC boxes

## Sensors
- HTU21D
- SHT21, SHT20, SHT25
- SHT30, SHT31, SHT35
- SHTC3 (new)
- SHT40, SHT41, SHT43, SHT45
- BME280 (air pressure sensor)
- DS18B20 (ubiquitous temperature sensor)
- MAX31865 (PT100 temperature Sensor)
- ![](https://github.com/nurazur/TiNo2/blob/e5b521a594be324584e5fc79e4e9750f60dd1295/New_smaller.png) MAX6675, MAX31855K, MAX31856, ADS1120 K-TypeThermocouple Sensors
- AM312 (PIR movement sensor)
- I2C and SPI Bus based sensors can easily be integrated
- brightness using a LDR
- enough digital GPIOs
- enough analog GPIO's

## Radio
- RFM69HCW FSK/OOK Module, RFM95 LoRa Module
- bidirectional communication
- ISM Band (Europe: 433MHz / 868MHz, US:315MHz / 915Mhz)
- 2GFSK Modulation
- Center frequency adjustable
- Frequency correction can be calibrated
- Transmit power from -2 dBm to 20dBm, 10 dBm typ.
- Link budget up to 125dB
- sensitivity -105 dBm typ.
- looong range
- RF communication encrypted
- FEC (Forward Error Correction)
- Interleaver

## Baseband
- Atmel (Microchip) ATMega4808
-  ![](https://github.com/nurazur/TiNo2/blob/e5b521a594be324584e5fc79e4e9750f60dd1295/New_smaller.png)supports Micrpchip AVR64DD28 and AVR64DD28 MCU's
- 48kByte Flash (64kByte Flash on AVR64DD)
- sleep current < 2µA
- 1 MHz clock allows operating voltage down to 1.8V
- 16 MHz clock for gateway receiver
- I2C bus
- SPI
- Serial Port (programming and configuration Port)
- min 12 additional GPIO
- CPU can be locked to avoid reading the flash code

## System / Software
- Open Source Software C++
- Software can easily be adopted individually
- supports Arduino IDE
- ![](https://github.com/nurazur/TiNo2/blob/e5b521a594be324584e5fc79e4e9750f60dd1295/New_smaller.png)supports PLatformIO
- Configuration of Nodes via serial interface (FTDI Adapter)
- configuration data and calibration data reside in EEPROM
- EEPROM encrypted
- Flashing
  - using UPDI Interface
  - serial with SerialUPDI Adapter and Bootloader
- up to 4 external interrupts (i.e. push buttons) can be configured

# How To Install with Arduino IDE
1. Install MegaCoreX
2. Install TiNo2 libraries

## Installation of  MegaCoreX via Boards Manager
* Open Arduino IDE.
* Open the **File > Preferences** menu item.
* Enter the following URL in **Additional Boards Manager URLs**:
    ```
    https://mcudude.github.io/MegaCoreX/package_MCUdude_MegaCoreX_index.json
    ```
* Separate the URLs using a comma ( **,** ) if you have more than one URL
* Open the **Tools > Board > Boards Manager...** menu item.
* Wait for the platform indexes to finish downloading.
* Scroll down until you see the **MegaCoreX** entry and click on it.
* Click **Install**.
* After installation is complete close the **Boards Manager** window.

## MegacoreX Manual Installation
Click on the "Download ZIP" button. Extract the ZIP file, and move the extracted folder to the location "**~/Documents/Arduino/hardware**". Create the "hardware" folder if it doesn't exist.
Open Arduino IDE and a new category in the boards menu called "MegaCoreX" will show up.

## Install TiNo2 package
* Open Arduino IDE.
* Open the **File > Preferences** menu item.
* Enter the following URL in **Additional Boards Manager URLs**:
    ```
    https://raw.githubusercontent.com/nurazur/TiNo2/master/package_nurazur_TiNo2_index.json
    ```
* Separate the URLs using a comma ( **,** ) if you have more than one URL
* Open the **Tools > Board > Boards Manager...** menu item.
* Wait for the platform indexes to finish downloading.
* Scroll down until you see the **nurazur TiNo2 Boards** entry and click on it.
* Click **Install**.
* After installation is complete, close the **Boards Manager** window.

#Burning a Bootloader
