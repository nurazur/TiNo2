# TiNo2
[German Version](https://github.com/nurazur/TiNo/blob/master/LIESMICH.md)

![](https://github.com/nurazur/TiNo/blob/master/matchbox.jpg)

"**TI**ny **NO**de": battery powered wireless sensor or wireless actor.
This is the second generation of TiNo. There are numerous technical improvements, mainly more flash memory and further reduction of power consumption. The antenna is now accommodated on the PCB. The project is based on Microchip Avr0 series processors (here the Atmega3208 or atmega4808).

The key points are still the same:

- low cost
- very small size (smaller than a matchbox)
- ultra low sleep current
- long battery life time: 5 years and more on a CR2032 cell
- long range (what ever this means :-), but its realy long)
- (even) simple(r) to build up
- communication security
- Plug&Play Firmware

Sensors can be almost any, like temperature, relative humidity, air pressure, altitude meter, light intensity, UV Index, movement detectors, Reed switches, etc. However sensors have to be specified to work down to 2.2V. Otherwise the charge of the battery can not be fully used.

PCBs fit into low cost PVC boxes with the size of a matchbox that are readily available on the market.  


# Features
## General
- Voltage from ca. 1.8V to 3.6V
- Operates with a CR2032 cell up to 5 years
- various PCBs fitting into selected PVC boxes

## Sensors
- HTU21D
- SHT21, SHT20, SHT25
- SHT30, SHT31, SHT35
- SHTC3
- SHT40, SHT41, SHT43, SHT45
- BME280 (air pressure sensor)
- DS18B20 (ubiquitous temperature sensor)
- MAX31865 (PT100 temperature Sensor)
- AM312 (PIR movement sensor)
- I2C and SPI Bus based sensors can easily be integrated
- brightness using a LDR
- enough digital GPIOs
- enough analog GPIO's

## Radio
- RFM69CW, RFM69HCW, RFM95 Module
- bidirectional communication
- ISM Band (Europe: 433MHz / 868MHz, US:315MHz / 915Mhz)
- 2GFSK Modulation
- Center frequency adjustable
- Frequency correction can be calibrated
- Transmit power from -18 dBm to 20dBm, 10 dBm typ.
- Link budget up to 120dB
- sensitivity -105 dBm typ.
- looong range
- RF communication encrypted
- FEC (Forward Error Correction)
- Interleaver

## Baseband
- Atmel (Microchip) ATMega4808
- 48kByte Flash
- sleep current < 2µA
- 1 MHz clock allows operating voltage down to 1.8V
- 16 MHz clock for gateway receiver
- I2C bus
- min 12 additional GPIO
- CPU can be locked to avoid reading the flash code

## System / Software
- Open Source Software C++
- Software can easily be adopted individually
- Programming tool is Arduino IDE
- Configuration of Nodes via serial interface (FTDI Adapter)
- configuration data and calibration data reside in EEPROM
- EEPROM encrypted
- Flashing
  - using UPDI Interface
  - serial with SerialUPDI Adapter and Bootloader
- up to 4 external interrupts (i.e. push buttons) can be configured
