//
#include "tino_max31856.h"
#include <stdlib.h>
#include <SPI.h>

//static const SPISettings settings = SPISettings(1000000, MSBFIRST, SPI_MODE1);
//static const uint16_t T_CONV_60Hz_ms = 155;
//static const uint16_t T_CONV_50Hz_ms = 185;

#define T_CONV_60Hz_ms 155
#define T_CONV_50Hz_ms 185
/**
    @brief  Basic constructor
    @param  cs [in]: pin used for CS signal
    @retval None
*/
Tino_MAX31856::Tino_MAX31856(const int8_t cs) : _cs(cs), 
_sck(-1), _miso(-1), _mosi(-1), _tc_type(MAX31856_TC_TYPE_K) {
}

/**
    @brief  Constructor with thermocouple definition
    @param  cs [in]: pin used for CS signal
    @param  tc [in]: thremocouple type
    @retval None
*/
Tino_MAX31856::Tino_MAX31856(const int8_t cs, const MAX31856_TCTypeT tc) :
  _cs(cs), _sck(-1), _miso(-1), _mosi(-1), _tc_type(tc) {
}

/**
    @brief  Constructor for software SPI
    @param  cs [in]: pin used for CS signal
    @param  mosi [in]: pin used for MOSI signal
    @param  miso [in]: pin used for MISO signal
    @param  sck [in]: pin used for SCK signal
    @retval None
*/
Tino_MAX31856::Tino_MAX31856(const int8_t cs, const int8_t mosi, const int8_t miso, const int8_t sck) 
:
_cs(cs), _sck(sck), _miso(miso), _mosi(mosi),  _tc_type(MAX31856_TC_TYPE_K) 
{  
}

/**
    @brief  Constructor for software SPI with thermocouple definition
    @param  cs [in]: pin used for CS signal
    @param  mosi [in]: pin used for MOSI signal
    @param  miso [in]: pin used for MISO signal
    @param  sck [in]: pin used for SCK signal
    @param  tc [in]: thremocouple type
    @retval None
*/
Tino_MAX31856::Tino_MAX31856(const int8_t cs, const int8_t mosi,
  const int8_t miso, const int8_t sck, const MAX31856_TCTypeT tc) :
  _cs(cs), _sck(sck), _miso(miso), _mosi(mosi), _tc_type(tc) {
}

/**
    @brief  Hardware configuration
    @param  None
    @retval None

void Tino_MAX31856::begin(void) {
  pinMode(_cs, OUTPUT);
  digitalWrite(_cs, HIGH);

  if (_sck != -1) {
    pinMode(_sck, OUTPUT);
    pinMode(_mosi, OUTPUT);
    pinMode(_miso, INPUT);
  } else {
    SPI.begin();
  }
  write(MAX31856_REG_CR0, 0);
  setThermocoupleType(_tc_type);
}
*/

/**
    @brief  Hardware configuration 2
    @param  None
    @retval None
*/
/*
uint8_t Tino_MAX31856::begin(uint8_t reg_cr0) 
{
  pinMode(_cs, OUTPUT);
  digitalWrite(_cs, HIGH);

  SPI.begin();

  write(MAX31856_REG_CR0, reg_cr0);
  setThermocoupleType(_tc_type);
  return read(MAX31856_REG_CR0);
}
*/


void Tino_MAX31856::begin(uint8_t reg_cr0)
{
  pinMode(_cs, OUTPUT);
  digitalWrite(_cs, HIGH);

  //SPI.begin();
  write(MAX31856_REG_CR0, reg_cr0);
  setThermocoupleType(_tc_type);
}



/**
    @brief  Setting thermocouple type
    @param  tc [in]: thremocouple type
    @retval None
*/
void Tino_MAX31856::setThermocoupleType(const MAX31856_TCTypeT tc) {
  uint8_t CR1 = read(MAX31856_REG_CR1) & 0xF0;
  CR1 |= (uint8_t)tc;
  write(MAX31856_REG_CR1, CR1);
}

/**
    @brief  Getting thermocouple type
    @param  None
    @retval Thermocouple type
*/
MAX31856_TCTypeT Tino_MAX31856::getThermocoupleType(void) {
  MAX31856_TCTypeT x = (MAX31856_TCTypeT)(read(MAX31856_REG_CR1) & 0x0F);
  return x;
}


uint8_t Tino_MAX31856::conversionComplete(void)
{
    if (getConversionMode() == MAX31856_ConversionMode_Auto)
        return 1;
    return !(read(MAX31856_REG_CR0) & MAX31856_REG_CR0_1SHOT);
}


/**
    @brief  Execute temperature conversion
    @param  None
    @retval None
    @note   Blocks until the end of conversion with timeout 250ms
*/
void Tino_MAX31856::convert(void) 
{
    // Trigger 1SHOT  conversion
    uint8_t CR0 = read(MAX31856_REG_CR0);
    CR0 &= ~MAX31856_REG_CR0_AUTOCONVERT;
    CR0 |= MAX31856_REG_CR0_1SHOT;
    write(MAX31856_REG_CR0, CR0);

    uint32_t start = millis();
    while (!conversionComplete()) 
    {
      if (millis() - start > 250)
        return NAN;
      delay(10);
    }
}



/**
    @brief  Reads hot junction temperature
    @param  None
    @retval Hot junction temperature in Celsius degrees
*/
float Tino_MAX31856::readThermocouple(void) {
  // Read Linearized TC temperature registers
  uint8_t buf[3];
  readMultiple(MAX31856_REG_LTCBH, buf, 3);

  // Converting linearized TC temperature code to real temperature
  int32_t temp_code = ((int32_t)buf[0] << 16) | ((int32_t)buf[1] << 8) | ((int32_t)buf[2] & 0xE0);
  if (temp_code & 0x800000) {
    temp_code |= 0xFF000000;
  }
  temp_code >>= 5;
  float temperature = (float) temp_code / (1 << 7);

  return temperature;
}

/**
    @brief  Reads cold junction temperature
    @param  None
    @retval Cold junction temperature in Celsius degrees
*/
float Tino_MAX31856::readColdJunction(void) {
  // Reading cold-junction temperature registers
  uint8_t buf[2];
  readMultiple(MAX31856_REG_CJTH, buf, 2);

  // Converting code to real temperature
  int16_t temp_code = ((int16_t)buf[0] << 8) | (int16_t)buf[1];
  float temperature = (float)temp_code / (1 << 8);

  return temperature;
}

/**
    @brief  Sets thermocouple voltage conversion averaging mode
    @param  avgMask[in] : number of samples to be averaged during conversion
    @retval None
    @note   Averaging mode can't be changed during conversion
*/
void Tino_MAX31856::setAvergingMode(const MAX31856_AVGSEL_MaskT avgMask) {
  // Disable automatic conversion for changing averaging mode
  MAX31856_ConversionModeT conversionMode = getConversionMode();
  if (MAX31856_ConversionMode_Auto == conversionMode) {
    setConversionMode(MAX31856_ConversionMode_NormOff);
  }

  // Set new averaging mode
  uint8_t CR1 = read(MAX31856_REG_CR1);
  CR1 &= 0x8F;
  CR1 |= (uint8_t)avgMask;
  write(MAX31856_REG_CR1, CR1);

  // Restore conversion mode
  if (MAX31856_ConversionMode_Auto == conversionMode) {
    setConversionMode(conversionMode);
  }
}

/**
    @brief  Gets thermocouple voltage conversion averaging mode
    @param  None
    @retval Number of samples used in one conversion
*/
uint8_t Tino_MAX31856::getAvergingMode(void) {
  // Read the bitfield
  uint8_t avgMode = (read(MAX31856_REG_CR1) >> 4) & 0x07;

  // Calculate the number of samples
  uint8_t nSamples = 0;
  if (avgMode > 3) {
    nSamples = 16;
  } else {
    nSamples = 1 << avgMode;
  }
  return nSamples;
}

/**
    @brief  Sets noise rejection filter
    @param  filter[in] : filter option
    @retval None
*/
void Tino_MAX31856::setNoiseFilter(const MAX31856_FilterT filter) {
  uint8_t CR0 = read(MAX31856_REG_CR0);
  CR0 &= ~0x01;

  // Set flag for 50Hz filter
  if (MAX31856_NoiseFilter50Hz == filter) {
    CR0 |= 0x01;
  }

  write(MAX31856_REG_CR0, CR0);
}

/**
    @brief  Gets noise rejection filter option
    @param  None
    @retval Noise rejection filter option
*/
MAX31856_FilterT Tino_MAX31856::getNoiseFilter(void) {
  uint8_t filterFlag = read(MAX31856_REG_CR0) & 0x01;

  return filterFlag ? MAX31856_NoiseFilter50Hz : MAX31856_NoiseFilter60Hz;
}


/**
    @brief  Reads fault register
    @param  None
    @retval Fault register value
*/
uint8_t Tino_MAX31856::readFault(void) {
  return read(MAX31856_REG_SR);
}

/**
    @brief  Sets the fault detection range for the thermocouple
    @param  low[in] : low border, Celsius degrees
    @param  high[in] : high border, Celsius degrees
    @retval None
*/
void Tino_MAX31856::setThermocoupleRange(const float low, const float high) {
  uint8_t buf[2];
  uint8_t sign = 0;

  // Setting low value
  float absVal = low;

  // Storing sign
  if (low < 0) {
    sign = 0x80;
    absVal = -low;
  }

  // Converting to register values
  buf[0] = (uint8_t)(absVal / (1 << 4)) & ~0x80; // MSB
  buf[0] |= sign; // Restoring sign
  buf[1] = (uint8_t)(absVal * (1 << 4)); // LSB

  // Writing low threshold
  writeMultiple(MAX31856_REG_LTLFTH, buf, 2);

  // Setting high value
  absVal = high;

  // Storing sign
  if (high < 0) {
    sign = 0x80;
    absVal = -high;
  }

  // Converting to register values
  buf[0] = (uint8_t)(absVal / (1 << 4)) & ~0x80; // MSB
  buf[0] |= sign; // Restoring sign
  buf[1] = (uint8_t)(absVal * (1 << 4)); // LSB

  // Writing high threshold
  writeMultiple(MAX31856_REG_LTHFTH, buf, 2);
}

/**
    @brief  Sets fault detection range for the cold junction
    @param  low[in] : low border, Celsius degrees
    @param  high[in] : high border, Celsius degrees
    @retval None
*/
void Tino_MAX31856::setColdJunctionRange(const int8_t low, const int8_t high) {
  write(MAX31856_REG_CJLF, (uint8_t)low);
  write(MAX31856_REG_CJHF, (uint8_t)high);
}

/**
    @brief  Sets the cold junction temperature offset
    @param  offset[in] : cold junction temperature offset [-8...7,9375], Celsius degrees
    @retval None
*/
void Tino_MAX31856::setColdJunctionOffset(const float offset) {
  float absVal = offset;
  uint8_t sign = 0;
  uint8_t regVal = 0;
  
  // Storing sign
  if (offset < 0) {
    sign = 0x80;
    absVal = -offset;
  }

  // Converting to register value
  regVal = (uint8_t)(absVal * (1 << 4)) & ~0x80;
  
  // Restoring sign
  regVal |= sign; 


  // Writing offset to register
  write(MAX31856_REG_CJTO, regVal);
}

/**
    @brief  Sets the cold junction temperature (for external sensor)
    @param  temperature[in] : cold junction temperature [-128, 127.98438], Celsius degrees
    @retval None
*/
void Tino_MAX31856::setColdJunctionTemperature(const float temperature) {
  uint8_t buf[2];
  uint8_t sign = 0;
  float absVal = temperature;

  // Storing sign
  if (temperature < 0) {
    sign = 0x80;
    absVal = -temperature;
  }

  // Converting to register values
  buf[0] = (uint8_t)(absVal) & ~0x80; // MSB
  buf[0] |= sign; // Restoring sign
  buf[1] = (uint8_t)(absVal * (1 << 8)) & 0xFC; // LSB

  // Writing offset tempreature
  writeMultiple(MAX31856_REG_CJTH, buf, 2);
}

/**
    @brief  Sets the conversion mode
    @param  mode[in] : conversion mode to be set
    @retval None
*/
void Tino_MAX31856::setConversionMode(const MAX31856_ConversionModeT mode) {
  uint8_t CR0 = read(MAX31856_REG_CR0);
  CR0 &= ~MAX31856_ConversionMode_Auto;
  CR0 |= mode;
  write(MAX31856_REG_CR0, CR0);
}

/**
    @brief  Gets the current conversion mode
    @param  None
    @retval Current conversion mode
*/
MAX31856_ConversionModeT Tino_MAX31856::getConversionMode(void) {
  return (MAX31856_ConversionModeT) (MAX31856_ConversionMode_Auto & read(MAX31856_REG_CR0));
}

/**
    @brief  Enables/disables cold junction temperature sensor
    @param  command[in] : new state
    @retval None
*/
void Tino_MAX31856::setColdJunctionEnable(MAX31856_ColdJunctionStateT state) {
  uint8_t CR0 = read(MAX31856_REG_CR0);
  CR0 &= ~MAX31856_ColdJunctionState_Disabled;
  CR0 |= state;
  write(MAX31856_REG_CR0, CR0);
}

/**
    @brief  Sets fault mode
    @param  mode[in] : new mode
    @retval None
*/
void Tino_MAX31856::setFaultMode(MAX31856_FaultModeT mode) {
  uint8_t CR0 = read(MAX31856_REG_CR0);
  CR0 &= ~MAX31856_FaultMode_Interrupt;
  CR0 |= mode;
  write(MAX31856_REG_CR0, CR0);
}

/**
    @brief  Clears fault flags
    @param  None
    @retval None
*/
void Tino_MAX31856::clearFaults(void) {
  uint8_t CR0 = read(MAX31856_REG_CR0);
  CR0 |= MAX31856_REG_CR0_FAULTCLR;
  write(MAX31856_REG_CR0, CR0);
}


/**
    @brief  Sets the open-circuit detection mode
    @param  mode[in] : open-ciruit detection mode to be set
    @retval None
*/
void Tino_MAX31856::setOCDetectionMode(const MAX31856_OCModeT mode) {
  uint8_t CR0 = read(MAX31856_REG_CR0);
  CR0 &= ~MAX31856_OCMode_100ms;
  CR0 |= mode;
  write(MAX31856_REG_CR0, CR0);
}

/**
    @brief  Gets the open-circuit detection mode
    @param  None
    @retval Current open-circuit detection mode
*/
MAX31856_OCModeT Tino_MAX31856::getOCDetectionMode(void) {
  return (MAX31856_OCModeT) (MAX31856_OCMode_100ms & read(MAX31856_REG_CR0));
}

//------------------------------ Private functions ----------------------------
/**
    @brief  Read MAX31856 register
    @param  address [in]: register address
    @retval Register value
*/
uint8_t Tino_MAX31856::read(const MAX31856_addressT address) {
  uint8_t value;
  readMultiple(address, &value, 1);
  return value;
}

/**
    @brief  Read MAX31856 multiple registers
    @param  address [in]: register address
    @param  rx_buf [out]: receiving buffer pointer
    @param  size [in]: number of bytes to read
    @retval None
*/
void Tino_MAX31856::readMultiple(const MAX31856_addressT address, uint8_t* const rx_buf, const uint8_t size) {
  /*
  //Start Transfer, Open Connection
  if (_sck == -1) {
    SPI.beginTransaction(settings);
  } else {
    digitalWrite(_sck, HIGH);
  }
  */
  digitalWrite(_cs, LOW);

  // Send address and read specified number of bytes
  transfer(address);
  for (uint8_t i = 0; i < size; ++i)
  {
    rx_buf[i] = transfer(0xFF);
  }

  /*
  //End Transfer, Close Connection for other programs to use.
  if (_sck == -1) {
    SPI.endTransaction();
  }
  */
  digitalWrite(_cs, HIGH);
}

/**
    @brief  Write MAX31856 register
    @param  address [in]: register address
    @param  value [in]: register value
    @retval None
*/
void Tino_MAX31856::write(const MAX31856_addressT address, const uint8_t value) {
  writeMultiple(address, &value, 1);
}

/**
    @brief  Write MAX31856 register
    @param  address [in]: register address
    @param  tx_buf [in]: data buffer pointer
    @param  size [in]: number of bytes to write
    @retval None
*/
void Tino_MAX31856::writeMultiple(const MAX31856_addressT address, 
  const uint8_t* const tx_buf, const uint8_t size) {

  /*
  //Start Transfer, Open Connection
  if (_sck == -1) {
    SPI.beginTransaction(settings);
  } else {
    digitalWrite(_sck, HIGH);
  }
  */
  digitalWrite(_cs, LOW);

  // Send address and specified number of bytes
  transfer(address | 0x80);
  for (uint8_t i = 0; i < size; ++i)
  {
    transfer(tx_buf[i]);
  }

  /*
  //End Transfer, Close Connection for other programs to use.
  if (_sck == -1) {
    SPI.endTransaction();
  }
  */

  digitalWrite(_cs, HIGH);
}

/**
    @brief  SPI byte transfer
    @param  val [in]: byte to transfer
    @retval Received byte
*/
uint8_t Tino_MAX31856::transfer(const uint8_t val) {

  //If Hardware SPI, use default SPI class.
  if (_sck == -1)
    return SPI.transfer(val);

  //If Software SPI, bitbang our way to glory.
  uint8_t out = 0;
  uint8_t bval = 0;
  for (uint8_t bit = 0; bit < 8; bit++) {
    digitalWrite(_sck, HIGH);
    bval = digitalRead(_miso);
    out >>= 1;
    out |= bval << 7;
    digitalWrite(_sck, LOW);
    digitalWrite(_mosi, val & (1 << bit) ? HIGH : LOW);
  }
  return out;
}


uint8_t TC_56::Measure(uint8_t enable, float* temp)
{
    uint8_t status=0xff;
    if (enable)
    {
        SPI.begin();
        SPI.setDataMode(SPI_MODE1);
        pinMode(this->power_pin,OUTPUT);
        digitalWrite(this->power_pin, 1);
        delay(1);
        
        this->begin();
        delay(1);
        
        this->setNoiseFilter(MAX31856_NoiseFilter50Hz);
        //this->setOCDetectionMode(MAX31856_OCMode_100ms); // does not work properly

        this->convert();
        
        temp[THERMOCOUPLE_TEMPERATURE] = this->readThermocouple();
        temp[COLD_JUNCTION_TEMPERATURE] = this->readColdJunction();
        status = this->readFault();
        //status = this->read(MAX31856_REG_CR0);
        //digitalWrite(power_pin, 0);
    }
    return status;
}


//Tino_MAX31856 *TC56 = NULL;
TC_56 *TC56 = NULL;

void ThermoCouple_Sleep_56(bool enable)
{
    if (TC56 && enable)
        digitalWrite(TC56->power_pin, LOW); // turn Sensor off to save power
}


void ThermoCouple_Init_56(uint8_t enable, uint8_t PowerPin, uint8_t CsPin)
{
    if (enable)
    {
        pinMode(CsPin, OUTPUT);  // SPI SS
        digitalWrite(CsPin, 1);
        TC56 = new TC_56(PowerPin, CsPin, MAX31856_TC_TYPE_K);
    }
}

uint8_t ThermoCouple_Measure_56(bool enable, float* temp)
{
    uint8_t status=0xff;
    if (TC56 && enable)
    {
        SPI.begin();
        SPI.setDataMode(SPI_MODE1);
        pinMode(TC56->power_pin,OUTPUT);
        digitalWrite(TC56->power_pin, 1);
        delay(1);
        
        TC56->begin();
        delay(1);
        
        TC56->setNoiseFilter(MAX31856_NoiseFilter50Hz);
        //TC56->setOCDetectionMode(MAX31856_OCMode_100ms); // does not work properly

        TC56->convert();
        
        temp[THERMOCOUPLE_TEMPERATURE] = TC56->readThermocouple();
        temp[COLD_JUNCTION_TEMPERATURE] = TC56->readColdJunction();
        status = TC56->readFault();
        //status = TC56->read(MAX31856_REG_CR0);
        //digitalWrite(TC56->power_pin, 0);
    }
    return status;
}

void ThermoCouple_Convert_56(bool enable)
{
    if (TC56 && enable)
    {
        SPI.begin();
        SPI.setDataMode(SPI_MODE1);
        pinMode(TC56->power_pin,OUTPUT);
        digitalWrite(TC56->power_pin, 1);
        delay(1);
        
        TC56->begin();
        delay(1);
        
        TC56->setNoiseFilter(MAX31856_NoiseFilter50Hz);
        //TC56->setOCDetectionMode(MAX31856_OCMode_100ms); // does not work properly

        // Trigger 1SHOT  conversion
        uint8_t CR0 = TC56->read(MAX31856_REG_CR0);
        CR0 &= ~MAX31856_REG_CR0_AUTOCONVERT;
        CR0 |= MAX31856_REG_CR0_1SHOT;
        TC56->write(MAX31856_REG_CR0, CR0);
        /*
        uint32_t start = millis();
        while (!TC56->conversionComplete()) 
        {
          if (millis() - start > 250)
            break;
          delay(10);
        }
        */
    }
}

uint8_t ThermoCouple_GetTemperatures(bool enable, float* temp)
{
    uint8_t status=0xff;
    if (TC56 && enable)
    {
        temp[THERMOCOUPLE_TEMPERATURE] = TC56->readThermocouple();
        temp[COLD_JUNCTION_TEMPERATURE] = TC56->readColdJunction();
        status = TC56->readFault();
    }
    return status;
}
