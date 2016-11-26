/**************************************************************************/
/*!
    @file     LIS3DH.cpp
    @author   K. Townsend / Limor Fried (Adafruit Industries)
    @license  BSD (see license.txt)

    This is a library for the Adafruit LIS3DH Accel breakout board
    ----> https://www.adafruit.com/products/2809

    Adafruit invests time and resources providing this open source code,
    please support Adafruit and open-source hardware by purchasing
    products from Adafruit!

    @section  HISTORY

    v1.0  - First release
*/
/**************************************************************************/

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <Wire.h>
#include "LIS3DH.h"


/**************************************************************************/
/*!
    @brief  Instantiates a new LIS3DH class in I2C or SPI mode
*/
/**************************************************************************/
// I2C
LIS3DH LIS3DH::create() { return LIS3DH(); }

#ifndef __AVR_ATtiny85__
LIS3DH LIS3DH::create(int8_t cspin) { return LIS3DH_SPI(cspin); };
LIS3DH LIS3DH::create(int8_t cspin, int8_t mosipin, int8_t misopin, int8_t sckpin)
{ return LIS3DH_SPI( cspin,  mosipin,  misopin,  sckpin); }
#endif

/**************************************************************************/
/*!
    @brief  Setups the HW (reads coefficients values, etc.)
*/
/**************************************************************************/
bool LIS3DH::begin(uint8_t i2caddr) {

  beginSerial(i2caddr);

  /*
  for (uint8_t i=0; i<0x30; i++) {
    Serial.print("$");
    Serial.print(i, HEX); Serial.print(" = 0x");
    Serial.println(readRegister8(i), HEX);
  }
  */

  /* Check connection */
  uint8_t deviceid = readRegister8(LIS3DH_REG_WHOAMI);
  if (deviceid != 0x33)
  {
    /* No LIS3DH detected ... return false */
//    Serial.println(deviceid, HEX);
    return false;
  }

  // enable all axes, normal mode
  writeRegister8(LIS3DH_REG_CTRL1, 0x07);
  // 400Hz rate
  setDataRate(LIS3DH_DATARATE_400_HZ);

  // High res & BDU enabled
  writeRegister8(LIS3DH_REG_CTRL4, 0x88);

  // DRDY on INT1
  writeRegister8(LIS3DH_REG_CTRL3, 0x10);

  // Turn on orientation config
  //writeRegister8(LIS3DH_REG_PL_CFG, 0x40);

  // enable adcs
  // writeRegister8(LIS3DH_REG_TEMPCFG, 0x80);
  
  // enable fifo -> mode stream  
  writeRegister8(LIS3DH_REG_CTRL5, 0x40);
  writeRegister8(LIS3DH_REG_FIFOCTRL, 0x80);

  /*
  for (uint8_t i=0; i<0x30; i++) {
    Serial.print("$");
    Serial.print(i, HEX); Serial.print(" = 0x");
    Serial.println(readRegister8(i), HEX);
  }
  */

  return true;
}


void LIS3DH::read(void) {
  readSerial();
  uint8_t range = getRange();
  uint16_t divider = 1;
  if (range == LIS3DH_RANGE_16_G) divider = 1365; // different sensitivity at 16g
  if (range == LIS3DH_RANGE_8_G) divider = 4096;
  if (range == LIS3DH_RANGE_4_G) divider = 8190;
  if (range == LIS3DH_RANGE_2_G) divider = 16380;

  x_g = (float)x.sens / divider;
  y_g = (float)y.sens / divider;
  z_g = (float)z.sens / divider;

}

/**************************************************************************/
/*!
    @brief  Read the auxilary ADC
*/
/**************************************************************************/

int16_t LIS3DH::readADC(uint8_t adc) {
  if ((adc < 1) || (adc > 3)) return 0;

  adc--;

  uint8_t reg = LIS3DH_REG_OUTADC1_L + adc*2;

  return readADCSerial(reg);
}


/**************************************************************************/
/*!
    @brief  Set INT to output for single or double click
*/
/**************************************************************************/

void LIS3DH::setClick(uint8_t c, uint8_t clickthresh, uint8_t timelimit, uint8_t timelatency, uint8_t timewindow) {
  if (!c) {
    //disable int
    uint8_t r = readRegister8(LIS3DH_REG_CTRL3);
    r &= ~(0x80); // turn off I1_CLICK
    writeRegister8(LIS3DH_REG_CTRL3, r);
    writeRegister8(LIS3DH_REG_CLICKCFG, 0);
    return;
  }
  // else...

  writeRegister8(LIS3DH_REG_CTRL3, 0x80); // turn on int1 click
  writeRegister8(LIS3DH_REG_CTRL5, 0x08); // latch interrupt on int1

  if (c == 1)
    writeRegister8(LIS3DH_REG_CLICKCFG, 0x15); // turn on all axes & singletap
  if (c == 2)
    writeRegister8(LIS3DH_REG_CLICKCFG, 0x2A); // turn on all axes & doubletap


  writeRegister8(LIS3DH_REG_CLICKTHS, clickthresh); // arbitrary
  writeRegister8(LIS3DH_REG_TIMELIMIT, timelimit); // arbitrary
  writeRegister8(LIS3DH_REG_TIMELATENCY, timelatency); // arbitrary
  writeRegister8(LIS3DH_REG_TIMEWINDOW, timewindow); // arbitrary
}

uint8_t LIS3DH::getClick(void) {
  return readRegister8(LIS3DH_REG_CLICKSRC);
}


/**************************************************************************/
/*!
    @brief  Sets the g range for the accelerometer
*/
/**************************************************************************/
void LIS3DH::setRange(lis3dh_range_t range)
{
  uint8_t r = readRegister8(LIS3DH_REG_CTRL4);
  r &= ~(0x30);
  r |= range << 4;
  writeRegister8(LIS3DH_REG_CTRL4, r);
}

/**************************************************************************/
/*!
    @brief  Sets the g range for the accelerometer
*/
/**************************************************************************/
lis3dh_range_t LIS3DH::getRange(void)
{
  /* Read the data format register to preserve bits */
  return (lis3dh_range_t)((readRegister8(LIS3DH_REG_CTRL4) >> 4) & 0x03);
}

/**************************************************************************/
/*!
    @brief  Sets the data rate for the LIS3DH (controls power consumption)
*/
/**************************************************************************/
void LIS3DH::setDataRate(lis3dh_dataRate_t dataRate)
{
  uint8_t ctl1 = readRegister8(LIS3DH_REG_CTRL1);
  ctl1 &= ~(0xF0); // mask off bits
  ctl1 |= (dataRate << 4);
  writeRegister8(LIS3DH_REG_CTRL1, ctl1);
}

/**************************************************************************/
/*!
    @brief  Sets the data rate for the LIS3DH (controls power consumption)
*/
/**************************************************************************/
lis3dh_dataRate_t LIS3DH::getDataRate(void)
{
  return (lis3dh_dataRate_t)((readRegister8(LIS3DH_REG_CTRL1) >> 4)& 0x0F);
}

#ifndef __AVR_ATtiny85__

/**************************************************************************/
/*!
    @brief  Gets the most recent sensor event
*/
/**************************************************************************/
bool LIS3DH::getEvent(sensors_event_t *event) {
  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  event->version   = sizeof(sensors_event_t);
  event->sensor_id = 0;  //_sensorID;
  event->type      = SENSOR_TYPE_ACCELEROMETER;
  event->timestamp = 0;

  read();

  event->acceleration.x = x_g * SENSORS_GRAVITY_STANDARD;
  event->acceleration.y = y_g * SENSORS_GRAVITY_STANDARD;
  event->acceleration.z = z_g * SENSORS_GRAVITY_STANDARD;
}

/**************************************************************************/
/*!
    @brief  Gets the sensor_t data
*/
/**************************************************************************/
void LIS3DH::getSensor(sensor_t *sensor) {
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy (sensor->name, "LIS3DH", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name)- 1] = 0;
  sensor->version     = 1;
  sensor->sensor_id   = 0;//_sensorID;
  sensor->type        = SENSOR_TYPE_ACCELEROMETER;
  sensor->min_delay   = 0;
  sensor->max_value   = 0;
  sensor->min_value   = 0;
  sensor->resolution  = 0;
}

#endif

// NEW DATA AVAILABLE
uint8_t LIS3DH::newDataAvailable() {
  uint8_t status = readRegister8(LIS3DH_REG_STATUS2) & 0x0F;
  return status;
}
// FIFO BUF CHECK
int LIS3DH::arrival() {
  uint8_t fifosrc = readRegister8(LIS3DH_REG_FIFOSRC) & 0x1F;
  return (int)fifosrc; 
}


// LIS3DH_I2C
LIS3DH::LIS3DH() {} 
bool LIS3DH::beginSerial(uint8_t i2caddr) {
  _i2caddr = i2caddr;
    // i2c
  Wire.begin();
  return true;
}
/**************************************************************************/
/*!
    @brief  Reads 8-bits from the specified register
*/
/**************************************************************************/
uint8_t LIS3DH::readRegister8(uint8_t reg) {
  uint8_t value;

  Wire.beginTransmission(_i2caddr);
  Wire.write((uint8_t)reg);
  Wire.endTransmission(false);

  Wire.requestFrom(_i2caddr, 1);
  value = Wire.read();

  return value;
}
/**************************************************************************/
/*!
    @brief  Writes 8-bits to the specified destination register
*/
/**************************************************************************/
void LIS3DH::writeRegister8(uint8_t reg, uint8_t value) {
    Wire.beginTransmission((uint8_t)_i2caddr);
    Wire.write((uint8_t)reg);
    Wire.write((uint8_t)value);
    Wire.endTransmission();
}
void LIS3DH::readSerial(void) {
  // read x y z at once

    // i2c
    Wire.beginTransmission(_i2caddr);
    Wire.write(LIS3DH_REG_OUT_X_L | 0x80); // 0x80 for autoincrement
    Wire.endTransmission(false);

    Wire.requestFrom(_i2caddr, 6);
    x.sens = Wire.read(); x.sens |= ((uint16_t)Wire.read()) << 8;
    y.sens = Wire.read(); y.sens |= ((uint16_t)Wire.read()) << 8;
    z.sens = Wire.read(); z.sens |= ((uint16_t)Wire.read()) << 8;

}

/**************************************************************************/
/*!
    @brief  Read the auxilary ADC
*/
/**************************************************************************/

int16_t LIS3DH::readADCSerial(uint8_t reg) {

  uint16_t value;

  // i2c
  Wire.beginTransmission(_i2caddr);
  Wire.write(reg | 0x80);   // 0x80 for autoincrement
  Wire.endTransmission();
  Wire.requestFrom(_i2caddr, 2);
  value = Wire.read();  value |= ((uint16_t)Wire.read()) << 8;
  return value;
}




#ifndef __AVR_ATtiny85__
// LIS3DH_SPI
LIS3DH_SPI::LIS3DH_SPI(int8_t cspin)
  : _cs(cspin), _mosi(-1), _miso(-1), _sck(-1)
{ }


LIS3DH_SPI::LIS3DH_SPI(int8_t cspin, int8_t mosipin, int8_t misopin, int8_t sckpin)
  : _cs(cspin), _mosi(mosipin), _miso(misopin), _sck(sckpin)
{ }

bool LIS3DH_SPI::beginSerial(uint8_t i2caddr) {

  digitalWrite(_cs, HIGH);
  pinMode(_cs, OUTPUT);

  if (_sck == -1) {
    // hardware SPI
    SPI.begin();
  } else {
    // software SPI
    pinMode(_sck, OUTPUT);
    pinMode(_mosi, OUTPUT);
    pinMode(_miso, INPUT);
  }
  return true;
}
/**************************************************************************/
/*!
    @brief  Reads 8-bits from the specified register
*/
/**************************************************************************/
uint8_t LIS3DH_SPI::readRegister8(uint8_t reg) {
  uint8_t value;

  if (_sck == -1)
    SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
  digitalWrite(_cs, LOW);
  spixfer(reg | 0x80); // read, bit 7 high
  value = spixfer(0);
  digitalWrite(_cs, HIGH);
  if (_sck == -1)
    SPI.endTransaction();              // release the SPI bus
  
  return value;
}
/**************************************************************************/
/*!
    @brief  Writes 8-bits to the specified destination register
*/
/**************************************************************************/
void LIS3DH_SPI::writeRegister8(uint8_t reg, uint8_t value) {
  if (_sck == -1)
    SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
  digitalWrite(_cs, LOW);
  spixfer(reg & ~0x80); // write, bit 7 low
  spixfer(value);
  digitalWrite(_cs, HIGH);
  if (_sck == -1)
    SPI.endTransaction();              // release the SPI bus
}
/**************************************************************************/
/*!
    @brief  Low level SPI
*/
/**************************************************************************/

uint8_t LIS3DH_SPI::spixfer(uint8_t x) {
  #ifndef __AVR_ATtiny85__
  if (_sck == -1)
    return SPI.transfer(x);

  // software spi
  //Serial.println("Software SPI");
  uint8_t reply = 0;
  for (int i=7; i>=0; i--) {
    reply <<= 1;
    digitalWrite(_sck, LOW);
    digitalWrite(_mosi, x & (1<<i));
    digitalWrite(_sck, HIGH);
    if (digitalRead(_miso))
      reply |= 1;
  }
  return reply;
  #endif
}
void LIS3DH_SPI::readSerial(void) {
  // read x y z at once

    if (_sck == -1)
      SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
    digitalWrite(_cs, LOW);
    spixfer(LIS3DH_REG_OUT_X_L | 0x80 | 0x40); // read multiple, bit 7&6 high

    x.sens = spixfer(); x.sens |= ((uint16_t)spixfer()) << 8;
    y.sens = spixfer(); y.sens |= ((uint16_t)spixfer()) << 8;
    z.sens = spixfer(); z.sens |= ((uint16_t)spixfer()) << 8;

    digitalWrite(_cs, HIGH);
    if (_sck == -1)
      SPI.endTransaction();              // release the SPI bus


}

/**************************************************************************/
/*!
    @brief  Read the auxilary ADC
*/
/**************************************************************************/

int16_t LIS3DH_SPI::readADCSerial(uint8_t reg) {
  uint16_t value;
    if (_sck == -1)
      SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
    digitalWrite(_cs, LOW);
    spixfer(reg | 0x80 | 0x40); // read multiple, bit 7&6 high

    value = spixfer(); value |= ((uint16_t)spixfer()) << 8;

    digitalWrite(_cs, HIGH);
    if (_sck == -1)
      SPI.endTransaction();              // release the SPI bus

  return value;
}


#endif
