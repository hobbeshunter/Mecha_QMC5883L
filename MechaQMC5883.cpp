#include "MechaQMC5883.h"

#include <Wire.h>

void MechaQMC5883::setAddress(uint8_t addr)
{
  address = addr;
}

void MechaQMC5883::writeReg(byte reg, byte val)
{
  Wire.beginTransmission(address); //start talking
  Wire.write(reg);                 // Choose register
  Wire.write(val);                 // Set the register
  Wire.endTransmission();
}

void MechaQMC5883::init()
{
  writeReg(0x0B, 0x01);
  //Define Set/Reset period
  setMode(Mode_Continuous, ODR_200Hz, RNG_8G, OSR_512);
  /*
  Define
  OSR = 512
  Full Scale Range = 8G(Gauss)
  ODR = 200HZ
  set continuous measurement mode
  */
}

void MechaQMC5883::setMode(uint16_t mode, uint16_t odr, uint16_t rng, uint16_t osr)
{
  writeReg(0x09, mode | odr | rng | osr);

  switch (rng)
  {
  case RNG_2G:
    lsb_to_g = RNG_2G_LSB_TO_G;
    break;
  case RNG_8G:
    lsb_to_g = RNG_8G_LSB_TO_G;
    break;
  }
}

void MechaQMC5883::softReset()
{
  writeReg(0x0A, 0x80);
}

/**
 * read values from device
 * @return status value:
 *  - 0:success
 *  - 1:data too long to fit in transmit buffer
 *  - 2:received NACK on transmit of address
 *  - 3:received NACK on transmit of data
 *  - 4:other error
 *  - 8:overflow (magnetic field too strong)
 */
int MechaQMC5883::readRaw(int16_t &x, int16_t &y, int16_t &z)
{
  Wire.beginTransmission(address);
  Wire.write(0x00);
  int err = Wire.endTransmission();
  if (err)
    return err;

  Wire.requestFrom(address, 6);
  x = Wire.read();       //LSB  x
  x |= Wire.read() << 8; //MSB  x
  y = Wire.read();       //LSB  z
  y |= Wire.read() << 8; //MSB z
  z = Wire.read();       //LSB y
  z |= Wire.read() << 8; //MSB y

  byte overflow = Wire.read() & 0x02;
  return overflow << 2;
}

int MechaQMC5883::read(float &x, float &y, float &z)
{
  int16_t ix, iy, iz;
  int err = readRaw(ix, iy, iz);
  x = (float)-iy * lsb_to_g;
  y = (float)-ix * lsb_to_g;
  z = (float)iz * lsb_to_g;

  return err;
}
