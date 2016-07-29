/*
 * File: bmp085.cpp
 * Author: Craig Hollinger
 *
 * Description:
 *
 * Firmware to read data from the Bosch BMP085 pressure sensor.  The formulas
 * used below for calculating the temperature and pressure from the raw data
 * are taken directly from the BMP085 data sheet.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */
#include <BMP085.h>
#include <WIRE.H>

// This union is used to manipulate a two byte unsigned int
typedef union _WORD_TYPE
{
  unsigned int val;
  struct
  {
    unsigned char LSB;
    unsigned char MSB;
  };
  unsigned char v[2];
} WORD_TYPE;

// This union is used to manipulate a two byte signed int
typedef union _SIGNED_WORD_TYPE
{
  int val;
  struct
  {
    unsigned char LSB;
    unsigned char MSB;
  };
  unsigned char v[2];
} SIGNED_WORD_TYPE;

// This union is used to manipulate a four byte signed long
typedef union _LONG_TYPE
{
  long val;
  struct
  {
    unsigned char LSB;
    unsigned char MSB;
    unsigned char b0;
    unsigned char b1;
  };
  unsigned char v[4];
} LONG_TYPE;

/*
 * Bosch BMP085 pressure and temperature module device slave address
 * - 0xee shifted 1-bit to right
 *
 * 0xee is the correct slave address for the Bosch device, Arduino's weird
 * implementation of the Wire library requires the shift.
 */
#define BMP085SlaveAdrs 0x77

/*
 * EEPROM registers containing the calibration data
 */
#define BMP085_EEPROM 0xaa
#define BMP085_AC1 0xaa
#define BMP085_AC2 0xac
#define BMP085_AC3 0xae
#define BMP085_AC4 0xb0
#define BMP085_AC5 0xb2
#define BMP085_AC6 0xb4
#define BMP085_B1  0xb6
#define BMP085_B2  0xb8
#define BMP085_MB  0xba
#define BMP085_MC  0xbc
#define BMP085_MD  0xbe
#define BMP085_MAX_CAL_FACTORS 22

/*
 * Register and the codes written to the register to start temperature and
 * pressure conversions.
 */
#define BMP085_START_REG 0xf4
#define BMP085_START_UT 0x2e // temperature
#define BMP085_START_UP 0x34 // pressure
#define BMP085_CONVERSION_TIME 4  // conversion time in ms

/*
 * Registers containing the 16-bit temperature and pressure data after a 
 * conversion.  The same register is used, what is read is determined by what
 * was previously written into the Start register.
 */
#define BMP085_UT 0xf6
#define BMP085_UP 0xf6

/*
 * Local registers for storing the calibration data read from the EEPROM.
 * These are setup as unions defined above so that they can be accessed one byte
 * at a time when read from the device EEPROM.
 */
SIGNED_WORD_TYPE Bosch_ac1,
                 Bosch_ac2,
                 Bosch_ac3;
WORD_TYPE Bosch_ac4,
          Bosch_ac5,
          Bosch_ac6;
SIGNED_WORD_TYPE Bosch_b1,
                 Bosch_b2,
                 Bosch_mb,
                 Bosch_mc,
                 Bosch_md;

/*
 * Raw temperature read from module
 */ 
LONG_TYPE Bosch_ut;

/*
 * Raw pressure read from module
 */ 
LONG_TYPE Bosch_up;

/*
 * Intermediate registers for calculations
 */ 
long Bosch_x1,
     Bosch_x2,
     Bosch_x3,
     Bosch_b3,
     Bosch_b5,
     Bosch_b6,
     Bosch_p,
     Bosch_t;

unsigned long Bosch_b4,
              Bosch_b7;

/*
* constructor()
*
* Read all 22 bytes of calibration data from the BMP085 EEPROM.  The factors
* are 16 bits each but are in big endian format (MSB first).  The byte order
* must be switched to make the factors into standard 16-bit integer format.
*/
BMP085::BMP085(void){

  /*
   * Arduino Wire automatically enables the internal pull-up resistors for SCL
   * and SDA.  When connected to a 3.3V slave device, the pull-ups have enough
   * strength to pull the slave device pins to 5V.  Not good for the device.
   * Write a zero to the PORT register for each of the pins to disable the
   * pull-ups.
   */
  PORTC &= 0b11001111;

  
  /*
   * Start the transfer by sending the Slave Address to the device.  Next, write
   * the starting address of the calibration EEPROM to set the read address in
   * the device.  Stop the transmission, but issue a repeated start on the I2C
   * bus, (endTransmission() with false parameter does this.  Finally, issue a
   * requestFrom() for the data.  Wait until all bytes have been read into the
   * buffer, then copy them to the local registers, swapping the MSB and LSB.
   */
  Wire.beginTransmission(BMP085SlaveAdrs);
  Wire.write(BMP085_EEPROM);
  Wire.endTransmission(false);
  Wire.requestFrom(BMP085SlaveAdrs, BMP085_MAX_CAL_FACTORS, true);
  
  /*
   * wait here until all data bytes have been read
   */   
  while(Wire.available() < BMP085_MAX_CAL_FACTORS);

  Bosch_ac1.MSB = Wire.read();
  Bosch_ac1.LSB = Wire.read();
  Bosch_ac2.MSB = Wire.read();
  Bosch_ac2.LSB = Wire.read();
  Bosch_ac3.MSB = Wire.read();
  Bosch_ac3.LSB = Wire.read();
  Bosch_ac4.MSB = Wire.read();
  Bosch_ac4.LSB = Wire.read();
  Bosch_ac5.MSB = Wire.read();
  Bosch_ac5.LSB = Wire.read();
  Bosch_ac6.MSB = Wire.read();
  Bosch_ac6.LSB = Wire.read();
  Bosch_b1.MSB = Wire.read();
  Bosch_b1.LSB = Wire.read();
  Bosch_b2.MSB = Wire.read();
  Bosch_b2.LSB = Wire.read();
  Bosch_mb.MSB = Wire.read();
  Bosch_mb.LSB = Wire.read();
  Bosch_mc.MSB = Wire.read();
  Bosch_mc.LSB = Wire.read();
  Bosch_md.MSB = Wire.read();
  Bosch_md.LSB = Wire.read();

}/* end constructor() */

/*
 *
 * getBoschRawTemperature()
 *
 * This routine starts the temperature conversion, then reads the raw
 * (uncalibrated) into storage.  The BMP085 sensor will take up to 4.5ms to do
 * a conversion.  This routine is written to not block during the conversion
 * time or need an interrupt (although one is available from the sensor).
 * Simply call this routine repeatedly and when the conversion is done the data
 * is read.  During the conversion time, the routine will return false.  After
 * the conversion time has elapsed and the data has been read, the routine will
 * return true.       
 */
boolean BMP085::getBoschRawTemperature(void){

  static boolean state = false;
  static unsigned long time = 0;
  
  if(state == false){
    Wire.beginTransmission(BMP085SlaveAdrs);
    Wire.write(BMP085_START_REG);
    Wire.write(BMP085_START_UT);
    Wire.endTransmission();
    
    time = millis();
    state = true;
    
    return(false);
  }
  else {
    if(millis() - time > BMP085_CONVERSION_TIME){
      Wire.beginTransmission(BMP085SlaveAdrs);
      Wire.write(BMP085_UT);
      Wire.endTransmission(false);
      Wire.requestFrom(BMP085SlaveAdrs, 2, true);

      while(Wire.available() < 2);
      Bosch_ut.MSB = Wire.read();
      Bosch_ut.LSB = Wire.read();
      Bosch_ut.b0 = 0;
      Bosch_ut.b1 = 0;
    
      state = false;
      return(true);
    }
    else{
      return(false);
    }
  }
}/* end getBoschRawTemperature() */

/*
 *
 * getBoschRawPressure()
 *
 * This routine starts the pressure conversion, then reads the raw
 * (uncalibrated) into storage.  The BMP085 sensor will take up to 4.5ms to do
 * a conversion.  This routine is written to not block during the conversion
 * time or need an interrupt (although one is available from the sensor).
 * Simply call this routine repeatedly and when the conversion is done the data
 * is read.  During the conversion time, the routine will return false.  After
 * the conversion time has elapsed and the data has been read, the routine will
 * return true.       
 */
boolean BMP085::getBoschRawPressure(void){

  static boolean state = false;
  static unsigned long time = 0;
  
  if(state == false){
    Wire.beginTransmission(BMP085SlaveAdrs);
    Wire.write(BMP085_START_REG);
    Wire.write(BMP085_START_UP);
    Wire.endTransmission();
  
    time = millis();
    state = true;
    
    return(false);
  }
  else {
    if(millis() - time > BMP085_CONVERSION_TIME){
  
      Wire.beginTransmission(BMP085SlaveAdrs);
      Wire.write(BMP085_UP);
      Wire.endTransmission(false);
      Wire.requestFrom(BMP085SlaveAdrs, 2, true);

      while(Wire.available() < 2);
      Bosch_up.MSB = Wire.read();
      Bosch_up.LSB = Wire.read();
      Bosch_up.b0 = 0;
      Bosch_up.b1 = 0;

      state = false;
      return(true);
    }
    else{
      return(false);
    }
  }
}/* end getBoschRawPressure() */

/*
 *
 * calculateBoschTemperature()
 *
 * Calculate the actual temperature from the Bosch BMP085 sensor.  The raw 
 * temperature is in Bosch_ut.  All other parameters have been previously read
 * from the sensor's calibration EEPROM.
 *
 * The formula is a direct copy from the sensor's data sheet.
 */
int BMP085::calculateBoschTemperature(void){
  Bosch_x1 = (Bosch_ut.val - Bosch_ac6.val) * Bosch_ac5.val / 32768L;
  Bosch_x2 = Bosch_mc.val * 2048L / (Bosch_x1 + Bosch_md.val);
  Bosch_b5 = Bosch_x1 + Bosch_x2;
  Bosch_t = (Bosch_b5 + 8) / 16L;
  
  return((int)Bosch_t);

}/* end calculateBoschTemperature() */

/*
 *
 * calculateBoschPressure()
 *
 * Calculate the actual pressure from the Bosch BMP085 sensor.  The raw pressure
 * is in Bosch_up.  All other parameters have been previously read from the 
 * sensor's calibration EEPROM.
 *
 * The formula is a direct copy from the sensor's data sheet.
 */
long BMP085::calculateBoschPressure(void){
  Bosch_b6 = Bosch_b5 - 4000L;
  Bosch_x1 = (Bosch_b2.val * (Bosch_b6 * Bosch_b6 / 4096L)) / 2048L;
  Bosch_x2 = Bosch_ac2.val * Bosch_b6 / 2048L;
  Bosch_x3 = Bosch_x1 + Bosch_x2;
  Bosch_b3 = ((Bosch_ac1.val * 4L + Bosch_x3) + 2L) / 4L;
  Bosch_x1 = Bosch_ac3.val * Bosch_b6 / 8192L;
  Bosch_x2 = (Bosch_b1.val * (Bosch_b6 * Bosch_b6 / 4096L)) / 65536;
  Bosch_x3 = ((Bosch_x1 + Bosch_x2) + 2L) / 4L;
  Bosch_b4 = Bosch_ac4.val * (unsigned long)(Bosch_x3 + 32768L) / 32768UL;
  Bosch_b7 = ((unsigned long)Bosch_up.val - Bosch_b3) * 50000UL;
  if(Bosch_b7 < 0x80000000){
    Bosch_p = (Bosch_b7 * 2L) / Bosch_b4;
  }
  else{
    Bosch_p = (Bosch_b7 / Bosch_b4) * 2L;
  }
  Bosch_x1 = (Bosch_p / 256L) * (Bosch_p / 256L);
  Bosch_x1 = (Bosch_x1 * 3038L) / 65536L;
  Bosch_x2 = (-7357L * Bosch_p) / 65536L;
  Bosch_p = Bosch_p + (Bosch_x1 + Bosch_x2 + 3791L) / 16;
  
  return(Bosch_p);

}/* end calculateBoschPressure() */
