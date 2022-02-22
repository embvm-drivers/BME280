/*
  Get basic environmental readings from the BME280
  By: Nathan Seidle
  SparkFun Electronics
  Date: March 9th, 2018
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/14348 - Qwiic Combo Board
  https://www.sparkfun.com/products/13676 - BME280 Breakout Board
  
  This example shows how to read humidity, pressure, and current temperature from the BME280 over I2C.

  Hardware connections:
  BME280 -> Arduino
  GND -> GND
  3.3 -> 3.3
  SDA -> A4
  SCL -> A5
*/

#include <Wire.h>
#include "SparkFunBME280.h"

BME280 mySensor;

void setup()
{
  Serial.begin(115200);
  Serial.println("Reading basic values from BME280");

  Wire.begin();

  if (mySensor.beginI2C() == false) //Begin communication over I2C
  {
    Serial.println("The sensor did not respond. Please check wiring.");
    while(1); //Freeze
  }
}

void loop()
{
  Serial.print("Humidity: ");
  Serial.print(mySensor.readFloatHumidity(), 0);

  Serial.print(" Pressure: ");
  Serial.print(mySensor.readFloatPressure(), 0);

  Serial.print(" Alt: ");
  //Serial.print(mySensor.readFloatAltitudeMeters(), 1);
  Serial.print(mySensor.readFloatAltitudeFeet(), 1);

  Serial.print(" Temp: ");
  //Serial.print(mySensor.readTempC(), 2);
  Serial.print(mySensor.readTempF(), 2);

  Serial.println();

  delay(50);
}



#if 0
// Initialization of SPI
    case SPI_MODE:
      // start the SPI library:
      SPI.begin();
      // initialize the  data ready and chip select pins:
      pinMode(settings.chipSelectPin, OUTPUT);
      digitalWrite(settings.chipSelectPin, HIGH);
      break;

void BME280::writeRegister(uint8_t offset, uint8_t dataToWrite)
{
  switch(settings.commInterface)
  {
    case I2C_MODE:
      // Write the byte

      switch(_wireType)
      {
        case(HARD_WIRE):
          _hardPort->beginTransmission(settings.I2CAddress);
          _hardPort->write(offset);
          _hardPort->write(dataToWrite);
          _hardPort->endTransmission();
          break;
      }
      break;

    case SPI_MODE:
      SPI.beginTransaction(settings.spiSettings);
      // take the chip select low to select the device:
      digitalWrite(settings.chipSelectPin, LOW);
      // send the device the register you want to read:
      SPI.transfer(offset & 0x7F);
      // send a value of 0 to read the first byte returned:
      SPI.transfer(dataToWrite);
      // decrement the number of bytes left to read:
      // take the chip select high to de-select:
      digitalWrite(settings.chipSelectPin, HIGH);
      SPI.endTransaction();
      break;

    default:
      break;
  }
}

//****************************************************************************//
void BME280::readRegisterRegion(uint8_t* outputPointer, uint8_t offset, uint8_t length)
{
  // define pointer that will point to the external space
  uint8_t i = 0;
  char c = 0;

  switch(settings.commInterface)
  {
    case I2C_MODE:
      switch(_wireType)
      {
        case(HARD_WIRE):
          _hardPort->beginTransmission(settings.I2CAddress);
          _hardPort->write(offset);
          _hardPort->endTransmission();

          // request bytes from slave device
          _hardPort->requestFrom(settings.I2CAddress, length);
          while((_hardPort->available()) &&
              (i < length)) // slave may send less than requested
          {
            c = _hardPort->read(); // receive a byte as character
            *outputPointer = c;
            outputPointer++;
            i++;
          }
          break;
        case(SOFT_WIRE):
#ifdef SoftwareWire_h
          _softPort->beginTransmission(settings.I2CAddress);
          _softPort->write(offset);
          _softPort->endTransmission();

          // request bytes from slave device
          _softPort->requestFrom(settings.I2CAddress, length);
          while((_softPort->available()) &&
              (i < length)) // slave may send less than requested
          {
            c = _softPort->read(); // receive a byte as character
            *outputPointer = c;
            outputPointer++;
            i++;
          }
#endif
          break;
      }
      break;

    case SPI_MODE:
      SPI.beginTransaction(settings.spiSettings);
      // take the chip select low to select the device:
      digitalWrite(settings.chipSelectPin, LOW);
      // send the device the register you want to read:
      SPI.transfer(offset | 0x80); // Ored with "read request" bit
      while(i < length) // slave may send less than requested
      {
        c = SPI.transfer(0x00); // receive a byte as character
        *outputPointer = c;
        outputPointer++;
        i++;
      }
      // take the chip select high to de-select:
      digitalWrite(settings.chipSelectPin, HIGH);
      SPI.endTransaction();
      break;

    default:
      break;
  }
}

uint8_t BME280::readRegister(uint8_t offset)
{
  // Return value
  uint8_t result = 0;
  uint8_t numBytes = 1;
  switch(settings.commInterface)
  {
    case I2C_MODE:
      switch(_wireType)
      {
        case(HARD_WIRE):
          _hardPort->beginTransmission(settings.I2CAddress);
          _hardPort->write(offset);
          _hardPort->endTransmission();

          _hardPort->requestFrom(settings.I2CAddress, numBytes);
          while(_hardPort->available()) // slave may send less than requested
          {
            result = _hardPort->read(); // receive a byte as a proper uint8_t
          }
          break;
      }

      break;

    case SPI_MODE:
      readRegisterRegion(&result, offset, 1);
      break;

    default:
      break;
  }
  return result;
}

#endif
