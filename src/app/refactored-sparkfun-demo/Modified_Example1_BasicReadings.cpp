/*
  Get basic environmental readings from the BME280
  By: Nathan Seidle
  SparkFun Electronics
  Date: March 9th, 2018
  License: This code is public domain but you buy me a beer if you use this and we meet someday
  (Beerware license).

  ***NOTE:***
  This example has been slightly modified to work with the refactored BME280 code.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/14348 - Qwiic Combo Board
  https://www.sparkfun.com/products/13676 - BME280 Breakout Board

  This example shows how to read humidity, pressure, and current temperature from the BME280 over
  I2C.

  Hardware connections:
  BME280 -> Arduino
  GND -> GND
  3.3 -> 3.3
  SDA -> A4
  SCL -> A5
*/

#include "SparkFunBME280_redesigned.h"
#include "environment_calculations.hpp"
#include <Arduino.h>
#include <Wire.h>
#include <assert.h>

uint8_t BME280_I2C_Addr_7bit = 0x76;
void BME280ArduinoI2CWrite(uint8_t register_addr, uint8_t data, void* private_data);
void BME280ArduinoI2CRead(uint8_t register_addr, uint8_t* data_out, size_t length,
						  void* private_data);

BME280 mySensor(BME280ArduinoI2CWrite, BME280ArduinoI2CRead, I2C_MODE,
				reinterpret_cast<void*>(&BME280_I2C_Addr_7bit));

void BME280ArduinoI2CWrite(uint8_t register_addr, uint8_t data, void* private_data)
{
	assert(private_data);

	uint8_t address = *static_cast<uint8_t*>(private_data);

	Wire.beginTransmission(address);
	Wire.write(register_addr);
	Wire.write(data);
	Wire.endTransmission();
}

void BME280ArduinoI2CRead(uint8_t register_addr, uint8_t* data_out, size_t length,
						  void* private_data)
{
	assert(data_out && private_data);

	uint8_t address = *static_cast<uint8_t*>(private_data);
	uint8_t i = 0;

	Wire.beginTransmission(address);
	Wire.write(register_addr);
	Wire.endTransmission();
	Wire.requestFrom(address, length);
	// Loop is to handle the case where device may send less than requested
	while(Wire.available() and i < length)
	{
		data_out[i++] = Wire.read();
	}
}

void setup()
{
	Serial.begin(115200);
	Serial.println("Reading basic values from BME280, refactored edition");

	Wire.begin();

	if(mySensor.begin() == false)
	{
		Serial.println("The sensor did not respond. Please check wiring.");
		while(1)
			; // Freeze
	}
}

void loop()
{
	auto pressure = mySensor.readFloatPressure();

	Serial.print("Humidity: ");
	Serial.print(mySensor.readFloatHumidity(), 0);

	Serial.print(" Pressure: ");
	Serial.print(pressure, 0);

	Serial.print(" Alt: ");
	Serial.print(metersToFeet(calculateAltitude(pressure)), 1);

	Serial.print(" Temp: ");
	// Serial.print(mySensor.readTempC(), 2);
	Serial.print(celsiusToFahrenheit(mySensor.readTemp()), 2);

	Serial.println();

	delay(50);
}
