/******************************************************************************
SparkFunBME280.cpp
BME280 Arduino and Teensy Driver
Marshall Taylor @ SparkFun Electronics
May 20, 2015
https://github.com/sparkfun/BME280_Breakout

Resources:
Uses Wire.h for i2c operation
Uses SPI.h for SPI operation

Development environment specifics:
Arduino IDE 1.8.5
Teensy loader 1.23

This code is released under the [MIT License](http://opensource.org/licenses/MIT).
Please review the LICENSE.md file included with this example. If you have any questions
or concerns with licensing, please contact techsupport@sparkfun.com.
Distributed as-is; no warranty is given.
******************************************************************************/
// See SparkFunBME280.h for additional topology notes.

#include "SparkFunBME280_redesigned.h"
#include "_private/bme280_defs.hpp"
#ifdef __AVR
// avr-gcc does not seem to supply the C++ versions of these headers
#include <assert.h>
#include <math.h>
#include <string.h>
#else
#include <cassert>
#include <cmath>
#include <cstring>
#endif

class BME280SettingChangeProtector
{
  public:
	BME280SettingChangeProtector(BME280& inst) : inst_(inst)
	{
		// Get the current mode so we can go back to it at the end
		originalMode = inst_.getMode();

		// Config will only be writeable in sleep mode, so first go to sleep mode
		inst_.setMode(BME280::op_mode::sleep);
	}

	~BME280SettingChangeProtector()
	{
		// Return to the original user's choice
		inst_.setMode(originalMode);
	}

  private:
	BME280& inst_;
	BME280::op_mode originalMode;
};

//****************************************************************************//
//
//  Settings and configuration
//
//****************************************************************************//

void BME280::readCompensationData()
{
	// Reading all compensation data, range 0x88:A1, 0xE1:E7
	calibration.dig_T1 = ((uint16_t)((readRegister(BME280_DIG_T1_MSB_REG) << 8) +
									 readRegister(BME280_DIG_T1_LSB_REG)));
	calibration.dig_T2 = ((int16_t)((readRegister(BME280_DIG_T2_MSB_REG) << 8) +
									readRegister(BME280_DIG_T2_LSB_REG)));
	calibration.dig_T3 = ((int16_t)((readRegister(BME280_DIG_T3_MSB_REG) << 8) +
									readRegister(BME280_DIG_T3_LSB_REG)));

	calibration.dig_P1 = ((uint16_t)((readRegister(BME280_DIG_P1_MSB_REG) << 8) +
									 readRegister(BME280_DIG_P1_LSB_REG)));
	calibration.dig_P2 = ((int16_t)((readRegister(BME280_DIG_P2_MSB_REG) << 8) +
									readRegister(BME280_DIG_P2_LSB_REG)));
	calibration.dig_P3 = ((int16_t)((readRegister(BME280_DIG_P3_MSB_REG) << 8) +
									readRegister(BME280_DIG_P3_LSB_REG)));
	calibration.dig_P4 = ((int16_t)((readRegister(BME280_DIG_P4_MSB_REG) << 8) +
									readRegister(BME280_DIG_P4_LSB_REG)));
	calibration.dig_P5 = ((int16_t)((readRegister(BME280_DIG_P5_MSB_REG) << 8) +
									readRegister(BME280_DIG_P5_LSB_REG)));
	calibration.dig_P6 = ((int16_t)((readRegister(BME280_DIG_P6_MSB_REG) << 8) +
									readRegister(BME280_DIG_P6_LSB_REG)));
	calibration.dig_P7 = ((int16_t)((readRegister(BME280_DIG_P7_MSB_REG) << 8) +
									readRegister(BME280_DIG_P7_LSB_REG)));
	calibration.dig_P8 = ((int16_t)((readRegister(BME280_DIG_P8_MSB_REG) << 8) +
									readRegister(BME280_DIG_P8_LSB_REG)));
	calibration.dig_P9 = ((int16_t)((readRegister(BME280_DIG_P9_MSB_REG) << 8) +
									readRegister(BME280_DIG_P9_LSB_REG)));

	calibration.dig_H1 = ((uint8_t)(readRegister(BME280_DIG_H1_REG)));
	calibration.dig_H2 = ((int16_t)((readRegister(BME280_DIG_H2_MSB_REG) << 8) +
									readRegister(BME280_DIG_H2_LSB_REG)));
	calibration.dig_H3 = ((uint8_t)(readRegister(BME280_DIG_H3_REG)));
	calibration.dig_H4 = ((int16_t)((readRegister(BME280_DIG_H4_MSB_REG) << 4) +
									(readRegister(BME280_DIG_H4_LSB_REG) & 0x0F)));
	calibration.dig_H5 = ((int16_t)((readRegister(BME280_DIG_H5_MSB_REG) << 4) +
									((readRegister(BME280_DIG_H4_LSB_REG) >> 4) & 0x0F)));
	calibration.dig_H6 = ((int8_t)readRegister(BME280_DIG_H6_REG));
}

bool BME280::checkChipID()
{
	bool valid = false;

	uint8_t chipID = readRegister(BME280_CHIP_ID_REG);
	if(chipID == 0x58 || chipID == 0x60) // Is this BMP or BME?
	{
		valid = true;
	}

	return valid;
}

bool BME280::begin()
{
	bool chip_valid = checkChipID();

	if(chip_valid)
	{
		readCompensationData();

		setStandbyTime(BME280::standby::for_0_5ms);
		setFilter(BME280::filtering::off);
		setPressureOverSample(BME280::oversampling::oversample_1x);
		setHumidityOverSample(BME280::oversampling::oversample_1x);
		setTempOverSample(BME280::oversampling::oversample_1x);

		setMode(op_mode::normal); // Go!
	}

	return chip_valid;
}

BME280::op_mode BME280::getMode()
{
	uint8_t controlData = readRegister(BME280_CTRL_MEAS_REG);
	uint8_t mode_bits = (controlData & 0b00000011); // Clear bits 7 through 2

	BME280::op_mode mode;
	switch(mode_bits)
	{
		case static_cast<uint8_t>(BME280::op_mode::normal):
		case static_cast<uint8_t>(BME280::op_mode::forced):
		case static_cast<uint8_t>(BME280::op_mode::sleep):
			mode = static_cast<BME280::op_mode>(mode_bits);
			break;
		default:
			assert(0); // Unexpected value!
	}

	return mode;
}

void BME280::setMode(BME280::op_mode mode)
{
	UpdateRegisterField(BME280_CTRL_MEAS_REG, BME280_CONFIG_MODE_MASK, BME280_CONFIG_MODE_SHIFT,
						static_cast<uint8_t>(mode));
}

void BME280::setStandbyTime(BME280::standby timeSetting)
{
	UpdateRegisterField(BME280_CONFIG_REG, BME280_CONFIG_STANDBY_MASK, BME280_CONFIG_STANDBY_SHIFT,
						static_cast<uint8_t>(timeSetting));
}

void BME280::setFilter(BME280::filtering filterSetting)
{
	UpdateRegisterField(BME280_CONFIG_REG, BME280_CONFIG_FILTER_MASK, BME280_CONFIG_FILTER_SHIFT,
						static_cast<uint8_t>(filterSetting));
}

void BME280::setTempOverSample(BME280::oversampling overSampleAmount)
{
	BME280SettingChangeProtector{*this};
	UpdateRegisterField(BME280_CTRL_MEAS_REG, BME280_CTRL_OVERSAMPLE_MASK,
						BME280_CTRL_TEMPERATURE_OVERSAMPLE_SHIFT,
						static_cast<uint8_t>(overSampleAmount));
}

void BME280::setPressureOverSample(BME280::oversampling overSampleAmount)
{
	BME280SettingChangeProtector{*this};
	UpdateRegisterField(BME280_CTRL_MEAS_REG, BME280_CTRL_OVERSAMPLE_MASK,
						BME280_CTRL_PRESSURE_OVERSAMPLE_SHIFT,
						static_cast<uint8_t>(overSampleAmount));
}

void BME280::setHumidityOverSample(BME280::oversampling overSampleAmount)
{
	BME280SettingChangeProtector{*this};
	UpdateRegisterField(BME280_CTRL_HUMIDITY_REG, BME280_CTRL_OVERSAMPLE_MASK,
						BME280_CTRL_HUMIDITY_OVERSAMPLE_SHIFT,
						static_cast<uint8_t>(overSampleAmount));
}

// Check the measuring bit and return true while device is taking measurement
bool BME280::isMeasuring(void)
{
	uint8_t stat = readRegister(BME280_STAT_REG);
	return (stat & BME280_STAT_STATUS_MASK);
}

// Strictly resets.  Run .begin() afterwards
void BME280::reset(void)
{
	writeRegister(BME280_RST_REG, 0xB6);
}

//****************************************************************************//
//
//  Burst Measurement Section
//
//****************************************************************************//

// Read all sensor registers as a burst. See BME280 Datasheet section 4. Data readout
void BME280::readAllMeasurements(BME280::Measurements& measurements)
{
	uint8_t dataBurst[8];
	readRegisterRegion(dataBurst, BME280_MEASUREMENTS_REG, 8);
	readTempFromBurst(dataBurst, measurements);
	readFloatPressureFromBurst(dataBurst, measurements);
	readFloatHumidityFromBurst(dataBurst, measurements);
}

//****************************************************************************//
//
//  Pressure Section
//
//****************************************************************************//
float BME280::convertPressure(int32_t raw_input)
{
	int64_t var1, var2, p_acc;
	var1 = ((int64_t)t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)calibration.dig_P6;
	var2 = var2 + ((var1 * (int64_t)calibration.dig_P5) << 17);
	var2 = var2 + (((int64_t)calibration.dig_P4) << 35);
	var1 = ((var1 * var1 * (int64_t)calibration.dig_P3) >> 8) +
		   ((var1 * (int64_t)calibration.dig_P2) << 12);
	var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)calibration.dig_P1) >> 33;
	if(var1 == 0)
	{
		return 0; // avoid exception caused by division by zero
	}
	p_acc = 1048576 - raw_input;
	p_acc = (((p_acc << 31) - var2) * 3125) / var1;
	var1 = (((int64_t)calibration.dig_P9) * (p_acc >> 13) * (p_acc >> 13)) >> 25;
	var2 = (((int64_t)calibration.dig_P8) * p_acc) >> 19;

	// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8
	// fractional bits). Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862
	// hPa
	p_acc = ((p_acc + var1 + var2) >> 8) + (((int64_t)calibration.dig_P7) << 4);

	return (float)p_acc / 256.0f;
}

float BME280::readFloatPressure(void)
{
	uint8_t buffer[3];
	readRegisterRegion(buffer, BME280_PRESSURE_MSB_REG, 3);
	int32_t adc_P = assembleRawTempPressure(buffer);
	return convertPressure(adc_P);
}

void BME280::readFloatPressureFromBurst(uint8_t buffer[], BME280::Measurements& measurements)
{
	int32_t adc_P = assembleRawTempPressure(buffer);
	measurements.pressure = convertPressure(adc_P);
}

//****************************************************************************//
//
//  Humidity Section
//
//****************************************************************************//
float BME280::convertHumidity(int32_t raw_input)
{
	int32_t var1 = (t_fine - ((int32_t)76800));
	var1 = (((((raw_input << 14) - (((int32_t)calibration.dig_H4) << 20) -
			   (((int32_t)calibration.dig_H5) * var1)) +
			  ((int32_t)16384)) >>
			 15) *
			(((((((var1 * ((int32_t)calibration.dig_H6)) >> 10) *
				 (((var1 * ((int32_t)calibration.dig_H3)) >> 11) + ((int32_t)32768))) >>
				10) +
			   ((int32_t)2097152)) *
				  ((int32_t)calibration.dig_H2) +
			  8192) >>
			 14));
	var1 = (var1 - (((((var1 >> 15) * (var1 >> 15)) >> 7) * ((int32_t)calibration.dig_H1)) >> 4));
	var1 = (var1 < 0 ? 0 : var1);
	var1 = (var1 > 419430400 ? 419430400 : var1);

	// At this point we have humidity in %RH as unsigned 32 bit integer in Q22.10 format
	// (22 integer and 10 fractional bits).
	// An Output value of “47445” represents 47445/1024 = 46.333 %RH
	// The final step here is to perform this conversion.
	return (float)(var1 >> 12) / 1024.0f;
}

float BME280::readFloatHumidity(void)
{
	uint8_t buffer[2];
	readRegisterRegion(buffer, BME280_HUMIDITY_MSB_REG, 2);
	int32_t adc_H = assembleRawHumidity(buffer);
	return convertHumidity(adc_H);
}

void BME280::readFloatHumidityFromBurst(uint8_t buffer[], BME280::Measurements& measurements)
{
	int32_t adc_H = assembleRawHumidity(&buffer[6]);
	measurements.humidity = convertHumidity(adc_H);
}

//****************************************************************************//
//
//  Temperature Section
//
//****************************************************************************//

void BME280::setTemperatureCorrection(float corr)
{
	tempCorrection_ = corr;
}

float BME280::convertTemperature(int32_t raw_input)
{
	// By datasheet, calibrate
	int64_t var1, var2;

	var1 = ((((raw_input >> 3) - ((int32_t)calibration.dig_T1 << 1))) *
			((int32_t)calibration.dig_T2)) >>
		   11;
	var2 = (((((raw_input >> 4) - ((int32_t)calibration.dig_T1)) *
			  ((raw_input >> 4) - ((int32_t)calibration.dig_T1))) >>
			 12) *
			((int32_t)calibration.dig_T3)) >>
		   14;
	t_fine = var1 + var2;

	float output = (t_fine * 5 + 128) >> 8;
	output = output / 100 + tempCorrection_;

	return output;
}

float BME280::readTemp(void)
{
	uint8_t buffer[3];
	readRegisterRegion(buffer, BME280_TEMPERATURE_MSB_REG, 3);
	int32_t adc_T = assembleRawTempPressure(buffer);
	return convertTemperature(adc_T);
}

void BME280::readTempFromBurst(uint8_t buffer[], BME280::Measurements& measurements)
{
	int32_t adc_T = assembleRawTempPressure(&buffer[3]);
	measurements.temperature = convertTemperature(adc_T);
}

//****************************************************************************//
//
//  Utility
//
//****************************************************************************//
void BME280::UpdateRegisterField(const uint8_t reg_addr, const uint8_t mask, const uint8_t shift,
								 const uint8_t data)
{
	uint8_t register_data = readRegister(reg_addr);
	register_data &= ~(mask << shift);
	register_data |= (data & mask) << shift;
	writeRegister(reg_addr, register_data);
}

int32_t BME280::assembleRawTempPressure(uint8_t* bytes)
{
	return ((uint32_t)bytes[0] << 12) | ((uint32_t)bytes[1] << 4) | ((bytes[2] >> 4) & 0x0F);
}

int32_t BME280::assembleRawHumidity(uint8_t* bytes)
{
	return ((uint32_t)bytes[0] << 8) | ((uint32_t)bytes[1]);
}

void BME280::readRegisterRegion(uint8_t* outputPointer, uint8_t offset, uint8_t length)
{
	assert(read_);

	if(commInterface_ == BME280::comm_mode::SPI)
	{
		offset |= 0x80; // set the read bit
	}

	memset(outputPointer, 0, length);

	read_(offset, outputPointer, length, private_data_);
}

uint8_t BME280::readRegister(uint8_t offset)
{
	uint8_t value = 0;

	readRegisterRegion(&value, offset, 1);

	return value;
}

int16_t BME280::readRegisterInt16(uint8_t offset)
{
	uint8_t myBuffer[2];
	readRegisterRegion(myBuffer, offset, 2); // Does memory transfer
	int16_t output = (int16_t)myBuffer[0] | int16_t(myBuffer[1] << 8);

	return output;
}

void BME280::writeRegister(uint8_t offset, uint8_t dataToWrite)
{
	assert(write_);

	if(commInterface_ == BME280::comm_mode::SPI)
	{
		offset &= 0x7F; // ensure offset does not include a read bit in position 0x80
	}

	write_(offset, dataToWrite, private_data_);
}
