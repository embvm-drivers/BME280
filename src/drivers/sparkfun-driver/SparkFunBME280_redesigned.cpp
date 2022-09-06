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
#include "_private/bme280_implementation_details.hpp"

//****************************************************************************//
//
//  Settings and configuration
//
//****************************************************************************//

bool BME280::begin()
{
	bool chip_valid = BME280Internals::checkChipID(*this);

	if(chip_valid)
	{
		BME280Internals::readCompensationData(*this);

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
	uint8_t controlData = BME280Internals::readRegister(*this, BME280_CTRL_MEAS_REG);
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
	BME280Internals::UpdateRegisterField(*this, BME280_CTRL_MEAS_REG, BME280_CONFIG_MODE_MASK,
										 BME280_CONFIG_MODE_SHIFT, static_cast<uint8_t>(mode));
}

void BME280::setStandbyTime(BME280::standby timeSetting)
{
	BME280Internals::UpdateRegisterField(*this, BME280_CONFIG_REG, BME280_CONFIG_STANDBY_MASK,
										 BME280_CONFIG_STANDBY_SHIFT,
										 static_cast<uint8_t>(timeSetting));
}

void BME280::setFilter(BME280::filtering filterSetting)
{
	BME280Internals::UpdateRegisterField(*this, BME280_CONFIG_REG, BME280_CONFIG_FILTER_MASK,
										 BME280_CONFIG_FILTER_SHIFT,
										 static_cast<uint8_t>(filterSetting));
}

void BME280::setTempOverSample(BME280::oversampling overSampleAmount)
{
	BME280SettingChangeProtector{*this};
	BME280Internals::UpdateRegisterField(*this, BME280_CTRL_MEAS_REG, BME280_CTRL_OVERSAMPLE_MASK,
										 BME280_CTRL_TEMPERATURE_OVERSAMPLE_SHIFT,
										 static_cast<uint8_t>(overSampleAmount));
}

void BME280::setPressureOverSample(BME280::oversampling overSampleAmount)
{
	BME280SettingChangeProtector{*this};
	BME280Internals::UpdateRegisterField(*this, BME280_CTRL_MEAS_REG, BME280_CTRL_OVERSAMPLE_MASK,
										 BME280_CTRL_PRESSURE_OVERSAMPLE_SHIFT,
										 static_cast<uint8_t>(overSampleAmount));
}

void BME280::setHumidityOverSample(BME280::oversampling overSampleAmount)
{
	BME280SettingChangeProtector{*this};
	BME280Internals::UpdateRegisterField(
		*this, BME280_CTRL_HUMIDITY_REG, BME280_CTRL_OVERSAMPLE_MASK,
		BME280_CTRL_HUMIDITY_OVERSAMPLE_SHIFT, static_cast<uint8_t>(overSampleAmount));
}

bool BME280::isMeasuring(void)
{
	uint8_t stat = BME280Internals::readRegister(*this, BME280_STAT_REG);
	return (stat & BME280_STAT_STATUS_MASK);
}

void BME280::reset(void)
{
	BME280Internals::writeRegister(*this, BME280_RST_REG, 0xB6);
}

void BME280::readAllMeasurements(BME280::Measurements& measurements)
{
	uint8_t dataBurst[8];
	BME280Internals::readRegisterRegion(*this, dataBurst, BME280_MEASUREMENTS_REG, 8);
	measurements.temperature =
		readTemperatureFromBurst(dataBurst, calibration, t_fine) + tempCorrection_;
	measurements.pressure = readPressureFromBurst(dataBurst, calibration, t_fine);
	measurements.humidity = readHumidityFromBurst(dataBurst, calibration, t_fine);
}

float BME280::readPressure(void)
{
	uint8_t buffer[3];
	BME280Internals::readRegisterRegion(*this, buffer, BME280_PRESSURE_MSB_REG, 3);
	int32_t adc_P = assembleRawTempPressure(buffer);
	return convertPressure(adc_P, calibration, t_fine);
}

float BME280::readHumidity(void)
{
	uint8_t buffer[2];
	BME280Internals::readRegisterRegion(*this, buffer, BME280_HUMIDITY_MSB_REG, 2);
	int32_t adc_H = assembleRawHumidity(buffer);
	return convertHumidity(adc_H, calibration, t_fine);
}

void BME280::setTemperatureCorrection(float corr)
{
	tempCorrection_ = corr;
}

float BME280::readTemperature(void)
{
	uint8_t buffer[3];
	BME280Internals::readRegisterRegion(*this, buffer, BME280_TEMPERATURE_MSB_REG, 3);
	int32_t adc_T = assembleRawTempPressure(buffer);
	return convertTemperature(adc_T, calibration, t_fine) + tempCorrection_;
}
