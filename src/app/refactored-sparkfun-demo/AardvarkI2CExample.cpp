// SPDX-FileCopyrightText: © 2022 Embedded Artistry LLC <contact@embeddedartistry.com>
// SPDX-License-Identifier: MIT

#include "aardvark.h"
#include <SparkFunBME280_redesigned.h>
#include <cassert>
#include <cstdio>
#include <environment_calculations.hpp>

static Aardvark handle_ = 0;
static AardvarkConfig mode_ = AA_CONFIG_SPI_I2C; // SPI + I2C
static uint8_t BME280_I2C_Addr_7bit = 0x76;

void aardvark_initialize()
{
	uint16_t devices;
	int devices_found;
	// Find the port instead of using the hard-wired one
	devices_found = aa_find_devices(1, &devices);
	assert(devices_found);
	assert(false == (AA_PORT_NOT_FREE & devices)); // Otherwise port is in uses
	handle_ = aa_open(devices);
	assert(handle_ > 0); // could not find aardvark device

	// Configure for I2C support
	aa_configure(handle_, mode_);

	// Enable target power
	aa_target_power(handle_, AA_TARGET_POWER_BOTH);
}

void aardvark_shutdown()
{
	aa_close(handle_);
	handle_ = 0;
}

void aardvark_i2c_write(uint8_t reg_addr, uint8_t data, void* private_data)
{
	uint8_t dev_addr = *reinterpret_cast<uint8_t*>(private_data);
	uint16_t num_written;
	uint8_t internal_buffer[2] = {reg_addr, data};

	int r = aa_i2c_write_ext(handle_, dev_addr, AA_I2C_NO_FLAGS, 2, internal_buffer, &num_written);
	assert(r == AA_OK);
	assert(2 == num_written);
}

void aardvark_i2c_read(uint8_t reg_addr, uint8_t* data, size_t length, void* private_data)
{
	uint8_t dev_addr = *reinterpret_cast<uint8_t*>(private_data);
	uint16_t num_written;
	uint16_t num_read;

	int r = aa_i2c_write_read(handle_, dev_addr, AA_I2C_NO_FLAGS, 1, &reg_addr, &num_written,
							  static_cast<uint16_t>(length), data, &num_read);
	assert(1 == num_written);
	assert(num_read == length);
	assert(r == AA_OK);
}

int main(void)
{
	printf("Running BME280 example with the Aardvark I2C driver\n");
	aardvark_initialize();

	BME280 mySensor(aardvark_i2c_write, aardvark_i2c_read, BME280::comm_mode::I2C,
					reinterpret_cast<void*>(&BME280_I2C_Addr_7bit));

	bool initialized = mySensor.begin();
	assert(initialized);

	int sample_count = 0;
	while(sample_count++ < 20)
	{
		auto pressure = mySensor.readPressure();
		auto humidity = static_cast<double>(mySensor.readHumidity());
		auto temperature = static_cast<double>(mySensor.readTemperature());
		auto altitude =
			static_cast<double>(metersToFeet(calculateAltitude(static_cast<float>(pressure))));
		auto dew_point = static_cast<double>(
			celsiusToFahrenheit(static_cast<float>(calculateDewPoint(temperature, humidity))));

		printf(
			"Humdity: %f %%Rh, Pressure: %f Pa, Altitude: %f ft, Temp: %f °F, Dew Point: %f °F\n",
			humidity, static_cast<double>(pressure), altitude,
			static_cast<double>(celsiusToFahrenheit(static_cast<float>(temperature))), dew_point);
	}

	printf("Shutting down aardvark\n");
	aardvark_shutdown();

	return 0;
}
