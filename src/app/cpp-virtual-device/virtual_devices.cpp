// SPDX-FileCopyrightText: © 2022 Embedded Artistry LLC <contact@embeddedartistry.com>
// SPDX-License-Identifier: MIT
// TODO: need to refactor the aardvark stuff here out into a hardware platform. Make reusable
// function implementations that can be supplied to the different driver typs
// TODO: in the hardware platform, need to set up the driver for the desired move (normal mode?
// forced mode?) set up the virtual function interfaces provide the virutal functions to the
// application

#include "SparkFunBME280.h"
#include "aardvark.h"
#include <cassert>
#include <cstdio>

static Aardvark handle_ = 0;
static AardvarkConfig mode_ = AA_CONFIG_SPI_I2C; // SPI + I2C
static uint8_t BME280_I2C_Addr_7bit = 0x76;

void aardvark_initialize()
{
	uint16_t devices, devices_found;
	// Find the port instead of using the hard-wired one
	devices_found = aa_find_devices(1, &devices);
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
	uint8_t dev_addr = *(uint8_t*)private_data;
	uint16_t num_written;
	uint8_t internal_buffer[2] = {reg_addr, data};

	int r = aa_i2c_write_ext(handle_, dev_addr, AA_I2C_NO_FLAGS, 2, internal_buffer, &num_written);
	assert(r == AA_OK);
	assert(2 == num_written);
}

void aardvark_i2c_read(uint8_t reg_addr, uint8_t* data, size_t length, void* private_data)
{
	uint8_t dev_addr = *(uint8_t*)private_data;
	uint16_t num_written;
	uint16_t num_read;

	int r = aa_i2c_write_read(handle_, dev_addr, AA_I2C_NO_FLAGS, 1, &reg_addr, &num_written,
							  length, data, &num_read);
	assert(1 == num_written);
	assert(num_read == length);
	assert(r == AA_OK);
}

int main(void)
{
	printf("Running BME280 example with the Aardvark I2C driver\n");
	aardvark_initialize();

	BME280 mySensor(aardvark_i2c_write, aardvark_i2c_read, I2C_MODE,
					reinterpret_cast<void*>(&BME280_I2C_Addr_7bit));

	bool initialized = mySensor.begin();
	assert(initialized);

	int sample_count = 0;
	while(sample_count++ < 20)
	{
		printf("Humdity: %f, Pressure: %f, Alt: %f, Temp: %f\n", mySensor.readFloatHumidity(),
			   mySensor.readFloatPressure(), mySensor.readFloatAltitudeFeet(),
			   mySensor.readTempF());
	}

	printf("Shutting down aardvark\n");
	aardvark_shutdown();

	return 0;
}
