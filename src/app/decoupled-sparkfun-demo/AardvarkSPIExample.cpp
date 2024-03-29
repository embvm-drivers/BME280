// SPDX-FileCopyrightText: © 2022 Embedded Artistry LLC <contact@embeddedartistry.com>
// SPDX-License-Identifier: MIT

#include "SparkFunBME280.h"
#include "aardvark.h"
#include <cassert>
#include <cstdio>
#include <cstring>

static Aardvark handle_ = 0;
static AardvarkConfig mode_ = AA_CONFIG_SPI_I2C; // SPI + I2C

void aardvark_initialize()
{
	int bitrate = 100;
	uint16_t devices;
	int devices_found;

	// Find the port instead of using the hard-wired one
	devices_found = aa_find_devices(1, &devices);
	assert(devices_found);
	assert(false == (AA_PORT_NOT_FREE & devices)); // Otherwise port is in uses
	handle_ = aa_open(devices);
	assert(handle_ > 0); // could not find aardvark device

	// Configure for SPI support
	aa_configure(handle_, mode_);
	aa_spi_configure(handle_, AA_SPI_POL_RISING_FALLING, AA_SPI_PHASE_SAMPLE_SETUP,
					 AA_SPI_BITORDER_MSB);
	aa_spi_master_ss_polarity(handle_, AA_SPI_SS_ACTIVE_LOW);

	bitrate = aa_spi_bitrate(handle_, bitrate);
	printf("Bitrate set to %d kHz\n", bitrate);

	// Enable target power
	aa_target_power(handle_, AA_TARGET_POWER_BOTH);
}

void aardvark_shutdown()
{
	aa_close(handle_);
	handle_ = 0;
}

void aardvark_spi_write(uint8_t reg_addr, uint8_t data, void* private_data)
{
	(void)private_data;
	uint8_t internal_buffer[2] = {reg_addr, data};

	int r = aa_spi_write(handle_, 2, internal_buffer, 0, internal_buffer);
	assert(r >= AA_OK);
}

void aardvark_spi_read(uint8_t reg_addr, uint8_t* data, size_t length, void* private_data)
{
	(void)private_data;

	// This is a requirement of the Aardvark SPI api - why they separate the read
	// and write data buffers and read sizes, I do not understand, because the write length
	// must match. So, we use a temporary internal buffer here to coerce the data
	// into the necessary form for this API.
	constexpr size_t INTERNAL_SPI_BUFFER_SIZE = 10;
	static uint8_t internal_buffer[INTERNAL_SPI_BUFFER_SIZE];
	assert(length + 1 < INTERNAL_SPI_BUFFER_SIZE);

	// Clear the data so that we aren't sending unexpected bytes to the device
	// when trying to read. This may be unnecessary, but I did not test it otherwise.
	memset(internal_buffer, 0, INTERNAL_SPI_BUFFER_SIZE);
	internal_buffer[0] = reg_addr;

	uint16_t target_length = static_cast<uint16_t>(length + 1);

	int r = aa_spi_write(handle_, target_length, internal_buffer, target_length, internal_buffer);
	assert(r >= AA_OK);

	memcpy(data, &internal_buffer[1], length);
}

int main(void)
{
	printf("Running BME280 example with the Aardvark SPI driver\n");
	aardvark_initialize();

	BME280 mySensor(aardvark_spi_write, aardvark_spi_read, SPI_MODE);

	bool initialized = mySensor.begin();
	assert(initialized);

	int sample_count = 0;
	while(sample_count++ < 20)
	{
		printf("Humdity: %f, Pressure: %f, Alt: %f, Temp: %f\n",
			   static_cast<double>(mySensor.readFloatHumidity()),
			   static_cast<double>(mySensor.readFloatPressure()),
			   static_cast<double>(mySensor.readFloatAltitudeFeet()),
			   static_cast<double>(mySensor.readTempF()));
	}

	printf("Shutting down aardvark\n");
	aardvark_shutdown();

	return 0;
}
