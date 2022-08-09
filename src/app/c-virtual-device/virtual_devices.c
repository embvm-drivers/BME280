// SPDX-FileCopyrightText: © 2022 Embedded Artistry LLC <contact@embeddedartistry.com>
// SPDX-License-Identifier: MIT

#include "aardvark.h"
#include "bme280.h"
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static Aardvark handle_ = 0;
static AardvarkConfig mode_ = AA_CONFIG_SPI_I2C; // SPI + I2C
static uint8_t BME280_I2C_Addr_7bit = 0x76;

void aardvark_initialize()
{
	uint16_t devices;
	int devices_found;
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

BME280_INTF_RET_TYPE aardvark_i2c_read(uint8_t reg_addr, uint8_t* reg_data, uint32_t length,
									   void* intf_ptr)
{
	uint8_t dev_addr = *(uint8_t*)intf_ptr;
	uint16_t num_written;
	uint16_t num_read;

	int r = aa_i2c_write_read(handle_, dev_addr, AA_I2C_NO_FLAGS, 1, &reg_addr, &num_written,
							  (uint16_t)length, reg_data, &num_read);
	assert(1 == num_written);
	assert(num_read == length);
	assert(r == AA_OK);

	return (int8_t)r;
}

int8_t aardvark_i2c_write(uint8_t reg_addr, const uint8_t* reg_data, uint32_t length,
						  void* intf_ptr)
{
	uint8_t dev_addr = *(uint8_t*)intf_ptr;
	uint16_t num_written;

	// Translate data
	uint8_t* internal_buffer = (uint8_t*)malloc(length + 1);
	internal_buffer[0] = reg_addr;
	memcpy(&internal_buffer[1], reg_data, length);

	int r = aa_i2c_write_ext(handle_, dev_addr, AA_I2C_NO_FLAGS, (uint16_t)length + 1,
							 internal_buffer, &num_written);
	assert(r == AA_OK);
	assert(2 == num_written);

	free(internal_buffer);

	return (int8_t)r;
}

void delay_us(uint32_t period, void* intf_ptr)
{
	(void)intf_ptr;
	usleep(period);
}

void print_sensor_data(struct bme280_data* comp_data)
{
	printf("%ld, %ld, %ld\r\n", comp_data->temperature, comp_data->pressure, comp_data->humidity);
}

int main(void)
{
	uint8_t settings_sel;
	struct bme280_data comp_data;
	struct bme280_dev dev = {
		.intf_ptr = &BME280_I2C_Addr_7bit,
		.intf = BME280_I2C_INTF,
		.read = aardvark_i2c_read,
		.write = aardvark_i2c_write,
		.delay_us = delay_us,
	};

	printf("Running C virtual device example\n");
	aardvark_initialize();

	int r = bme280_init(&dev);
	assert(r == BME280_OK);

	/* Recommended mode of operation: Indoor navigation */
	dev.settings.osr_h = BME280_OVERSAMPLING_1X;
	dev.settings.osr_p = BME280_OVERSAMPLING_16X;
	dev.settings.osr_t = BME280_OVERSAMPLING_2X;
	dev.settings.filter = BME280_FILTER_COEFF_16;
	dev.settings.standby_time = BME280_STANDBY_TIME_62_5_MS;

	settings_sel = BME280_OSR_PRESS_SEL;
	settings_sel |= BME280_OSR_TEMP_SEL;
	settings_sel |= BME280_OSR_HUM_SEL;
	settings_sel |= BME280_STANDBY_SEL;
	settings_sel |= BME280_FILTER_SEL;
	r = bme280_set_sensor_settings(settings_sel, &dev);
	r = bme280_set_sensor_mode(BME280_NORMAL_MODE, &dev);

	printf("Temperature, Pressure, Humidity\r\n");
	while(1)
	{
		/* Delay while the sensor completes a measurement */
		dev.delay_us(70000, dev.intf_ptr);
		r = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);
		print_sensor_data(&comp_data);
	}

#if 0
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
#endif
	printf("Shutting down aardvark\n");
	aardvark_shutdown();

	return 0;
}
