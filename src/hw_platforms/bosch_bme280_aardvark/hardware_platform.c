// SPDX-FileCopyrightText: © 2022 Embedded Artistry LLC <contact@embeddedartistry.com>
// SPDX-License-Identifier: MIT

#include "hardware_platform.h"
#include "aardvark.h"
#include "bme280.h"
#include "virtual_bosch_bme280_shim.h"
#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h> // for usleep()

#pragma mark - Prototypes -

// Aardvark section
static void aardvark_initialize(void);
static void aardvark_shutdown(void);
static int8_t aardvark_i2c_read(uint8_t reg_addr, uint8_t* reg_data, uint32_t length,
								void* intf_ptr);
static int8_t aardvark_i2c_write(uint8_t reg_addr, const uint8_t* reg_data, uint32_t length,
								 void* intf_ptr);

// BME280 driver section
static void delay_us(uint32_t period, void* intf_ptr);
static void initialize_bme280(void);

#pragma mark - Declarations -

static Aardvark handle_ = 0;
static AardvarkConfig mode_ = AA_CONFIG_SPI_I2C; // SPI + I2C
static uint8_t BME280_I2C_Addr_7bit = 0x76;

const TemperatureSensor_withCb* temp0 = NULL;
const BarometricSensor_withCb* baro0 = NULL;
const HumiditySensor_withCb* humidity0 = NULL;

/// BME280 driver instance, mapped to our desired Aardvark implementations
struct bme280_dev bme280_inst = {
	.intf_ptr = &BME280_I2C_Addr_7bit,
	.intf = BME280_I2C_INTF,
	.read = aardvark_i2c_read,
	.write = aardvark_i2c_write,
	.delay_us = delay_us,
};

#pragma mark - Aardvark Functions -

/** Convenience function to initilize the Aardvark device with our
 * expected settings.
 *
 * @post Aardvark device has been successfully initialized
 */
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

int8_t aardvark_i2c_read(uint8_t reg_addr, uint8_t* reg_data, uint32_t length, void* intf_ptr)
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

#pragma mark - BME280 Driver Functions -

void delay_us(uint32_t period, void* intf_ptr)
{
	(void)intf_ptr;
	usleep(period);
}

/** Convenience function to initilize BME280 with our
 * expected settings.
 *
 * @pre Aardvark device has been initialized
 * @post BME280 is successfully initialized
 */
void initialize_bme280(void)
{
	uint8_t settings_sel;

	int r = bme280_init(&bme280_inst);
	assert(r == BME280_OK);

	/* Recommended mode of operation: Weather monitoring */
	bme280_inst.settings.osr_h = BME280_OVERSAMPLING_1X;
	bme280_inst.settings.osr_p = BME280_OVERSAMPLING_1X;
	bme280_inst.settings.osr_t = BME280_OVERSAMPLING_1X;
	bme280_inst.settings.filter = BME280_FILTER_COEFF_OFF;
	// TODO: confirm if needed
	// bme280_inst.settings.standby_time = BME280_STANDBY_TIME_62_5_MS;

	// In weather mode:
	// only low data rate is needed - 1/s, or 1/min
	// Noise of pressure values is of no concern.

	settings_sel = BME280_OSR_PRESS_SEL;
	settings_sel |= BME280_OSR_TEMP_SEL;
	settings_sel |= BME280_OSR_HUM_SEL;
	settings_sel |= BME280_STANDBY_SEL;
	// TODO: confirm if needed
	// settings_sel |= BME280_FILTER_SEL;
	r = bme280_set_sensor_settings(settings_sel, &bme280_inst);
	assert(r == BME280_OK);
	r = bme280_set_sensor_mode(BME280_FORCED_MODE, &bme280_inst);
	assert(r == BME280_OK);
}

#pragma mark - Public Interfaces -

void hwPlatform_initialize()
{
	aardvark_initialize();
	initialize_bme280();
	BoschBME280VirtualInterfaces intf = virtualBME280_initialize(&bme280_inst);
	temp0 = intf.temperature;
	baro0 = intf.barometer;
	humidity0 = intf.humidity;
}

/** Prepare the hardware platform for shutdown
 *
 * @post System can be safely shut down
 */
void hwPlatform_shutdown()
{
	aardvark_shutdown();
}

const TemperatureSensor_withCb* hwPlatform_GetTemp0()
{
	assert(temp0);
	return temp0;
}
const BarometricSensor_withCb* hwPlatform_GetBaro0()
{
	assert(baro0);
	return baro0;
}

const HumiditySensor_withCb* hwPlatform_GetHumidity0()
{
	assert(humidity0);
	return humidity0;
}
