// SPDX-FileCopyrightText: © 2022 Embedded Artistry LLC <contact@embeddedartistry.com>
// SPDX-License-Identifier: MIT

#include <assert.h>
#include <hardware_platform.h>
#include <stdio.h>
#include <unistd.h> // for usleep

#pragma mark - Declarations -

/// This will be set by the signal handler, which controls when to
/// quit based on keyboard input
volatile bool abort_program_ = false;

uint32_t latest_pressure = 0;
int32_t latest_altitude = 0;
int16_t latest_temperature = 0;
uint8_t latest_humidity = 0;

#pragma mark - Functions -

// This enables graceful termination of our program using CTRL+C
void signal_handler(int signal)
{
	(void)signal;
	abort_program_ = true;
}

static void print_float_vals(int16_t temperature, uint8_t humidity, uint32_t pressure,
							 int32_t altitude)
{
	double temp_converted = (double)temperature / (double)(1 << 8);
	double pressure_converted = (double)pressure / (double)(1 << 10);
	double altitude_converted = (double)altitude / (double)(1 << 10);

	printf("Temperature: %.2f°C, Humidity: %d%%, Pressure: %.2f hPa, Altitude: %.2f m\n",
		   temp_converted, humidity, pressure_converted, altitude_converted);
}

static void baro0_sample_cb(uint32_t pressure, int32_t altitude)
{
	latest_pressure = pressure;
	latest_altitude = altitude;
}

static void humidity0_sample_cb(uint8_t humidity)
{
	latest_humidity = humidity;
}

#pragma mark - Main -

int main(void)
{
	printf("Running C virtual device example\n");
	hwPlatform_initialize();
	const TemperatureSensor_withCb* const temp0 = hwPlatform_GetTemp0();
	const BarometricSensor_withCb* const baro0 = hwPlatform_GetBaro0();
	const HumiditySensor_withCb* const humidity0 = hwPlatform_GetHumidity0();

	bool valid = false;

	// Note that pressure, altitude, and humidity only get
	// updated through callbacks
	baro0->registerNewSampleCb(baro0_sample_cb);
	humidity0->registerNewSampleCb(humidity0_sample_cb);

	while(!abort_program_)
	{
		// This shows synchronous reads of the sensor
		valid = temp0->readTemperature(&latest_temperature);
		assert(valid);

		print_float_vals(latest_temperature, latest_humidity, latest_pressure, latest_altitude);

		usleep(5 * 1000 * 1000);
	}

	printf("Shutting down aardvark\n");
	hwPlatform_shutdown();

	return 0;
}
