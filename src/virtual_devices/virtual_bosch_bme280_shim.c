// SPDX-FileCopyrightText: © 2022 Embedded Artistry LLC <contact@embeddedartistry.com>
// SPDX-License-Identifier: MIT

#include "virtual_bosch_bme280_shim.h"
#include "bme280.h"
#include "fixed_point_math.h"
#include <assert.h>
#include <linkedlist/ll.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

// NOTES:
// - Error handling is not shown here
// - Fetching new samples could be purely asynchronous if driven by a timer
//   or thread, but this implementation requires someone to call readX() to
//   trigger a new value.

#pragma mark - Prototypes -

// temp0 section
static bool temp0_readTemperature(int16_t* const temperature);
static void temp0_registerNewSampleCb(const NewTemperatureSampleCb callback);
static void temp0_unregisterNewSampleCb(const NewTemperatureSampleCb callback);
static void temp0_registerErrorCb(const TemperatureErrorCb callback);
static void temp0_unregisterErrorCb(const TemperatureErrorCb callback);

// barometric0 section
static bool baro0_readPressure(uint32_t* const pressure);
static bool baro0_readAltitude(int32_t* const altitude);
static void baro0_setSLP(uint32_t slp);
static void baro0_registerNewSampleCb(const NewBarometricSampleCb callback);
static void baro0_unregisterNewSampleCb(const NewBarometricSampleCb callback);
static void baro0_registerErrorCb(const BarometricErrorCb callback);
static void baro0_unregisterErrorCb(const BarometricErrorCb callback);

// humidity0 section
static bool humidity0_readHumidity(uint8_t* const humidity);
static void humidity0_registerNewSampleCb(const NewHumiditySampleCb callback);
static void humidity0_unregisterNewSampleCb(const NewHumiditySampleCb callback);
static void humidity0_registerErrorCb(const HumidityErrorCb callback);
static void humidity0_unregisterErrorCb(const HumidityErrorCb callback);

#pragma mark - Definitions -

typedef struct
{
	ll_t node;
	NewTemperatureSampleCb cb;
} TempNewSampleCallbackNode;

typedef struct
{
	ll_t node;
	TemperatureErrorCb cb;
} TempErrorCallbackNode;

typedef struct
{
	ll_t node;
	NewHumiditySampleCb cb;
} HumidityNewSampleCallbackNode;

typedef struct
{
	ll_t node;
	HumidityErrorCb cb;
} HumidityErrorCallbackNode;

typedef struct
{
	ll_t node;
	NewBarometricSampleCb cb;
} BaroNewSampleCallbackNode;

typedef struct
{
	ll_t node;
	BarometricErrorCb cb;
} BaroErrorCallbackNode;

/** This macro reduces duplication for assigning items to the
 * appropriate callback list
 */
#define ALLOC_AND_ASSIGN_NODE_TO_LIST(node_type, input_cb, list)   \
	{                                                              \
		node_type* new_cb = (node_type*)malloc(sizeof(node_type)); \
		assert(new_cb);                                            \
		new_cb->cb = input_cb;                                     \
		list_add(&new_cb->node, &list);                            \
	}

/** This macro reduces duplication for removing items from the
 * appropriate callback list
 */
#define FIND_AND_REMOVE_NODE_FROM_LIST(node_type, cb_input, list) \
	{                                                             \
		if(cb_input)                                              \
		{                                                         \
			node_type* current_node;                              \
			list_for_each_entry(current_node, &list, node)        \
			{                                                     \
				if(current_node->cb == cb_input)                  \
				{                                                 \
					list_del(&current_node->node);                \
					break;                                        \
				}                                                 \
			}                                                     \
		}                                                         \
	}

#pragma mark - Declarations -

struct bme280_dev* our_bme280_inst = NULL;

/// Current sea level pressure setting
/// Defaults to 1013.25 hPa
/// Internal storage: UQ22.10
static uint32_t slp_ = 1037568;
/// Indicates whether the current cached BME280 samples are valid
static bool samples_are_valid = false;
static time_t last_sampling_time;
static uint32_t sampling_interval_in_s = 60;
static uint8_t latest_humidity = 0;
static int16_t latest_temperature = 0;
static uint32_t latest_pressure = 0;
static int32_t latest_altitude = 0;

/// Mapping for virtual humidity sensor interface
static const HumiditySensor_withCb humidity0 = {
	.readHumidity = humidity0_readHumidity,
	.registerNewSampleCb = humidity0_registerNewSampleCb,
	.unregisterNewSampleCb = humidity0_unregisterNewSampleCb,
	.registerErrorCb = humidity0_registerErrorCb,
	.unregisterErrorCb = humidity0_unregisterErrorCb,
};

/// Mapping for virtual barometric sensor interface
static const BarometricSensor_withCb barometric0 = {
	.readPressure = baro0_readPressure,
	.readAltitude = baro0_readAltitude,
	.setSeaLevelPressure = baro0_setSLP,
	.registerNewSampleCb = baro0_registerNewSampleCb,
	.unregisterNewSampleCb = baro0_unregisterNewSampleCb,
	.registerErrorCb = baro0_registerErrorCb,
	.unregisterErrorCb = baro0_unregisterErrorCb,
};

/// Mapping for virtual temperature sensor interface
static const TemperatureSensor_withCb temp0 = {
	.readTemperature = temp0_readTemperature,
	.registerNewSampleCb = temp0_registerNewSampleCb,
	.unregisterNewSampleCb = temp0_unregisterNewSampleCb,
	.registerErrorCb = temp0_registerErrorCb,
	.unregisterErrorCb = temp0_unregisterErrorCb,
};

// List Declarations
LIST_INIT(tempSampleCbList);
LIST_INIT(tempErrorCbList);
LIST_INIT(baroSampleCbList);
LIST_INIT(baroErrorCbList);
LIST_INIT(humiditySampleCbList);
LIST_INIT(humidityErrorCbList);

#pragma mark - Supporting Functionality -

/**
 * Takes the current time
 * and then checks if the desired interval has elapsed
 * if so, a new sample will be acquired
 */
static bool checkIfNewSampleNeeded()
{
	time_t current_time = time(NULL);

	return ((current_time - last_sampling_time) > sampling_interval_in_s);
}

static int32_t compute_altitude(uint32_t pressure)
{
	// For this, we will use H = 44330 * [1 - (P/p0)^(1/5.255) ]
	// where H = altitude, P = pressure input, p0 = SLP
	// This calculation does not include temperature, but that can be
	// improved if necessary. The reason is that our pressure is already
	// temperature corrected.
	int64_t working_altitude = (int64_t)44330
							   << 10; // starting constant, converted to fixed point format
	// 10 here represents the fractional bit count for both values- UQ10
	int32_t pressure_ratio = (int32_t)(((int64_t)pressure * (1 << 10)) / slp_);

	// For this next step, we need to upgrade to Q16 so we can work properly with the fp_pow
	// function (1/5.255) -> 0.19029495718 In fixed-point Q16, that leads to 12471
	const int32_t exponent_value = 12471;
	int32_t pressure_component = fp_pow(pressure_ratio << 6, exponent_value);
	// Then we downgrade back to Q10
	pressure_component = pressure_component >> 6;

	// need to convert 1 to 1.0 in fixed point format, then calculate [1 - pressure_component]
	pressure_component = ((int32_t)(1 << 10) - pressure_component);

	// Final multiplication step: 44330 * pressure_component, adjusted for Q10 fixed point
	// multiplication If that final scaling step isn't done, then you get a HUUUGE number!
	working_altitude = (working_altitude * pressure_component) / (1 << 10);

	return (int32_t)working_altitude;
}

static void invoke_callbacks()
{
	BaroNewSampleCallbackNode* baro_node = NULL;
	list_for_each_entry(baro_node, &baroSampleCbList, node)
	{
		baro_node->cb(latest_pressure, latest_altitude);
	}

	TempNewSampleCallbackNode* temp_node = NULL;
	list_for_each_entry(temp_node, &tempSampleCbList, node)
	{
		temp_node->cb(latest_temperature);
	}

	HumidityNewSampleCallbackNode* humidity_node = NULL;
	list_for_each_entry(humidity_node, &humiditySampleCbList, node)
	{
		humidity_node->cb(latest_humidity);
	}
}

static bool get_new_samples()
{
	struct bme280_data compensated_data;
	int r = bme280_get_sensor_data(BME280_ALL, &compensated_data, our_bme280_inst);

	if(r == BME280_OK)
	{
#ifndef BME280_64BIT_ENABLE
#error This conversion code is dependent on 64-bit output
#endif

#ifdef VIRTUAL_BME280_VERBOSE_SAMPLING_PRINTS
		printf(
			"Retrieved new data from sensors: %d (100 * °C), %u (1024 * %%RH), %u (100 * Pascal)\n",
			compensated_data.temperature, compensated_data.humidity, compensated_data.pressure);
#endif

		// This was successful, so we take the current timestamp in order to gate the next
		// access of data from the sensor.
		last_sampling_time = time(NULL);

		// correct from 100 * °C to fixed point Q7.8
		// Surely this could be done more efficiently,
		// but I would also likely modify the compensation code
		// to return a more sensible value
		latest_temperature = (int16_t)((compensated_data.temperature * (1 << 8)) / 100);

		// convert from 1024 * % RH to integer %
		// this would have worked well as UQ10 without any conversion,
		// if you need that precision.
		latest_humidity = (uint8_t)(compensated_data.humidity >> 10);

		// correct from 100 * Pa to hPa with fixed point format UQ22.10;
		latest_pressure = (uint32_t)(((uint64_t)compensated_data.pressure << 10) / 10000);
		latest_altitude = compute_altitude(latest_pressure);

#ifdef VIRTUAL_BME280_VERBOSE_SAMPLING_PRINTS
		printf("Converted to: %d (°C in Q7.8), %d (%%RH, integral), %u (hPa, UQ22.10), %u (m, "
			   "Q21.10)\n",
			   latest_temperature, latest_humidity, latest_pressure, latest_altitude);
		printf("Sampling time: %ld s\n", last_sampling_time);
#endif

		invoke_callbacks();
	}

	return r == BME280_OK;
}

#pragma mark - Virtual Barometric Sensor Functions -

bool baro0_readPressure(uint32_t* const pressure)
{
	bool new_sample_needed = checkIfNewSampleNeeded();
	if(new_sample_needed)
	{
		// If successful, callbacks are invoked here
		samples_are_valid = get_new_samples();
	}

	if(samples_are_valid && pressure)
	{
		*pressure = latest_pressure;
	}

	return samples_are_valid;
}

bool baro0_readAltitude(int32_t* const altitude)
{
	bool new_sample_needed = checkIfNewSampleNeeded();
	if(new_sample_needed)
	{
		// If successful, callbacks are invoked here
		samples_are_valid = get_new_samples();
	}

	if(samples_are_valid && altitude)
	{
		*altitude = latest_altitude;
	}

	return samples_are_valid;
}

void baro0_setSLP(uint32_t slp)
{
	// Here, we simply update the SLP variable used to calculate the
	// altitude with the input value
	slp_ = slp;
}

void baro0_registerNewSampleCb(const NewBarometricSampleCb callback)
{
	assert(callback);
	ALLOC_AND_ASSIGN_NODE_TO_LIST(BaroNewSampleCallbackNode, callback, baroSampleCbList);
}

void baro0_unregisterNewSampleCb(const NewBarometricSampleCb callback)
{
	FIND_AND_REMOVE_NODE_FROM_LIST(BaroNewSampleCallbackNode, callback, baroSampleCbList);
}

void baro0_registerErrorCb(const BarometricErrorCb callback)
{
	ALLOC_AND_ASSIGN_NODE_TO_LIST(BaroErrorCallbackNode, callback, baroErrorCbList);
}

void baro0_unregisterErrorCb(const BarometricErrorCb callback)
{
	FIND_AND_REMOVE_NODE_FROM_LIST(BaroErrorCallbackNode, callback, baroErrorCbList);
}

#pragma mark - Virtual Humidity Sensor Functions -

bool humidity0_readHumidity(uint8_t* const humidity)
{
	bool new_sample_needed = checkIfNewSampleNeeded();
	if(new_sample_needed)
	{
		// If successful, callbacks are invoked here
		samples_are_valid = get_new_samples();
	}

	if(samples_are_valid && humidity)
	{
		*humidity = latest_humidity;
	}

	return samples_are_valid;
}

void humidity0_registerNewSampleCb(const NewHumiditySampleCb callback)
{
	assert(callback);
	ALLOC_AND_ASSIGN_NODE_TO_LIST(HumidityNewSampleCallbackNode, callback, humiditySampleCbList);
}

void humidity0_unregisterNewSampleCb(const NewHumiditySampleCb callback)
{
	FIND_AND_REMOVE_NODE_FROM_LIST(HumidityNewSampleCallbackNode, callback, humiditySampleCbList);
}

void humidity0_registerErrorCb(const HumidityErrorCb callback)
{
	assert(callback);
	ALLOC_AND_ASSIGN_NODE_TO_LIST(HumidityErrorCallbackNode, callback, humidityErrorCbList);
}

void humidity0_unregisterErrorCb(const HumidityErrorCb callback)
{
	FIND_AND_REMOVE_NODE_FROM_LIST(HumidityErrorCallbackNode, callback, humidityErrorCbList);
}

#pragma mark - Virtual Temperature Sensor Functions -

bool temp0_readTemperature(int16_t* const temperature)
{
	bool new_sample_needed = checkIfNewSampleNeeded();
	if(new_sample_needed)
	{
		// If successful, callbacks are invoked here
		samples_are_valid = get_new_samples();
	}

	if(samples_are_valid && temperature)
	{
		*temperature = latest_temperature;
	}

	return samples_are_valid;
}

void temp0_registerNewSampleCb(const NewTemperatureSampleCb callback)
{
	assert(callback);
	ALLOC_AND_ASSIGN_NODE_TO_LIST(TempNewSampleCallbackNode, callback, tempSampleCbList);
}

void temp0_unregisterNewSampleCb(const NewTemperatureSampleCb callback)
{
	FIND_AND_REMOVE_NODE_FROM_LIST(TempNewSampleCallbackNode, callback, tempSampleCbList);
}

void temp0_registerErrorCb(const TemperatureErrorCb callback)
{
	assert(callback);
	ALLOC_AND_ASSIGN_NODE_TO_LIST(TempErrorCallbackNode, callback, tempErrorCbList);
}

void temp0_unregisterErrorCb(const TemperatureErrorCb callback)
{
	FIND_AND_REMOVE_NODE_FROM_LIST(TempErrorCallbackNode, callback, tempErrorCbList);
}

#pragma mark - Public Interfaces -

BoschBME280VirtualInterfaces initialize_bme280_virtual_devices(void* const input_inst)
{
	assert(input_inst);
	our_bme280_inst = (struct bme280_dev*)input_inst;

	// NOTE:
	// Other actions can be taken here:
	// - start a thread that periodically reads new samples from the device
	//   so that you have a pure async method
	// - set up a timer to do the same thing

	return (BoschBME280VirtualInterfaces){&temp0, &barometric0, &humidity0};
}
