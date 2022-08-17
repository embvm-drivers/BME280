// SPDX-FileCopyrightText: © 2022 Embedded Artistry LLC <contact@embeddedartistry.com>
// SPDX-License-Identifier: MIT

#ifndef VIRTUAL_BME280_SHIM
#define VIRTUAL_BME280_SHIM

#include <barometric_sensor.h>
#include <humidity_sensor.h>
#include <temperature_sensor.h>

/// This definition can be used to configure the sampling interval for the
/// underlying virtual devices. New samples will only be retrieved after this
/// number of seconds have passed.
#ifndef VIRTUAL_BME280_SAMPLE_INTERVAL_IN_S
#define VIRTUAL_BME280_SAMPLE_INTERVAL_IN_S 60
#endif

/// Default sea level pressure setting that is used in altitude calculations
/// when SLP is not set by the application.
/// Internal storage format is UQ22.10
/// Defaults to 1013.25 hPa (1037568 in UQ22.10)
#ifndef VIRTUAL_BME280_DEFAULT_SEA_LEVEL_PRESSURE_IN_HPA
#define VIRTUAL_BME280_DEFAULT_SEA_LEVEL_PRESSURE_IN_HPA 1037568
#endif

/** @file Bosch BME280 Virtual Device Shims
 *
 * This module implements the temperature, pressure/altitude, and humidity
 * virtual devices using the Bosch-provided BME280 driver.
 *
 * The expected usage pattern is as follows:
 * - Hardware platform initialization configures the BME280 driver according to
 *   the application's needs
 * - Hardware platform invokes virtualBME280_initialize(), passing the
 *   BME280 driver's instance structure as an input pointer
 * - The function returns a struct containing pointers to the virtual devices
 * - The hardware platform puts these virtual device pointers in the necessary
 *   locations so they can be accessed by application code (e.g., an API called
 *   get_temp0()).
 * - The full BME280 driver is hidden from the application, and only the virtual
 *   devices are exposed.
 *
 * @note This implementation _could_ support multiple implementations, but right
 * now it only supports one BME280 device in the system. To support multiple, this
 * basic approach would work, but you would need to duplicate the shim functions
 * by name to support the number of target implementations. You could add an index
 * input parameter to the virtualBME280_initialize() function so you can
 * access the correct set. Or you could manage the allocated count internally,
 * storing associated instance pointers in an array or list, and having implementations
 * map to a particular index in the list (temp0 => inst[0], temp1 => inst[1]).
 */

typedef struct
{
	const TemperatureSensor_withCb* const temperature;
	const BarometricSensor_withCb* const barometer;
	const HumiditySensor_withCb* const humidity;
} BoschBME280VirtualInterfaces;

/** Initialize the virtual devices for the Bosch BME280 Implementation
 *
 * @pre The BME280 driver has already been initialized and configured by
 * the hardware platform initialization code.
 * @post The virtual devices are ready to be used by application software.
 *
 * @param[in] input_inst Pointer to the BME280 instance that these virtual devices
 * will map to.
 * 	@note This pointer is passed as a void* to prevent this header from requiring
 *        knowledge of bme280.h
 *
 * @returns a struct containing pointers to the corresponding virtual devices
 * that can be used by generic application code.
 */
BoschBME280VirtualInterfaces virtualBME280_initialize(void* const input_inst);

#endif // VIRTUAL_BME280_SHIM
