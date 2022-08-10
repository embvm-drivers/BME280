// SPDX-FileCopyrightText: © 2022 Embedded Artistry LLC <contact@embeddedartistry.com>
// SPDX-License-Identifier: MIT

#ifndef EXAMPLE_HARDWARE_PLATFORM_H_
#define EXAMPLE_HARDWARE_PLATFORM_H_

#include <barometric_sensor.h>
#include <humidity_sensor.h>
#include <temperature_sensor.h>

/** Initialize System Hardware
 *
 * This function should be invoked by main() (or other appropriate function in
 * the initialization path)
 *
 *	@post System hardware has been initialized and configured according
 *  to the application's needs.
 */
void hwPlatform_initialize();

/** Prepare the hardware platform for shutdown
 *
 * @post System can be safely shut down
 */
void hwPlatform_shutdown();

const TemperatureSensor_withCb* hwPlatform_GetTemp0();
const BarometricSensor_withCb* hwPlatform_GetBaro0();
const HumiditySensor_withCb* hwPlatform_GetHumidity0();

#endif // EXAMPLE_HARDWARE_PLATFORM_H_
