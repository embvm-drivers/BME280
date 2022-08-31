#ifndef ENVIRONMENT_CALCULATIONS_HPP_
#define ENVIRONMENT_CALCULATIONS_HPP_

float celsiusToFahrenheit(float celsius);
float metersToFeet(float meters);

/// Calculate altitude using barometric pressure
/// @param[in] pressure Pressure in units of Pascals (Pa)
/// @param[in] slp Sea level pressure in Pascals (Pa). If a value is not
///		specified, a default SLP will be supplied.
/// @returns altitude in meters
float calculateAltitude(float pressure, float slp = 101325.0);

#endif // ENVIRONMENT_CALCULATIONS_HPP_
