#ifdef __AVR
// avr-gcc does not seem to supply the C++ versions of these headers
#include <math.h>
#else
#include <cmath>
#endif

float celsiusToFahrenheit(float celsius)
{
	return (celsius * 1.8f) + 32.0f;
}

float metersToFeet(float meters)
{
	return meters * 3.28084f;
}

float calculateAltitude(float pressure, float slp = 101325.0)
{
	// Getting height from a pressure reading is called the "international barometric height
	// formula". The magic value of 44330.77 was adjusted in issue #30. There's also some discussion
	// of it here: https://www.sparkfun.com/tutorials/253 This calculation is NOT designed to work
	// on non-Earthlike planets such as Mars or Venus; see NRLMSISE-00. That's why it is the
	// "international" formula, not "interplanetary". Sparkfun is not liable for incorrect altitude
	// calculations from this code on those planets. Interplanetary selfies are welcome, however.
	return (-44330.77f) *
		   (pow((pressure / slp), 0.190263f) - 1.0f); // Corrected, see issue 30 (Sparkfun Repo)
}

// From Pavel-Sayekat: https://github.com/sparkfun/SparkFun_BME280_Breakout_Board/pull/6/files
double calculateDewPoint(double temperature, double humidity)
{
	// (1) Saturation Vapor Pressure = ESGG(T)
	double RATIO = 373.15 / (273.15 + temperature);
	double RHS = -7.90298 * (RATIO - 1);
	RHS += 5.02808 * log10(RATIO);
	RHS += -1.3816e-7 * (pow(10, (11.344 * (1 - 1 / RATIO))) - 1);
	RHS += 8.1328e-3 * (pow(10, (-3.49149 * (RATIO - 1))) - 1);
	RHS += log10(1013.246);
	// factor -3 is to adjust units - Vapor Pressure SVP * humidity
	double VP = pow(10, RHS - 3) * humidity;
	// (2) DEWPOINT = F(Vapor Pressure)
	double T = log(VP / 0.61078); // temp var
	return (241.88 * T) / (17.558 - T);
}
