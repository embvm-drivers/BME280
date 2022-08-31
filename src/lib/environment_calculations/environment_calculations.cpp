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
