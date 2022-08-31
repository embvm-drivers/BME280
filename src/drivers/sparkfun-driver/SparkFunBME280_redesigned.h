/******************************************************************************
This implementation was derived from SparkFunBME280.h
This implementation deviates from the original by demonstrating design
for change principles, and breaks the dependency on the Arduino SDK.

Original:
BME280 Arduino and Teensy Driver
Marshall Taylor @ SparkFun Electronics
May 20, 2015
https://github.com/sparkfun/BME280_Breakout

This code is released under the [MIT License](http://opensource.org/licenses/MIT).
Please review the LICENSE.md file included with this example. If you have any questions
or concerns with licensing, please contact techsupport@sparkfun.com.
Distributed as-is; no warranty is given.

******************************************************************************/

// Test derived class for base class SparkFunIMU
#ifndef __BME280_H__
#define __BME280_H__

#ifdef __AVR
// avr-gcc does not supply the C++ versions of these headers
#include <stddef.h>
#include <stdint.h>
#else
#include <cstddef>
#include <cstdint>
#endif

enum bme280_comm_mode
{
	I2C_MODE = 0,
	SPI_MODE
};

// TODO: make an enumeration
#define MODE_SLEEP 0b00
#define MODE_FORCED 0b01
#define MODE_NORMAL 0b11

// TODO: move to a separate header, only available for implementation
// Register names:
#define BME280_DIG_T1_LSB_REG 0x88
#define BME280_DIG_T1_MSB_REG 0x89
#define BME280_DIG_T2_LSB_REG 0x8A
#define BME280_DIG_T2_MSB_REG 0x8B
#define BME280_DIG_T3_LSB_REG 0x8C
#define BME280_DIG_T3_MSB_REG 0x8D
#define BME280_DIG_P1_LSB_REG 0x8E
#define BME280_DIG_P1_MSB_REG 0x8F
#define BME280_DIG_P2_LSB_REG 0x90
#define BME280_DIG_P2_MSB_REG 0x91
#define BME280_DIG_P3_LSB_REG 0x92
#define BME280_DIG_P3_MSB_REG 0x93
#define BME280_DIG_P4_LSB_REG 0x94
#define BME280_DIG_P4_MSB_REG 0x95
#define BME280_DIG_P5_LSB_REG 0x96
#define BME280_DIG_P5_MSB_REG 0x97
#define BME280_DIG_P6_LSB_REG 0x98
#define BME280_DIG_P6_MSB_REG 0x99
#define BME280_DIG_P7_LSB_REG 0x9A
#define BME280_DIG_P7_MSB_REG 0x9B
#define BME280_DIG_P8_LSB_REG 0x9C
#define BME280_DIG_P8_MSB_REG 0x9D
#define BME280_DIG_P9_LSB_REG 0x9E
#define BME280_DIG_P9_MSB_REG 0x9F
#define BME280_DIG_H1_REG 0xA1
#define BME280_CHIP_ID_REG 0xD0 // Chip ID
#define BME280_RST_REG 0xE0 // Softreset Reg
#define BME280_DIG_H2_LSB_REG 0xE1
#define BME280_DIG_H2_MSB_REG 0xE2
#define BME280_DIG_H3_REG 0xE3
#define BME280_DIG_H4_MSB_REG 0xE4
#define BME280_DIG_H4_LSB_REG 0xE5
#define BME280_DIG_H5_MSB_REG 0xE6
#define BME280_DIG_H6_REG 0xE7
#define BME280_CTRL_HUMIDITY_REG 0xF2 // Ctrl Humidity Reg
#define BME280_STAT_REG 0xF3 // Status Reg
#define BME280_CTRL_MEAS_REG 0xF4 // Ctrl Measure Reg
#define BME280_CONFIG_REG 0xF5 // Configuration Reg
#define BME280_MEASUREMENTS_REG 0xF7 // Measurements register start
#define BME280_PRESSURE_MSB_REG 0xF7 // Pressure MSB
#define BME280_PRESSURE_LSB_REG 0xF8 // Pressure LSB
#define BME280_PRESSURE_XLSB_REG 0xF9 // Pressure XLSB
#define BME280_TEMPERATURE_MSB_REG 0xFA // Temperature MSB
#define BME280_TEMPERATURE_LSB_REG 0xFB // Temperature LSB
#define BME280_TEMPERATURE_XLSB_REG 0xFC // Temperature XLSB
#define BME280_HUMIDITY_MSB_REG 0xFD // Humidity MSB
#define BME280_HUMIDITY_LSB_REG 0xFE // Humidity LSB

// Class BME280_SensorSettings.  This object is used to hold settings data.  The application
// uses this classes' data directly.  The settings are adopted and sent to the sensor
// at special times, such as .begin.  Some are used for doing math.
//
// This is a kind of bloated way to do this.  The trade-off is that the user doesn't
// need to deal with #defines or enums with bizarre names.
//
// A power user would strip out BME280_SensorSettings entirely, and send specific read and
// write command directly to the IC. (ST #defines below)
//
struct BME280_SensorSettings
{
	// Main Interface selection
	bme280_comm_mode commInterface;

	// Deprecated settings
	uint8_t runMode;
	uint8_t tStandby;
	uint8_t filter;
	uint8_t tempOverSample;
	uint8_t pressOverSample;
	uint8_t humidOverSample;
	float tempCorrection; // correction of temperature - added to the result
};

// Used to hold the calibration constants.  These are used
// by the driver as measurements are being taking
struct SensorCalibration
{
	uint16_t dig_T1;
	int16_t dig_T2;
	int16_t dig_T3;

	uint16_t dig_P1;
	int16_t dig_P2;
	int16_t dig_P3;
	int16_t dig_P4;
	int16_t dig_P5;
	int16_t dig_P6;
	int16_t dig_P7;
	int16_t dig_P8;
	int16_t dig_P9;

	uint8_t dig_H1;
	int16_t dig_H2;
	uint8_t dig_H3;
	int16_t dig_H4;
	int16_t dig_H5;
	int8_t dig_H6;
};

struct BME280_SensorMeasurements
{
	float temperature;
	float pressure;
	float humidity;
};

/** BME280 Driver Interface
 *
 * This driver supports both I2C and SPI communications. By default, the driver
 * uses I2C, but this value can be overriden using the `comm_mode` argument in
 * the constructor.
 *
 * To make this driver work, you need to implement a `write_func` and `read_func`
 * that handles the data interactions between the BME280 driver and the underlying
 * communication bus implementation. Please see the notes for those type definitions for mor
 * information. These values are supplied to the driver during construction.
 * Example implementations of these functions can be found in the src/app directory.
 *
 * Class usage pattern:
 * 	- Construct the driver with a write_implementation, a read_implementation,
 *    a desired communication mode, and optionally, private data to pass to your write/read
 *  - Call begin() after the device has been powered on and the communication bus has been
 *    initialized
 *  - Change modes and/or read data as needed
 *
 * Notes for I2C communications:
 * 	- Default 7-bit address is 0x77. 0x76 can also be used by tying SDO to GND
 *  - Device supports speeds up to 3.4 MHz
 *
 * Notes SPI communications:
 * 	- Configure the bus to use SPI mode 0, with MSB first
 *  - Device supports both 3- and 4-wire SPI
 *  - Device supports speeds up to 10 MHz
 *
 * For more information on this device, see the datasheet:
 * 	https://cdn.sparkfun.com/assets/e/7/3/b/1/BME280_Datasheet.pdf
 */
class BME280
{
  public:
	/** Prototype for the driver's write function abstraction
	 *
	 * Users of this driver must supply an implementation for the write function, which is what
	 * enables the BME280 driver to communicate through the underlying platform.
	 *
	 * @note Your driver may require sending a single buffer instead of multiple independent bytes.
	 *  In this case, you can combine the two bytes into a single buffer for use with your driver.
	 *
	 * @pre Device is powered on
	 * @pre begin() has been called
	 * @post write operation has completed successfully
	 *
	 * @param register The register address to send to the device
	 * @param data The data byte to send to the device
	 * @param private_data A pointer to private data reserved for the implementor's use. Common
	 * 	uses of this parameter are to store a `this` pointer for a particular class, a pointer
	 *  to the I2C address for this device, a pointer to a GPIO line to be used for the CS pin
	 */
	using write_func = void (*)(uint8_t /*register*/, uint8_t /*data*/, void* /*private data*/);

	/** Prototype for the driver's read function abstraction
	 *
	 * Users of this driver must supply an implementation for the read function, which is what
	 * enables the BME280 driver to communicate through the underlying platform.
	 *
	 * @note This function is actually a mix of write/read: First you must send one byte (register
	 * address) to the device, and then read the data.
	 *  - For I2C, you will need to send the register in a write operation, and then follow that
	 * with a read operation. A repeated start condition is valid here.
	 *  - For SPI, your driver may let you send one byte and then switch to reading by sending 0x0.
	 * 		Other SPI drivers may require you to combine the data into a single buffer, where the
	 * first byte is the register address and the remaining bytes are 0x0. Total buffer size will be
	 * length+1 in this scenario
	 *
	 * @pre Device is powered on
	 * @pre begin() has been called
	 * @post read operation has completed successfully, and the data is stored in data_out
	 *
	 *
	 * @param register The register address to send to the device.
	 * @param data_out Buffer used to store the returned data for the driver
	 * @param length The amount of data to read from the device
	 * @param private_data A pointer to private data reserved for the implementor's use. Common
	 * 	uses of this parameter are to store a `this` pointer for a particular class, a pointer
	 *  to the I2C address for this device, a pointer to a GPIO line to be used for the CS pin
	 */
	using read_func = void (*)(uint8_t /*register*/, uint8_t* /*data_out*/, size_t /*length*/,
							   void* /*private data*/);

  public:
	// settings
	BME280_SensorSettings settings;
	SensorCalibration calibration;
	int32_t t_fine;

	// Constructor generates default BME280_SensorSettings.
	//(over-ride after construction if desired)
	BME280(write_func write_implementation, read_func read_implementation,
		   bme280_comm_mode comm_mode = I2C_MODE, void* private_data = nullptr);
	//~BME280() = default;

	/** Initialize the Device
	 *
	 * This function initializes the device, applies the sensor settings, and
	 * retrieves the SensorCalibration constants from the device.
	 *
	 * @pre Before begin() is called, this driver assumes the following is true:
	 * 	- The device is powered
	 * 	- 2ms has elapsed between supplying power and calling begin()
	 * 	- The communication bus has been properly configured and initialized
	 *  - Valid  implementations of write_func and read_func have been supplied
	 *
	 * @returns
	 * 	- true if the device was initialized and a valid chip ID was detected
	 *  - false otherwise
	 */
	bool begin();

	uint8_t getMode(void); // Get the current mode: sleep, forced, or normal
	void setMode(uint8_t mode); // Set the current mode

	void setTempOverSample(uint8_t overSampleAmount); // Set the temperature sample mode
	void setPressureOverSample(uint8_t overSampleAmount); // Set the pressure sample mode
	void setHumidityOverSample(uint8_t overSampleAmount); // Set the humidity sample mode
	void setStandbyTime(uint8_t timeSetting); // Set the standby time between measurements
	void setFilter(uint8_t filterSetting); // Set the filter

	void setReferencePressure(
		float refPressure); // Allows user to set local sea level reference pressure
	float getReferencePressure();

	bool isMeasuring(void); // Returns true while the device is taking measurement

	// Software reset routine
	void reset(void);
	void readAllMeasurements(BME280_SensorMeasurements* measurements);

	// Returns the values as floats.
	float readFloatPressure(void);
	float readFloatAltitudeMeters(void);
	float readFloatAltitudeFeet(void);
	void readFloatPressureFromBurst(uint8_t buffer[], BME280_SensorMeasurements* measurements);

	float readFloatHumidity(void);
	void readFloatHumidityFromBurst(uint8_t buffer[], BME280_SensorMeasurements* measurements);

	// Temperature related methods
	void setTemperatureCorrection(float corr);
	float readTemp(void);
	float readTempFromBurst(uint8_t buffer[]);

	// Dewpoint related methods
	// From Pavel-Sayekat: https://github.com/sparkfun/SparkFun_BME280_Breakout_Board/pull/6/files
	double dewPointC(void);
	double dewPointF(void);

	// The following utilities read and write

	// ReadRegisterRegion takes a uint8 array address as input and reads
	// a chunk of memory into that array.
	void readRegisterRegion(uint8_t*, uint8_t, uint8_t);
	// readRegister reads one register
	uint8_t readRegister(uint8_t);
	// Reads two regs, LSByte then MSByte order, and concatenates them
	// Used for two-byte reads
	int16_t readRegisterInt16(uint8_t offset);
	// Writes a byte;
	void writeRegister(uint8_t, uint8_t);

  private:
	uint8_t checkSampleValue(uint8_t userValue); // Checks for valid over sample values
	void readTempCFromBurst(uint8_t buffer[], BME280_SensorMeasurements* measurements);

	/// Converts from the raw sensor output to target representation
	/// @param[in] raw_input the value returned from the BME280 sensor
	/// @returns humidity in %RH
	float convertHumidity(int32_t raw_input);

	float _referencePressure = 101325.0; // Default but is changeable
	write_func write_ = nullptr;
	read_func read_ = nullptr;
	void* private_data_ = nullptr;
};

#endif // End of __BME280_H__ definition check
