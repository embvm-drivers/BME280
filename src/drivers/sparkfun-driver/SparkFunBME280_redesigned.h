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
	/// Indicates whether the driver will communicate with the device
	/// via SPI or I2C.
	enum class comm_mode
	{
		I2C,
		SPI
	};

	struct Measurements
	{
		float temperature;
		float pressure;
		float humidity;
	};

	/// Operational modes supported by the BME280 part
	enum class op_mode : uint8_t
	{
		sleep = 0b00,
		forced = 0b01,
		normal = 0b11,
	};

	/// Oversampling during measurement deraeses noise, but also
	/// consumes more power.
	enum class oversampling : uint8_t
	{
		/// None means "skip data altogether"
		none = 0,
		oversample_1x,
		oversample_2x,
		oversample_4x,
		oversample_8x,
		oversample_16x,
	};

	/// This controls the time between measurements on the BME280 sensor.
	/// Higher times = lower power consumption.
	enum class standby : uint8_t
	{
		for_0_5ms = 0,
		for_62_5_ms,
		for_125_ms,
		for_250_ms,
		for_500_ms,
		for_1000_ms,
		for_10_ms,
		for_20_ms,
	};

	/// Filtering essentially changes the bandwidth of the
	/// temperature and pressure output signals.
	/// The output of the measurement step becomes:
	/// data_filtered = (data_filtered_old * (filter_coefficient - 1) + data_ADC)/filter_coefficient
	enum class filtering : uint8_t
	{
		/// Filter is turned off
		off = 0,
		coeff_2,
		coeff_4,
		coeff_8,
		coeff_16
	};

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
	/// BME280 Constructor
	/// @param[in] write_implementation The implementation of the write_func that will be used by
	/// 	the driver.
	/// @param[in] read_implementation The implementation of the read_func that will be used by
	/// 	the driver.
	/// @param[in] comm_mode The communciation method that corresponds with the chosen read/write
	/// 	implementation. If not specified, defaults to I2C communications.
	/// @param[in] private_data Optional pointer to user-specified private data. This value will
	///		be supplied to both the read_func and write_func implementations.
	BME280(write_func write_implementation, read_func read_implementation,
		   comm_mode comm_mode = comm_mode::I2C, void* private_data = nullptr)
		: commInterface_(comm_mode), write_(write_implementation), read_(read_implementation),
		  private_data_(private_data)
	{
	}
	~BME280() = default;

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
	 * @post The device is initialized, calibration data has been read from the device,
	 * 	and the following default settings have been supplied:
	 * 	- Standby time of 0.5ms
	 *  - filtering off
	 *  - 1x temperature, pressure, and humidity oversampling
	 *
	 * @returns
	 * 	- true if the device was initialized and a valid chip ID was detected
	 *  - false otherwise
	 */
	bool begin();

	/// Perform a software reset
	/// Run .begin() afterwards to use the device.
	void reset(void);

	/// Read the current operating mode from the device.
	op_mode getMode(void);
	/// Set a new operating mode on the device
	void setMode(op_mode mode);

	/// Read all sensor registers as a burst. See BME280 Datasheet section 4. Data readout
	void readAllMeasurements(BME280::Measurements& measurements);
	/// Read the current pressure from the device
	float readPressure(void);
	/// Read the current humidity from the device
	float readHumidity(void);
	/// Read the current temperature from the device
	float readTemperature(void);
	/// Checks the measuring bit on the device.
	/// @returns true if the device is currently making a measurement
	bool isMeasuring(void);

	/// Set the driver-side temperature offset value.
	/// This value is applied AFTER compensation calculations.
	/// @param[in] corr Offset to apply to temperature measurements.
	void setTemperatureCorrection(float corr);
	/// Set temperature sensor oversampling value (default: 1x)
	void setTempOverSample(oversampling overSampleAmount);
	/// Set pressure sensor oversampling value (default: 1x)
	void setPressureOverSample(oversampling overSampleAmount);
	/// Set humidity sensor oversampling value (default: 1x)
	void setHumidityOverSample(oversampling overSampleAmount);
	/// Set the time between measurements on the sensor side (default: 0.5ms)
	void setStandbyTime(standby timeSetting);
	/// Set the device-side filter coefficient (default: off)
	void setFilter(filtering filterSetting);

  public:
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

  private:
	/// Selection of I2C/SPI communication interface
	comm_mode commInterface_;
	/// correction of temperature - added to the result of the compensation calculation
	float tempCorrection_ = 0.0f;
	/// Implementation of the write_ abstraction
	write_func write_ = nullptr;
	/// Implementation of the read_ abstraction
	read_func read_ = nullptr;
	/// Optional user-specified private data pointer that will be supplied to the
	/// read_ and write_ abstractions
	void* private_data_ = nullptr;
	SensorCalibration calibration;
	int32_t t_fine;

	// This class handles some internal details, keeping these interfaces out
	// of the header while still accessing instance-specific config/calibration information.
	friend class BME280Internals;
};

#endif // End of __BME280_H__ definition check
