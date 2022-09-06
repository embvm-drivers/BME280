#ifndef BME280_PRIVATE_IMPLEMENTATION_DETAILS_HPP_
#define BME280_PRIVATE_IMPLEMENTATION_DETAILS_HPP_

#ifdef __AVR
// avr-gcc does not seem to supply the C++ versions of these headers
#include <assert.h>
#include <math.h>
#include <string.h>
#else
#include <cassert>
#include <cmath>
#include <cstring>
#endif

static int32_t assembleRawTempPressure(uint8_t* bytes)
{
	return ((uint32_t)bytes[0] << 12) | ((uint32_t)bytes[1] << 4) | ((bytes[2] >> 4) & 0x0F);
}

static int32_t assembleRawHumidity(uint8_t* bytes)
{
	return ((uint32_t)bytes[0] << 8) | ((uint32_t)bytes[1]);
}

/// Converts from the raw sensor output to the °C
/// This routine performs compensation as well, as described in the datasheet.
/// @param[in] raw_input The raw value returned from the sensor
/// @param[in] calibration The BME280 sensor calibration data for the target component
/// @param[out] t_fine Reference to the t_fine variable in the BME280 sensor instance,
///		which will be updated with the fine-resolution temperature calculation
/// use.
/// @returns temperature in °C
static float convertTemperature(int32_t raw_input, const BME280::SensorCalibration& calibration,
								int32_t& t_fine)
{
	// By datasheet, calibrate
	int64_t var1, var2;

	var1 = ((((raw_input >> 3) - ((int32_t)calibration.dig_T1 << 1))) *
			((int32_t)calibration.dig_T2)) >>
		   11;
	var2 = (((((raw_input >> 4) - ((int32_t)calibration.dig_T1)) *
			  ((raw_input >> 4) - ((int32_t)calibration.dig_T1))) >>
			 12) *
			((int32_t)calibration.dig_T3)) >>
		   14;
	t_fine = var1 + var2;

	float output = (t_fine * 5 + 128) >> 8;
	output = output / 100;

	return output;
}

/// Converts from the raw sensor output to pressure
/// This routine performs compensation for temperature, as described in the datasheet.
/// @param[in] raw_input The raw value returned from the sensor
/// @param[in] calibration The BME280 sensor calibration data for the target component
/// @param[in] t_fine Reference to the t_fine variable in the BME280 sensor instance
/// use.
/// @returns pressure in Pascals (Pa)
static float convertPressure(int32_t raw_input, const BME280::SensorCalibration& calibration,
							 const int32_t t_fine)
{
	int64_t var1, var2, p_acc;
	var1 = ((int64_t)t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)calibration.dig_P6;
	var2 = var2 + ((var1 * (int64_t)calibration.dig_P5) << 17);
	var2 = var2 + (((int64_t)calibration.dig_P4) << 35);
	var1 = ((var1 * var1 * (int64_t)calibration.dig_P3) >> 8) +
		   ((var1 * (int64_t)calibration.dig_P2) << 12);
	var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)calibration.dig_P1) >> 33;
	if(var1 == 0)
	{
		return 0; // avoid exception caused by division by zero
	}
	p_acc = 1048576 - raw_input;
	p_acc = (((p_acc << 31) - var2) * 3125) / var1;
	var1 = (((int64_t)calibration.dig_P9) * (p_acc >> 13) * (p_acc >> 13)) >> 25;
	var2 = (((int64_t)calibration.dig_P8) * p_acc) >> 19;

	// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8
	// fractional bits). Output value of “24674867” represents 24674867/256 = 96386.2 Pa =
	// 963.862 hPa
	p_acc = ((p_acc + var1 + var2) >> 8) + (((int64_t)calibration.dig_P7) << 4);

	return (float)p_acc / 256.0f;
}

/// Converts from the raw sensor output to target representation
/// @param[in] raw_input The raw value returned from the sensor
/// @param[in] calibration The BME280 sensor calibration data for the target component
/// @param[in] t_fine Reference to the t_fine variable in the BME280 sensor instance
/// @returns humidity in %RH
static float convertHumidity(int32_t raw_input, const BME280::SensorCalibration& calibration,
							 const int32_t t_fine)
{
	int32_t var1 = (t_fine - ((int32_t)76800));
	var1 = (((((raw_input << 14) - (((int32_t)calibration.dig_H4) << 20) -
			   (((int32_t)calibration.dig_H5) * var1)) +
			  ((int32_t)16384)) >>
			 15) *
			(((((((var1 * ((int32_t)calibration.dig_H6)) >> 10) *
				 (((var1 * ((int32_t)calibration.dig_H3)) >> 11) + ((int32_t)32768))) >>
				10) +
			   ((int32_t)2097152)) *
				  ((int32_t)calibration.dig_H2) +
			  8192) >>
			 14));
	var1 = (var1 - (((((var1 >> 15) * (var1 >> 15)) >> 7) * ((int32_t)calibration.dig_H1)) >> 4));
	var1 = (var1 < 0 ? 0 : var1);
	var1 = (var1 > 419430400 ? 419430400 : var1);

	// At this point we have humidity in %RH as unsigned 32 bit integer in Q22.10 format
	// (22 integer and 10 fractional bits).
	// An Output value of “47445” represents 47445/1024 = 46.333 %RH
	// The final step here is to perform this conversion.
	return (float)(var1 >> 12) / 1024.0f;
}

static float readPressureFromBurst(uint8_t buffer[], const BME280::SensorCalibration& calibration,
								   const int32_t t_fine)
{
	int32_t adc_P = assembleRawTempPressure(buffer);
	return convertPressure(adc_P, calibration, t_fine);
}

static float readTemperatureFromBurst(uint8_t buffer[],
									  const BME280::SensorCalibration& calibration, int32_t& t_fine)
{
	int32_t adc_T = assembleRawTempPressure(&buffer[3]);
	return convertTemperature(adc_T, calibration, t_fine);
}

static float readHumidityFromBurst(uint8_t buffer[], const BME280::SensorCalibration& calibration,
								   const int32_t t_fine)
{
	int32_t adc_H = assembleRawHumidity(&buffer[6]);
	return convertHumidity(adc_H, calibration, t_fine);
}

class BME280SettingChangeProtector
{
  public:
	BME280SettingChangeProtector(BME280& inst) : inst_(inst)
	{
		// Get the current mode so we can go back to it at the end
		originalMode = inst_.getMode();

		// Config will only be writeable in sleep mode, so first go to sleep mode
		inst_.setMode(BME280::op_mode::sleep);
	}

	~BME280SettingChangeProtector()
	{
		// Return to the original user's choice
		inst_.setMode(originalMode);
	}

  private:
	BME280& inst_;
	BME280::op_mode originalMode;
};

class BME280Internals
{
  public:
	static void readCompensationData(BME280& inst)
	{
		auto calibration = inst.calibration;
		// Reading all compensation data, range 0x88:A1, 0xE1:E7
		calibration.dig_T1 = ((uint16_t)((readRegister(inst, BME280_DIG_T1_MSB_REG) << 8) +
										 readRegister(inst, BME280_DIG_T1_LSB_REG)));
		calibration.dig_T2 = ((int16_t)((readRegister(inst, BME280_DIG_T2_MSB_REG) << 8) +
										readRegister(inst, BME280_DIG_T2_LSB_REG)));
		calibration.dig_T3 = ((int16_t)((readRegister(inst, BME280_DIG_T3_MSB_REG) << 8) +
										readRegister(inst, BME280_DIG_T3_LSB_REG)));

		calibration.dig_P1 = ((uint16_t)((readRegister(inst, BME280_DIG_P1_MSB_REG) << 8) +
										 readRegister(inst, BME280_DIG_P1_LSB_REG)));
		calibration.dig_P2 = ((int16_t)((readRegister(inst, BME280_DIG_P2_MSB_REG) << 8) +
										readRegister(inst, BME280_DIG_P2_LSB_REG)));
		calibration.dig_P3 = ((int16_t)((readRegister(inst, BME280_DIG_P3_MSB_REG) << 8) +
										readRegister(inst, BME280_DIG_P3_LSB_REG)));
		calibration.dig_P4 = ((int16_t)((readRegister(inst, BME280_DIG_P4_MSB_REG) << 8) +
										readRegister(inst, BME280_DIG_P4_LSB_REG)));
		calibration.dig_P5 = ((int16_t)((readRegister(inst, BME280_DIG_P5_MSB_REG) << 8) +
										readRegister(inst, BME280_DIG_P5_LSB_REG)));
		calibration.dig_P6 = ((int16_t)((readRegister(inst, BME280_DIG_P6_MSB_REG) << 8) +
										readRegister(inst, BME280_DIG_P6_LSB_REG)));
		calibration.dig_P7 = ((int16_t)((readRegister(inst, BME280_DIG_P7_MSB_REG) << 8) +
										readRegister(inst, BME280_DIG_P7_LSB_REG)));
		calibration.dig_P8 = ((int16_t)((readRegister(inst, BME280_DIG_P8_MSB_REG) << 8) +
										readRegister(inst, BME280_DIG_P8_LSB_REG)));
		calibration.dig_P9 = ((int16_t)((readRegister(inst, BME280_DIG_P9_MSB_REG) << 8) +
										readRegister(inst, BME280_DIG_P9_LSB_REG)));

		calibration.dig_H1 = ((uint8_t)(readRegister(inst, BME280_DIG_H1_REG)));
		calibration.dig_H2 = ((int16_t)((readRegister(inst, BME280_DIG_H2_MSB_REG) << 8) +
										readRegister(inst, BME280_DIG_H2_LSB_REG)));
		calibration.dig_H3 = ((uint8_t)(readRegister(inst, BME280_DIG_H3_REG)));
		calibration.dig_H4 = ((int16_t)((readRegister(inst, BME280_DIG_H4_MSB_REG) << 4) +
										(readRegister(inst, BME280_DIG_H4_LSB_REG) & 0x0F)));
		calibration.dig_H5 = ((int16_t)((readRegister(inst, BME280_DIG_H5_MSB_REG) << 4) +
										((readRegister(inst, BME280_DIG_H4_LSB_REG) >> 4) & 0x0F)));
		calibration.dig_H6 = ((int8_t)readRegister(inst, BME280_DIG_H6_REG));
	}

	static bool checkChipID(const BME280& inst)
	{
		bool valid = false;

		uint8_t chipID = readRegister(inst, BME280_CHIP_ID_REG);
		if(chipID == 0x58 || chipID == 0x60) // Is this BMP or BME?
		{
			valid = true;
		}

		return valid;
	}

	static void UpdateRegisterField(const BME280& inst, const uint8_t reg_addr, const uint8_t mask,
									const uint8_t shift, const uint8_t data)
	{
		uint8_t register_data = readRegister(inst, reg_addr);
		register_data &= ~(mask << shift);
		register_data |= (data & mask) << shift;
		writeRegister(inst, reg_addr, register_data);
	}

	static void readRegisterRegion(const BME280& inst, uint8_t* outputPointer, uint8_t offset,
								   uint8_t length)
	{
		assert(inst.read_);

		if(inst.commInterface_ == BME280::comm_mode::SPI)
		{
			offset |= 0x80; // set the read bit
		}

		memset(outputPointer, 0, length);

		inst.read_(offset, outputPointer, length, inst.private_data_);
	}

	static uint8_t readRegister(const BME280& inst, uint8_t offset)
	{
		uint8_t value = 0;

		readRegisterRegion(inst, &value, offset, 1);

		return value;
	}

	static int16_t readRegisterInt16(const BME280& inst, uint8_t offset)
	{
		uint8_t myBuffer[2];
		readRegisterRegion(inst, myBuffer, offset, 2); // Does memory transfer
		int16_t output = (int16_t)myBuffer[0] | int16_t(myBuffer[1] << 8);

		return output;
	}

	static void writeRegister(const BME280& inst, uint8_t offset, uint8_t dataToWrite)
	{
		assert(inst.write_);

		if(inst.commInterface_ == BME280::comm_mode::SPI)
		{
			offset &= 0x7F; // ensure offset does not include a read bit in position 0x80
		}

		inst.write_(offset, dataToWrite, inst.private_data_);
	}
};

#endif // BME280_PRIVATE_IMPLEMENTATION_DETAILS_HPP_
