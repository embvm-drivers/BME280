####################
# Aardvark Example #
####################

executable('aardvark_i2c_example',
	files('AardvarkI2CExample.cpp'),
	dependencies: [
		redesigned_sparkfun_bme280_driver_dep,
		aardvark_vendor_native_driver_dep,
		environment_calculations_dep
	],
	install: false,
	native: true
)

executable('aardvark_spi_example',
	files('AardvarkSPIExample.cpp'),
	dependencies: [
		redesigned_sparkfun_bme280_driver_dep,
		aardvark_vendor_native_driver_dep,
		environment_calculations_dep
	],
	install: false,
	native: true
)

##################
# ATMega Example #
##################

if meson.is_cross_build() and host_machine.cpu_family() == 'avr' and meson.is_subproject() == false
	modified_example1_basic_readings = executable('modified_example1_basic_readings',
		sources: files('Modified_Example1_BasicReadings.cpp'),
		dependencies:
		[
			redesigned_sparkfun_bme280_driver_dep,
			environment_calculations_dep,
			arduinocore_main_dep,
			arduinocore_wire_dep
		],
		install: false
	)

	modified_example1_basic_readings_bin = custom_target('modified_example1_basic_readings.hex',
		input: modified_example1_basic_readings,
		output: 'modified_example1_basic_readings.hex',
		command: host_hex_conversion,
		build_by_default: true
	)

	modified_example1_basic_readings_hex = custom_target('modified_example1_basic_readings.bin',
		input: modified_example1_basic_readings,
		output: 'modified_example1_basic_readings.bin',
		command: host_bin_conversion,
		build_by_default: true
	)

# AVRDUDE command
# /Users/phillip/Library/Arduino15/packages/arduino/tools/avrdude/6.3.0-arduino17/bin/avrdude -C/Users/phillip/Library/Arduino15/packages/arduino/tools/avrdude/6.3.0-arduino17/etc/avrdude.conf -v -patmega2560 -cwiring -P/dev/cu.usbmodem2101 -b115200 -D -Uflash:w:buildresults/src/example-app/modified_example1_basic_readings.hex:i
endif
