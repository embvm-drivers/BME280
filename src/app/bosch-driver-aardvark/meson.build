###################################
# Bosch Driver + Aardvark Example #
###################################

executable('bosch_aardvark_i2c_example',
	files('AardvarkI2CExample.c'),
	dependencies: [
		bosch_bme280_driver_dep,
		aardvark_vendor_native_driver_dep
	],
	install: false,
	native: true
)
