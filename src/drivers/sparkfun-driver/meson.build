sparkfun_bme280_driver = static_library('sparkfun_bme280_driver',
	sources: files('SparkFunBME280.cpp'),
	cpp_args: [
		'-Wno-old-style-cast',
		'-Wno-sign-conversion',
		'-Wno-shorten-64-to-32',
		'-Wno-double-promotion',
		'-Wno-implicit-float-conversion'
	]
)

sparkfun_bme280_driver_dep = declare_dependency(
	link_with: sparkfun_bme280_driver,
	include_directories: include_directories('.'),
)

redesigned_sparkfun_bme280_driver = static_library('redesigned_sparkfun_bme280_driver',
	sources: files('SparkFunBME280_redesigned.cpp'),
	# REMOVE ME AND FIX WARNINGS IN THIS FILE
	cpp_args: [
		'-Wno-old-style-cast',
		'-Wno-sign-conversion',
		'-Wno-shorten-64-to-32',
		'-Wno-double-promotion'
	],
)

redesigned_sparkfun_bme280_driver_dep = declare_dependency(
	link_with: redesigned_sparkfun_bme280_driver,
	include_directories: include_directories('.'),
)
