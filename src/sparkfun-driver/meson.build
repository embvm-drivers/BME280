
bme280_driver = static_library('bme280_driver',
	sources: files('SparkFunBME280.cpp'),
)

bme280_driver_dep = declare_dependency(
	link_with: bme280_driver,
	include_directories: include_directories('.'),
)
