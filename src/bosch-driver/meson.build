
bosch_bme280_driver = static_library('bosch_bme280_driver',
	sources: files('bme280.c'),
)

bosch_bme280_driver_dep = declare_dependency(
	link_with: bosch_bme280_driver,
	include_directories: include_directories('.', is_system: true),
)
