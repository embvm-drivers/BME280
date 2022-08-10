bosch_bme280_virtual_devices_lib = static_library('bosch_bme280_virtual_devices',
	sources: [
		'virtual_bosch_bme280_shim.c'
	],
	include_directories: include_directories('.'),
	dependencies: [
 		bosch_bme280_driver_dep,
		fixed_point_math_dep,
		c_virtual_device_intf_dep,
		c_linked_list_dep,
	],
	c_args: '-DVIRTUAL_BME280_VERBOSE_SAMPLING_PRINTS'
)

bosch_bme280_virtual_devices_dep = declare_dependency(
	link_with: bosch_bme280_virtual_devices_lib,
	include_directories: include_directories('.'),
	dependencies: c_virtual_device_intf_dep,
)
