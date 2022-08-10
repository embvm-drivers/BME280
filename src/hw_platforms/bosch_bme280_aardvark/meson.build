bosch_bme280_aardvark_hardware_platform_lib = static_library('hardware_platform',
	sources: [
		'hardware_platform.c',
	],
	include_directories: include_directories('.'),
	dependencies: [
		bosch_bme280_driver_dep,
 		bosch_bme280_virtual_devices_dep,
		aardvark_vendor_native_driver_dep,
		c_virtual_device_intf_dep
	]
)

bosch_bme280_aardvark_hardware_platform_dep = declare_dependency(
	link_with: bosch_bme280_aardvark_hardware_platform_lib,
	include_directories: include_directories('.'),
	dependencies: c_virtual_device_intf_dep,
)
