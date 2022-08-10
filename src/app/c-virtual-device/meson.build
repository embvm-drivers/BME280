executable('c_virtual_device_bosch',
	files('virtual_devices.c'),
	dependencies: [
		bosch_bme280_driver_dep,
		aardvark_vendor_native_driver_dep,
		c_virtual_device_intf_dep,
		c_linked_list_dep
	],
	install: false,
	native: true
)
