environment_calculations_lib = static_library('environment_calculations',
	sources: 'environment_calculations.cpp',
	include_directories: include_directories('.'),
	build_by_default: meson.is_subproject() == false
)

environment_calculations_dep = declare_dependency(
	link_with: environment_calculations_lib,
	include_directories: include_directories('.')
)
