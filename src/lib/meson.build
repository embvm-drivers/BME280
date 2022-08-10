fixed_point_math_lib = static_library('fixed_point_math',
	sources: 'fixed_point_math.c',
	include_directories: include_directories('.'),
	c_args: '-Wno-shorten-64-to-32'
)

fixed_point_math_dep = declare_dependency(
	link_with: fixed_point_math_lib,
	include_directories: include_directories('.')
)
