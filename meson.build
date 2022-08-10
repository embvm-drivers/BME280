project('BME280 Driver',
	['c', 'cpp'],
	default_options : [
		'warning_level=3',
		'werror=false',
		# This project defaults to a release build
		'debug=false',
		'optimization=2',
		# `build.*` options affect `native: true targets`
		# plain options affect `native: false` targets.
		'c_std=c11', 'build.c_std=c11',
		'cpp_std=c++17', 'build.cpp_std=c++17',
	],
	meson_version: '>=0.56.0',
	version: '0.1'
)

################################
# Project Options and Settings #
################################

# Pick up our common compiler variables + desired_*_flags variables
subdir('build/compiler')
subdir('build/compiler/c')
subdir('build/compiler/cpp')

if get_option('debug')
	add_project_arguments('-DDEBUG',
		language: ['c', 'cpp'])
	add_project_arguments('-DDEBUG',
		language: ['c', 'cpp'], native: true)
endif

if get_option('disable-builtins')
	desired_common_compile_flags += '-fno-builtin'
endif

if get_option('disable-stack-protection')
	desired_common_compile_flags += '-fno-stack-protector'
endif

if get_option('disable-rtti')
	desired_cpp_compile_flags += '-fno-rtti'
	desired_native_cpp_compile_flags += '-fno-rtti'
endif

if get_option('disable-exceptions')
	desired_cpp_compile_flags += ['-fno-exceptions', '-fno-unwind-tables']
	desired_native_cpp_compile_flags += ['-fno-exceptions', '-fno-unwind-tables']
endif

if get_option('enable-pedantic')
	desired_common_compile_flags += '-pedantic'
else
	desired_common_compile_flags += '-Wno-pedantic'
endif

if get_option('enable-pedantic-error')
	desired_common_compile_flags += '-pedantic-error'
endif

# Silence the warning when -nostdinc and -nostdinc++ are specified together
desired_common_compile_flags += ['-Wno-unused-command-line-argument']

compile_settings_list = [
	{'lang': 'c', 'compiler': host_c_compiler, 'flags': desired_c_compile_flags, 'isnative': false},
	{'lang': 'c', 'compiler': native_c_compiler, 'flags': desired_native_c_compile_flags, 'isnative': true},
	{'lang': 'cpp', 'compiler': host_cpp_compiler, 'flags': desired_cpp_compile_flags, 'isnative': false},
	{'lang': 'cpp', 'compiler': native_cpp_compiler, 'flags': desired_native_cpp_compile_flags, 'isnative': true},
]

link_settings_list = [
	{'lang': 'c', 'compiler': host_c_compiler, 'flags': [], 'isnative': false},
	{'lang': 'c', 'compiler': native_c_compiler, 'flags': [], 'isnative': true},
	{'lang': 'cpp', 'compiler': host_cpp_compiler, 'flags': [], 'isnative': false},
	{'lang': 'cpp', 'compiler': native_cpp_compiler, 'flags': [], 'isnative': true},
]

# Process the compilation flags
subdir('build/compiler/check-and-apply-flags')

#################
# Build Modules #
#################

# Include reusable build modules here via subdir() calls
subdir('build/linker/linker-script-as-property')
subdir('build/linker/linker-map')
subdir('build/test/cmocka')
subdir('build/objcopy')

###############
# Subprojects #
###############

# TODO: Uncomment if you want to use the embvm-core constructs in this implementation
#embvm_core_subproject = subproject('embvm-core')
#framework_include_dep = embvm_core_subproject.get_variable('framework_include_dep')
#framework_host_include_dep = embvm_core_subproject.get_variable('framework_host_include_dep')
#framework_native_include_dep = embvm_core_subproject.get_variable('framework_native_include_dep')

# This dep needs to be refactored - currently pulls in emvbm, and it will fail to compile for cross buidl with AVR
# because libc does not have AVR support. We do not need the embvm-core for this project at this time.
if meson.is_cross_build() == false
	aardvark_driver_subproject = subproject('aardvark')
	aardvark_vendor_native_driver_dep = aardvark_driver_subproject.get_variable('aardvark_vendor_native_driver_dep')
endif

if meson.is_cross_build() and host_machine.cpu_family() == 'avr' and meson.is_subproject() == false
	arduinocore = subproject('arduinocore-avr')
	arduinocore_dep = arduinocore.get_variable('arduinocore_dep')
	arduinocore_main_dep = arduinocore.get_variable('arduinocore_main_dep')
	arduinocore_wire_dep = arduinocore.get_variable('arduinocore_wire_dep')
endif

c_interfaces = subproject('c-interfaces')
c_virtual_device_intf_dep = c_interfaces.get_variable('c_virtual_device_intf_dep')

#######################
# Process Source Tree #
#######################

# Add files to this variable if you want them analyzed by clang-tidy
clangtidy_files = []
# Add dependencies to this array for testing with Catch2
catch2_tests_dep = []

# TODO: update the following trees!
subdir('src')
subdir('test')

# Defined after src and test so catch2_dep is fully populated
# when creating the built-in targets
subdir('build/test/catch2')

###################
# Tooling Modules #
###################

subdir('build/analysis/clang-tidy')
subdir('build/analysis/complexity')
subdir('build/analysis/cppcheck')
subdir('build/analysis/sloccount')
subdir('build/analysis/vale')
subdir('build/format')
subdir('build/docs/doxygen')

#############
# Packaging #
#############

# These macros allow you to simplify the declaration of includes for common
# include paths. Examples:
#	build_root_include.format('doc'),
#	app_include.format('APP')
build_root_include = meson.build_root() + ':@0@'
app_include = meson.build_root() / 'src/app:@0@'
lib_include = meson.build_root() / 'src/lib:@0@'

host_pkg_files = [
	build_root_include.format('docs'),
	app_include.format('APP')
]

native_pkg_files = [
	build_root_include.format('docs'),
	app_include.format('APP_native')
]

# Invoke the package module
subdir('build/package')
