# BME280 Driver

BME280 driver implementation designed according to [Design for Change](https://embeddedartistry.com/course/designing-embedded-systems-for-change/) principles, allowing it to work with any system by merely supplying the necessary abstraction implementations.

## Table of Contents

1. [About the Project](#about-the-project)
2. [Project Status](#project-status)
3. [Getting Started](#getting-started)
    1. [Requirements](#requirements)
        1. [git-lfs](#git-lfs)
        1. [Meson Build System](#meson-build-system)
    2. [Getting the Source](#getting-the-source)
    3. [Building](#building)
        1. [Enabling Link-time Optimization](#enabling-link-time-optimization)
    4. [Installation](#installation)
    5. [Usage](#usage)
4. [Configuration Options](#configuration-options)
5. [Documentation](#documentation)
6. [Need Help?](#need-help)
7. [Contributing](#contributing)
8. [Further Reading](#further-reading)
9. [Authors](#authors)
10. [License](#license)
11. [Acknowledgments](#acknowledgements)

# About the Project

This project provides a portable and reusable BME280 driver that is not tied to any particular ecosystem.

This repository, in addition to providing an actual driver, is also meant as a demonstration for our course [Designing Embedded Software for Change](https://embeddedartistry.com/course/designing-embedded-systems-for-change/). As such, we have refactored [Sparkfun's BME280 Library](https://github.com/sparkfun/SparkFun_BME280_Arduino_Library) for Arduino, as well as provided a driver implementation on top of [Bosch's BME280 driver](https://github.com/BoschSensortec/BME280_driver).

**[Back to top](#table-of-contents)**

# Project Status

This project is currently under development as part of our [Designing Embedded Software for Change](https://embeddedartistry.com/course/designing-embedded-systems-for-change/) course.

- We have adjusted the [Sparkfun BME280 driver](https://github.com/sparkfun/SparkFun_BME280_Arduino_Library), removing its dependency on the Arduino SDK. 
    + An example application is provided for the ATMega256 (`make CROSS=avr:arduino_mega`) demonstrating that the library still works on Arduino
    + An example application is provided using the [Aardvark USB adapter](https://github.com/embvm-drivers/aardvark) (only compiles in non-cross-compilation mode for now due to an embvm-core issue with AVR)

**[Back to top](#table-of-contents)**

## Getting Started

### Requirements

This project uses [Embedded Artistry's standard Meson build system](https://embeddedartistry.com/fieldatlas/embedded-artistrys-standardized-meson-build-system/), and dependencies are described in detail [on our website](https://embeddedartistry.com/fieldatlas/embedded-artistrys-standardized-meson-build-system/).

At a minimum you will need:

* [`git-lfs`](https://git-lfs.github.com), which is used to store binary files in this repository
* [Meson](#meson-build-system) is the build system
* Some kind of compiler for your target system.
    - This repository has been tested with:
        - gcc-7, gcc-8, gcc-9
        - arm-none-eabi-gcc
        - Apple clang
        - Mainline clang

#### git-lfs

This project stores some files using [`git-lfs`](https://git-lfs.github.com).

To install `git-lfs` on Linux:

```
sudo apt install git-lfs
```

To install `git-lfs` on OS X:

```
brew install git-lfs
```

Additional installation instructions can be found on the [`git-lfs` website](https://git-lfs.github.com).

#### Meson Build System

The [Meson](https://mesonbuild.com) build system depends on `python3` and `ninja-build`.

To install on Linux:

```
sudo apt-get install python3 python3-pip ninja-build
```

To install on OSX:

```
brew install python3 ninja
```

Meson can be installed through `pip3`:

```
pip3 install meson
```

If you want to install Meson globally on Linux, use:

```
sudo -H pip3 install meson
```

**[Back to top](#table-of-contents)**

### Getting the Source

This project uses [`git-lfs`](https://git-lfs.github.com), so please install it before cloning. If you cloned prior to installing `git-lfs`, simply run `git lfs pull` after installation.

This project is hosted on GitHub. You can clone the project directly using this command:

```
git clone --recursive https://github.com/embeddedartistry/project-skeleton
```

If you don't clone recursively, be sure to run the following command in the repository or your build will fail:

```
git submodule update --init
```

**[Back to top](#table-of-contents)**

### Building

If Make is installed, the library can be built by issuing the following command:

```
make
```

This will build all targets for your current architecture.

You can clean builds using:

```
make clean
```

You can eliminate the generated `buildresults` folder using:

```
make distclean
```

You can also use  `meson` directly for compiling.

Create a build output folder:

```
meson buildresults
```

And build all targets by running

```
ninja -C buildresults
```

Cross-compilation is handled using `meson` cross files. Example files are included in the [`build/cross`](build/cross/) folder. You can write your own cross files for your specific processor by defining the toolchain, compilation flags, and linker flags. These settings will be used to compile the project.

Cross-compilation must be configured using the meson command when creating the build output folder. For example:

```
meson buildresults --cross-file build/cross/gcc_arm_cortex-m4.txt
```

Following that, you can run `make` (at the project root) or `ninja` to build the project.

Tests will not be cross-compiled. They will only be built for the native platform.

**Full instructions for building the project, using alternate toolchains, and running supporting tooling are documented in [Embedded Artistry's Standardized Meson Build System](https://embeddedartistry.com/fieldatlas/embedded-artistrys-standardized-meson-build-system/) on our website.**

**[Back to top](#table-of-contents)**

### Enabling Link-time Optimization

Link-time Optimization (LTO) can be enabled during the meson configuration stage by setting the built-in option `b_lto` to `true`:

```
meson buildresults -Db_lto=true
```

This can be combined with other build options.

**[Back to top](#table-of-contents)**

### Testing

The tests for this library are written with CMocka, which is included as a subproject and does not need to be installed on your system. You can run the tests by issuing the following command:

```
make test
```

By default, test results are generated for use by the CI server and are formatted in JUnit XML. The test results XML files can be found in `buildresults/test/`.

**[Back to top](#table-of-contents)**

## Configuration Options

The following meson project options can be set for this library when creating the build results directory with `meson`, or by using `meson configure`:

* `disable-builtins` will tell the compiler not to generate built-in function
* `disable-stack-protection` will tell the compiler not to insert stack protection calls
* `disable-rtti` will disable RTTI for C++ projects
* `disable-exceptions` will disable exceptions for C++ projects
* `enable-threading` can be used to control threaded targets and libc++ threading support
* `enable-pedantic`: Turn on `pedantic` warnings
* `enable-pedantic-error`: Turn on `pedantic` warnings and errors
* `hide-unimplemented-libc-apis`: Hides the header definitions for functions which are not actually implemented
* `enable-gnu-extensions` will enable GNU libc extensions that are implemented in this library

Options can be specified using `-D` and the option name:

```
meson buildresults -Ddisable-builtins=false
```

The same style works with `meson configure`:

```
cd buildresults
meson configure -Ddisable-builtins=false
```

**[Back to top](#table-of-contents)**

## Documentation

Documentation can be built locally by running the following command:

```
make docs
```

Documentation can be found in `buildresults/docs`, and the root page is `index.html`.

**[Back to top](#table-of-contents)**

## Need help?

If you need further assistance or have any questions, please file a GitHub issue or send us an email using the [Embedded Artistry Contact Form](http://embeddedartistry.com/contact).

You can also [reach out on Twitter: mbeddedartistry](https://twitter.com/mbeddedartistry/).

## Contributing

If you are interested in contributing to this project, please read our [contributing guidelines](docs/CONTRIBUTING.md).

## Authors

* **[Phillip Johnston](https://github.com/phillipjohnston)**

## License

Copyright © 2021 Embedded Artistry LLC

See the [LICENSE](LICENSE) file for licensing details.

For other open-source licenses, please see the [Software Inventory](docs/software_inventory.xlsx).

## Acknowledgments

Make any public acknowledgments here

**[Back to top](#table-of-contents)**

