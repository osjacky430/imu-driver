[![Build Status](https://travis-ci.org/osjacky430/imu_driver.svg?branch=master)](https://travis-ci.org/osjacky430/imu_driver)
# IMU driver
Handles the master side of communication of openIMU300ZI, this is a generic imu driver that can be used on different driver, but mainly for STM32F4. Ports to known driver such as HAL, LL, LibopenCM3 are implemented.
## Getting started
The instruction will help you to build the static library, the example, and the test.
### Prerequisites
The toolchain in use is arm-none-eabi-gcc (xPack GNU ARM Embedded GCC, 64-bit) 8.3.1 20190703 (release), which is shipped with GNU MCU Eclipse, however, newer version of arm-none-eabi-gcc will be fine, too.
After you downloaded the toolchain, remember to set the binary file folder to PATH, i.e.
```
export PATH="$PATH:/path/to/arm-gcc-toolchain/bin"
```
The examples are all Eclipse projects, therefore Eclipse IDE, with GNU MCU Eclipse plugin installed, is preferred.
### Build static library
To build static library, just create the build file, and run cmake with flag -DBUILD_STATIC_LIBRARY=ON.
```
cd /path/to/imu_driver
mkdir build && cd build
cmake -DBUILD_STATIC_LIBRARY=ON ..
cmake --build ./
```
### Build examples
If you are working on Eclipse, then simply open projects from the file system will do the work, however, some properties need to be reconfigured. For HAL and LL, you can also create the project using STM32CubeMX, and drag the whole file (or imu_driver/include & imu_driver/src) to your desire directory (e.g. folder "Lib" in the examples). As for OCM3, there are tons of things to be reconfigured, the configuration procedure will be added in the future if needed.
### Build tests
Currently the test is done using Visual Studio Test Explorer under google test framework, this part will be added if I manage to conduct unit test using arm-none-eabi-gcc toolchain.
## Built with
[cmake](https://cmake.org/) - Build, and test.
