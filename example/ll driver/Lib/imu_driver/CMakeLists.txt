cmake_minimum_required(VERSION 3.11.4 FATAL_ERROR)

option(BUILD_STATIC_LIBRARY "Build static library" OFF)

set(CMAKE_TOOLCHAIN_FILE "ARMToolchain.cmake")

project(imu_driver C CXX)

if(BUILD_STATIC_LIBRARY)
    add_library(imu_driver STATIC src/imu_driver.c)
endif()
