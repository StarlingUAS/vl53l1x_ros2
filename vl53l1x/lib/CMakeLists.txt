cmake_minimum_required(VERSION 3.5)
project(vl53l1x_vendor)

add_library(rpi_vl53l1x STATIC "")

FILE(GLOB vl53l1x_core core/*.c)
FILE(GLOB vl53l1x_platform platform/*.c)

target_sources(rpi_vl53l1x PRIVATE
                ${vl53l1x_core}
                ${vl53l1x_platform}
                i2c/i2c.cpp
            )
target_include_directories(rpi_vl53l1x PUBLIC
            core
            platform
            i2c
            rpi_vl53l1x
        )