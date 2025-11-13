# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(plant_project)

target_sources(     app PRIVATE src/main.c
                     src/sensors/soil_sensor.c
                     src/sensors/light_sensor.c
                     src/sensors/color_sensor.c
                     src/sensors/gps.c
                     src/sensors/accelerometer.c )

target_include_directories(app PRIVATE)