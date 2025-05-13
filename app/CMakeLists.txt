#-------------------------------------------------------------------------------
# Zephyr Example Application
#
# Copyright (c) 2021 Nordic Semiconductor ASA
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
add_compile_options(-Wall -Wextra -Wpedantic)

project(app LANGUAGES C)

target_sources(app PRIVATE
    src/main.c
    src/bluetooth_support.c
    src/sensor_processing.c
    src/posture_detection.c
    src/vibration.c
    src/telemetry_storage.c)
