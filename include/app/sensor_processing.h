#pragma once

#include <zephyr/device.h>

void sensor_processing_start(const struct device *const accel_sensor);
void sensor_processing_stop(void);
