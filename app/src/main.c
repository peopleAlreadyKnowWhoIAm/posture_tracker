/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#include "zephyr/bluetooth/bluetooth.h"
#include "zephyr/bluetooth/services/nus.h"
#include "zephyr/settings/settings.h"
#include "zephyr/sys/printk.h"
#include <math.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "app/sensor_processing.h"

#include <app_version.h>

LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

#define BLINK_PERIOD_MS_STEP 100U
#define BLINK_PERIOD_MS_MAX 1000U

int init_device(const struct device *const dev) {

        if (!device_is_ready(dev)) {
                LOG_ERR("Device %s is not ready\n", dev->name);
                return 1;
        }

        printk("Device %p name is %s\n", dev, dev->name);

        return 0;
}

int main(void) {
        const struct device *const bmi160 = DEVICE_DT_GET_ANY(bosch_bmi160);

        printk("Zephyr not Example Application %s\n", APP_VERSION_STRING);

        // int err = bt_nus_cb_register(&nus_listener, NULL);
        // if (err) {
        //         LOG_ERR("Failed to register NUS callback: %d\n", err);
        //         return err;
        // }

	// if (IS_ENABLED(CONFIG_SETTINGS)) {
	// 	settings_load();
	// }

        if (init_device(bmi160)) {
                LOG_ERR("Gyro not ready");
                return 0;
        }

        printk("Initialization complete\n");

        sensor_processing_start(bmi160);

        while (1) {
                // if (sensor_sample_fetch(bmi160) < 0) {
                //         printf("Magnetometer sample update error.\n");
                // }
                // struct sensor_value val[3];
                // if (sensor_channel_get(bmi160, SENSOR_CHAN_ACCEL_XYZ, val) < 0) {
                //         printf("Cannot read qmc5883 magnetometer channels.\n");
                //         return 0;
                // } 
                // if (val[0].val1 < min_x) {
                //         min_x = val[0].val1;
                //         needPrint = true;
                // }
                // if (val[0].val1 > max_x) {
                //         max_x = val[0].val1;
                //         needPrint = true;
                // }
                // if (val[1].val1 < min_y) {
                //         min_y = val[1].val1;
                //         needPrint = true;
                // }
                // if (val[1].val1 > max_y) {
                //         max_y = val[1].val1;
                //         needPrint = true;
                // }
                // if (val[2].val1 < min_z) {
                //         min_z = val[2].val1;
                //         needPrint = true;
                // }
                // if (val[2].val1 > max_z) {
                //         max_z = val[2].val1;
                //         needPrint = true;
                // }
                // if (needPrint) {
                //         printf("Min: %d, %d, %d\n", min_x, min_y, min_z);
                //         printf("Max: %d, %d, %d\n", max_x, max_y, max_z);
                // }
                // printf("X: %d, Y: %d, Z: %d\n", val[0].val1, val[1].val1, val[2].val1);
                // if (sensor_sample_fetch(bmi160) < 0) {
                //         printf("Gyro sample update error.\n");
                // }
                // print_gyro_data(bmi160);
                // print_magnetometer_data(qmc5883);
                // printf("Magnetometer (uT): X=%d, Y=%d, Z=%d\n",
                //        (val[0].val1 - x_offset) * x_scale / 1000,
                //        (val[1].val1 - y_offset) * y_scale / 1000,
                //        (val[2].val1 - z_offset) * z_scale / 1000);
                k_sleep(K_MSEC(500));
        }                // if (sensor_sample_fetch(bmi160) < 0) {
                //         printf("Gyro sample update error.\n");
                // }
// `X: 178.086029, Y: 60.604840, Z: -9.371119                                       
// X: 163.209274, Y: -46.786598, Z: -38.774082                                     
// X: 178.521027, Y: -21.187965, Z: 32.469353                                      

        return 0;
}
