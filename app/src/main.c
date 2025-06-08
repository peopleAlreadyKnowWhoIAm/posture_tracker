/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#include "app/bluetooth_support.h"
#include "zephyr/drivers/gpio.h"
#include "zephyr/sys/poweroff.h"
#include "zephyr/sys/printk.h"
#include <zephyr/drivers/sensor.h>
#include <zephyr/input/input.h>
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

        if (init_device(bmi160)) {
                LOG_ERR("Gyro not ready");
                return 0;
        }

        printk("Initialization complete\n");

        sensor_processing_start(bmi160);

        while (1) {
                k_sleep(K_MSEC(500));
        } 

        return 0;
}
static void input_callback(struct input_event *event, void *user_data) {
        LOG_INF("Input event: type=%d, code=%d, value=%d", event->type,
                event->code, event->value);
        if (event->type != INPUT_EV_KEY || event->value != 0) {
                return;
        }
        if (event->code == INPUT_KEY_POWER) {

                const struct gpio_dt_spec power_button =
                    GPIO_DT_SPEC_GET(DT_NODELABEL(button_0), gpios);
                int rc = gpio_pin_configure_dt(&power_button, GPIO_INPUT);
                if (rc < 0) {
                        printf("Could not configure sw0 GPIO (%d)\n", rc);
                        return;
                }

                rc = gpio_pin_interrupt_configure_dt(&power_button,
                                                     GPIO_INT_LEVEL_ACTIVE);

                if (rc < 0) {
                        printf("Could not configure sw0 GPIO interrupt (%d)\n",
                               rc);
                        return;
                }
                printf("Power button pressed, shutting down...\n");
                sys_poweroff();
        } else if (event->code == INPUT_KEY_DELETE) {
                bluetooth_remove_bonded_peer();
        }
}

INPUT_CALLBACK_DEFINE(NULL, input_callback, NULL);
