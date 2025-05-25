#include "app/vibration.h"

#include <zephyr/drivers/gpio.h>

static const struct gpio_dt_spec vibration_control =
    GPIO_DT_SPEC_GET(DT_NODELABEL(vibration_output), gpios);
static bool is_vibrating = false;

static void vibration_work_handler(struct k_work *) { vibration_stop(); }

static K_WORK_DELAYABLE_DEFINE(stop_vibration_work, vibration_work_handler);

void vibration_start(void) {
        if (is_vibrating) {
                return;
        }
        is_vibrating = true;
        (void)gpio_pin_set_dt(&vibration_control, 1);
}

void vibration_short_start(void) {
        vibration_start();
        k_work_schedule(&stop_vibration_work, K_MSEC(VIBRATION_SHORT_DURATION));
}

void vibration_stop(void) {
        if (!is_vibrating) {
                return;
        }
        is_vibrating = false;
        gpio_pin_set_dt(&vibration_control, 0);
}

static int vibration_gpio_init(void) {
        if (!device_is_ready(vibration_control.port)) {
                return -1;
        }

        int ret =
            gpio_pin_configure_dt(&vibration_control, GPIO_OUTPUT_INACTIVE);
        return ret;
}
SYS_INIT(vibration_gpio_init, APPLICATION, 1);
