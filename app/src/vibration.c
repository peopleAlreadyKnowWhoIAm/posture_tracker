#include "app/vibration.h"

#include <zephyr/drivers/gpio.h>

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static bool is_vibrating = false;

static void vibration_work_handler(struct k_work *) {
    vibration_stop();
}

static K_WORK_DELAYABLE_DEFINE(stop_vibration_work, vibration_work_handler);

void vibration_start(void) {
    if (is_vibrating) {
        return;
    }
    is_vibrating = true;
   (void)gpio_pin_set_dt(&led, 1); 
}

void vibration_short_start(void) {
    if (is_vibrating) {
        return;
    }
    is_vibrating = true;
   gpio_pin_set_dt(&led, 1);
   k_work_schedule(&stop_vibration_work, K_MSEC(VIBRATION_SHORT_DURATION));
   is_vibrating = false;
}

void vibration_stop(void) {
    if (!is_vibrating) {
        return;
    }
    is_vibrating = false;
   gpio_pin_set_dt(&led, 0);
}

static int vibration_gpio_init(void) {
    if (!device_is_ready(led.port)) {
        return -1;
    }

    int ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
    return ret;
}
SYS_INIT(vibration_gpio_init, APPLICATION, 1);
