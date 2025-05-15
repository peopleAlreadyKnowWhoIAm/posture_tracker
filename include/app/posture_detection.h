#pragma once

#include <stdint.h>
#include <stdbool.h>

struct posture_data {
    int16_t x_angle;
    int16_t y_angle;
    unsigned cm_s2_max_accel_diff;
};

struct posture_settings {
    uint8_t detection_time;
    uint8_t detection_range;
    bool is_notifying;
    int8_t x_angle_calibration;
};

enum posture_state {
    POSTURE_STATE_CORRECT,
    POSTURE_STATE_INVALID,
    POSTURE_STATE_MOVEMENTS,
    POSTURE_STATE_INCORRECT,
};

void posture_detection_update(struct posture_data *data);

enum posture_state posture_detection_get_state(void);

void posture_detection_do_calibration(void);
void posture_detection_set_timeout(uint8_t timeout);
void posture_detection_set_enabled(bool enabled);
void posture_detection_set_working_range(uint8_t range);
struct posture_settings posture_detection_get_settings(void);
