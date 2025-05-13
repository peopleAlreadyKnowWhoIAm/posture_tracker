#pragma once

#include <stdint.h>

struct posture_data {
    int16_t x_angle;
    int16_t y_angle;
    unsigned cm_s2_max_accel_diff;
};

enum posture_state {
    POSTURE_STATE_CORRECT,
    POSTURE_STATE_INVALID,
    POSTURE_STATE_MOVEMENTS,
    POSTURE_STATE_INCORRECT,
};

void posture_detection_update(struct posture_data *data);

enum posture_state posture_detection_get_state(void);
