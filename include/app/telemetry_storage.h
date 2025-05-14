#pragma once

#include <stddef.h>
#include <stdint.h>

struct telemetry {
    uint32_t timestamp;
    uint8_t posture_notifications;
    uint8_t activeness_notifications;
    uint16_t seconds_not_moving;
    uint16_t seconds_in_bad_posture;
    uint16_t seconds_in_good_posture;
};

void telemetry_storage_submit(struct telemetry *telemetry);

int telemetry_get_portion(uint8_t *buf, size_t* len);
