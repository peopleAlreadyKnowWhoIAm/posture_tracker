#pragma once

#include "posture_detection.h"

void bluetooth_support_notify_posture(void);
void bluetooth_support_notify_movement(void);
void bluetooth_support_notify_state(enum posture_state state);
void bluetooth_remove_bonded_peer(void);