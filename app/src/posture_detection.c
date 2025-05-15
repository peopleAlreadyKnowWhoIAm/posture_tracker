#include "app/posture_detection.h"

#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "app/telemetry_storage.h"
#include "app/vibration.h"
#include "app/bluetooth_support.h"

LOG_MODULE_REGISTER(posture_detection, LOG_LEVEL_INF);

#define MOVEMENT_THRESHOLD 1000u

#define POSTURE_DETECTION_ANGLE_THRESHOLD 50

#define POSTURE_NO_MOVEMENT_TIMEOUT_S (60u * 30u)

#define TELEMETRY_SUBMIT_TIMEOUT_S (60u * 30u)

#define HISTEREZIS_ANGLE 2

static bool calibration_flag = false;

static struct posture_settings settings = {
	.detection_range = 15,
	.detection_time = 10,
	.is_notifying = true,
	.x_angle_calibration = 5,
};

struct posture_work {
	struct k_work work;
	struct posture_data data;

	enum posture_state state;
	int64_t state_start_ts;
	int64_t movement_notification_ts;
	int64_t telemetry_submit_ts;

	bool is_vibrating : 1;

	struct telemetry telemetry;
};

static inline bool is_posture_angle_in_valid_range(const struct posture_data *data,
						   enum posture_state state) {
	int histeresis_angle = HISTEREZIS_ANGLE;
	if (state == POSTURE_STATE_INVALID) {
		histeresis_angle = -HISTEREZIS_ANGLE;
	}
	return abs(data->x_angle) - histeresis_angle < POSTURE_DETECTION_ANGLE_THRESHOLD &&
	       abs(data->y_angle) - histeresis_angle < POSTURE_DETECTION_ANGLE_THRESHOLD;
}

static inline bool is_posture_angle_correct(const struct posture_data *data,
					    enum posture_state state) {
	int histeresis_angle =
	    (state == POSTURE_STATE_CORRECT) ? HISTEREZIS_ANGLE : -HISTEREZIS_ANGLE;
	return (abs(data->x_angle + settings.x_angle_calibration) - histeresis_angle <
		    settings.detection_range &&
		abs(data->y_angle) - histeresis_angle < settings.detection_range);
}

static void update_stats(struct posture_work *posture_work) {
	unsigned seconds_state = (k_uptime_get() - posture_work->state_start_ts) / 1000;
	LOG_DBG("Posture state changed from %d, old state %u seconds", posture_work->state,
		seconds_state);
	struct telemetry *telemetry = &posture_work->telemetry;
	if (posture_work->state != POSTURE_STATE_MOVEMENTS) {
		telemetry->seconds_not_moving += seconds_state;
	}

	if (posture_work->state == POSTURE_STATE_CORRECT ) {
		telemetry->seconds_in_good_posture += seconds_state;
	}

	if (posture_work->state == POSTURE_STATE_INCORRECT) {
		telemetry->seconds_in_bad_posture += seconds_state;
	}	
}

static void process_data(struct k_work *work) {
	struct posture_work *posture_work = CONTAINER_OF(work, struct posture_work, work);
	struct posture_data data = posture_work->data;
	if (posture_work->state_start_ts == 0) {
		posture_work->state_start_ts = k_uptime_get();
		posture_work->movement_notification_ts = k_uptime_get();
		posture_work->telemetry_submit_ts = k_uptime_get();
	}

	enum posture_state wanted_state = POSTURE_STATE_CORRECT;
	bool is_moving = data.cm_s2_max_accel_diff > MOVEMENT_THRESHOLD;

	if (is_moving) {
		wanted_state = POSTURE_STATE_MOVEMENTS;
	} else if (!is_posture_angle_in_valid_range(&data, posture_work->state)) {
		wanted_state = POSTURE_STATE_INVALID;
	} else if (!is_posture_angle_correct(&data, posture_work->state)) {
		wanted_state = POSTURE_STATE_INCORRECT;
	} else {
		wanted_state = POSTURE_STATE_CORRECT;
	}

	if (calibration_flag && wanted_state == POSTURE_STATE_CORRECT) {
		calibration_flag = false;
		settings.x_angle_calibration = (int8_t)data.x_angle;
		LOG_INF("Calibration done, new offset %d", settings.x_angle_calibration);
	}

	if (!settings.is_notifying && wanted_state != POSTURE_STATE_MOVEMENTS) {
		wanted_state = POSTURE_STATE_INVALID;
	}

	uint32_t movement_time_diff_s =
	    (k_uptime_get() - posture_work->movement_notification_ts) / 1000;
	if (posture_work->state != POSTURE_STATE_MOVEMENTS &&
	    movement_time_diff_s > POSTURE_NO_MOVEMENT_TIMEOUT_S) {
		vibration_short_start();
		bluetooth_support_notify_movement();

		posture_work->telemetry.activeness_notifications++;
		posture_work->movement_notification_ts = k_uptime_get();
	}

	bool is_telemetry_timeout_expired =
	    (k_uptime_get() - posture_work->telemetry_submit_ts) / 1000 >
	    TELEMETRY_SUBMIT_TIMEOUT_S;
	if (is_telemetry_timeout_expired) {
		update_stats(posture_work);
		posture_work->state_start_ts = k_uptime_get();
		posture_work->telemetry_submit_ts = k_uptime_get();
		telemetry_storage_submit(&posture_work->telemetry);
		posture_work->telemetry = (struct telemetry){0};
	}

	if (wanted_state == posture_work->state) {
		bool is_incorrect_timeout_expired =
		    (k_uptime_get() - posture_work->state_start_ts) / 1000 >
		    settings.detection_time;
		if (wanted_state == POSTURE_STATE_INCORRECT && is_incorrect_timeout_expired &&
		    !posture_work->is_vibrating) {
			posture_work->is_vibrating = true;
			bluetooth_support_notify_posture();
			vibration_start();

			posture_work->telemetry.posture_notifications++;
		}
		// Nothing to do
		return;
	}

	if (posture_work->is_vibrating) {
		vibration_stop();
		posture_work->is_vibrating = false;
	}

	if (posture_work->state == POSTURE_STATE_MOVEMENTS) {
		posture_work->movement_notification_ts = k_uptime_get();
	}

	update_stats(posture_work);
	posture_work->state = wanted_state;
	LOG_INF("Posture state changed to %d", posture_work->state);
	posture_work->state_start_ts = k_uptime_get();
	bluetooth_support_notify_state(wanted_state);
}

static struct posture_work process_data_work = {
    .work = Z_WORK_INITIALIZER(process_data),
};
void posture_detection_update(struct posture_data *data) {
	if (k_work_is_pending(&process_data_work.work)) {
		return;
	}
	process_data_work.data = *data;
	k_work_submit(&process_data_work.work);
}
void posture_detection_do_calibration(void) {
	calibration_flag = true;
}

void posture_detection_set_timeout(uint8_t timeout) {
	settings.detection_time = timeout;
}

void posture_detection_set_enabled(bool enabled) {
	settings.is_notifying = enabled;
}

void posture_detection_set_working_range(uint8_t range) {
	settings.detection_range = range;
}

enum posture_state posture_detection_get_state(void) {
	return process_data_work.state;
}

struct posture_settings posture_detection_get_settings() {
	return settings;
}
