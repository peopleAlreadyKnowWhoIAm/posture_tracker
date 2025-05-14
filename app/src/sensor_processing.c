#include "zephyr/drivers/sensor.h"
#include "zephyr/logging/log.h"
#include <math.h>

#include "app/sensor_processing.h"
#include "app/posture_detection.h"

LOG_MODULE_REGISTER(sensor_processing, LOG_LEVEL_INF);

#define M_PI 3.14159265358979323846

#define MOVEMENT_THRESHOLD 200u

#define MEASUREMENTS_POOL 10

struct accel_cm_s2_ts {
  int16_t x;
  int16_t y;
  int16_t z;
  int32_t timestamp;
};

struct proceess_sensor_arg {
  struct k_work_delayable work;
  const struct device *accel_sensor;
  struct accel_cm_s2_ts last_measurements[MEASUREMENTS_POOL];
  unsigned measurement_used;
  int64_t start_ts;
};

struct angle {
  int16_t main;
  int16_t side;
};

static inline int16_t convert_to_cm_s2(const struct sensor_value val) {
  return (int16_t)(val.val1 * 1000 + val.val2 / 1000);
}

static struct accel_cm_s2_ts
from_sensor_vals(const struct sensor_value val[3]) {
  return (struct accel_cm_s2_ts){
      .x = convert_to_cm_s2(val[0]),
      .y = convert_to_cm_s2(val[1]),
      .z = convert_to_cm_s2(val[2]),
      .timestamp = (uint32_t)k_uptime_get(),
  };
}

static struct angle
accel_to_avg_angle(const struct accel_cm_s2_ts accels[MEASUREMENTS_POOL]) {
  int32_t x = 0;
  int32_t y = 0;
  int32_t z = 0;
  for (unsigned i = 0; i < MEASUREMENTS_POOL; i++) {
    x += accels[i].x;
    y += accels[i].y;
    z += accels[i].z;
  }
  x /= MEASUREMENTS_POOL;
  y /= MEASUREMENTS_POOL;
  z /= MEASUREMENTS_POOL;

  float main_angle_rad = atan2f((float)y, (float)x);
  float side_angle_rad = atan2f((float)z, (float)x);
  return (struct angle){
      .main = (int16_t)(main_angle_rad * 180.0f / (float)M_PI),
      .side = (int16_t)(side_angle_rad * 180.0f / (float)M_PI),
  };
}

static inline unsigned norm_accel(const struct accel_cm_s2_ts accel) {
  return sqrtf(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);
}

static unsigned
max_accel_diff(const struct accel_cm_s2_ts values[MEASUREMENTS_POOL]) {
    unsigned max_diff = 0;
    int norm = norm_accel(values[0]);
  for (unsigned i = 1; i < MEASUREMENTS_POOL; i++) {
    int norm_i = norm_accel(values[i]);
    unsigned diff = abs(norm_i - norm);
    if (diff > max_diff) {
        max_diff = diff;
    }
    norm = norm_i;
  }
  return max_diff;
}

static void process_sensor(struct k_work *work) {
  struct k_work_delayable *dwork = k_work_delayable_from_work(work);
  struct proceess_sensor_arg *arg_struct =
      CONTAINER_OF(dwork, struct proceess_sensor_arg, work);
  const struct device *sensor = arg_struct->accel_sensor;
  if (sensor == NULL) {
    LOG_ERR("Sensor not present. Stoping processing");
    return;
  }
  // In case the function is called for the first time
  if (arg_struct->start_ts == 0) {
    arg_struct->start_ts = k_uptime_get();
  }

  if (sensor_sample_fetch(sensor) < 0) {
    LOG_ERR("Sensor fetch error. Stopping processing");
    return;
  }
  struct sensor_value val[3];

  if (sensor_channel_get(sensor, SENSOR_CHAN_ACCEL_XYZ, val) < 0) {
    LOG_ERR("Sensor data get error. Stopping processing");
    return;
  }

  struct accel_cm_s2_ts measurement = from_sensor_vals(val);
  arg_struct->last_measurements[arg_struct->measurement_used] = measurement;
  arg_struct->measurement_used++;

  if (arg_struct->measurement_used >= MEASUREMENTS_POOL) {
    unsigned max_acc_diff =
        max_accel_diff(arg_struct->last_measurements);
    struct angle angles = accel_to_avg_angle(arg_struct->last_measurements);
    LOG_DBG("Movement_detected: %d, Angles: %d, %d", max_acc_diff,
            angles.main, angles.side);
    struct posture_data data = {
        .x_angle = angles.main,
        .y_angle = angles.side,
        .cm_s2_max_accel_diff = max_acc_diff,
    };
    posture_detection_update(&data);
    // TODO export data to other consumers

    arg_struct->measurement_used = 0;
    arg_struct->start_ts = k_uptime_get();
  }

  k_work_reschedule(&arg_struct->work, K_MSEC(50));
}

static struct proceess_sensor_arg sensor_arg = {
    .accel_sensor = NULL,
    .measurement_used = 0,
    .start_ts = 0,
};

void sensor_processing_start(const struct device *const accel_sensor) {
  sensor_arg.accel_sensor = accel_sensor;
  k_work_init_delayable(&sensor_arg.work, process_sensor);
  k_work_schedule(&sensor_arg.work, K_MSEC(50));
}

void sensor_processing_stop(void) {
  if (sensor_arg.accel_sensor == NULL) {
    return;
  }
  struct k_work_sync sync;
  k_work_cancel_delayable_sync(&sensor_arg.work, &sync);
  sensor_arg = (struct proceess_sensor_arg){};
}
