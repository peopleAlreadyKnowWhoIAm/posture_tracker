#ifndef DRIVERS_SENSOR_QMC5883L_QMC5883L_H_
#define DRIVERS_SENSOR_QMC5883L_QMC5883L_H_

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>

#define QMC5883_XYZ_DATA_START 0x00u

#define QMC5883_STATUS_REGISTER 0x06u
#define QMC5883_STATUS_DATA_READY BIT(0u)
#define QMC5883_STATUS_OVERFLOW BIT(1u)
#define QMC5883_STATUS_DATA_SKIP BIT(2u)

#define QMC5883_TEMPERATURE_START 0x07u

#define QMC5883_CONTROL_REGISTER_1 0x09u
#define QMC5883_MODE_MASK 0b11u
#define QMC5883_MODE_STANDBY 0b00u
#define QMC5883_MODE_CONTINUOUS 0b01u
#define QMC5883_DATA_RATE_MASK 0b1100u
#define QMC5883_DATA_RATE_SHIFT 2u
const static uint8_t qmc5883_data_rate_values[] = {10, 50, 100, 200};

#define QMC5883_RANGE_MASK 0b110000u
#define QMC5883_RANGE_SHIFT 4u
const static uint8_t qmc5883_range_values[] = {2, 8};
#define QMC5883_OVERSAMPLING_MASK 0b11000000u
#define QMC5883_OVERSAMPLING_SHIFT 6u
const static uint16_t qmc5883_oversampling_values[] = {512, 256, 128, 64};

#define QMC5883_CONTROL_REGISTER_2 0x0Au
#define QMC5883_INTERRUPT_ENABLE BIT(0u)
#define QMC5883_ROLL_POINTER BIT(6u)
#define QMC5883_SOFT_RESET BIT(7u)

#define QMC5883_PERIOD_FBR_REGISTER 0x0Bu
#define QMC5883_PERIOD_FBR_VALUE 0x01u 

#define QMC5883_CHIP_ID_REGISTER 0x0Du
#define QMC5883_CHIP_ID_VALUE 0xFFu

struct qmc5883_data {
        int16_t x_sample;
        int16_t y_sample;
        int16_t z_sample;
        int16_t temperature;
};
struct qmc5883_config {
        struct i2c_dt_spec i2c;
        uint8_t data_rate;
        uint8_t magnetic_range;
        uint16_t oversampling;
};

#endif /* DRIVERS_SENSOR_QMC5883L_QMC5883L_H_ */
