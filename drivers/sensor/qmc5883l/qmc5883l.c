#include "zephyr/devicetree.h"
#define DT_DRV_COMPAT qst_qmc5883l

#include "qmc5883l.h"
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(HMC5883L, CONFIG_SENSOR_LOG_LEVEL);

static int qmc5883_update_config(const struct device *dev) {
        const struct qmc5883_config *config = dev->config;

        uint8_t sampling_frequency = UINT8_MAX;
        for (uint8_t i = 0; i < ARRAY_SIZE(qmc5883_data_rate_values); i++) {
                if (qmc5883_data_rate_values[i] == config->data_rate) {
                        sampling_frequency = i;
                        break;
                }
        }

        uint8_t magnetic_range = UINT8_MAX;
        for (uint8_t i = 0; i < ARRAY_SIZE(qmc5883_range_values); i++) {
                if (qmc5883_range_values[i] == config->magnetic_range) {
                        magnetic_range = i;
                        break;
                }
        }

        uint8_t oversampling = UINT8_MAX;
        for (uint8_t i = 0; i < ARRAY_SIZE(qmc5883_oversampling_values); i++) {
                if (qmc5883_oversampling_values[i] == config->oversampling) {
                        oversampling = i;
                        break;
                }
        }
        if (sampling_frequency == UINT8_MAX || magnetic_range == UINT8_MAX ||
            oversampling == UINT8_MAX) {
                LOG_ERR("Invalid configuration values.");
                return -EINVAL;
        }

        uint8_t config_value = (sampling_frequency << QMC5883_DATA_RATE_SHIFT) |
                               (magnetic_range << QMC5883_RANGE_SHIFT) |
                               (oversampling << QMC5883_OVERSAMPLING_SHIFT) |
                               (QMC5883_MODE_CONTINUOUS);

        if (i2c_reg_write_byte_dt(&config->i2c, QMC5883_CONTROL_REGISTER_1,
                                  config_value) < 0) {
                LOG_ERR("Failed to write configuration register.");
                return -EIO;
        }
        return 0;
}

static int qmc5883_sample_fetch(const struct device *dev,
                                 enum sensor_channel chan) {
        struct qmc5883_data *data = dev->data;
        const struct qmc5883_config *config = dev->config;

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL);

        uint8_t buf[6];
        if (i2c_burst_read_dt(&config->i2c, QMC5883_XYZ_DATA_START, buf, 6) < 0) {
                LOG_ERR("Failed to fetch sample.");
                return -EIO;
        }

        data->x_sample = (int16_t)((buf[1] << 8) | buf[0]);
        data->y_sample = (int16_t)((buf[3] << 8) | buf[2]);
        data->z_sample = (int16_t)((buf[5] << 8) | buf[4]);

        return 0;
}

static int qmc5883_channel_get(const struct device *dev,
                                enum sensor_channel chan,
                                struct sensor_value *val) {
        const struct qmc5883_data *data = dev->data;

        switch (chan) {
        case SENSOR_CHAN_MAGN_X:
                val->val1 = data->x_sample;
                break;
        case SENSOR_CHAN_MAGN_Y:
                val->val1 = data->y_sample;
                break;
        case SENSOR_CHAN_MAGN_Z:
                val->val1 = data->z_sample;
                break;
        case SENSOR_CHAN_MAGN_XYZ:
                val[0].val1 = data->x_sample;
                val[1].val1 = data->y_sample;
                val[2].val1 = data->z_sample;
                break;
        default:
                return -ENOTSUP;
        }
        return 0;
}

static DEVICE_API(sensor, qmc5883_driver_api) = {
        .sample_fetch = qmc5883_sample_fetch,
        .channel_get = qmc5883_channel_get,
};

int qmc5883_init(const struct device *dev) {
        const struct qmc5883_config *config = dev->config;

        if (!device_is_ready(config->i2c.bus)) {
                LOG_ERR("I2C bus device not ready");
                return -ENODEV;
        }

        /* check chip ID */
        uint8_t id;
        if (i2c_reg_read_byte_dt(&config->i2c, QMC5883_CHIP_ID_REGISTER, &id) <
            0) {
                LOG_ERR("Failed to read chip ID.");
                return -EIO;
        }

        if (id != QMC5883_CHIP_ID_VALUE) {
                LOG_ERR("Invalid chip ID.");
                return -EINVAL;
        }

        if (i2c_reg_write_byte_dt(&config->i2c, QMC5883_CONTROL_REGISTER_2,
                                  QMC5883_SOFT_RESET) < 0) {
                LOG_ERR("Failed to reset chip.");
                return -EIO;
        }
        k_sleep(K_MSEC(10));

        if (i2c_reg_write_byte_dt(&config->i2c, QMC5883_PERIOD_FBR_REGISTER,
                                  QMC5883_PERIOD_FBR_VALUE) < 0) {
                LOG_ERR("Failed to set period.");
                return -EIO;
        }

        if (qmc5883_update_config(dev) < 0) {
                LOG_ERR("Failed to update configuration.");
                return -EIO;
        }
        return 0;
}

#define QMC5883_DEFINE(inst)                                                        \
        static struct qmc5883_data qmc5883_data_##inst;                               \
        static struct qmc5883_config qmc5883_config_##inst = {                  \
                .i2c = I2C_DT_SPEC_INST_GET(inst),                                   \
                .data_rate = DT_INST_PROP(inst, sampling_frequency),            \
                .magnetic_range = DT_INST_PROP(inst, magnetic_field_range), \
                .oversampling = DT_INST_PROP(inst, oversampling),                   \
        };\
        SENSOR_DEVICE_DT_INST_DEFINE(inst, qmc5883_init, NULL,\
                &qmc5883_data_##inst, &qmc5883_config_##inst, POST_KERNEL, \
                CONFIG_SENSOR_INIT_PRIORITY, &qmc5883_driver_api); 

DT_INST_FOREACH_STATUS_OKAY(QMC5883_DEFINE)