#include <zephyr.h>
#include <device.h>
#include <drivers/sensor.h>
#include <logging/log.h>
#include <cstdio>
#include "message.hpp"
#include "thread_runner.hpp"

k_msgq imu_controller_msgq;

namespace {

LOG_MODULE_REGISTER(imu_controller, LOG_LEVEL_INF);

char __aligned(4) imu_controller_msgq_buffer[10 * sizeof (imu_message)];

class imu_controller {
public:
    int setup() {
        dev = device_get_binding("ADIS16470");
        if (dev == nullptr)
            return -1;
        for (int i = 0; i < 3; ++i) {
            message.accel[i] = 0;
            message.gyro[i] = 0;
        }
        message.temp = 0;
        k_msgq_init(&imu_controller_msgq, imu_controller_msgq_buffer, sizeof (imu_message), 10);
        LOG_INF("IMU controller for LexxPluss board. (%p)", dev);
        return 0;
    }
    void loop() {
        if (sensor_sample_fetch_chan(dev, SENSOR_CHAN_ALL) == 0) {
            sensor_value v;
            sensor_channel_get(dev, SENSOR_CHAN_ACCEL_X, &v);
            message.accel[0] = static_cast<float>(v.val1) + static_cast<float>(v.val2) * 1e-6f;
            sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Y, &v);
            message.accel[1] = static_cast<float>(v.val1) + static_cast<float>(v.val2) * 1e-6f;
            sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Z, &v);
            message.accel[2] = static_cast<float>(v.val1) + static_cast<float>(v.val2) * 1e-6f;
            sensor_channel_get(dev, SENSOR_CHAN_GYRO_X, &v);
            message.gyro[0] = static_cast<float>(v.val1) + static_cast<float>(v.val2) * 1e-6f;
            sensor_channel_get(dev, SENSOR_CHAN_GYRO_Y, &v);
            message.gyro[1] = static_cast<float>(v.val1) + static_cast<float>(v.val2) * 1e-6f;
            sensor_channel_get(dev, SENSOR_CHAN_GYRO_Z, &v);
            message.gyro[2] = static_cast<float>(v.val1) + static_cast<float>(v.val2) * 1e-6f;
            sensor_channel_get(dev, SENSOR_CHAN_DIE_TEMP, &v);
            message.temp = static_cast<float>(v.val1) + static_cast<float>(v.val2) * 1e-6f;
        }
        while (k_msgq_put(&imu_controller_msgq, &message, K_NO_WAIT) != 0)
            k_msgq_purge(&imu_controller_msgq);
        k_msleep(10);
    }
private:
    const device *dev = nullptr;
    imu_message message;
};

LEXX_THREAD_RUNNER(imu_controller);

}

/* vim: set expandtab shiftwidth=4: */
