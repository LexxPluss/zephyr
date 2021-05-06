#include <zephyr.h>
#include <device.h>
#include <drivers/sensor.h>
#include <logging/log.h>
#include <cstdio>
#include "message.hpp"
#include "thread_runner.hpp"

k_msgq sonar_controller_msgq;

namespace {

LOG_MODULE_REGISTER(sonar_controller, LOG_LEVEL_INF);

char __aligned(4) sonar_controller_msgq_buffer[10 * sizeof (sonar_message)];

class sonar_driver {
public:
    int init() {
        for (int i = 0; i < sonar_message::SENSORS; ++i) {
            char label[16];
            snprintf(label, sizeof label, "HC-SR04_%d", i);
            dev[i] = device_get_binding(label);
            if (dev[i] == nullptr)
                return -1;
            LOG_INF("SONAR controller for LexxPluss board. (%p)", dev[i]);
        }
        return 0;
    }
    void poll(uint32_t distance[sonar_message::SENSORS]) {
        for (int i = 0; i < sonar_message::SENSORS; ++i) {
            if (sensor_sample_fetch_chan(dev[i], SENSOR_CHAN_ALL) == 0) {
                sensor_value v;
                sensor_channel_get(dev[i], SENSOR_CHAN_DISTANCE, &v);
                distance[i] = v.val1 * 1000 + v.val2 / 1000;
            }
        }
    }
private:
    const device *dev[sonar_message::SENSORS] = {nullptr, nullptr, nullptr, nullptr};
};

class sonar_controller {
public:
    int setup() {
        for (int i = 0; i < sonar_message::SENSORS; ++i)
            message.distance[i] = 0;
        k_msgq_init(&sonar_controller_msgq, sonar_controller_msgq_buffer, sizeof (sonar_message), 10);
        return driver.init();
    }
    void loop() {
        driver.poll(message.distance);
        while (k_msgq_put(&sonar_controller_msgq, &message, K_NO_WAIT) != 0)
            k_msgq_purge(&sonar_controller_msgq);
        k_msleep(1);
    }
private:
    sonar_driver driver;
    sonar_message message;
};

LEXX_THREAD_RUNNER(sonar_controller);

}

/*
vi:et:sw=4:
*/
