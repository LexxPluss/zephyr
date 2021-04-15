#include <zephyr.h>
#include <device.h>
#include <drivers/sensor.h>
#include <logging/log.h>
#include "thread_runner.hpp"

LOG_MODULE_REGISTER(sonar_controller, LOG_LEVEL_INF);

class sonar_controller {
public:
    int init() {
        dev = device_get_binding("HC-SR04_0");
        LOG_INF("SONAR controller for LexxPluss board. (%p)", dev);
        return dev == nullptr ? -1 : 0;
    }
    void run() {
        while (true) {
            measure();
            k_sleep(K_MSEC(100));
        }
    }
private:
    void measure() {
        if (sensor_sample_fetch_chan(dev, SENSOR_CHAN_ALL) == 0) {
            struct sensor_value distance;
            if (sensor_channel_get(dev, SENSOR_CHAN_DISTANCE, &distance) == 0)
                LOG_INF("%s: %d.%03dM", dev->name, distance.val1, (distance.val2 / 1000));
        }
    }
    const device *dev = nullptr;
};

LEXX_THREAD_RUNNER(sonar_controller);
