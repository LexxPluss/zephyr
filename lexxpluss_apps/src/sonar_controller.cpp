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

class locker {
public:
    locker(k_mutex &mutex) : mutex(mutex) {k_mutex_lock(&mutex, K_FOREVER);}
    ~locker() {k_mutex_unlock(&mutex);}
private:
    k_mutex &mutex;
};

class sonar_driver {
public:
    static void runner(void *p1, void *p2, void *p3) {
        sonar_driver *self = static_cast<sonar_driver*>(p1);
        if (self->init(static_cast<const char*>(p2)) == 0)
            self->run();
    }
    bool is_sampled() {
        locker l(mutex);
        return sampled;
    }
    uint32_t get_distance() {
        locker l(mutex);
        sampled = false;
        return distance;
    }
private:
    int init(const char *label) {
        k_mutex_init(&mutex);
        dev = device_get_binding(label);
        LOG_INF("SONAR controller for LexxPluss board. (%p)", dev);
        return dev == nullptr ? -1 : 0;
    }
    void run() {
        while (true) {
            uint32_t start = k_uptime_get_32();
            if (sensor_sample_fetch_chan(dev, SENSOR_CHAN_ALL) == 0) {
                sensor_value v;
                sensor_channel_get(dev, SENSOR_CHAN_DISTANCE, &v);
                locker l(mutex);
                distance = v.val1 * 1000 + v.val2 / 1000;
            }
            {
                locker l(mutex);
                sampled = true;
            }
            uint32_t now = k_uptime_get_32();
            uint32_t elapsed = now - start;
            uint32_t waitms = 1;
            if (elapsed < 60)
                waitms = 60 - elapsed;
            k_msleep(waitms);
        }
    }
    k_mutex mutex;
    const device *dev = nullptr;
    uint32_t distance = 0;
    bool sampled = false;
} drivers[sonar_message::SENSORS];

K_THREAD_DEFINE(tid_driver_0, 2048, &sonar_driver::runner, &drivers[0], "HC-SR04_0", nullptr, 5, K_FP_REGS, 1000);
K_THREAD_DEFINE(tid_driver_1, 2048, &sonar_driver::runner, &drivers[1], "HC-SR04_1", nullptr, 5, K_FP_REGS, 1000);
K_THREAD_DEFINE(tid_driver_2, 2048, &sonar_driver::runner, &drivers[2], "HC-SR04_2", nullptr, 5, K_FP_REGS, 1000);
K_THREAD_DEFINE(tid_driver_3, 2048, &sonar_driver::runner, &drivers[3], "HC-SR04_3", nullptr, 5, K_FP_REGS, 1000);

class sonar_controller {
public:
    int setup() {
        for (int i = 0; i < sonar_message::SENSORS; ++i)
            message.distance[i] = 0;
        k_msgq_init(&sonar_controller_msgq, sonar_controller_msgq_buffer, sizeof (sonar_message), 10);
        return 0;
    }
    void loop() {
        while (true) {
            bool sampled = true;
            for (int i = 0; i < sonar_message::SENSORS; ++i) {
                if (!drivers[i].is_sampled()) {
                    sampled = false;
                    break;
                }
            }
            if (sampled)
                break;
            k_msleep(1);
        }
        for (int i = 0; i < sonar_message::SENSORS; ++i)
            message.distance[i] = drivers[i].get_distance() / 10;
        while (k_msgq_put(&sonar_controller_msgq, &message, K_NO_WAIT) != 0)
            k_msgq_purge(&sonar_controller_msgq);
        k_msleep(1);
    }
private:
    sonar_message message;
};

LEXX_THREAD_RUNNER(sonar_controller);

}

/* vim: set expandtab shiftwidth=4: */
