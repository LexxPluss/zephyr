#include <zephyr.h>
#include <device.h>
#include <drivers/sensor.h>
#include "message.hpp"
#include "thread_runner.hpp"

k_msgq msgq_uss2ros;

namespace {

char __aligned(4) msgq_uss2ros_buffer[10 * sizeof (msg_uss2ros)];

class uss_fetcher {
public:
    static void runner(void *p1, void *p2, void *p3) {
        uss_fetcher *self = static_cast<uss_fetcher*>(p1);
        if (self->init(static_cast<const char*>(p2), static_cast<const char*>(p3)) == 0)
            self->run();
    }
    void get_distance(uint32_t distance[2]) const {
        distance[0] = this->distance[0];
        distance[1] = this->distance[1];
    }
private:
    int init(const char *label0, const char *label1) {
        dev[0] = device_get_binding(label0);
        if (dev[0] == nullptr)
            return -1;
        if (label1 != nullptr) {
            dev[1] = device_get_binding(label1);
            if (dev[1] == nullptr)
                return -1;
        }
        return 0;
    }
    void run() {
        while (true) {
            if (sensor_sample_fetch_chan(dev[0], SENSOR_CHAN_ALL) == 0) {
                sensor_value v;
                sensor_channel_get(dev[0], SENSOR_CHAN_DISTANCE, &v);
                distance[0] = v.val1 * 1000 + v.val2 / 1000;
            }
            if (dev[1] != nullptr) {
                if (sensor_sample_fetch_chan(dev[1], SENSOR_CHAN_ALL) == 0) {
                    sensor_value v;
                    sensor_channel_get(dev[1], SENSOR_CHAN_DISTANCE, &v);
                    distance[1] = v.val1 * 1000 + v.val2 / 1000;
                }
            }
            k_msleep(1);
        }
    }
    const device *dev[2]{nullptr, nullptr};
    uint32_t distance[2]{0, 0};
} fetcher[4];
K_THREAD_DEFINE(tid_fetcher_0, 2048, &uss_fetcher::runner, &fetcher[0], "MB1604_0", "MB1604_1", 6, K_FP_REGS, 1000);
K_THREAD_DEFINE(tid_fetcher_1, 2048, &uss_fetcher::runner, &fetcher[1], "MB1604_2", nullptr, 6, K_FP_REGS, 1000);
K_THREAD_DEFINE(tid_fetcher_2, 2048, &uss_fetcher::runner, &fetcher[2], "MB1604_3", nullptr, 6, K_FP_REGS, 1000);
K_THREAD_DEFINE(tid_fetcher_3, 2048, &uss_fetcher::runner, &fetcher[3], "MB1604_4", nullptr, 6, K_FP_REGS, 1000);

class uss_controller {
public:
    int setup() {
        k_msgq_init(&msgq_uss2ros, msgq_uss2ros_buffer, sizeof (msg_uss2ros), 10);
        return 0;
    }
    void loop() {
        msg_uss2ros message;
        uint32_t distance[2];
        fetcher[0].get_distance(distance);
        message.front_left = distance[0];
        message.front_right = distance[1];
        fetcher[1].get_distance(distance);
        message.left = distance[0];
        fetcher[2].get_distance(distance);
        message.right = distance[0];
        fetcher[3].get_distance(distance);
        message.back = distance[0];
        while (k_msgq_put(&msgq_uss2ros, &message, K_NO_WAIT) != 0)
            k_msgq_purge(&msgq_uss2ros);
        k_msleep(50);
    }
};

LEXX_THREAD_RUNNER(uss_controller);

}

/* vim: set expandtab shiftwidth=4: */
