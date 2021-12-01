#include <zephyr.h>
#include <logging/log.h>
#include "adc_reader.hpp"
#include "tof_controller.hpp"

k_msgq msgq_tof2ros;

namespace {

LOG_MODULE_REGISTER(tof);

char __aligned(4) msgq_tof2ros_buffer[8 * sizeof (msg_tof2ros)];

class tof_controller_impl {
public:
    int init() const {
        k_msgq_init(&msgq_tof2ros, msgq_tof2ros_buffer, sizeof (msg_tof2ros), 8);
        return 0;
    }
    void run() const {
        while (true) {
            msg_tof2ros message;
            message.left = adc_reader::get(adc_reader::INDEX_DOWNWARD_L);
            message.right = adc_reader::get(adc_reader::INDEX_DOWNWARD_R);
            while (k_msgq_put(&msgq_tof2ros, &message, K_NO_WAIT) != 0)
                k_msgq_purge(&msgq_tof2ros);
            k_msleep(20);
        }
    }
} impl;

}

void tof_controller::init()
{
    impl.init();
}

void tof_controller::run(void *p1, void *p2, void *p3)
{
    impl.run();
}

k_thread tof_controller::thread;

// vim: set expandtab shiftwidth=4:
