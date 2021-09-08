#include <zephyr.h>
#include "message.hpp"
#include "thread_runner.hpp"

k_msgq msgq_actuator2ros;
k_msgq msgq_ros2actuator;

namespace {

char __aligned(4) msgq_actuator2ros_buffer[10 * sizeof (msg_actuator2ros)];
char __aligned(4) msgq_ros2actuator_buffer[10 * sizeof (msg_ros2actuator)];

class actuator_controller_impl {
};

class actuator_controller {
public:
    int setup() {
        k_msgq_init(&msgq_actuator2ros, msgq_actuator2ros_buffer, sizeof (msg_actuator2ros), 10);
        k_msgq_init(&msgq_ros2actuator, msgq_ros2actuator_buffer, sizeof (msg_ros2actuator), 10);
        return 0;
    }
    void loop() {
        k_msleep(10);
    }
private:
};

LEXX_THREAD_RUNNER(actuator_controller);

}

/* vim: set expandtab shiftwidth=4: */