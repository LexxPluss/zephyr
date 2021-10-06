#include <zephyr.h>
#include "actuator_controller.hpp"
#include "adc_reader.hpp"
#include "imu_controller.hpp"
#include "led_controller.hpp"
#include "pgv_controller.hpp"
#include "rosserial.hpp"
#include "tof_controller.hpp"
#include "uss_controller.hpp"

namespace {

K_THREAD_STACK_DEFINE(actuator_controller_stack, 2048);
K_THREAD_STACK_DEFINE(adc_reader_stack, 2048);
K_THREAD_STACK_DEFINE(imu_controller_stack, 2048);
K_THREAD_STACK_DEFINE(led_controller_stack, 2048);
K_THREAD_STACK_DEFINE(pgv_controller_stack, 2048);
K_THREAD_STACK_DEFINE(rosserial_stack, 2048);
K_THREAD_STACK_DEFINE(tof_controller_stack, 2048);
K_THREAD_STACK_DEFINE(uss_controller_stack, 2048);

#define RUN(name) \
    k_thread_create(&name::thread, name##_stack, K_THREAD_STACK_SIZEOF(name##_stack), \
                    name::run, nullptr, nullptr, nullptr, 5, K_FP_REGS, K_NO_WAIT);

}

void main()
{
    actuator_controller::init();
    adc_reader::init();
    imu_controller::init();
    led_controller::init();
    pgv_controller::init();
    rosserial::init();
    tof_controller::init();
    uss_controller::init();
    RUN(actuator_controller);
    RUN(adc_reader);
    RUN(imu_controller);
    RUN(led_controller);
    RUN(pgv_controller);
    RUN(tof_controller);
    RUN(uss_controller);
    RUN(rosserial); // The rosserial thread will be started last.
    while (true) {
        k_msleep(1000);
    }
}

// vim: set expandtab shiftwidth=4:
