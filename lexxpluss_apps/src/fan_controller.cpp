#include <zephyr.h>
#include "thread_runner.hpp"
#include "fan_controller.hpp"

int fan_controller::init()
{
    return 0;
}

void fan_controller::run()
{
    while (true) {
        k_msleep(1000);
    }
}

LEXX_THREAD(fan_controller);
