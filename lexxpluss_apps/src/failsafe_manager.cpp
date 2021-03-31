#include <zephyr.h>
#include "thread_runner.hpp"
#include "failsafe_manager.hpp"

int failsafe_manager::init()
{
    return 0;
}

void failsafe_manager::run()
{
    while (true) {
        k_msleep(1000);
    }
}

LEXX_THREAD(failsafe_manager);
