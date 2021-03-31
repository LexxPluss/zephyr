#include <zephyr.h>
#include "thread_runner.hpp"
#include "host_communicator.hpp"

int host_communicator::init()
{
    return 0;
}

void host_communicator::run()
{
    while (true) {
        k_msleep(1000);
    }
}

LEXX_THREAD(host_communicator);
