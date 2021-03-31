#include <zephyr.h>
#include "thread_runner.hpp"
#include "commander.hpp"

int commander::init()
{
    return 0;
}

void commander::run()
{
    while (true) {
        k_msleep(1000);
    }
}

LEXX_THREAD(commander);
