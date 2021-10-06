#pragma once

#include <zephyr.h>

struct tof_controller {
    static void init();
    static void run(void *p1, void *p2, void *p3);
    static k_thread thread;
};

// vim: set expandtab shiftwidth=4:
