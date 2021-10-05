#pragma once

#include <zephyr.h>

struct adc_reader {
    static void init();
    static void run(void *p1, void *p2, void *p3);
    static uint16_t get(int index);
    static k_thread thread;
    static constexpr int INDEX_DOWNWARD_L{0};
    static constexpr int INDEX_DOWNWARD_R{1};
    static constexpr int INDEX_ACTUATOR_0{2};
    static constexpr int INDEX_ACTUATOR_1{3};
    static constexpr int INDEX_ACTUATOR_2{4};
    static constexpr int INDEX_CONNECT_CART{5};
};

// vim: set expandtab shiftwidth=4:
