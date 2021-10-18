#pragma once

#include <zephyr.h>

struct msg_ros2actuator {
    uint16_t data[3];
    uint8_t type;
    static constexpr uint8_t CWCCW{0};
    static constexpr uint8_t DUTY{1};
} __attribute__((aligned(4)));

struct msg_actuator2ros {
    int32_t encoder_count[3];
    int32_t current[3];
    int32_t connect;
} __attribute__((aligned(4)));

struct actuator_controller {
    static void init();
    static void run(void *p1, void *p2, void *p3);
    static k_thread thread;
};

extern k_msgq msgq_actuator2ros;
extern k_msgq msgq_ros2actuator;

// vim: set expandtab shiftwidth=4:
