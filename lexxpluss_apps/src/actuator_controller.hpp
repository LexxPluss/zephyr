#pragma once

#include <zephyr.h>

struct msg_ros2actuator {
    struct {
        int32_t location;
        int8_t direction;
        uint8_t power;
    } actuators[3];
    uint8_t type;
    static constexpr int8_t DOWN{-1}, STOP{0}, UP{1};
    static constexpr uint8_t CONTROL{0}, LOCATION{1};
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
