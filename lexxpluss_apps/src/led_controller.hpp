#pragma once

#include <zephyr.h>

struct msg_ros2led {
    uint32_t pattern;
    uint8_t rgb[3];
    static constexpr uint32_t NONE = 0;
    static constexpr uint32_t EMERGENCY_STOP = 1;
    static constexpr uint32_t AMR_MODE = 2;
    static constexpr uint32_t AGV_MODE = 3;
    static constexpr uint32_t MISSION_PAUSE = 4;
    static constexpr uint32_t PATH_BLOCKED = 5;
    static constexpr uint32_t MANUAL_DRIVE = 6;
    static constexpr uint32_t CHARGING = 10;
    static constexpr uint32_t WAITING_FOR_JOB = 11;
    static constexpr uint32_t LEFT_WINKER = 12;
    static constexpr uint32_t RIGHT_WINKER = 13;
    static constexpr uint32_t BOTH_WINKER = 14;
    static constexpr uint32_t MOVE_ACTUATOR = 15;
    static constexpr uint32_t SHOWTIME = 10000;
    static constexpr uint32_t RGB = 10001;
} __attribute__((aligned(4)));

struct led_controller {
    static void init();
    static void run(void *p1, void *p2, void *p3);
    static k_thread thread;
};

extern k_msgq msgq_ros2led;

// vim: set expandtab shiftwidth=4:
