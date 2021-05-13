#pragma once

#include <zephyr.h>

struct led_message {
    uint32_t pattern;
    static constexpr uint32_t NONE = 0;
    static constexpr uint32_t EMERGENCY_STOP = 1;
    static constexpr uint32_t AMR_MODE = 2;
    static constexpr uint32_t AGV_MODE = 3;
    static constexpr uint32_t MISSION_PAUSE = 4;
    static constexpr uint32_t PATH_BLOCKED = 5;
    static constexpr uint32_t MANUAL_DRIVE = 6;
    static constexpr uint32_t LEFT_INDICATOR = 7;
    static constexpr uint32_t RIGHT_INDICATOR = 8;
    static constexpr uint32_t DOCK_MODE = 9;
    static constexpr uint32_t CHARGING = 10;
    static constexpr uint32_t WAITING_FOR_JOB = 11;
    static constexpr uint32_t PATTERNS = 12;
} __attribute__((aligned(4)));

struct sonar_message {
    uint32_t distance[4];
    static constexpr int SENSORS = 4;
} __attribute__((aligned(4)));


extern k_msgq led_controller_msgq;
extern k_msgq sonar_controller_msgq;

/* vim: set expandtab shiftwidth=4: */
