#pragma once

#include <zephyr.h>

#define M_PI 3.14159265358979323846

struct msg_ros2led {
    uint32_t pattern;
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
} __attribute__((aligned(4)));

struct msg_pgv2ros {
    uint32_t xp, tag;
    int32_t xps;
    int16_t yps;
    uint16_t ang, cc1, cc2, wrn;
    uint8_t addr, lane, o1, s1, o2, s2;
    struct {
        bool cc2, cc1, wrn, np, err, tag, rp, nl, ll, rl;
    } f;
} __attribute__((aligned(4)));

struct msg_ros2pgv {
    uint8_t dir_command;
} __attribute__((aligned(4)));

struct msg_ros2actuator {
    uint16_t data[3];
    uint8_t type;
    static constexpr uint8_t CWCCW = 0;
    static constexpr uint8_t DUTY = 1;
} __attribute__((aligned(4)));

struct msg_actuator2ros {
    int32_t encoder_count[3];
    uint16_t current[3];
} __attribute__((aligned(4)));

struct msg_uss2ros {
    uint32_t front_left, front_right;
    uint32_t left, right, back;
} __attribute__((aligned(4)));

extern k_msgq msgq_ros2led;
extern k_msgq msgq_pgv2ros;
extern k_msgq msgq_ros2pgv;
extern k_msgq msgq_actuator2ros;
extern k_msgq msgq_ros2actuator;
extern k_msgq msgq_uss2ros;
