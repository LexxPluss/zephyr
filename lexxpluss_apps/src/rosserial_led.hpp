#pragma once

#include <zephyr.h>
#include "ros/node_handle.h"
#include "std_msgs/String.h"
#include "led_controller.hpp"

class ros_led {
public:
    void init(ros::NodeHandle &nh) {
        nh.subscribe(sub_string);
        nh.subscribe(sub_direct);
    }
    void poll() {}
private:
    void callback_string(const std_msgs::String& req) {
        msg_ros2led message;
        if      (strcmp(req.data, "emergency_stop")  == 0) message.pattern = msg_ros2led::EMERGENCY_STOP;
        else if (strcmp(req.data, "amr_mode")        == 0) message.pattern = msg_ros2led::AMR_MODE;
        else if (strcmp(req.data, "agv_mode")        == 0) message.pattern = msg_ros2led::AGV_MODE;
        else if (strcmp(req.data, "mission_pause")   == 0) message.pattern = msg_ros2led::MISSION_PAUSE;
        else if (strcmp(req.data, "path_blocked")    == 0) message.pattern = msg_ros2led::PATH_BLOCKED;
        else if (strcmp(req.data, "manual_drive")    == 0) message.pattern = msg_ros2led::MANUAL_DRIVE;
        else if (strcmp(req.data, "charging")        == 0) message.pattern = msg_ros2led::CHARGING;
        else if (strcmp(req.data, "waiting_for_job") == 0) message.pattern = msg_ros2led::WAITING_FOR_JOB;
        else if (strcmp(req.data, "left_winker")     == 0) message.pattern = msg_ros2led::LEFT_WINKER;
        else if (strcmp(req.data, "right_winker")    == 0) message.pattern = msg_ros2led::RIGHT_WINKER;
        else if (strcmp(req.data, "both_winker")     == 0) message.pattern = msg_ros2led::BOTH_WINKER;
        else if (strcmp(req.data, "move_actuator")   == 0) message.pattern = msg_ros2led::MOVE_ACTUATOR;
        else                                               message.pattern = msg_ros2led::NONE;
        while (k_msgq_put(&msgq_ros2led, &message, K_NO_WAIT) != 0)
            k_msgq_purge(&msgq_ros2led);
    }
    void callback_direct(const std_msgs::String& req) {
        msg_ros2led message;
        message.pattern = msg_ros2led::RGB;
        message.rgb[0] = 0x00;
        message.rgb[1] = 0x00;
        message.rgb[2] = 0x00;
        while (k_msgq_put(&msgq_ros2led, &message, K_NO_WAIT) != 0)
            k_msgq_purge(&msgq_ros2led);
    }
    ros::Subscriber<std_msgs::String, ros_led> sub_string{"/body_control/led", &ros_led::callback_string, this};
    ros::Subscriber<std_msgs::String/*@@*/, ros_led> sub_direct{"/body_control/led_direct", &ros_led::callback_direct, this};
};

// vim: set expandtab shiftwidth=4:
