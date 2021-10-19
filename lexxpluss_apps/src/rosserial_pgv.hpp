#pragma once

#include <zephyr.h>
#include <cstdio>
#include "ros/node_handle.h"
#include "pf_pgv100/pgv_dir_msg.h"
#include "pf_pgv100/pgv_scan_data.h"
#include "pgv_controller.hpp"

#define M_PI 3.14159265358979323846

class ros_pgv {
public:
    void init(ros::NodeHandle &nh) {
        nh.advertise(pub);
        nh.subscribe(sub);
    }
    void poll() {
        msg_pgv2ros message;
        if (k_msgq_get(&msgq_pgv2ros, &message, K_NO_WAIT) == 0)
            publish(message);
    }
private:
    void publish(const msg_pgv2ros &message) {
        double ang{static_cast<double>(message.ang) / 10.0};
        if (ang < 180.0)
            ang *= -1.0;
        else
            ang = 360.0 - ang;
        double xpos{static_cast<double>(message.xps)};
        if (message.f.tag) {
            if (xpos > 2000.0)
                xpos -= 0x01000001;
        }
        double ypos{static_cast<double>(message.yps)};
        if (ypos > 2000.0)
            ypos -= 16383.0;
        if (message.f.np)
            ypos *= -1.0;
        msg.angle = ang * M_PI / 180.0;
        msg.x_pos = xpos / 10000.0;
        msg.y_pos = ypos / 10000.0;
        msg.direction = direction;
        msg.color_lane_count = message.lane;
        msg.no_color_lane = message.f.nl;
        msg.no_pos = message.f.np;
        msg.tag_detected = message.f.tag;
        msg.control_code1_detected = message.f.cc1;
        msg.control_code2_detected = message.f.cc2;
        pub.publish(&msg);
    }
    void callback(const pf_pgv100::pgv_dir_msg &req) {
        switch (req.dir_command) {
        case 0:
            snprintf(direction, sizeof direction, "No lane is selected");
            break;
        case 1:
            snprintf(direction, sizeof direction, "Right lane is selected");
            break;
        case 2:
            snprintf(direction, sizeof direction, "Left lane is selected");
            break;
        case 3:
            snprintf(direction, sizeof direction, "Straight Ahead");
            break;
        }
        msg_ros2pgv ros2pgv;
        ros2pgv.dir_command = req.dir_command;
        while (k_msgq_put(&msgq_ros2pgv, &ros2pgv, K_NO_WAIT) != 0)
            k_msgq_purge(&msgq_ros2pgv);
    }
    pf_pgv100::pgv_scan_data msg;
    ros::Publisher pub{"pgv100_scan", &msg};
    ros::Subscriber<pf_pgv100::pgv_dir_msg, ros_pgv> sub{"/pgv_dir", &ros_pgv::callback, this};
    char direction[64]{"Straight Ahead"};
};

// vim: set expandtab shiftwidth=4:
