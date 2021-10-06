#pragma once

#include <zephyr.h>
#include "ros/node_handle.h"
#include "std_msgs/Float64MultiArray.h"
#include "message.hpp"

class ros_tof {
public:
    void init(ros::NodeHandle &nh) {
        nh.advertise(pub);
        msg.data = msg_data;
        msg.data_length = sizeof msg_data / sizeof msg_data[0];
    }
    void poll() {
        msg_tof2ros message;
        if (k_msgq_get(&msgq_tof2ros, &message, K_NO_WAIT) == 0) {
            msg.data[0] = message.left * 1e-3f;
            msg.data[1] = message.right * 1e-3f;
            pub.publish(&msg);
        }
    }
private:
    std_msgs::Float64MultiArray msg;
    float msg_data[2];
    ros::Publisher pub{"downward", &msg};
};

// vim: set expandtab shiftwidth=4:
