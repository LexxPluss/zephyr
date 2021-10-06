#pragma once

#include <zephyr.h>
#include "ros/node_handle.h"
#include "std_msgs/ByteMultiArray.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/UInt16MultiArray.h"
#include "message.hpp"

class ros_actuator {
public:
    void init(ros::NodeHandle &nh) {
        nh.subscribe(sub_cwccw);
        nh.subscribe(sub_duty);
        msg_encoder.data = msg_encoder_data;
        msg_encoder.data_length = sizeof msg_encoder_data / sizeof msg_encoder_data[0];
        msg_current.data = msg_current_data;
        msg_current.data_length = sizeof msg_current_data / sizeof msg_current_data[0];
    }
    void poll() {
        msg_actuator2ros message;
        if (k_msgq_get(&msgq_actuator2ros, &message, K_NO_WAIT) == 0) {
            for (int i = 0; i < 3; ++i) {
                msg_encoder.data[i] = message.encoder_count[i];
                msg_current.data[i] = message.current[i];
            }
            pub_encoder.publish(&msg_encoder);
        }
    }
private:
    void callback_cwccw(const std_msgs::ByteMultiArray& req) {
        msg_ros2actuator ros2actuator;
        ros2actuator.data[0] = req.data[0];
        ros2actuator.data[1] = req.data[1];
        ros2actuator.data[2] = req.data[2];
        ros2actuator.type = msg_ros2actuator::CWCCW;
        while (k_msgq_put(&msgq_ros2actuator, &ros2actuator, K_NO_WAIT) != 0)
            k_msgq_purge(&msgq_ros2actuator);
    }
    void callback_duty(const std_msgs::UInt16MultiArray& req) {
        msg_ros2actuator ros2actuator;
        ros2actuator.data[0] = req.data[0];
        ros2actuator.data[1] = req.data[1];
        ros2actuator.data[2] = req.data[2];
        ros2actuator.type = msg_ros2actuator::DUTY;
        while (k_msgq_put(&msgq_ros2actuator, &ros2actuator, K_NO_WAIT) != 0)
            k_msgq_purge(&msgq_ros2actuator);
    }
    std_msgs::Int32MultiArray msg_encoder;
    std_msgs::UInt16MultiArray msg_current;
    int32_t msg_encoder_data[3];
    uint16_t msg_current_data[3];
    ros::Publisher pub_encoder{"encoder_count", &msg_encoder};
    ros::Publisher pub_current{"current_value", &msg_current};
    ros::Subscriber<std_msgs::ByteMultiArray, ros_actuator> sub_cwccw{"request_direction_of_rotation", &ros_actuator::callback_cwccw, this};
    ros::Subscriber<std_msgs::UInt16MultiArray, ros_actuator> sub_duty{"request_actuator_power", &ros_actuator::callback_duty, this};
};

// vim: set expandtab shiftwidth=4:
