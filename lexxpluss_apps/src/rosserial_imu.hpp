#pragma once

#include <zephyr.h>
#include "ros/node_handle.h"
#include "geometry_msgs/Vector3.h"
#include "message.hpp"

class ros_imu {
public:
    void init(ros::NodeHandle &nh) {
        gyro.init(nh);
        accel.init(nh);
        angle.init(nh);
        velocity.init(nh);
    }
    void poll() {
        msg_imu2ros message;
        if (k_msgq_get(&msgq_imu2ros, &message, K_NO_WAIT) == 0) {
            gyro.poll(message.gyro);
            accel.poll(message.accel);
            angle.poll(message.delta_ang);
            velocity.poll(message.delta_vel);
        }
    }
private:
    class imupub {
    public:
        imupub(const char *name) : pub(name, &msg) {}
        void init(ros::NodeHandle &nh) {nh.advertise(pub);}
        void poll(const float *data) {
            msg.x = data[0];
            msg.y = data[1];
            msg.z = data[2];
            pub.publish(&msg);
        }
        ros::Publisher pub;
    private:
        geometry_msgs::Vector3 msg;
    };
    imupub gyro{"adis16470_gyro_data"};
    imupub accel{"adis16470_accel_data"};
    imupub angle{"adis16470_ang_data"};
    imupub velocity{"adis16470_vel_data"};
};

// vim: set expandtab shiftwidth=4:
