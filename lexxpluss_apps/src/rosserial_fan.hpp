#include <zephyr.h>
#include "ros/node_handle.h"
#include "std_msgs/UInt8MultiArray.h"
#include "can_controller.hpp"

class ros_fan {
public:
    void init(ros::NodeHandle &nh) {
        nh.advertise(pub);
        msg.data = msg_data;
        msg.data_length = sizeof msg_data / sizeof msg_data[0];
    }
    void poll() {
        msg_powerboard2ros message;
        if (k_msgq_get(&msgq_powerboard2ros, &message, K_NO_WAIT) == 0) {
            msg.data[0] = message.fan_duty;
            pub.publish(&msg);
        }
    }
private:
    std_msgs::UInt8MultiArray msg;
    uint8_t msg_data[1];
    ros::Publisher pub{"/sensor_set/fan", &msg};
};

// vim: set expandtab shiftwidth=4:
