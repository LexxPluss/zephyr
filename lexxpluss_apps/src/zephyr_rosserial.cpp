#include <math.h>
#include "std_msgs/String.h"
#include "message.hpp"
#include "rosserial_hardware_zephyr.hpp"
#include "thread_runner.hpp"

namespace {

class ros_led;
class ros_led {
public:
    void init(ros::NodeHandle &nh) {
        nh.subscribe(sub);
    }
    void poll() {
        if (updated) {
            updated = false;
        }
    }
private:
    void callback(const std_msgs::String& req) {
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
        updated = true;
    }
    ros::Subscriber<std_msgs::String, ros_led> sub{"/body_control/led", &ros_led::callback, this};
    msg_ros2led message{msg_ros2led::NONE};
    bool updated{false};
};

class zephyr_rosserial {
public:
    int setup() {
        nh.initNode(const_cast<char*>("UART_1"));
        led.init(nh);
        return 0;
    }
    void loop() {
        nh.spinOnce();
        led.poll();
        k_msleep(1);
    }
private:
    ros::NodeHandle nh;
    ros_led led;
};

LEXX_THREAD_RUNNER(zephyr_rosserial);

}
