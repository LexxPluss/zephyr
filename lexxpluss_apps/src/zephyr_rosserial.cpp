#include <cmath>
#include <cstdio>
#include "std_msgs/ByteMultiArray.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt16MultiArray.h"
#include "pf_pgv100/pgv_dir_msg.h"
#include "pf_pgv100/pgv_scan_data.h"
#include "message.hpp"
#include "rosserial_hardware_zephyr.hpp"
#include "thread_runner.hpp"

namespace {

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
        double ang = static_cast<double>(message.ang) / 10.0;
        if (ang < 180.0)
            ang *= -1.0;
        else
            ang = 360.0 - ang;
        double xpos = message.xps;
        if (message.f.tag) {
            if (xpos > 2000.0)
                xpos -= 0x01000001;
        }
        double ypos = message.yps;
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
    void callback(const pf_pgv100::pgv_dir_msg& req) {
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
    }
    pf_pgv100::pgv_scan_data msg;
    ros::Publisher pub{"pgv100_scan", &msg};
    ros::Subscriber<pf_pgv100::pgv_dir_msg, ros_pgv> sub{"/pgv_dir", &ros_pgv::callback, this};
    char direction[64]{"Straight Ahead"};
};

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

class zephyr_rosserial {
public:
    int setup() {
        nh.initNode(const_cast<char*>("UART_1"));
        led.init(nh);
        pgv.init(nh);
        actuator.init(nh);
        return 0;
    }
    void loop() {
        nh.spinOnce();
        led.poll();
        pgv.poll();
        actuator.poll();
        k_msleep(1);
    }
private:
    ros::NodeHandle nh;
    ros_led led;
    ros_pgv pgv;
    ros_actuator actuator;
};

LEXX_THREAD_RUNNER(zephyr_rosserial);

}
