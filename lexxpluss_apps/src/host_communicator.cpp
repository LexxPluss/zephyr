#include <zephyr.h>
#include <logging/log.h>
#include "ros.h"
#include "geometry_msgs/Vector3.h"
#include "lexxauto_msgs/Led.h"
#include "lexxauto_msgs/ultrasound.h"
#include "message.hpp"
#include "thread_runner.hpp"

namespace {

LOG_MODULE_REGISTER(host_controller, LOG_LEVEL_INF);

class ros_led_server;
class ros_led_server {
public:
    void callback(const lexxauto_msgs::LedRequest &req, lexxauto_msgs::LedResponse &res) {
        LOG_INF("LED %s", log_strdup(req.pattern));
        if      (strcmp(req.pattern, "emergency_stop")  == 0) message.pattern = led_message::EMERGENCY_STOP;
        else if (strcmp(req.pattern, "amr_mode")        == 0) message.pattern = led_message::AMR_MODE;
        else if (strcmp(req.pattern, "agv_mode")        == 0) message.pattern = led_message::AGV_MODE;
        else if (strcmp(req.pattern, "mission_pause")   == 0) message.pattern = led_message::MISSION_PAUSE;
        else if (strcmp(req.pattern, "path_blocked")    == 0) message.pattern = led_message::PATH_BLOCKED;
        else if (strcmp(req.pattern, "manual_drive")    == 0) message.pattern = led_message::MANUAL_DRIVE;
        else if (strcmp(req.pattern, "left_indicator")  == 0) message.pattern = led_message::LEFT_INDICATOR;
        else if (strcmp(req.pattern, "right_indicator") == 0) message.pattern = led_message::RIGHT_INDICATOR;
        else if (strcmp(req.pattern, "dock_mode")       == 0) message.pattern = led_message::DOCK_MODE;
        else if (strcmp(req.pattern, "charging")        == 0) message.pattern = led_message::CHARGING;
        else if (strcmp(req.pattern, "waiting_for_job") == 0) message.pattern = led_message::WAITING_FOR_JOB;
        else if (strcmp(req.pattern, "left_winker")     == 0) message.pattern = led_message::LEFT_WINKER;
        else if (strcmp(req.pattern, "right_winker")    == 0) message.pattern = led_message::RIGHT_WINKER;
        else if (strcmp(req.pattern, "both_winker")     == 0) message.pattern = led_message::BOTH_WINKER;
        else if (strcmp(req.pattern, "move_actuator")   == 0) message.pattern = led_message::MOVE_ACTUATOR;
        else                                                  message.pattern = led_message::NONE;
        updated = true;
        res.success = true;
    }
    void poll() {
        if (updated) {
            while (k_msgq_put(&led_controller_msgq, &message, K_NO_WAIT) != 0)
                k_msgq_purge(&led_controller_msgq);
            updated = false;
        }
    }
    ros::ServiceServer<lexxauto_msgs::LedRequest, lexxauto_msgs::LedResponse, ros_led_server>
        server{"/body_control/led", &ros_led_server::callback, this};
private:
    led_message message = {led_message::NONE};
    bool updated = false;
};

class ros_sonar_server {
public:
    void poll(ros::NodeHandle &nh) {
        sonar_message message;
        if (k_msgq_get(&sonar_controller_msgq, &message, K_NO_WAIT) == 0) {
            ros_msg.sensor0 = message.distance[0];
            ros_msg.sensor1 = message.distance[1];
            ros_msg.sensor2 = message.distance[2];
            ros_msg.sensor3 = message.distance[3];
            pub.publish(&ros_msg);
        }
    }
    ros::Publisher pub{"ultrasound_measured_data", &ros_msg};
private:
    lexxauto_msgs::ultrasound ros_msg;
};

class ros_imu_server {
private:
    class imupub {
    public:
        imupub(const char *name) : pub(name, &ros_msg) {}
        void poll(ros::NodeHandle &nh, const float *data) {
            ros_msg.x = data[0];
            ros_msg.y = data[1];
            ros_msg.z = data[2];
            pub.publish(&ros_msg);
        }
        ros::Publisher pub;
    private:
        geometry_msgs::Vector3 ros_msg;
    };
public:
    void poll(ros::NodeHandle &nh) {
        imu_message message;
        if (k_msgq_get(&imu_controller_msgq, &message, K_NO_WAIT) == 0) {
            gyro.poll(nh, message.gyro);
            accel.poll(nh, message.accel);
            angle.poll(nh, message.delta_ang);
            velocity.poll(nh, message.delta_vel);
        }
    }
    imupub gyro{"adis16470_gyro_data"};
    imupub accel{"adis16470_accel_data"};
    imupub angle{"adis16470_ang_data"};
    imupub velocity{"adis16470_vel_data"};
};

#if 0
class debug_scenario {
public:
    void init() {
        prev = k_uptime_get();
    }
    void poll(ros_led_server &led) {
        static const char *patterns[] = {
            "emergency_stop",
            "amr_mode",
            "agv_mode",
            "mission_pause",
            "path_blocked",
            "manual_drive",
            "left_indicator",
            "right_indicator",
            "dock_mode",
            "charging",
            "waiting_for_job",
        };
        auto now = k_uptime_get();
        if (now - prev > 30000) {
            prev = now;
            lexxauto_msgs::LedRequest req;
            lexxauto_msgs::LedResponse res;
            req.pattern = patterns[index];
            led.callback(req, res);
            if (++index >= sizeof patterns / sizeof patterns[0])
                index = 0;
        }
    }
private:
    int64_t prev = 0;
    uint32_t index = 0;
};
#endif

class host_communicator {
public:
    int setup() {
        nh.initNode();
        nh.advertise(imu.gyro.pub);
        nh.advertise(imu.accel.pub);
        nh.advertise(imu.angle.pub);
        nh.advertise(imu.velocity.pub);
        // nh.advertiseService(led.server);
        // nh.advertise(sonar.pub);
        // debug.init();
        return 0;
    }
    void loop() {
        nh.spinOnce();
        imu.poll(nh);
        // led.poll();
        // sonar.poll(nh);
        // debug.poll(led);
        k_msleep(1);
    }
private:
    ros::NodeHandle nh;
    ros_imu_server imu;
    // ros_led_server led;
    // ros_sonar_server sonar;
    // debug_scenario debug;
};

LEXX_THREAD_RUNNER(host_communicator);

}

/* vim: set expandtab shiftwidth=4: */
