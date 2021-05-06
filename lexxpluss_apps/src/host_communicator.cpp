#include <zephyr.h>
#include "ros.h"
#include "lexxauto_msgs/Led.h"
#include "lexxauto_msgs/ultrasound.h"
#include "message.hpp"
#include "thread_runner.hpp"

namespace {

class ros_led_server {
public:
    void callback(const lexxauto_msgs::LedRequest &req, lexxauto_msgs::LedResponse &res) {
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
private:
    led_message message = {led_message::NONE};
    bool updated = false;
} ros_led_server_instance;

class {
public:
    void poll(ros::NodeHandle &nh, ros::Publisher &pub) {
        sonar_message message;
        if (k_msgq_get(&sonar_controller_msgq, &message, K_NO_WAIT) == 0) {
            ros_msg.sensor0 = message.distance[0];
            ros_msg.sensor1 = message.distance[1];
            ros_msg.sensor2 = message.distance[2];
            ros_msg.sensor3 = message.distance[3];
            nh.publish(pub.id_, &ros_msg); // workaround, can not call publisher virtual funcion.
            printk("distance: %d %d %d %d\n",
                   message.distance[0],
                   message.distance[1],
                   message.distance[2],
                   message.distance[3]);
        }
    }
    lexxauto_msgs::ultrasound ros_msg;
} ros_sonar_server_instance;

ros::NodeHandle nh;
ros::ServiceServer<lexxauto_msgs::LedRequest, lexxauto_msgs::LedResponse, ros_led_server>
    led("/body_control/led", &ros_led_server::callback, &ros_led_server_instance);
ros::Publisher sonar_pub("ultrasound_measured_data", &ros_sonar_server_instance.ros_msg);

class debug_scenario {
public:
    void init() {
        prev = k_uptime_get();
    }
    void poll() {
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
            ros_led_server_instance.callback(req, res);
            if (++index >= sizeof patterns / sizeof patterns[0])
                index = 0;
        }
    }
private:
    int64_t prev = 0;
    uint32_t index = 0;
} debug_scenario_instance;

class host_communicator {
public:
    int setup() const {
        nh.initNode();
        nh.advertiseService(led);
        nh.advertise(sonar_pub);
        debug_scenario_instance.init();
        return 0;
    }
    void loop() const {
        nh.spinOnce();
        ros_led_server_instance.poll();
        ros_sonar_server_instance.poll(nh, sonar_pub);
        debug_scenario_instance.poll();
        k_msleep(1);
    }
};

LEXX_THREAD_RUNNER(host_communicator);

}

/*
vi:et:sw=4:
*/
