#include "rosserial_hardware_zephyr.hpp"
#include "rosserial_actuator.hpp"
#include "rosserial_imu.hpp"
#include "rosserial_led.hpp"
#include "rosserial_pgv.hpp"
#include "rosserial_tof.hpp"
#include "rosserial_uss.hpp"
#include "rosserial.hpp"

namespace {

class rosserial_impl {
public:
    int init() {
        nh.initNode(const_cast<char*>("UART_1"));
        led.init(nh);
        pgv.init(nh);
        actuator.init(nh);
        uss.init(nh);
        imu.init(nh);
        tof.init(nh);
        return 0;
    }
    void run() {
        while (true) {
            nh.spinOnce();
            led.poll();
            pgv.poll();
            actuator.poll();
            uss.poll();
            imu.poll();
            tof.poll();
            k_msleep(1);
        }
    }
private:
    ros::NodeHandle nh;
    ros_led led;
    ros_pgv pgv;
    ros_actuator actuator;
    ros_uss uss;
    ros_imu imu;
    ros_tof tof;
} impl;

}

void rosserial::init()
{
    impl.init();
}

void rosserial::run(void *p1, void *p2, void *p3)
{
    impl.run();
}

k_thread rosserial::thread;

// vim: set expandtab shiftwidth=4:
