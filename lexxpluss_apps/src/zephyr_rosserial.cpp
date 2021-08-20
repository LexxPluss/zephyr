#include <math.h>
#include <zephyr.h>
#include "ros/node_handle.h"
#include "rosserial_hardware_zephyr.hpp"
#include "thread_runner.hpp"

namespace {

class zephyr_rosserial {
public:
    int setup() {
        nh.initNode(const_cast<char*>("UART_1"));
        return 0;
    }
    void loop() {
        nh.spinOnce();
        k_msleep(1);
    }
private:
    ros::NodeHandle_<rosserial_hardware_zephyr> nh;
};

LEXX_THREAD_RUNNER(zephyr_rosserial);

}
