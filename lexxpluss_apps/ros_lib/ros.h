#ifndef _ROS_H_
#define _ROS_H_

#include "ros/node_handle.h"
#include "zephyr_hardware.h"

namespace ros
{
    typedef NodeHandle_<ZephyrHardware> NodeHandle;
}

#endif
