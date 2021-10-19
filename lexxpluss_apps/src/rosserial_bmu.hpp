#pragma once

#include <zephyr.h>
#include <cstdio>
#include "ros/node_handle.h"
#include "sensor_msgs/BatteryState.h"
#include "can_controller.hpp"

class ros_bmu {
public:
    void init(ros::NodeHandle &nh) {nh.advertise(pub);}
    void poll() {
        msg_bmu2ros message;
        if (k_msgq_get(&msgq_bmu2ros, &message, K_NO_WAIT) == 0) {
            float cell_voltage[2];
            cell_voltage[0] = message.max_cell_voltage.value;
            cell_voltage[1] = message.min_cell_voltage.value;
            char serial[8];
            snprintf(serial, sizeof serial, "%04x", message.serial);
            if (message.mod_status1 & 0b01000000)
                msg.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_FULL;
            else if (message.charging_current > 0)
                msg.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
            else
                msg.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
            if (message.mod_status1 & 0b00100000 ||
                message.bmu_status == 0x07 ||
                message.bmu_status == 0x09 ||
                message.bmu_alarm1 == 0b10000010)
                msg.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_OVERHEAT;
            else if (message.mod_status1 & 0b00011000)
                msg.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_OVERVOLTAGE;
            else if (message.mod_status2 & 0b11100000)
                msg.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_DEAD;
            else if (message.mod_status1 & 0b10000111 ||
                     message.mod_status2 & 0b00000001 ||
                     message.bmu_status == 0xf0 ||
                     message.bmu_status == 0xf1 ||
                     message.bmu_alarm1 == 0b01111101 ||
                     message.bmu_alarm2 == 0b00000001)
                msg.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
            else
                msg.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
            msg.voltage = message.pack_voltage * 1e-3f;
            msg.current = message.pack_current * 1e-2f;
            msg.charge = message.remain_capacity * 1e-2f;
            msg.capacity = message.full_charge_capacity * 1e-2f;
            msg.design_capacity = message.design_capacity * 1e-2f;
            msg.percentage = message.rsoc * 1e-2f;
            msg.power_supply_technology = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
            msg.present = true;
            msg.cell_voltage_length = 2;
            msg.cell_voltage = cell_voltage;
            msg.location = "0";
            msg.serial_number = serial;
            pub.publish(&msg);
        }
    }
private:
    sensor_msgs::BatteryState msg;
    ros::Publisher pub{"/sensor_set/battery", &msg};
};

// vim: set expandtab shiftwidth=4:
