#include <zephyr.h>
#include <device.h>
#include <drivers/can.h>
#include "adc_reader.hpp"
#include "can_controller.hpp"

k_msgq msgq_bmu2ros;
k_msgq msgq_powerboard2ros;

namespace {

char __aligned(4) msgq_bmu2ros_buffer[10 * sizeof (msg_bmu2ros)];
char __aligned(4) msgq_powerboard2ros_buffer[10 * sizeof (msg_powerboard2ros)];

CAN_DEFINE_MSGQ(msgq_bmu, 10);
CAN_DEFINE_MSGQ(msgq_power, 10);

class can_controller_impl {
public:
    int init() {
        k_msgq_init(&msgq_bmu2ros, msgq_bmu2ros_buffer, sizeof (msg_bmu2ros), 10);
        k_msgq_init(&msgq_powerboard2ros, msgq_powerboard2ros_buffer, sizeof (msg_powerboard2ros), 10);
        dev = device_get_binding("CAN_1");
        prev_cycle_bmu = prev_cycle_power = k_cycle_get_32();
        return dev == nullptr ? -1 : 0;
    }
    void run() {
        setup_can_filter();
        while (true) {
            zcan_frame frame;
            if (k_msgq_get(&msgq_bmu, &frame, K_NO_WAIT) == 0)
                handler_bmu(frame);
            if (k_msgq_get(&msgq_power, &frame, K_NO_WAIT) == 0)
                handler_power(frame);
            uint32_t now_cycle = k_cycle_get_32();
            uint32_t dt_ms = k_cyc_to_ms_near32(now_cycle - prev_cycle_bmu);
            if (dt_ms > 500) {
                prev_cycle_bmu = now_cycle;
                while (k_msgq_put(&msgq_bmu2ros, &bmu2ros, K_NO_WAIT) != 0)
                    k_msgq_purge(&msgq_bmu2ros);
            }
            dt_ms = k_cyc_to_ms_near32(now_cycle - prev_cycle_power);
            if (dt_ms > 100) {
                prev_cycle_power = now_cycle;
                while (k_msgq_put(&msgq_powerboard2ros, &powerboard2ros, K_NO_WAIT) != 0)
                    k_msgq_purge(&msgq_powerboard2ros);
            }
            send_trolley_status();
            k_msleep(30);
        }
    }
private:
    void setup_can_filter() const {
        static const zcan_filter filter_bmu{
            .id = 0x100,
            .rtr = CAN_DATAFRAME,
            .id_type = CAN_STANDARD_IDENTIFIER,
            .id_mask = 0x7e0,
            .rtr_mask = 1
        };
        static const zcan_filter filter_power{
            .id = 1000,
            .rtr = CAN_DATAFRAME,
            .id_type = CAN_STANDARD_IDENTIFIER,
            .id_mask = CAN_STD_ID_MASK,
            .rtr_mask = 1
        };
        can_attach_msgq(dev, &msgq_bmu, &filter_bmu);
        can_attach_msgq(dev, &msgq_power, &filter_power);
    }
    void handler_bmu(zcan_frame &frame) {
        if (frame.id == 0x100) {
            bmu2ros.mod_status1 = frame.data[0];
            bmu2ros.bmu_status = frame.data[1];
            bmu2ros.asoc = frame.data[2];
            bmu2ros.rsoc = frame.data[3];
            bmu2ros.soh = frame.data[4];
            bmu2ros.fet_temp = (frame.data[5] << 8) | frame.data[6];
        } else if (frame.id == 0x101) {
            bmu2ros.pack_current = (frame.data[0] << 8) | frame.data[1];
            bmu2ros.charging_current = (frame.data[2] << 8) | frame.data[3];
            bmu2ros.pack_voltage = (frame.data[4] << 8) | frame.data[5];
            bmu2ros.mod_status2 = frame.data[6];
        } else if (frame.id == 0x103) {
            bmu2ros.design_capacity = (frame.data[0] << 8) | frame.data[1];
            bmu2ros.full_charge_capacity = (frame.data[2] << 8) | frame.data[3];
            bmu2ros.remain_capacity = (frame.data[4] << 8) | frame.data[5];
        } else if (frame.id == 0x110) {
            bmu2ros.max_voltage.value = (frame.data[0] << 8) | frame.data[1];
            bmu2ros.max_voltage.id = frame.data[2];
            bmu2ros.min_voltage.value = (frame.data[4] << 8) | frame.data[5];
            bmu2ros.min_voltage.id = frame.data[6];
        } else if (frame.id == 0x111) {
            bmu2ros.max_temp.value = (frame.data[0] << 8) | frame.data[1];
            bmu2ros.max_temp.id = frame.data[2];
            bmu2ros.min_temp.value = (frame.data[4] << 8) | frame.data[5];
            bmu2ros.min_temp.id = frame.data[6];
        } else if (frame.id == 0x112) {
            bmu2ros.max_current.value = (frame.data[0] << 8) | frame.data[1];
            bmu2ros.max_current.id = frame.data[2];
            bmu2ros.min_current.value = (frame.data[4] << 8) | frame.data[5];
            bmu2ros.min_current.id = frame.data[6];
        } else if (frame.id == 0x113) {
            bmu2ros.bmu_fw_ver = frame.data[0];
            bmu2ros.mod_fw_ver = frame.data[1];
            bmu2ros.serial_config = frame.data[2];
            bmu2ros.parallel_config = frame.data[3];
            bmu2ros.bmu_alarm1 = frame.data[4];
            bmu2ros.bmu_alarm2 = frame.data[5];
        } else if (frame.id == 0x120) {
            bmu2ros.min_cell_voltage.value = (frame.data[0] << 8) | frame.data[1];
            bmu2ros.min_cell_voltage.id = frame.data[2];
            bmu2ros.max_cell_voltage.value = (frame.data[4] << 8) | frame.data[5];
            bmu2ros.max_cell_voltage.id = frame.data[6];
        } else if (frame.id == 0x130) {
            bmu2ros.manufacturing = (frame.data[0] << 8) | frame.data[1];
            bmu2ros.inspection = (frame.data[2] << 8) | frame.data[3];
            bmu2ros.serial = (frame.data[4] << 8) | frame.data[5];
        }
    }
    void handler_power(zcan_frame &frame) {
        powerboard2ros.bumper_switch[0] = (frame.data[0] & 0b00001000) != 0;
        powerboard2ros.bumper_switch[1] = (frame.data[0] & 0b00010000) != 0;
        powerboard2ros.emergency_switch[0] = (frame.data[0] & 0b00000010) != 0;
        powerboard2ros.emergency_switch[1] = (frame.data[0] & 0b00000100) != 0;
        powerboard2ros.power_switch = (frame.data[0] & 0b00000001) != 0;
        powerboard2ros.auto_charging = (frame.data[1] & 0b00000010) != 0;
        powerboard2ros.manual_charging = (frame.data[1] & 0b00000001) != 0;
        powerboard2ros.c_fet = (frame.data[2] & 0b00010000) != 0;
        powerboard2ros.d_fet = (frame.data[2] & 0b00100000) != 0;
        powerboard2ros.p_dsg = (frame.data[2] & 0b01000000) != 0;
        powerboard2ros.v5_fail = (frame.data[2] & 0b00000001) != 0;
        powerboard2ros.v16_fail = (frame.data[2] & 0b00000010) != 0;
        powerboard2ros.wheel_disable[0] = (frame.data[3] & 0b00000001) != 0;
        powerboard2ros.wheel_disable[1] = (frame.data[3] & 0b00000010) != 0;
        powerboard2ros.fan_duty = frame.data[4];
        powerboard2ros.charge_connector_temp[0] = frame.data[5];
        powerboard2ros.charge_connector_temp[1] = frame.data[6];
        powerboard2ros.board_temp = frame.data[7];
    }
    void send_trolley_status() const {
        zcan_frame frame{
            .id = 1001,
            .rtr = CAN_DATAFRAME,
            .id_type = CAN_STANDARD_IDENTIFIER,
            .dlc = 1,
            .data{get_trolley_status()}
        };
        can_send(dev, &frame, K_MSEC(100), nullptr, nullptr);
    }
    uint8_t get_trolley_status() const {
        int32_t mv = adc_reader::get(adc_reader::INDEX_TROLLEY);
        if (mv > 3300 * 3 / 4)
            return 2;
        else if (mv > 3300 / 4)
            return 1;
        else
            return 0;
    }
    msg_bmu2ros bmu2ros{0};
    msg_powerboard2ros powerboard2ros{0};
    uint32_t prev_cycle_bmu{0}, prev_cycle_power{0};
    const device *dev{nullptr};
} impl;

}

void can_controller::init()
{
    impl.init();
}

void can_controller::run(void *p1, void *p2, void *p3)
{
    impl.run();
}

k_thread can_controller::thread;

/* vim: set expandtab shiftwidth=4: */
