#include <zephyr.h>
#include <device.h>
#include <drivers/can.h>
#include "adc_reader.hpp"
#include "can_controller.hpp"

k_msgq msgq_bmu2ros;
k_msgq msgq_board2ros;
k_msgq msgq_ros2board;

namespace {

char __aligned(4) msgq_bmu2ros_buffer[10 * sizeof (msg_bmu2ros)];
char __aligned(4) msgq_board2ros_buffer[10 * sizeof (msg_board2ros)];
char __aligned(4) msgq_ros2board_buffer[10 * sizeof (msg_ros2board)];

CAN_DEFINE_MSGQ(msgq_can_bmu, 10);
CAN_DEFINE_MSGQ(msgq_can_board, 10);

class can_controller_impl {
public:
    int init() {
        k_msgq_init(&msgq_bmu2ros, msgq_bmu2ros_buffer, sizeof (msg_bmu2ros), 10);
        k_msgq_init(&msgq_board2ros, msgq_board2ros_buffer, sizeof (msg_board2ros), 10);
        k_msgq_init(&msgq_ros2board, msgq_ros2board_buffer, sizeof (msg_ros2board), 10);
        dev = device_get_binding("CAN_1");
        return dev == nullptr ? -1 : 0;
    }
    void run() {
        if (!device_is_ready(dev))
            return;
        setup_can_filter();
        while (true) {
            zcan_frame frame;
            if (k_msgq_get(&msgq_can_bmu, &frame, K_NO_WAIT) == 0) {
                if (handler_bmu(frame)) {
                    while (k_msgq_put(&msgq_bmu2ros, &bmu2ros, K_NO_WAIT) != 0)
                        k_msgq_purge(&msgq_bmu2ros);
                }
            }
            if (k_msgq_get(&msgq_can_board, &frame, K_NO_WAIT) == 0) {
                handler_board(frame);
                while (k_msgq_put(&msgq_board2ros, &board2ros, K_NO_WAIT) != 0)
                    k_msgq_purge(&msgq_board2ros);
            }
            k_msgq_get(&msgq_ros2board, &ros2board, K_NO_WAIT);
            send_message();
            k_msleep(30);
        }
    }
private:
    void setup_can_filter() const {
        static const zcan_filter filter_bmu{
            .id{0x100},
            .rtr{CAN_DATAFRAME},
            .id_type{CAN_STANDARD_IDENTIFIER},
            .id_mask{0x7e0},
            .rtr_mask{1}
        };
        static const zcan_filter filter_board{
            .id{1000},
            .rtr{CAN_DATAFRAME},
            .id_type{CAN_STANDARD_IDENTIFIER},
            .id_mask{CAN_STD_ID_MASK},
            .rtr_mask{1}
        };
        can_attach_msgq(dev, &msgq_can_bmu, &filter_bmu);
        can_attach_msgq(dev, &msgq_can_board, &filter_board);
    }
    bool handler_bmu(zcan_frame &frame) {
        bool result{false};
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
            result = true;
        }
        return result;
    }
    void handler_board(zcan_frame &frame) {
        board2ros.bumper_switch[0] = (frame.data[0] & 0b00001000) != 0;
        board2ros.bumper_switch[1] = (frame.data[0] & 0b00010000) != 0;
        board2ros.emergency_switch[0] = (frame.data[0] & 0b00000010) != 0;
        board2ros.emergency_switch[1] = (frame.data[0] & 0b00000100) != 0;
        board2ros.power_switch = (frame.data[0] & 0b00000001) != 0;
        board2ros.auto_charging = (frame.data[1] & 0b00000010) != 0;
        board2ros.manual_charging = (frame.data[1] & 0b00000001) != 0;
        board2ros.c_fet = (frame.data[2] & 0b00010000) != 0;
        board2ros.d_fet = (frame.data[2] & 0b00100000) != 0;
        board2ros.p_dsg = (frame.data[2] & 0b01000000) != 0;
        board2ros.v5_fail = (frame.data[2] & 0b00000001) != 0;
        board2ros.v16_fail = (frame.data[2] & 0b00000010) != 0;
        board2ros.wheel_disable[0] = (frame.data[3] & 0b00000001) != 0;
        board2ros.wheel_disable[1] = (frame.data[3] & 0b00000010) != 0;
        board2ros.fan_duty = frame.data[4];
        board2ros.charge_connector_temp[0] = frame.data[5];
        board2ros.charge_connector_temp[1] = frame.data[6];
        board2ros.board_temp = frame.data[7];
    }
    void send_message() const {
        zcan_frame frame{
            .id{1001},
            .rtr{CAN_DATAFRAME},
            .id_type{CAN_STANDARD_IDENTIFIER},
            .dlc{3},
            .data{get_trolley_status(), ros2board.emergency_stop, ros2board.power_off}
        };
        can_send(dev, &frame, K_MSEC(100), nullptr, nullptr);
    }
    uint8_t get_trolley_status() const {
        int32_t mv{adc_reader::get(adc_reader::INDEX_TROLLEY)};
        if (mv > 3300 * 3 / 4)
            return 2;
        else if (mv > 3300 / 4)
            return 1;
        else
            return 0;
    }
    msg_bmu2ros bmu2ros{0};
    msg_board2ros board2ros{0};
    msg_ros2board ros2board{0};
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
