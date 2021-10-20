#include <device.h>
#include <drivers/i2c.h>
#include "misc_controller.hpp"

namespace {

class misc_controller_impl {
public:
    int init() {
        dev = device_get_binding("I2C_1");
        if (dev != nullptr) {
            uint8_t wbuf[1]{0x0b}, rbuf[2];
            if (i2c_write_read(dev, ADDR, wbuf, sizeof wbuf, rbuf, sizeof rbuf) == 0 &&
                (rbuf[0] & 0b11111000) == 0b11001000) {
                uint8_t initbuf[2]{0x03, 0b10000000};
                i2c_write(dev, initbuf, sizeof initbuf, ADDR);
            }
        }
        return dev == nullptr ? -1 : 0;
    }
    void run() {
        if (!device_is_ready(dev))
            return;
        while (true) {
            uint8_t wbuf[1]{0x00}, rbuf[2];
            if (i2c_write_read(dev, ADDR, wbuf, sizeof wbuf, rbuf, sizeof rbuf) == 0) {
                int16_t value = (rbuf[0] << 8) | rbuf[1];
                main_board_temperature = value / 128.0f;
            }
        }
    }
    float get_main_board_temp() const {return main_board_temperature;}
    float get_actuator_board_temp() const {return actuator_board_temperature;}
private:
    const device *dev{nullptr};
    float main_board_temperature{0.0f}, actuator_board_temperature{0.0f};
    static constexpr uint8_t ADDR{0b1001000};
} impl;

}

void misc_controller::init()
{
    impl.init();
}

void misc_controller::run(void *p1, void *p2, void *p3)
{
    impl.run();
}

float misc_controller::get_main_board_temp()
{
    return impl.get_main_board_temp();
}

float misc_controller::get_actuator_board_temp()
{
    return impl.get_actuator_board_temp();
}

k_thread misc_controller::thread;

// vim: set expandtab shiftwidth=4:
