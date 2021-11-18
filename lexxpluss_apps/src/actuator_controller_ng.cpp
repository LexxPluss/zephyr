#include <device.h>
#include <drivers/pwm.h>
#include "actuator_controller.hpp"

namespace {

static constexpr uint32_t ACTUATOR_NUM{3};

class actuator_driver_pin {
public:
    struct config {
        const char *name;
        uint32_t pin;
    };
    int init(const config &data) {
        dev = device_get_binding(data.name);
        pin = data.pin;
        return dev == nullptr ? -1 : 0;
    }
    bool ready() const {
        return device_is_ready(dev);
    }
    void H() const {
        pwm_pin_set_nsec(dev, pin, CONTROL_PERIOD_NS, CONTROL_PERIOD_NS, PWM_POLARITY_NORMAL);
    }
private:
    uint32_t pin{0};
    const device *dev{nullptr};
    static constexpr uint32_t CONTROL_HZ{5000};
    static constexpr uint32_t CONTROL_PERIOD_NS{1000000000ULL / CONTROL_HZ};
};

class actuator_driver {
public:
    int init(const actuator_driver_pin::config (&data)[2]) {
        for (int i{0}; i < 2; ++i) {
            if (pin[i].init(data[i]) != 0)
                return -1;
        }
        return 0;
    }
    void run() {
        for (const auto &i : pin) {
            if (!i.ready())
                return;
        }
    }
private:
    void stop() const {
        for (const auto &i : pin)
            i.H();
    }
    actuator_driver_pin pin[2];
} driver[ACTUATOR_NUM];

}

void a()
{
    actuator_driver_pin::config data[]{{"a", 1}, {"b", 2}};
    driver[0].init(data);
}

// vim: set expandtab shiftwidth=4:
