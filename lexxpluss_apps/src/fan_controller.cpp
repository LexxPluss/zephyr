#include <zephyr.h>
#include <device.h>
#include <drivers/pwm.h>
#include "message.hpp"
#include "thread_runner.hpp"

k_msgq fan_controller_msgq;

namespace {

char __aligned(4) fan_controller_msgq_buffer[10 * sizeof (fan_message)];

class fan_controller {
public:
    int setup() {
        k_msgq_init(&fan_controller_msgq, fan_controller_msgq_buffer, sizeof (fan_message), 10);
#if 0
        pwm_pin_set_usec(dev[0], 1, 100, 10, 0);
        pwm_pin_set_usec(dev[1], 1, 100, 10, 0);
        dev[0] = device_get_binding("PWM_10");
        dev[1] = device_get_binding("PWM_11");
        return dev == nullptr ? -1 : 0;
#else
        return 0;
#endif
    }
    void loop() const {
        fan_message message;
        if (k_msgq_get(&led_controller_msgq, &message, K_FOREVER) == 0) {
#if 0
            pwm_pin_set_usec(dev[0], 1, PWM_USEC, message.duty_percent[0] * PWM_USEC / 100, 0);
            pwm_pin_set_usec(dev[1], 1, PWM_USEC, message.duty_percent[1] * PWM_USEC / 100, 0);
#endif
        }
    }
private:
    const device *dev[2] = {nullptr, nullptr};
    static constexpr uint32_t PWM_USEC = 100;
};

LEXX_THREAD_RUNNER(fan_controller);

}

/* vim: set expandtab shiftwidth=4: */
