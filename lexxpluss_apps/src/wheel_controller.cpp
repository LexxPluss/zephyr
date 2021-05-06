#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include "thread_runner.hpp"

namespace {

#define LED   DT_ALIAS(led3)
#define LABEL DT_GPIO_LABEL(LED, gpios)
#define PIN   DT_GPIO_PIN(LED, gpios)
#define FLAG  DT_GPIO_FLAGS(LED, gpios)

class wheel_controller {
public:
    int setup() {
        dev = device_get_binding(LABEL);
        if (dev)
            gpio_pin_configure(dev, PIN, GPIO_OUTPUT_ACTIVE | FLAG);
        return dev == nullptr ? -1 : 0;
    }
    void loop() const {
        gpio_pin_set(dev, PIN, 1);
        k_msleep(200);
        gpio_pin_set(dev, PIN, 0);
        k_msleep(200);
    }
private:
    const device *dev = nullptr;
};

LEXX_THREAD_RUNNER(wheel_controller);

}
