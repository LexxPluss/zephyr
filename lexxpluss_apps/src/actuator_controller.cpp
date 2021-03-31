#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include "thread_runner.hpp"
#include "actuator_controller.hpp"

#define LED   DT_ALIAS(led0)
#define LABEL DT_GPIO_LABEL(LED, gpios)
#define PIN   DT_GPIO_PIN(LED, gpios)
#define FLAG  DT_GPIO_FLAGS(LED, gpios)

int actuator_controller::init()
{
    dev = device_get_binding(LABEL);
    if (dev)
        gpio_pin_configure(dev, PIN, GPIO_OUTPUT_ACTIVE | FLAG);
    return dev == nullptr ? -1 : 0;
}

void actuator_controller::run()
{
    while (true) {
        gpio_pin_set(dev, PIN, 1);
        k_msleep(500);
        gpio_pin_set(dev, PIN, 0);
        k_msleep(500);
    }
}

LEXX_THREAD(actuator_controller);
