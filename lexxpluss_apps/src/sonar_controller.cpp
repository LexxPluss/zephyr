#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include "thread_runner.hpp"
#include "sonar_controller.hpp"

#define LED   DT_ALIAS(led2)
#define LABEL DT_GPIO_LABEL(LED, gpios)
#define PIN   DT_GPIO_PIN(LED, gpios)
#define FLAG  DT_GPIO_FLAGS(LED, gpios)

int sonar_controller::init()
{
    dev = device_get_binding(LABEL);
    if (dev)
        gpio_pin_configure(dev, PIN, GPIO_OUTPUT_ACTIVE | FLAG);
    return dev == nullptr ? -1 : 0;
}

void sonar_controller::run()
{
    while (true) {
        gpio_pin_set(dev, PIN, 1);
        k_msleep(300);
        gpio_pin_set(dev, PIN, 0);
        k_msleep(300);
    }
}

LEXX_THREAD(sonar_controller);
