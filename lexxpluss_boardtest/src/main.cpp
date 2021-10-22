#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>

void main()
{
    const device *gpiog = device_get_binding("GPIOG");
    const device *gpiok = device_get_binding("GPIOK");
    gpio_pin_configure(gpiog, 12, GPIO_OUTPUT_LOW | GPIO_ACTIVE_HIGH);
    gpio_pin_configure(gpiog, 13, GPIO_OUTPUT_LOW | GPIO_ACTIVE_HIGH);
    gpio_pin_configure(gpiog, 15, GPIO_OUTPUT_LOW | GPIO_ACTIVE_HIGH);
    gpio_pin_configure(gpiok, 3, GPIO_OUTPUT_LOW | GPIO_ACTIVE_HIGH);
    gpio_pin_configure(gpiok, 4, GPIO_OUTPUT_LOW | GPIO_ACTIVE_HIGH);
    gpio_pin_configure(gpiok, 5, GPIO_OUTPUT_LOW | GPIO_ACTIVE_HIGH);
    gpio_pin_configure(gpiok, 6, GPIO_OUTPUT_LOW | GPIO_ACTIVE_HIGH);
    gpio_pin_configure(gpiok, 7, GPIO_OUTPUT_LOW | GPIO_ACTIVE_HIGH);
    int led{1};
    while (true) {
        gpio_pin_set(gpiog, 12, led);
        gpio_pin_set(gpiog, 13, led);
        gpio_pin_set(gpiog, 15, led);
        gpio_pin_set(gpiok, 3, led);
        gpio_pin_set(gpiok, 4, led);
        gpio_pin_set(gpiok, 5, led);
        gpio_pin_set(gpiok, 6, led);
        gpio_pin_set(gpiok, 7, led);
        led = !led;
        k_msleep(1000);
    }
}

// vim: set expandtab shiftwidth=4:
