#include <zephyr.h>
#include <device.h>
#include <drivers/led_strip.h>
#include "thread_runner.hpp"

#define LED    DT_ALIAS(led_strip)
#define LABEL  DT_LABEL(LED)
#define PIXELS DT_PROP(LED, chain_length)

class led_controller {
public:
    int init() {
        colors[0] = {.r = 0xff, .g = 0x00, .b = 0x00};
        colors[1] = {.r = 0x00, .g = 0xff, .b = 0x00};
        colors[2] = {.r = 0x00, .g = 0x00, .b = 0xff};
        dev = device_get_binding(LABEL);
        return dev == nullptr ? -1 : 0;
    }
#if 0
    void run() {
        int index = 0, color = 0;
        while (true) {
            clear();
            pixeldata[index] = colors[color];
            led_strip_update_rgb(dev, pixeldata, PIXELS);
            if (++index >= PIXELS) {
                index = 0;
                if (++color >= 3)
                    color = 0;
            }
            k_sleep(K_MSEC(200));
        }
    }
#endif
    void run() {
        int dim = 0, color = 0, step = 1;
        while (true) {
            setup_pixel(colors[color], dim);
            led_strip_update_rgb(dev, pixeldata, PIXELS);
            dim += step;
            if (dim < 0 || dim >= 100) {
                if (dim < 0) {
                    dim = 0;
                    if (++color >= 3)
                        color = 0;
                }
                if (dim > 100)
                    dim = 100;
                step *= -1;
            }
            k_sleep(K_MSEC(8));
        }
    }
private:
    void setup_pixel(led_rgb color, int dim) {
        color.r = color.r * dim / 100;
        color.g = color.g * dim / 100;
        color.b = color.b * dim / 100;
        for (int i = 0; i < PIXELS; ++i)
            pixeldata[i] = color;
    }
    void clear() {
        static const led_rgb black = {.r = 0x00, .g = 0x00, .b = 0x00};
        for (int i = 0; i < PIXELS; ++i)
            pixeldata[i] = black;
    }
    const device *dev = nullptr;
    led_rgb pixeldata[PIXELS];
    led_rgb colors[3];
};

LEXX_THREAD_RUNNER(led_controller);
