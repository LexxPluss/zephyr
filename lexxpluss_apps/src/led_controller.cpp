#include <zephyr.h>
#include <device.h>
#include <drivers/led_strip.h>
#include <logging/log.h>
#include "message.hpp"
#include "thread_runner.hpp"

k_msgq led_controller_msgq;

namespace {

LOG_MODULE_REGISTER(led_controller, LOG_LEVEL_INF);

char __aligned(4) led_controller_msgq_buffer[10 * sizeof (led_message)];

#define PIXELS DT_PROP(DT_NODELABEL(led_strip0), chain_length)

class led_driver {
public:
    int init() {
        dev[0] = device_get_binding("WS2812_0");
        dev[1] = device_get_binding("WS2812_1");
        if (dev[0] == nullptr || dev[1] == nullptr)
            return -1;
        LOG_INF("LED strip for LexxPluss board.");
        fill(black);
        update_led();
        return 0;
    }
    void poll(uint32_t pattern) {
        switch (pattern) {
        default:
        case led_message::NONE:            fill(black);                                break;
        case led_message::EMERGENCY_STOP:  fill_strobe(emergency_stop, counter);       break;
        case led_message::AMR_MODE:        fill(amr_mode);                             break;
        case led_message::AGV_MODE:        fill(agv_mode);                             break;
        case led_message::MISSION_PAUSE:   fill(mission_pause);                        break;
        case led_message::PATH_BLOCKED:    fill(path_blocked);                         break;
        case led_message::MANUAL_DRIVE:    fill(manual_drive);                         break;
        case led_message::LEFT_INDICATOR:  fill_blink(orange, counter, SELECT::LEFT);  break;
        case led_message::RIGHT_INDICATOR: fill_blink(orange, counter, SELECT::RIGHT); break;
        case led_message::DOCK_MODE:       fill_fade(dock_mode, counter);              break;
        case led_message::CHARGING:        fill_rainbow(counter);                      break;
        case led_message::WAITING_FOR_JOB: fill_fade(waiting_for_job, counter);        break;
        }
        update_led();
        ++counter;
    }
    void reset() {counter = 0;}
private:
    enum class SELECT : int {LEFT = 0, RIGHT = 1, BOTH = 2};
    void update_led() {
        led_strip_update_rgb(dev[0], pixeldata[0], PIXELS);
        led_strip_update_rgb(dev[1], pixeldata[1], PIXELS);
    }
    void fill(const led_rgb &color, SELECT select = SELECT::BOTH) {
        if (select == SELECT::BOTH) {
            for (int i = 0; i < PIXELS; ++i) {
                pixeldata[0][i] = color;
                pixeldata[1][i] = color;
            }
        } else {
            for (int i = 0; i < PIXELS; ++i)
                pixeldata[static_cast<int>(select)][i] = color;
        }
    }
    void fill_blink(const led_rgb &color, uint32_t &counter, SELECT select = SELECT::BOTH) {
        static constexpr uint32_t thres = 64;
        if (counter > thres)
            counter = 0;
        if (counter == 0)
            fill(color, select);
        else if (counter == thres / 2)
            fill(black, select);
    }
    void fill_strobe(const led_rgb &color, uint32_t &counter) {
        static constexpr uint32_t thres = 20;
        static constexpr uint32_t pause = 40;
        if (counter < thres) {
            if ((counter % 4) == 0)
                fill(color);
            else if ((counter % 4) == 2)
                fill(black);
        } else if (counter > thres + pause) {
            counter = 0;
        }
    }
    void fill_rainbow(uint32_t &counter, SELECT select = SELECT::BOTH) {
        if (counter > 256 * 3)
            counter = 0;
        if (select == SELECT::BOTH) {
            for (int i = 0; i < PIXELS; ++i) {
                auto color = wheel(((i * 256 / PIXELS) + counter / 3) & 255);
                pixeldata[0][i] = color;
                pixeldata[1][i] = color;
            }
        } else {
            for (int i = 0; i < PIXELS; ++i)
                pixeldata[static_cast<int>(select)][i] = wheel(((i * 256 / PIXELS) + counter / 3) & 255);
        }
    }
    void fill_fade(const led_rgb &color, uint32_t &counter) {
        static constexpr uint32_t thres = 130;
        if (counter >= thres * 2)
            counter = 0;
        int percent;
        if (counter < thres)
            percent = counter * 100 / thres;
        else
            percent = (thres * 2 - counter) * 100 / thres;
        fill(fader(color, percent));
    }
    led_rgb fader(const led_rgb &color, int percent) const {
        led_rgb color_;
        color_.r = color.r * percent / 100;
        color_.g = color.g * percent / 100;
        color_.b = color.b * percent / 100;
        return color_;
    }
    led_rgb wheel(uint32_t wheelpos) const {
        static constexpr uint32_t thres = 256 / 3;
        led_rgb color;
        if (wheelpos < thres) {
            color.r = wheelpos * 3;
            color.g = 255 - wheelpos * 3;
            color.b = 0;
        } else if (wheelpos < thres * 2) {
            wheelpos -= thres;
            color.r = 255 - wheelpos * 3;
            color.g = 0;
            color.b = wheelpos * 3;
        } else {
            wheelpos -= thres * 2;
            color.r = 0;
            color.g = wheelpos * 3;
            color.b = 255 - wheelpos * 3;
        }
        return color;
    }
    const device *dev[2]{nullptr, nullptr};
    led_rgb pixeldata[2][PIXELS];
    uint32_t counter = 0;
    static const led_rgb emergency_stop, amr_mode, agv_mode, mission_pause, path_blocked, manual_drive, dock_mode, waiting_for_job, orange, black;
};
const led_rgb led_driver::emergency_stop  = {.r = 0xff, .g = 0x00, .b = 0x00};
const led_rgb led_driver::amr_mode        = {.r = 0x00, .g = 0x80, .b = 0x80};
const led_rgb led_driver::agv_mode        = {.r = 0x45, .g = 0xff, .b = 0x00};
const led_rgb led_driver::mission_pause   = {.r = 0xff, .g = 0xff, .b = 0x00};
const led_rgb led_driver::path_blocked    = {.r = 0xe6, .g = 0x08, .b = 0xff};
const led_rgb led_driver::manual_drive    = {.r = 0xfe, .g = 0xf4, .b = 0xff};
const led_rgb led_driver::dock_mode       = {.r = 0x00, .g = 0x00, .b = 0xff};
const led_rgb led_driver::waiting_for_job = {.r = 0xff, .g = 0xff, .b = 0xff};
const led_rgb led_driver::orange          = {.r = 0xff, .g = 0x80, .b = 0x00};
const led_rgb led_driver::black           = {.r = 0x00, .g = 0x00, .b = 0x00};

class led_controller {
public:
    int setup() {
        prev = k_uptime_get();
        k_msgq_init(&led_controller_msgq, led_controller_msgq_buffer, sizeof (led_message), 10);
        return driver.init();
    }
    void loop() {
        if (k_msgq_get(&led_controller_msgq, &message, K_NO_WAIT) == 0)
            driver.reset();
        auto now = k_uptime_get();
        if (now - prev >= 25) {
            prev = now;
            driver.poll(message.pattern);
        }
        k_msleep(1);
    }
private:
    led_driver driver;
    led_message message{led_message::NONE};
    int64_t prev = 0;
};

LEXX_THREAD_RUNNER(led_controller);

}

/*
vi:et:sw=4:
*/
