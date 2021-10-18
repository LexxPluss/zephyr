#include <device.h>
#include <devicetree.h>
#include <drivers/led_strip.h>
#include "led_controller.hpp"

k_msgq msgq_ros2led;

namespace {

char __aligned(4) msgq_ros2led_buffer[10 * sizeof (msg_ros2led)];

class led_controller_impl {
public:
    int init() {
        k_msgq_init(&msgq_ros2led, msgq_ros2led_buffer, sizeof (msg_ros2led), 10);
        message.pattern = msg_ros2led::SHOWTIME;
        dev[LED_LEFT] = device_get_binding("WS2812_0");
        dev[LED_RIGHT] = device_get_binding("WS2812_1");
        if (dev[LED_LEFT] == nullptr || dev[LED_RIGHT] == nullptr)
            return -1;
        fill(black);
        update();
        return 0;
    }
    void run() {
        if (!device_is_ready(dev[LED_LEFT]) || !device_is_ready(dev[LED_RIGHT]))
            return;
        while (true) {
            if (k_msgq_get(&msgq_ros2led, &message, K_MSEC(DELAY_MS)) == 0)
                counter = 0;
            poll();
        }
    }
private:
    void poll() {
        switch (message.pattern) {
        default:
        case msg_ros2led::NONE:            fill(black); break;
        case msg_ros2led::EMERGENCY_STOP:  fill_strobe(emergency_stop, 10, 50, 1000); break;
        case msg_ros2led::AMR_MODE:        fill(amr_mode); break;
        case msg_ros2led::AGV_MODE:        fill(agv_mode); break;
        case msg_ros2led::MISSION_PAUSE:   fill(mission_pause); break;
        case msg_ros2led::PATH_BLOCKED:    fill(path_blocked); break;
        case msg_ros2led::MANUAL_DRIVE:    fill(manual_drive); break;
        case msg_ros2led::CHARGING:        fill_rainbow(); break;
        case msg_ros2led::WAITING_FOR_JOB: fill_fade(waiting_for_job); break;
        case msg_ros2led::LEFT_WINKER:     fill_blink_sequence(sequence, LED_LEFT); break;
        case msg_ros2led::RIGHT_WINKER:    fill_blink_sequence(sequence, LED_RIGHT); break;
        case msg_ros2led::BOTH_WINKER:     fill_blink_sequence(sequence, LED_BOTH); break;
        case msg_ros2led::MOVE_ACTUATOR:   fill_strobe(move_actuator, 10, 200, 200); break;
        case msg_ros2led::SHOWTIME:        fill_toggle(showtime); break;
        case msg_ros2led::RGB:             fill(led_rgb{.r{message.rgb[0]}, .g{message.rgb[1]}, .b{message.rgb[2]}}); break;
        update();
        }
        ++counter;
    }
    void update() {
        led_strip_update_rgb(dev[LED_LEFT], pixeldata[LED_LEFT], PIXELS);
        led_strip_update_rgb(dev[LED_RIGHT], pixeldata[LED_RIGHT], PIXELS);
    }
    void fill(const led_rgb &color, uint32_t select = LED_BOTH) {
        if (select == LED_BOTH) {
            for (uint32_t i{0}; i < PIXELS; ++i)
                pixeldata[LED_LEFT][i] = pixeldata[LED_RIGHT][i] = color;
        } else {
            for (uint32_t i{0}; i < PIXELS; ++i)
                pixeldata[select][i] = color;
        }
    }
    void fill_strobe(const led_rgb &color, uint32_t nstrobe, uint32_t strobedelay, uint32_t endpause) {
        if (counter < nstrobe * strobedelay / DELAY_MS) {
            if ((counter % (strobedelay * 2 / DELAY_MS)) == 0)
                fill(color);
            else if ((counter % (strobedelay * 2 / DELAY_MS)) == strobedelay / DELAY_MS)
                fill(black);
        } else if (counter == (nstrobe * strobedelay + endpause) / DELAY_MS) {
            fill(black);
            counter = 0;
        }
    }
    void fill_rainbow(uint32_t select = LED_BOTH) {
        if (counter % 3 == 0)
            return;
        if (counter > 256 * 3)
            counter = 0;
        if (select == LED_BOTH) {
            for (uint32_t i{0}; i < PIXELS; ++i)
                pixeldata[LED_LEFT][i] = pixeldata[LED_RIGHT][i] = wheel(((i * 256 / PIXELS) + counter / 3) & 255);
        } else {
            for (uint32_t i{0}; i < PIXELS; ++i)
                pixeldata[select][i] = wheel(((i * 256 / PIXELS) + counter / 3) & 255);
        }
    }
    void fill_fade(const led_rgb &color) {
        static constexpr uint32_t thres{130};
        if (counter >= thres * 2)
            counter = 0;
        int percent;
        if (counter < thres)
            percent = counter * 100 / thres;
        else
            percent = (thres * 2 - counter) * 100 / thres;
        fill(fader(color, percent));
    }
    void fill_blink_sequence(const led_rgb &color, uint32_t select = LED_BOTH) {
        uint32_t n{0};
        if (counter >= 8 && counter < 25) {
            n = (counter - 8) * 6;
            if (n > PIXELS)
                n = PIXELS;
        }
        if (select == LED_BOTH) {
            for (uint32_t i{0}; i < PIXELS; ++i)
                pixeldata[LED_LEFT][i] = pixeldata[LED_RIGHT][i] = i < n ? color : black;
        } else {
            for (uint32_t i{0}; i < PIXELS; ++i)
                pixeldata[select][i] = i < n ? color : black;
            fill(black, select == LED_LEFT ? LED_RIGHT : LED_LEFT);
        }
        if (counter > 25)
            counter = 0;
    }
    void fill_toggle(const led_rgb &color) {
        static constexpr uint32_t thres{10};
        if (counter >= thres * 2)
            counter = 0;
        led_rgb c0, c1;
        if (counter < thres)
            c0 = color, c1 = black;
        else
            c0 = black, c1 = color;
        for (uint32_t i{0}, end{PIXELS / 8}; i < end; i += 8) {
            for (uint32_t j{0}; j < 4; ++j)
                pixeldata[LED_LEFT][i + j] = pixeldata[LED_RIGHT][i + j] = c0;
            for (uint32_t j{4}; j < 8; ++j)
                pixeldata[LED_LEFT][i + j] = pixeldata[LED_RIGHT][i + j] = c1;
        }
    }
    led_rgb fader(const led_rgb &color, int percent) const {
        led_rgb color_;
        color_.r = color.r * percent / 100;
        color_.g = color.g * percent / 100;
        color_.b = color.b * percent / 100;
        return color_;
    }
    led_rgb wheel(uint32_t wheelpos) const {
        static constexpr uint32_t thres{256 / 3};
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
    msg_ros2led message;
    static constexpr uint32_t PIXELS{DT_PROP(DT_NODELABEL(led_strip0), chain_length)};
    static constexpr uint32_t LED_LEFT{0}, LED_RIGHT{1}, LED_BOTH{2}, LED_NUM{2};
    static constexpr uint32_t DELAY_MS{50};
    const device *dev[LED_NUM]{nullptr, nullptr};
    led_rgb pixeldata[LED_NUM][PIXELS];
    uint32_t counter{0};
    static const led_rgb emergency_stop, amr_mode, agv_mode, mission_pause, path_blocked, manual_drive;
    static const led_rgb dock_mode, waiting_for_job, orange, sequence, move_actuator, showtime, black;
} impl;
const led_rgb led_controller_impl::emergency_stop {.r{0x80}, .g{0x00}, .b{0x00}};
const led_rgb led_controller_impl::amr_mode       {.r{0x00}, .g{0x80}, .b{0x80}};
const led_rgb led_controller_impl::agv_mode       {.r{0x45}, .g{0xff}, .b{0x00}};
const led_rgb led_controller_impl::mission_pause  {.r{0xff}, .g{0xff}, .b{0x00}};
const led_rgb led_controller_impl::path_blocked   {.r{0xe6}, .g{0x08}, .b{0xff}};
const led_rgb led_controller_impl::manual_drive   {.r{0xfe}, .g{0xf4}, .b{0xff}};
const led_rgb led_controller_impl::dock_mode      {.r{0x00}, .g{0x00}, .b{0xff}};
const led_rgb led_controller_impl::waiting_for_job{.r{0xff}, .g{0xff}, .b{0x00}};
const led_rgb led_controller_impl::orange         {.r{0xff}, .g{0xa5}, .b{0x00}};
const led_rgb led_controller_impl::sequence       {.r{0x90}, .g{0x20}, .b{0x00}};
const led_rgb led_controller_impl::move_actuator  {.r{0x45}, .g{0xff}, .b{0x00}};
const led_rgb led_controller_impl::showtime       {.r{0x0f}, .g{0xb6}, .b{0xc8}};
const led_rgb led_controller_impl::black          {.r{0x00}, .g{0x00}, .b{0x00}};

}

void led_controller::init()
{
    impl.init();
}

void led_controller::run(void *p1, void *p2, void *p3)
{
    impl.run();
}

k_thread led_controller::thread;

// vim: set expandtab shiftwidth=4:
