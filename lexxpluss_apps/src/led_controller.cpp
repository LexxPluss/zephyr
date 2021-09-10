#include <device.h>
#include <devicetree.h>
#include <drivers/led_strip.h>
#include "message.hpp"
#include "thread_runner.hpp"

k_msgq msgq_ros2led;

namespace {

char __aligned(4) msgq_ros2led_buffer[10 * sizeof (msg_ros2led)];

class led_controller_impl {
public:
    int init() {
        dev[LED_LEFT] = device_get_binding("WS2812_0");
        dev[LED_RIGHT] = device_get_binding("WS2812_1");
        if (dev[LED_LEFT] == nullptr || dev[LED_RIGHT] == nullptr)
            return -1;
        fill(black);
        update();
        return 0;
    }
    void reset() {counter = 0;}
    void poll(uint32_t pattern) {
        switch (pattern) {
        default:
        case msg_ros2led::NONE:            fill(black); break;
        case msg_ros2led::EMERGENCY_STOP:  fill_strobe(emergency_stop, 10, 50, 1000); break;
        case msg_ros2led::AMR_MODE:        fill(amr_mode); break;
        case msg_ros2led::AGV_MODE:        fill(agv_mode); break;
        case msg_ros2led::MISSION_PAUSE:   fill(mission_pause); break;
        case msg_ros2led::PATH_BLOCKED:    fill(path_blocked); break;
        case msg_ros2led::MANUAL_DRIVE:    fill(manual_drive); break;
        case msg_ros2led::CHARGING:        break;
        case msg_ros2led::WAITING_FOR_JOB: break;
        case msg_ros2led::LEFT_WINKER:     fill_blink_sequence(sequence, LED_LEFT); break;
        case msg_ros2led::RIGHT_WINKER:    fill_blink_sequence(sequence, LED_RIGHT); break;
        case msg_ros2led::BOTH_WINKER:     fill_blink_sequence(sequence, LED_BOTH); break;
        case msg_ros2led::MOVE_ACTUATOR:   fill_strobe(move_actuator, 10, 200, 200); break;
        }
        ++counter;
    }
    static constexpr uint32_t DELAY_MS{50};
private:
    void update() {
        led_strip_update_rgb(dev[LED_LEFT], pixeldata[LED_LEFT], PIXELS);
        led_strip_update_rgb(dev[LED_RIGHT], pixeldata[LED_RIGHT], PIXELS);
    }
    void fill(const led_rgb &color, uint32_t select = LED_BOTH) {
        if (select == LED_BOTH) {
            for (uint32_t i = 0; i < PIXELS; ++i)
                pixeldata[LED_LEFT][i] = pixeldata[LED_RIGHT][i] = color;
        } else {
            for (uint32_t i = 0; i < PIXELS; ++i)
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
    void fill_blink_sequence(const led_rgb &color, uint32_t select = LED_BOTH) {
        uint32_t n{0};
        if (counter >= 8 && counter < 25) {
            n = (counter - 8) * 6;
            if (n > PIXELS)
                n = PIXELS;
        }
        if (select == LED_BOTH) {
            for (uint32_t i = 0; i < PIXELS; ++i)
                pixeldata[LED_LEFT][i] = pixeldata[LED_RIGHT][i] = i < n ? color : black;
        } else {
            for (uint32_t i = 0; i < PIXELS; ++i)
                pixeldata[select][i] = i < n ? color : black;
            fill(black, select == LED_LEFT ? LED_RIGHT : LED_LEFT);
        }
        if (counter > 25)
            counter = 0;
    }
    static constexpr uint32_t PIXELS{DT_PROP(DT_NODELABEL(led_strip0), chain_length)};
    static constexpr uint32_t LED_LEFT{0}, LED_RIGHT{1}, LED_BOTH{2}, LED_NUM{2};
    const device *dev[LED_NUM]{nullptr, nullptr};
    led_rgb pixeldata[LED_NUM][PIXELS];
    uint32_t counter{0};
    static const led_rgb emergency_stop, amr_mode, agv_mode, mission_pause, path_blocked, manual_drive;
    static const led_rgb dock_mode, waiting_for_job, orange, sequence, move_actuator, black;
};
const led_rgb led_controller_impl::emergency_stop {.r = 0x80, .g = 0x00, .b = 0x00};
const led_rgb led_controller_impl::amr_mode       {.r = 0x00, .g = 0x80, .b = 0x80};
const led_rgb led_controller_impl::agv_mode       {.r = 0x45, .g = 0xff, .b = 0x00};
const led_rgb led_controller_impl::mission_pause  {.r = 0xff, .g = 0xff, .b = 0x00};
const led_rgb led_controller_impl::path_blocked   {.r = 0xe6, .g = 0x08, .b = 0xff};
const led_rgb led_controller_impl::manual_drive   {.r = 0xfe, .g = 0xf4, .b = 0xff};
const led_rgb led_controller_impl::dock_mode      {.r = 0x00, .g = 0x00, .b = 0xff};
const led_rgb led_controller_impl::waiting_for_job{.r = 0xff, .g = 0xff, .b = 0x00};
const led_rgb led_controller_impl::orange         {.r = 0xff, .g = 0xa5, .b = 0x00};
const led_rgb led_controller_impl::sequence       {.r = 0x90, .g = 0x20, .b = 0x00};
const led_rgb led_controller_impl::move_actuator  {.r = 0x45, .g = 0xff, .b = 0x00};
const led_rgb led_controller_impl::black          {.r = 0x00, .g = 0x00, .b = 0x00};

class led_controller {
public:
    int setup() {
        k_msgq_init(&msgq_ros2led, msgq_ros2led_buffer, sizeof (msg_ros2led), 10);
        return impl.init();
    }
    void loop() {
        msg_ros2led message;
        if (k_msgq_get(&msgq_ros2led, &message, K_MSEC(led_controller_impl::DELAY_MS)) == 0)
            impl.reset();
        impl.poll(message.pattern);
    }
private:
    led_controller_impl impl;
};

LEXX_THREAD_RUNNER(led_controller);

}
