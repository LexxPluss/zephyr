#include <device.h>
#include <drivers/adc.h>
#include "adc_reader.hpp"

namespace {

class adc_reader_impl {
public:
    int init() {
        dev = device_get_binding("ADC_1");
        if (dev != nullptr) {
            static constexpr uint8_t ch[NUM_CHANNELS]{0, 1, 10, 11, 12, 13};
            adc_channel_cfg channel_cfg{
                .gain{ADC_GAIN_1},
                .reference{ADC_REF_INTERNAL},
                .acquisition_time{ADC_ACQ_TIME_DEFAULT},
                .differential{0}
            };
            for (auto i : ch) {
                sequence.channels |= BIT(i);
                channel_cfg.channel_id = i;
                adc_channel_setup(dev, &channel_cfg);
            }
        }
        return dev == nullptr ? -1 : 0;
    }
    void run() {
        while (true) {
            adc_read(dev, &sequence);
            k_msleep(50);
        }
    }
    uint16_t get(int index) {
        int32_t ref = adc_ref_internal(dev);
        int32_t value = buffer[index];
        if (ref > 0)
            adc_raw_to_millivolts(ref, ADC_GAIN_1, 12, &value);
        return value;
    }
private:
    static constexpr int NUM_CHANNELS{6};
    const device *dev = nullptr;
    adc_sequence sequence{
        .options{nullptr},
        .channels{0},
        .buffer{buffer},
        .buffer_size{sizeof buffer},
        .resolution{12},
        .oversampling{0},
        .calibrate{0}
    };
    uint16_t buffer[NUM_CHANNELS];
} impl;

}

void adc_reader::init()
{
    impl.init();
}

void adc_reader::run(void *p1, void *p2, void *p3)
{
    impl.run();
}

uint16_t adc_reader::get(int index)
{
    return impl.get(index);
}

k_thread adc_reader::thread;

// vim: set expandtab shiftwidth=4:
