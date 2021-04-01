#include <zephyr.h>
#include <device.h>
#include <drivers/spi.h>
#include "thread_runner.hpp"

class led_controller {
public:
    int init() {
        dev = device_get_binding(DT_LABEL(DT_NODELABEL(spi4)));
        return dev == nullptr ? -1 : 0;
    }
    void run() {
        while (true) {
            k_msleep(1000);
        }
    }
private:
    const device *dev = nullptr;
    const spi_config config = {
        .frequency = 0,
        .operation = SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8),
        .slave = 0,
        .cs = nullptr
    };
};

LEXX_THREAD(led_controller);
