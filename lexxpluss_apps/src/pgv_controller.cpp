#include <zephyr.h>
#include <device.h>
#include <drivers/uart.h>
#include <sys/ring_buffer.h>
#include "message.hpp"
#include "thread_runner.hpp"

k_msgq pgv_controller_msgq;

namespace {

char __aligned(4) pgv_controller_msgq_buffer[10 * sizeof (pgv_message)];

class pgv_driver {
public:
    int init() {
        ring_buf_init(&rxbuf.rb, sizeof rxbuf.buf, rxbuf.buf);
        ring_buf_init(&txbuf.rb, sizeof txbuf.buf, txbuf.buf);
        dev = device_get_binding("UART_@@");
        if (dev != nullptr) {
            uart_config config{
                .baudrate = 115200,
                .parity = UART_CFG_PARITY_EVEN,
                .stop_bits = UART_CFG_STOP_BITS_1,
                .data_bits = UART_CFG_DATA_BITS_8,
                .flow_ctrl = UART_CFG_FLOW_CTRL_NONE
            };
            uart_configure(dev, &config);
            uart_irq_rx_disable(dev);
            uart_irq_tx_disable(dev);
            uart_irq_callback_user_data_set(dev, uart_isr_trampoline, this);
            uart_irq_rx_enable(dev);
            direction_decision(DIR::STRAIGHT);
        }
        return dev == nullptr ? -1 : 0;
    }
    bool get_position(pgv_message &data) {
        ring_buf_reset(&rxbuf.rb);
        uint8_t req[2];
        req[0] = 0xc8;
        req[1] = ~req[0];
        send(req, sizeof req);
        for (int i = 0; i < 100; ++i) {
            if (rb_count(&rxbuf.rb) >= 21) 
                break;
            k_msleep(10);
        }
        uint8_t buf[64];
        int n = ring_buf_get(&rxbuf.rb, buf, sizeof buf);
        if (n < 21 || validate(buf, 21))
            return false;
        decode(buf, data);
        return true;
    }
private:
    enum class DIR {
        RIGHT,
        LEFT,
        STRAIGHT
    };
    void decode(const uint8_t *buf, pgv_message &data) const {
        data.f.cc2 =  (buf[ 0] & 0x40) != 0;
        data.addr  =  (buf[ 0] & 0x30) >> 4;
        data.f.cc1 =  (buf[ 0] & 0x08) != 0;
        data.f.wrn =  (buf[ 0] & 0x04) != 0;
        data.f.np  =  (buf[ 0] & 0x02) != 0;
        data.f.err =  (buf[ 0] & 0x01) != 0;
        data.f.tag =  (buf[ 1] & 0x40) != 0;
        data.lane  =  (buf[ 1] & 0x30) >> 4;
        data.f.rp  =  (buf[ 1] & 0x08) != 0;
        data.f.nl  =  (buf[ 1] & 0x04) != 0;
        data.f.ll  =  (buf[ 1] & 0x02) != 0;
        data.f.rl  =  (buf[ 1] & 0x01) != 0;
        data.yps   =  (buf[ 6] & 0x40 ? 0xc000 : 0) |
                     ((buf[ 6] & 0x7f) << 7) |
                      (buf[ 7] & 0x7f);
        data.ang   = ((buf[10] & 0x7f) << 7) |
                      (buf[11] & 0x7f);
        data.wrn   = ((buf[18] & 0x7f) << 7) |
                      (buf[19] & 0x7f);
        if (data.f.tag) { // data matrix tag
            data.xp  = 0;
            data.o1  = 0;
            data.s1  = 0;
            data.cc1 = 0;
            data.o2  = 0;
            data.s2  = 0;
            data.cc2 = 0;
            data.xps =  (buf[ 2] & 0x04 ? 0xff00000000 : 0) |
                       ((buf[ 2] & 0x07) << 21) |
                       ((buf[ 3] & 0x7f) << 14) |
                       ((buf[ 4] & 0x7f) <<  7) |
                        (buf[ 5] & 0x7f);
            data.tag = ((buf[14] & 0x7f) << 21) |
                       ((buf[15] & 0x7f) << 14) |
                       ((buf[16] & 0x7f) <<  7) |
                        (buf[17] & 0x7f);
        } else { // lane tracking
            data.xp  = ((buf[ 2] & 0x07) << 21) |
                       ((buf[ 3] & 0x7f) << 14) |
                       ((buf[ 4] & 0x7f) <<  7) |
                        (buf[ 5] & 0x7f);
            data.o1  =  (buf[14] & 0x60) >> 5;
            data.s1  =  (buf[14] & 0x18) >> 3;
            data.cc1 = ((buf[14] & 0x07) << 7) |
                        (buf[15] & 0x7f);
            data.o2  =  (buf[16] & 0x60) >> 5;
            data.s2  =  (buf[16] & 0x18) >> 3;
            data.cc2 = ((buf[16] & 0x07) << 7) |
                        (buf[17] & 0x7f);
            data.xps = 0;
            data.tag = 0;
        }
    }
    void direction_decision(DIR dir) {
        switch (dir) {
        case DIR::RIGHT:
        case DIR::LEFT:
        case DIR::STRAIGHT:
            break;
        }
    }
    bool validate(const uint8_t *buf, uint32_t length) const {
        uint32_t tail = length - 1;
        uint8_t check = buf[0];
        for (uint32_t i = 1; i < tail; ++i)
            check ^= buf[i];
        return check == buf[tail];
    }
    uint32_t rb_count(const ring_buf *rb) const {
	return rb->tail - rb->head;
    }
    void send(const uint8_t *buf, uint32_t length) {
        if (dev != nullptr) {
            while (length > 0) {
                int n = ring_buf_put(&txbuf.rb, buf, length);
                uart_irq_tx_enable(dev);
                buf += n;
                length -= n;
            }
        }
    }
    bool wait_data(uint32_t length) const {
        for (int i = 0; i < 100; ++i) {
            if (rb_count(&rxbuf.rb) >= length) 
                return true;
            k_msleep(10);
            return false;
        }
    }
    int recv(uint8_t *buf, uint32_t length) {
        return ring_buf_get(&rxbuf.rb, buf, length);
    }
    bool is_received() {
        return !ring_buf_is_empty(&rxbuf.rb);
    }
    void uart_isr() {
        while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
            uint8_t buf[64];
            if (uart_irq_rx_ready(dev)) {
                int n = uart_fifo_read(dev, buf, sizeof buf);
                if (n > 0)
                    ring_buf_put(&rxbuf.rb, buf, n);
            }
            if (uart_irq_tx_ready(dev)) {
                int n = ring_buf_get(&txbuf.rb, buf, 1);
                if (n > 0)
                    uart_fifo_fill(dev, buf, 1);
                else
                    uart_irq_tx_disable(dev);
            }
        }
    }
    static void uart_isr_trampoline(const device *dev, void *user_data) {
        pgv_driver *self = static_cast<pgv_driver*>(user_data);
        self->uart_isr();
    }
    struct {
        ring_buf rb;
        uint32_t buf[256 / sizeof (uint32_t)];
    } txbuf, rxbuf;
    const device *dev = nullptr;
};

class pgv_controller {
public:
    int setup() {
        k_msgq_init(&pgv_controller_msgq, pgv_controller_msgq_buffer, sizeof (pgv_message), 10);
        return driver.init();
    }
    void loop() {
        pgv_message message;
        if (driver.get_position(message)) {
            while (k_msgq_put(&pgv_controller_msgq, &message, K_NO_WAIT) != 0)
                k_msgq_purge(&pgv_controller_msgq);
        }
        k_msleep(30);
    }
private:
    pgv_driver driver;
};

LEXX_THREAD_RUNNER(pgv_controller);

}

/* vim: set expandtab shiftwidth=4: */
