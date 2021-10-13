#include <zephyr.h>
#include <device.h>
#include <drivers/uart.h>
#include <sys/ring_buffer.h>
#include "pgv_controller.hpp"

k_msgq msgq_pgv2ros;
k_msgq msgq_ros2pgv;

namespace {

char __aligned(4) msgq_pgv2ros_buffer[10 * sizeof (msg_pgv2ros)];
char __aligned(4) msgq_ros2pgv_buffer[10 * sizeof (msg_ros2pgv)];

class pgv_controller_impl {
public:
    int init() {
        k_msgq_init(&msgq_pgv2ros, msgq_pgv2ros_buffer, sizeof (msg_pgv2ros), 10);
        k_msgq_init(&msgq_ros2pgv, msgq_ros2pgv_buffer, sizeof (msg_ros2pgv), 10);
        ring_buf_init(&rxbuf.rb, sizeof rxbuf.buf, rxbuf.buf);
        ring_buf_init(&txbuf.rb, sizeof txbuf.buf, txbuf.buf);
        dev = device_get_binding("UART_6");
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
        }
        return dev == nullptr ? -1 : 0;
    }
    void run() {
        if (!device_is_ready(dev))
            return;
        for (int i = 0; i < 30; ++i) {
            ring_buf_reset(&rxbuf.rb);
            set_direction_decision(DIR::STRAIGHT);
            if (wait_data(3))
                break;
        }
        while (true) {
            msg_pgv2ros pgv2ros;
            if (get_position(pgv2ros)) {
                while (k_msgq_put(&msgq_pgv2ros, &pgv2ros, K_NO_WAIT) != 0)
                    k_msgq_purge(&msgq_pgv2ros);
            }
            msg_ros2pgv ros2pgv;
            if (k_msgq_get(&msgq_ros2pgv, &ros2pgv, K_NO_WAIT) == 0) {
                switch (ros2pgv.dir_command) {
                    case 0: set_direction_decision(DIR::NOLANE);   break;
                    case 1: set_direction_decision(DIR::RIGHT);    break;
                    case 2: set_direction_decision(DIR::LEFT);     break;
                    default:
                    case 3: set_direction_decision(DIR::STRAIGHT); break;
                }
                wait_data(3);
            }
            k_msleep(30);
        }
    }
private:
    enum class DIR {
        NOLANE,
        RIGHT,
        LEFT,
        STRAIGHT
    };
    bool get_position(msg_pgv2ros &data) {
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
    void set_direction_decision(DIR dir) {
        uint8_t req[2];
        switch (dir) {
        case DIR::NOLANE:   req[0] = 0xe0; break;
        case DIR::RIGHT:    req[0] = 0xe4; break;
        case DIR::LEFT:     req[0] = 0xe8; break;
        case DIR::STRAIGHT: req[0] = 0xec; break;
        }
        req[1] = ~req[0];
        send(req, sizeof req);
    }
    void decode(const uint8_t *buf, msg_pgv2ros &data) const {
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
        }
        return false;
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
        pgv_controller_impl *self = static_cast<pgv_controller_impl*>(user_data);
        self->uart_isr();
    }
    struct {
        ring_buf rb;
        uint32_t buf[256 / sizeof (uint32_t)];
    } txbuf, rxbuf;
    const device *dev = nullptr;
} impl;

}

void pgv_controller::init()
{
    impl.init();
}

void pgv_controller::run(void *p1, void *p2, void *p3)
{
    impl.run();
}

k_thread pgv_controller::thread;

/* vim: set expandtab shiftwidth=4: */
