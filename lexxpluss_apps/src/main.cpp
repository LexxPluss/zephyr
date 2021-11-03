#include <zephyr.h>
#include <disk/disk_access.h>
#include <drivers/gpio.h>
#include <fs/fs.h>
#include <ff.h>
#include "actuator_controller.hpp"
#include "adc_reader.hpp"
#include "can_controller.hpp"
#include "imu_controller.hpp"
#include "led_controller.hpp"
#include "misc_controller.hpp"
#include "pgv_controller.hpp"
#include "rosserial.hpp"
#include "tof_controller.hpp"
#include "uss_controller.hpp"

namespace {

K_THREAD_STACK_DEFINE(actuator_controller_stack, 2048);
K_THREAD_STACK_DEFINE(adc_reader_stack, 2048);
K_THREAD_STACK_DEFINE(can_controller_stack, 2048);
K_THREAD_STACK_DEFINE(imu_controller_stack, 2048);
K_THREAD_STACK_DEFINE(led_controller_stack, 2048);
K_THREAD_STACK_DEFINE(misc_controller_stack, 2048);
K_THREAD_STACK_DEFINE(pgv_controller_stack, 2048);
K_THREAD_STACK_DEFINE(rosserial_stack, 2048);
K_THREAD_STACK_DEFINE(tof_controller_stack, 2048);
K_THREAD_STACK_DEFINE(uss_controller_stack, 2048);

#define RUN(name, prio) \
    k_thread_create(&name::thread, name##_stack, K_THREAD_STACK_SIZEOF(name##_stack), \
                    name::run, nullptr, nullptr, nullptr, prio, K_FP_REGS, K_MSEC(2000));

}

FATFS fatfs;
fs_mount_t mount;

void main()
{
    actuator_controller::init();
    adc_reader::init();
    can_controller::init();
    imu_controller::init();
    led_controller::init();
    misc_controller::init();
    pgv_controller::init();
    rosserial::init();
    tof_controller::init();
    uss_controller::init();
    RUN(actuator_controller, 1);
    RUN(adc_reader, 1);
    RUN(can_controller, 3);
    RUN(imu_controller, 1);
    RUN(led_controller, 1);
    RUN(misc_controller, 1);
    RUN(pgv_controller, 1);
    RUN(tof_controller, 1);
    RUN(uss_controller, 1);
    RUN(rosserial, 2); // The rosserial thread will be started last.
    if (disk_access_init("SD") == 0) {
        mount.type = FS_FATFS;
        mount.fs_data = &fatfs;
        mount.mnt_point = "/SD:";
        fs_mount(&mount);
    }
    const device *gpiog = device_get_binding("GPIOG");
    if (gpiog != nullptr)
        gpio_pin_configure(gpiog, 12, GPIO_OUTPUT_LOW | GPIO_ACTIVE_HIGH);
    int heartbeat_led{1};
    while (true) {
        if (gpiog != nullptr) {
            gpio_pin_set(gpiog, 12, heartbeat_led);
            heartbeat_led = !heartbeat_led;
        }
        k_msleep(1000);
    }
}

// vim: set expandtab shiftwidth=4:
