#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include "thread_runner.hpp"

class sonar_controller {
public:
    int init() {
        return 0;
    }
    void run() {
        while (true) {
            k_msleep(300);
        }
    }
private:
    const device *dev = nullptr;
};

LEXX_THREAD_RUNNER(sonar_controller);
