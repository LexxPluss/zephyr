#include <zephyr.h>
#include "thread_runner.hpp"

class led_controller {
public:
    int init() {
        return 0;
    }
    void run() {
        while (true) {
            k_msleep(1000);
        }
    }
private:
};

LEXX_THREAD_RUNNER(led_controller);
