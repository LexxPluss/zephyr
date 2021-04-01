#include <zephyr.h>
#include "thread_runner.hpp"

class failsafe_manager {
public:
    int init() {
        return 0;
    }
    void run() {
        while (true) {
            k_msleep(1000);
        }
    }
};

LEXX_THREAD(failsafe_manager);
