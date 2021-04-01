#include <zephyr.h>
#include "thread_runner.hpp"

class host_communicator {
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

LEXX_THREAD(host_communicator);
