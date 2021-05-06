#include <zephyr.h>
#include "thread_runner.hpp"

namespace {

class commander {
public:
    int setup() const {
        return 0;
    }
    void loop() const {
        k_msleep(1000);
    }
};

LEXX_THREAD_RUNNER(commander);

}
