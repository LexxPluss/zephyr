#pragma once

#include <zephyr.h>

#define LEXX_THREAD_RUNNER(name) \
    static name instance; \
    K_THREAD_DEFINE(tid_##name, 2048, &thread_runner<name>::runner, &instance, nullptr, nullptr, 5, K_FP_REGS, 1000)

template <class T>
struct thread_runner {
    static void runner(void *p1, void *p2, void *p3) {
        if (p1 != nullptr) {
            T *target = static_cast<T*>(p1);
            if (target->setup() == 0) {
                while (true)
                    target->loop();
            }
        }
    }
};
