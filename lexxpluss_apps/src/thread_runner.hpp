#pragma once

#define LEXX_THREAD_RUNNER(name) \
    static name instance; \
    K_THREAD_DEFINE(tid_##name, 2048, &thread_runner<name>::runner, &instance, nullptr, nullptr, 5, 0, 0)

template <class T>
struct thread_runner {
    static void runner(void *p1, void *p2, void *p3) {
        if (p1 != nullptr) {
            T *target = static_cast<T*>(p1);
            if (target->init() == 0)
                target->run();
        }
    }
};
