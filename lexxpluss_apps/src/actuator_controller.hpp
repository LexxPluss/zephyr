#pragma once

struct device;

class actuator_controller {
public:
    int init();
    void run();
private:
    const device *dev = nullptr;
};
