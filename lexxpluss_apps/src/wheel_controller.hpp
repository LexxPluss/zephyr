#pragma once

struct device;

class wheel_controller {
public:
    int init();
    void run();
private:
    const device *dev = nullptr;
};
