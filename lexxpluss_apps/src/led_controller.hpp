#pragma once

struct device;

class led_controller {
public:
    int init();
    void run();
private:
    const device *dev = nullptr;
};
