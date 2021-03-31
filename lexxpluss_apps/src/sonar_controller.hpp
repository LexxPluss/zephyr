#pragma once

struct device;

class sonar_controller {
public:
    int init();
    void run();
private:
    const device *dev = nullptr;
};
