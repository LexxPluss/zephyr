#pragma once

class adc_reader {
public:
    int setup();
    void loop();
    static uint16_t get(int index);
    static constexpr int INDEX_DOWNWARD_L{0};
    static constexpr int INDEX_DOWNWARD_R{1};
    static constexpr int INDEX_ACTUATOR_0{2};
    static constexpr int INDEX_ACTUATOR_1{3};
    static constexpr int INDEX_ACTUATOR_2{4};
    static constexpr int INDEX_CONNECT_CART{5};
private:
};

// vim: set expandtab shiftwidth=4:
