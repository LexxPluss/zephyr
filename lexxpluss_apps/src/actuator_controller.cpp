#include <device.h>
#include <drivers/gpio.h>
#include <drivers/pwm.h>
#include "message.hpp"
#include "thread_runner.hpp"

k_msgq msgq_actuator2ros;
k_msgq msgq_ros2actuator;

namespace {

char __aligned(4) msgq_actuator2ros_buffer[10 * sizeof (msg_actuator2ros)];
char __aligned(4) msgq_ros2actuator_buffer[10 * sizeof (msg_ros2actuator)];

class timer_hal_helper {
public:
    int init() {
        if (init_tim1() != 0 || init_tim5() != 0 || init_tim8() != 0)
            return -1;
        return 0;
    }
    void get_count(int16_t data[3]) const {
        data[0] = TIM1->CNT;
        TIM1->CNT = 0;
        data[1] = TIM5->CNT;
        TIM5->CNT = 0;
        data[2] = TIM8->CNT;
        TIM8->CNT = 0;
    }
private:
    int init_tim1() {
        TIM_Encoder_InitTypeDef sConfig{0};
        timh[0].Instance = TIM1;
        timh[0].Init.Prescaler = 0;
        timh[0].Init.CounterMode = TIM_COUNTERMODE_UP;
        timh[0].Init.Period = 65535;
        timh[0].Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
        timh[0].Init.RepetitionCounter = 0;
        timh[0].Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
        sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
        sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
        sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
        sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
        sConfig.IC1Filter = 0;
        sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
        sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
        sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
        sConfig.IC2Filter = 0;
        if (HAL_TIM_Encoder_Init(&timh[0], &sConfig) != HAL_OK)
            return -1;
        TIM_MasterConfigTypeDef sMasterConfig{0};
        sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
        sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
        sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
        if (HAL_TIMEx_MasterConfigSynchronization(&timh[0], &sMasterConfig) != HAL_OK)
            return -1;
        HAL_TIM_Encoder_Start(&timh[0], TIM_CHANNEL_ALL);
        return 0;
    }
    int init_tim5() {
        TIM_Encoder_InitTypeDef sConfig{0};
        timh[1].Instance = TIM5;
        timh[1].Init.Prescaler = 0;
        timh[1].Init.CounterMode = TIM_COUNTERMODE_UP;
        timh[1].Init.Period = 4294967295;
        timh[1].Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
        timh[1].Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
        sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
        sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
        sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
        sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
        sConfig.IC1Filter = 0;
        sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
        sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
        sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
        sConfig.IC2Filter = 0;
        if (HAL_TIM_Encoder_Init(&timh[1], &sConfig) != HAL_OK)
            return -1;
        TIM_MasterConfigTypeDef sMasterConfig{0};
        sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
        sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
        if (HAL_TIMEx_MasterConfigSynchronization(&timh[1], &sMasterConfig) != HAL_OK)
            return -1;
        return 0;
    }
    int init_tim8() {
        TIM_Encoder_InitTypeDef sConfig{0};
        timh[2].Instance = TIM8;
        timh[2].Init.Prescaler = 0;
        timh[2].Init.CounterMode = TIM_COUNTERMODE_UP;
        timh[2].Init.Period = 65535;
        timh[2].Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
        timh[2].Init.RepetitionCounter = 0;
        timh[2].Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
        sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
        sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
        sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
        sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
        sConfig.IC1Filter = 0;
        sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
        sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
        sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
        sConfig.IC2Filter = 0;
        if (HAL_TIM_Encoder_Init(&timh[2], &sConfig) != HAL_OK)
            return -1;
        TIM_MasterConfigTypeDef sMasterConfig{0};
        sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
        sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
        sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
        if (HAL_TIMEx_MasterConfigSynchronization(&timh[2], &sMasterConfig) != HAL_OK)
            return -1;
        return 0;
    }
    TIM_HandleTypeDef timh[3];
};

class actuator_controller_impl {
public:
    int init() {
        dev_pwm[0] = device_get_binding("PWM_2");
        dev_pwm[1] = device_get_binding("PWM_4");
        dev_pwm[2] = device_get_binding("PWM_9");
        for (auto i = 0; i < 3; ++i) {
            if (dev_pwm[i] == nullptr)
                return -1;
        }
        dev_dir = device_get_binding("GPIOF");
        if (dev_dir == nullptr)
            return -1;
        gpio_pin_configure(dev_dir, 3, GPIO_OUTPUT | GPIO_ACTIVE_HIGH);
        gpio_pin_configure(dev_dir, 4, GPIO_OUTPUT | GPIO_ACTIVE_HIGH);
        gpio_pin_configure(dev_dir, 5, GPIO_OUTPUT | GPIO_ACTIVE_HIGH);
        return helper.init();
    }
    void handle(const msg_ros2actuator *msg) {
        if (msg != nullptr) {
            if (msg->type == msg_ros2actuator::CWCCW)
                control_cwccw(msg->data);
            else if (msg->type == msg_ros2actuator::DUTY)
                control_duty(msg->data);
        }
    }
    void poll_encoder(int32_t data[3]) {
        int16_t d[3];
        helper.get_count(d);
        for (auto i = 0; i < 3; ++i)
            data[i] = d[i];
    }
    void poll_current(uint16_t data[3]) {
        for (auto i = 0; i < 3; ++i)
            data[i] = 0;
    }
private:
    void control_cwccw(const uint16_t data[3]) {
        gpio_pin_set(dev_dir, 3, data[0] == 0 ? 0 : 1);
        gpio_pin_set(dev_dir, 4, data[1] == 0 ? 0 : 1);
        gpio_pin_set(dev_dir, 5, data[2] == 0 ? 0 : 1);
    }
    void control_duty(const uint16_t data[3]) {
        for (auto i = 0; i < 3; ++i) {
            uint32_t pulse_ns = data[i] * CONTROL_PERIOD_NS / 65535;
            pwm_pin_set_nsec(dev_pwm[i], 1, CONTROL_PERIOD_NS, pulse_ns, PWM_POLARITY_NORMAL);
        }
    }
    timer_hal_helper helper;
    static constexpr uint32_t ACTUATOR_NUM{3};
    const device *dev_pwm[ACTUATOR_NUM], *dev_dir;
    static constexpr uint32_t CONTROL_HZ{5000};
    static constexpr uint32_t CONTROL_PERIOD_NS{1000000000ULL / CONTROL_HZ};
};

class actuator_controller {
public:
    int setup() {
        k_msgq_init(&msgq_actuator2ros, msgq_actuator2ros_buffer, sizeof (msg_actuator2ros), 10);
        k_msgq_init(&msgq_ros2actuator, msgq_ros2actuator_buffer, sizeof (msg_ros2actuator), 10);
        prev_cycle = k_cycle_get_32();
        return impl.init();
    }
    void loop() {
        msg_ros2actuator ros2actuator;
        if (k_msgq_get(&msgq_ros2actuator, &ros2actuator, K_NO_WAIT) == 0)
            impl.handle(&ros2actuator);
        uint32_t now_cycle = k_cycle_get_32();
        uint32_t dt_ms = k_cyc_to_ms_near32(now_cycle - prev_cycle);
        if (dt_ms > 100) {
            prev_cycle = now_cycle;
            msg_actuator2ros actuator2ros;
            impl.poll_encoder(actuator2ros.encoder_count);
            impl.poll_current(actuator2ros.current);
            while (k_msgq_put(&msgq_actuator2ros, &actuator2ros, K_NO_WAIT) != 0)
                k_msgq_purge(&msgq_actuator2ros);
        }
        k_msleep(10);
    }
private:
    actuator_controller_impl impl;
    uint32_t prev_cycle{0};
};

LEXX_THREAD_RUNNER(actuator_controller);

}

/* vim: set expandtab shiftwidth=4: */