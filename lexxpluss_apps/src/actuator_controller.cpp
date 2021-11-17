#include <device.h>
#include <drivers/gpio.h>
#include <drivers/pwm.h>
#include "adc_reader.hpp"
#include "actuator_controller.hpp"

k_msgq msgq_actuator2ros;
k_msgq msgq_ros2actuator;

extern "C" void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *htim_encoder)
{
    GPIO_InitTypeDef GPIO_InitStruct{0};
    if(htim_encoder->Instance==TIM1) {
        __HAL_RCC_TIM1_CLK_ENABLE();
        __HAL_RCC_GPIOE_CLK_ENABLE();
        GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_11;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
        HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
    } else if(htim_encoder->Instance==TIM5) {
        __HAL_RCC_TIM5_CLK_ENABLE();
        __HAL_RCC_GPIOH_CLK_ENABLE();
        GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
        HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
    } else if(htim_encoder->Instance==TIM8) {
        __HAL_RCC_TIM8_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();
        GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    }
}

namespace {

char __aligned(4) msgq_actuator2ros_buffer[8 * sizeof (msg_actuator2ros)];
char __aligned(4) msgq_ros2actuator_buffer[8 * sizeof (msg_ros2actuator)];

static constexpr uint32_t ACTUATOR_NUM{3};

class timer_hal_helper {
public:
    int init() {
        if (init_tim1() != 0 || init_tim5() != 0 || init_tim8() != 0)
            return -1;
        return 0;
    }
    void get_count(int16_t data[ACTUATOR_NUM]) const {
        data[0] = TIM1->CNT;
        TIM1->CNT = 0;
        data[1] = TIM8->CNT;
        TIM8->CNT = 0;
        data[2] = TIM5->CNT;
        TIM5->CNT = 0;
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
    TIM_HandleTypeDef timh[ACTUATOR_NUM];
};

static const struct {
    const char *name;
    uint32_t pin;
} config[ACTUATOR_NUM][2]{
    {{"PWM_12", 1}, {"PWM_2", 3}},
    {{"PWM_14", 1}, {"PWM_4", 1}},
    {{"PWM_3",  3}, {"PWM_9", 1}}
};

class actuator_controller_impl {
public:
    int init() {
        k_msgq_init(&msgq_actuator2ros, msgq_actuator2ros_buffer, sizeof (msg_actuator2ros), 8);
        k_msgq_init(&msgq_ros2actuator, msgq_ros2actuator_buffer, sizeof (msg_ros2actuator), 8);
        prev_cycle = k_cycle_get_32();
        for (uint32_t i{0}; i < ACTUATOR_NUM; ++i) {
            for (uint32_t j{0}; j < 2; ++j) {
                dev_pwm[i][j] = device_get_binding(config[i][j].name);
                if (dev_pwm[i][j] == nullptr)
                    return -1;
            }
        }
        dev_power = device_get_binding("GPIOB");
        dev_fail_01 = device_get_binding("GPIOF");
        dev_fail_2 = device_get_binding("GPIOJ");
        if (dev_power == nullptr || dev_fail_01 == nullptr || dev_fail_2 == nullptr)
            return -1;
        gpio_pin_configure(dev_power, 1, GPIO_OUTPUT_HIGH | GPIO_ACTIVE_HIGH);
        gpio_pin_configure(dev_fail_01, 11, GPIO_INPUT | GPIO_ACTIVE_HIGH);
        gpio_pin_configure(dev_fail_01, 12, GPIO_INPUT | GPIO_ACTIVE_HIGH);
        gpio_pin_configure(dev_fail_2, 4, GPIO_INPUT | GPIO_ACTIVE_HIGH);
        return helper.init();
    }
    void run() {
        for (uint32_t i{0}; i < ACTUATOR_NUM; ++i) {
            for (uint32_t j{0}; j < 2; ++j) {
                if (!device_is_ready(dev_pwm[i][j]))
                    return;
                pwm_pin_set_nsec(dev_pwm[i][j], config[i][j].pin, CONTROL_PERIOD_NS, CONTROL_PERIOD_NS, PWM_POLARITY_NORMAL);
            }
        }
        const device *gpiok = device_get_binding("GPIOK");
        if (gpiok != nullptr)
            gpio_pin_configure(gpiok, 4, GPIO_OUTPUT_LOW | GPIO_ACTIVE_HIGH);
        int heartbeat_led{1};
        while (true) {
            msg_ros2actuator message;
            if (k_msgq_get(&msgq_ros2actuator, &message, K_NO_WAIT) == 0)
                handle_control(message);
            read_fail();
            uint32_t now_cycle{k_cycle_get_32()};
            uint32_t dt_ms{k_cyc_to_ms_near32(now_cycle - prev_cycle)};
            if (dt_ms > 100) {
                prev_cycle = now_cycle;
                msg_actuator2ros actuator2ros;
                get_encoder(actuator2ros.encoder_count);
                get_current(actuator2ros.current);
                actuator2ros.connect = get_trolley();
                get_fail(actuator2ros.fail);
                while (k_msgq_put(&msgq_actuator2ros, &actuator2ros, K_NO_WAIT) != 0)
                    k_msgq_purge(&msgq_actuator2ros);
                if (gpiok != nullptr) {
                    gpio_pin_set(gpiok, 4, heartbeat_led);
                    heartbeat_led = !heartbeat_led;
                }
            }
            k_msleep(10);
        }
    }
private:
    void handle_control(const msg_ros2actuator *msg) const {
        for (uint32_t i{0}; i < ACTUATOR_NUM; ++i) {
            if (msg->actuators[i].direction == 0) {
                for (uint32_t j{0}; j < 2; ++j)
                    pwm_pin_set_nsec(dev_pwm[i][j], config[i][j].pin, CONTROL_PERIOD_NS, CONTROL_PERIOD_NS, PWM_POLARITY_NORMAL);
            } else {
                uint32_t pwm{100U - msg->actuators[i].power};
                uint32_t pulse_ns{pwm * CONTROL_PERIOD_NS / 100};
                if (msg->actuators[0].direction < 0) {
                    pwm_pin_set_nsec(dev_pwm[i][0], config[i][0].pin, CONTROL_PERIOD_NS, pulse_ns, PWM_POLARITY_NORMAL);
                    pwm_pin_set_nsec(dev_pwm[i][1], config[i][1].pin, CONTROL_PERIOD_NS, CONTROL_PERIOD_NS, PWM_POLARITY_NORMAL);
                } else {
                    pwm_pin_set_nsec(dev_pwm[i][0], config[i][0].pin, CONTROL_PERIOD_NS, CONTROL_PERIOD_NS, PWM_POLARITY_NORMAL);
                    pwm_pin_set_nsec(dev_pwm[i][1], config[i][1].pin, CONTROL_PERIOD_NS, pulse_ns, PWM_POLARITY_NORMAL);
                }
            }
        }
    }
    }
    void read_fail() {
        fail[0] = gpio_pin_get(dev_fail_01, 11) == 0;
        fail[1] = gpio_pin_get(dev_fail_01, 12) == 0;
        fail[2] = gpio_pin_get(dev_fail_2, 4) == 0;
    }
    void get_encoder(int32_t data[ACTUATOR_NUM]) const {
        int16_t d[ACTUATOR_NUM];
        helper.get_count(d);
        for (uint32_t i{0}; i < ACTUATOR_NUM; ++i)
            data[i] = d[i];
    }
    void get_current(int32_t data[ACTUATOR_NUM]) const {
        data[0] = adc_reader::get(adc_reader::INDEX_ACTUATOR_0);
        data[1] = adc_reader::get(adc_reader::INDEX_ACTUATOR_1);
        data[2] = adc_reader::get(adc_reader::INDEX_ACTUATOR_2);
    }
    int32_t get_trolley() const {
        return adc_reader::get(adc_reader::INDEX_TROLLEY);
    }
    void get_fail(bool fail[ACTUATOR_NUM]) const {
        for (uint32_t i{0}; i < ACTUATOR_NUM; ++i)
            fail[i] = this->fail[i];
    }
    timer_hal_helper helper;
    uint32_t prev_cycle{0};
    const device *dev_pwm[ACTUATOR_NUM][2]{{nullptr, nullptr}, {nullptr, nullptr}, {nullptr, nullptr}};
    const device *dev_power{nullptr}, *dev_fail_01{nullptr}, *dev_fail_2{nullptr};
    bool fail[3]{false, false, false};
    static constexpr uint32_t CONTROL_HZ{5000};
    static constexpr uint32_t CONTROL_PERIOD_NS{1000000000ULL / CONTROL_HZ};
} impl;

}

void actuator_controller::init()
{
    impl.init();
}

void actuator_controller::run(void *p1, void *p2, void *p3)
{
    impl.run();
}

k_thread actuator_controller::thread;

// vim: set expandtab shiftwidth=4:
