
#include "motorController.h"
#include <driver/ledc.h>


MotorController::MotorController(gpio_num_t motorA1_, gpio_num_t motorA2_, gpio_num_t motorAPwm_, 
                                 gpio_num_t motorB1_, gpio_num_t motorB2_, gpio_num_t motorBPwm_ )
{
    motorA1 = motorA1_;
    motorA2 = motorA2_;
    motorAPwm = motorAPwm_;
    motorB1 = motorB1_;
    motorB2 = motorB2_;
    motorBPwm = motorBPwm_;
}

void MotorController::Init()
{
    gpio_config_t io_conf = {};

    // Configure pins for MotorA
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << motorA1) | (1ULL << motorA2) | (1ULL << motorAPwm);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    // Configure pins for MotorB
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << motorB1) | (1ULL << motorB2) | (1ULL << motorBPwm);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    gpio_set_level(motorA1, 0);
    gpio_set_level(motorA2, 0);
    //gpio_set_level(motorAPwm, 1);
    gpio_set_level(motorB1, 0);
    gpio_set_level(motorB2, 0);
    //gpio_set_level(motorBPwm, 1);

    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = LEDC_TIMER_10_BIT ,
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = 10000,  // Set output frequency at 10 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .gpio_num       = motorAPwm,
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER_0,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    ledc_channel.gpio_num = motorBPwm;
    ledc_channel.channel = LEDC_CHANNEL_1;
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));



    // Set duty to 50%
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 1024));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 1024));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1));

}


void MotorController::setDirection(DIRECTION dir)
{
    switch(dir)
    {
        case DIRECTION::STOP: 
            gpio_set_level(motorA1, 1);
            gpio_set_level(motorA2, 1);
            gpio_set_level(motorB1, 1);
            gpio_set_level(motorB2, 1);
            break;
        case DIRECTION::BACK: 
            gpio_set_level(motorA1, 1);
            gpio_set_level(motorA2, 0);
            gpio_set_level(motorB1, 1);
            gpio_set_level(motorB2, 0);
            break;
        case DIRECTION::RIGHT: 
            gpio_set_level(motorA1, 0);
            gpio_set_level(motorA2, 1);
            gpio_set_level(motorB1, 1);
            gpio_set_level(motorB2, 0);
            break;
        case DIRECTION::LEFT: 
            gpio_set_level(motorA1, 1);
            gpio_set_level(motorA2, 0);
            gpio_set_level(motorB1, 0);
            gpio_set_level(motorB2, 1);
            break;
        case DIRECTION::FWD: 
            gpio_set_level(motorA1, 0);
            gpio_set_level(motorA2, 1);
            gpio_set_level(motorB1, 0);
            gpio_set_level(motorB2, 1);
        break;
    }
}