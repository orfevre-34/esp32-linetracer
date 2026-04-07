#include "motor_driver.hpp"
#include "esp_log.h"
#include <algorithm>

static const char* TAG = "MotorDriver";

// PWM configuration
static constexpr int PWM_FREQ = 500;      // 500Hz
static constexpr ledc_timer_bit_t PWM_RESOLUTION = LEDC_TIMER_10_BIT;  // 0-1023
static constexpr int PWM_MAX = (1 << PWM_RESOLUTION) - 1;  // 1023

Motor::Motor(gpio_num_t pin_a, gpio_num_t pin_b,
             ledc_channel_t ch_a, ledc_channel_t ch_b,
             ledc_timer_t timer)
    : pin_a_(pin_a), pin_b_(pin_b), ch_a_(ch_a), ch_b_(ch_b), timer_(timer)
{
}

esp_err_t Motor::init()
{
    // Configure LEDC timer
    ledc_timer_config_t timer_conf = {};
    timer_conf.speed_mode = LEDC_LOW_SPEED_MODE;
    timer_conf.duty_resolution = PWM_RESOLUTION;
    timer_conf.timer_num = timer_;
    timer_conf.freq_hz = PWM_FREQ;
    timer_conf.clk_cfg = LEDC_AUTO_CLK;
    
    esp_err_t ret = ledc_timer_config(&timer_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LEDC timer: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Configure channel A
    ledc_channel_config_t ch_conf_a = {};
    ch_conf_a.speed_mode = LEDC_LOW_SPEED_MODE;
    ch_conf_a.channel = ch_a_;
    ch_conf_a.timer_sel = timer_;
    ch_conf_a.intr_type = LEDC_INTR_DISABLE;
    ch_conf_a.gpio_num = pin_a_;
    ch_conf_a.duty = 0;
    ch_conf_a.hpoint = 0;
    
    ret = ledc_channel_config(&ch_conf_a);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LEDC channel A: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Configure channel B
    ledc_channel_config_t ch_conf_b = {};
    ch_conf_b.speed_mode = LEDC_LOW_SPEED_MODE;
    ch_conf_b.channel = ch_b_;
    ch_conf_b.timer_sel = timer_;
    ch_conf_b.intr_type = LEDC_INTR_DISABLE;
    ch_conf_b.gpio_num = pin_b_;
    ch_conf_b.duty = 0;
    ch_conf_b.hpoint = 0;
    
    ret = ledc_channel_config(&ch_conf_b);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LEDC channel B: %s", esp_err_to_name(ret));
        return ret;
    }
    
    initialized_ = true;
    ESP_LOGI(TAG, "Motor initialized (pins %d/%d)", pin_a_, pin_b_);
    return ESP_OK;
}

void Motor::set_pwm(ledc_channel_t ch, int duty)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
}

void Motor::set(MotorDirection dir, int speed)
{
    if (!initialized_) return;
    
    speed = std::max(0, std::min(100, speed));
    int duty = (speed * PWM_MAX) / 100;
    
    current_speed_ = speed;
    current_dir_ = dir;
    
    switch (dir) {
        case MotorDirection::FORWARD:
            set_pwm(ch_a_, duty);
            set_pwm(ch_b_, 0);
            break;
            
        case MotorDirection::BACKWARD:
            set_pwm(ch_a_, 0);
            set_pwm(ch_b_, duty);
            break;
            
        case MotorDirection::BRAKE:
            // Both HIGH = brake (for some drivers)
            set_pwm(ch_a_, PWM_MAX);
            set_pwm(ch_b_, PWM_MAX);
            break;
            
        case MotorDirection::STOP:
        default:
            set_pwm(ch_a_, 0);
            set_pwm(ch_b_, 0);
            break;
    }
}

// DifferentialDrive implementation
DifferentialDrive::DifferentialDrive(Motor& left, Motor& right)
    : left_(left), right_(right)
{
}

void DifferentialDrive::forward(int speed)
{
    left_.forward(speed);
    right_.forward(speed);
}

void DifferentialDrive::backward(int speed)
{
    left_.backward(speed);
    right_.backward(speed);
}

void DifferentialDrive::stop()
{
    left_.stop();
    right_.stop();
}

void DifferentialDrive::brake()
{
    left_.brake();
    right_.brake();
}

void DifferentialDrive::turn_left(int speed)
{
    left_.backward(speed);
    right_.forward(speed);
}

void DifferentialDrive::turn_right(int speed)
{
    left_.forward(speed);
    right_.backward(speed);
}

void DifferentialDrive::drive(int speed, int steering)
{
    speed = std::max(0, std::min(100, speed));
    steering = std::max(-100, std::min(100, steering));
    
    int left_speed = speed;
    int right_speed = speed;
    
    if (steering > 0) {
        // Turn right: reduce right motor
        right_speed = speed * (100 - steering) / 100;
    } else if (steering < 0) {
        // Turn left: reduce left motor
        left_speed = speed * (100 + steering) / 100;
    }
    
    left_.forward(left_speed);
    right_.forward(right_speed);
}

void DifferentialDrive::set_motors(int left_speed, int right_speed)
{
    if (left_speed >= 0) {
        left_.forward(left_speed);
    } else {
        left_.backward(-left_speed);
    }
    
    if (right_speed >= 0) {
        right_.forward(right_speed);
    } else {
        right_.backward(-right_speed);
    }
}
