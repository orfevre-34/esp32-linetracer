#pragma once

#include <cstdint>
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_err.h"

// Motor direction
enum class MotorDirection {
    STOP,
    FORWARD,
    BACKWARD,
    BRAKE
};

// Single motor control (2 GPIO pins for H-bridge)
class Motor {
public:
    Motor(gpio_num_t pin_a, gpio_num_t pin_b, 
          ledc_channel_t ch_a, ledc_channel_t ch_b,
          ledc_timer_t timer = LEDC_TIMER_0);
    
    esp_err_t init();
    
    // Set motor speed and direction
    // speed: 0-100 (percentage)
    void set(MotorDirection dir, int speed);
    
    // Convenience methods
    void forward(int speed) { set(MotorDirection::FORWARD, speed); }
    void backward(int speed) { set(MotorDirection::BACKWARD, speed); }
    void stop() { set(MotorDirection::STOP, 0); }
    void brake() { set(MotorDirection::BRAKE, 0); }
    
    int get_speed() const { return current_speed_; }
    MotorDirection get_direction() const { return current_dir_; }

private:
    gpio_num_t pin_a_, pin_b_;
    ledc_channel_t ch_a_, ch_b_;
    ledc_timer_t timer_;
    int current_speed_ = 0;
    MotorDirection current_dir_ = MotorDirection::STOP;
    bool initialized_ = false;
    
    void set_pwm(ledc_channel_t ch, int duty);
};

// Differential drive controller (2 motors)
class DifferentialDrive {
public:
    DifferentialDrive(Motor& left, Motor& right);
    
    // Basic movement
    void forward(int speed);
    void backward(int speed);
    void stop();
    void brake();
    
    // Turning
    void turn_left(int speed);   // Pivot turn (right forward, left backward)
    void turn_right(int speed);  // Pivot turn (left forward, right backward)
    
    // Differential steering (-100 to +100 for steering)
    // speed: forward speed (0-100)
    // steering: -100 (full left) to +100 (full right)
    void drive(int speed, int steering);
    
    // Direct motor control
    void set_motors(int left_speed, int right_speed);
    
    Motor& left() { return left_; }
    Motor& right() { return right_; }

private:
    Motor& left_;
    Motor& right_;
};
