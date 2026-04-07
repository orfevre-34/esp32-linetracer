#pragma once

#include <cstdint>
#include <functional>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"

// Button event types
enum class ButtonEvent {
    PRESSED,
    RELEASED,
    CLICKED,
    LONG_PRESS,
    DOUBLE_CLICK
};

// Button configuration
struct ButtonConfig {
    gpio_num_t pin;
    bool active_low = true;          // true: pressed = LOW, false: pressed = HIGH
    uint32_t debounce_ms = 20;       // Debounce time
    uint32_t long_press_ms = 800;    // Long press threshold
    uint32_t double_click_ms = 100;  // Double click window
};

class Button {
public:
    using EventCallback = std::function<void(ButtonEvent)>;
    
    Button(const ButtonConfig& config);
    ~Button();
    
    // Initialize the button
    esp_err_t init();
    
    // Set event callback
    void set_callback(EventCallback callback) { callback_ = callback; }
    
    // Check if button is currently pressed
    bool is_pressed() const;
    
    // Get raw state (for polling mode)
    bool read_raw() const;
    
    // Process button state (call this in a loop for polling mode)
    void update();
    
    // Get button pin
    gpio_num_t get_pin() const { return config_.pin; }
    
private:
    ButtonConfig config_;
    EventCallback callback_;
    
    bool last_state_ = false;
    bool current_state_ = false;
    bool pressed_ = false;
    
    int64_t last_change_time_ = 0;
    int64_t press_start_time_ = 0;
    int64_t last_click_time_ = 0;
    
    bool long_press_fired_ = false;
    int click_count_ = 0;
    bool pending_click_ = false;  // Wait for double-click window before firing CLICKED
};

// Button manager for handling multiple buttons with interrupt support
class ButtonManager {
public:
    static constexpr int MAX_BUTTONS = 4;
    
    ButtonManager();
    ~ButtonManager();
    
    // Add a button to manage
    bool add_button(Button* button);
    
    // Start the button polling task
    void start(uint32_t poll_interval_ms = 10);
    
    // Stop the polling task
    void stop();
    
private:
    static void poll_task(void* arg);
    
    Button* buttons_[MAX_BUTTONS] = {nullptr};
    int button_count_ = 0;
    TaskHandle_t task_handle_ = nullptr;
    bool running_ = false;
    uint32_t poll_interval_ms_ = 10;
};
