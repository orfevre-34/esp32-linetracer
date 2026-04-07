#include "button.hpp"
#include "esp_log.h"

static const char* TAG = "Button";

Button::Button(const ButtonConfig& config)
    : config_(config)
{
}

Button::~Button()
{
}

esp_err_t Button::init()
{
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << config_.pin);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = config_.active_low ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = config_.active_low ? GPIO_PULLDOWN_DISABLE : GPIO_PULLDOWN_ENABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO %d: %s", config_.pin, esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize state
    last_state_ = read_raw();
    current_state_ = last_state_;
    
    ESP_LOGI(TAG, "Button initialized on GPIO %d", config_.pin);
    return ESP_OK;
}

bool Button::read_raw() const
{
    int level = gpio_get_level(config_.pin);
    return config_.active_low ? (level == 0) : (level == 1);
}

bool Button::is_pressed() const
{
    return pressed_;
}

void Button::update()
{
    bool raw_state = read_raw();
    int64_t now = esp_timer_get_time() / 1000;  // Convert to ms
    
    // Debounce
    if (raw_state != last_state_) {
        last_change_time_ = now;
        last_state_ = raw_state;
    }
    
    if ((now - last_change_time_) >= config_.debounce_ms) {
        if (raw_state != current_state_) {
            current_state_ = raw_state;
            
            if (current_state_) {
                // Button pressed
                pressed_ = true;
                press_start_time_ = now;
                long_press_fired_ = false;
                
                if (callback_) {
                    callback_(ButtonEvent::PRESSED);
                }
            } else {
                // Button released
                pressed_ = false;
                
                if (callback_) {
                    callback_(ButtonEvent::RELEASED);
                }
                
                // Check for click (only if long press wasn't fired)
                if (!long_press_fired_) {
                    int64_t press_duration = now - press_start_time_;
                    
                    if (press_duration < config_.long_press_ms) {
                        // Check for double click
                        if (pending_click_ && (now - last_click_time_) < config_.double_click_ms) {
                            // Double click detected
                            click_count_ = 0;
                            pending_click_ = false;
                            if (callback_) {
                                callback_(ButtonEvent::DOUBLE_CLICK);
                            }
                        } else {
                            // First click - wait for potential double click
                            click_count_++;
                            last_click_time_ = now;
                            pending_click_ = true;
                        }
                    }
                }
            }
        }
        
        // Check for long press while button is held
        if (current_state_ && !long_press_fired_) {
            int64_t press_duration = now - press_start_time_;
            if (press_duration >= config_.long_press_ms) {
                long_press_fired_ = true;
                if (callback_) {
                    callback_(ButtonEvent::LONG_PRESS);
                }
            }
        }
    }
    
    // Fire pending click after double-click window expires
    if (pending_click_ && (now - last_click_time_) >= config_.double_click_ms) {
        pending_click_ = false;
        click_count_ = 0;
        if (callback_) {
            callback_(ButtonEvent::CLICKED);
        }
    }
}

// ButtonManager implementation
ButtonManager::ButtonManager()
{
}

ButtonManager::~ButtonManager()
{
    stop();
}

bool ButtonManager::add_button(Button* button)
{
    if (button_count_ >= MAX_BUTTONS) {
        ESP_LOGE(TAG, "Cannot add more buttons, max is %d", MAX_BUTTONS);
        return false;
    }
    buttons_[button_count_++] = button;
    return true;
}

void ButtonManager::start(uint32_t poll_interval_ms)
{
    if (running_) return;
    
    poll_interval_ms_ = poll_interval_ms;
    running_ = true;
    
    xTaskCreate(poll_task, "btn_poll", 2048, this, 10, &task_handle_);
    ESP_LOGI(TAG, "Button manager started with %d buttons", button_count_);
}

void ButtonManager::stop()
{
    if (!running_) return;
    
    running_ = false;
    if (task_handle_) {
        vTaskDelete(task_handle_);
        task_handle_ = nullptr;
    }
}

void ButtonManager::poll_task(void* arg)
{
    ButtonManager* mgr = static_cast<ButtonManager*>(arg);
    
    while (mgr->running_) {
        for (int i = 0; i < mgr->button_count_; i++) {
            if (mgr->buttons_[i]) {
                mgr->buttons_[i]->update();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(mgr->poll_interval_ms_));
    }
    
    vTaskDelete(nullptr);
}
