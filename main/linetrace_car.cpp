/*
   ライントレースカー 完全実装
   - 6チャンネルセンサー
   - 2モーター差動駆動
   - OLED表示（待機/動作モード切替）
   - ボタン制御
*/

// ===== Compile Switches (from Kconfig) =====
// ENABLE_STRAIGHT_BOOST is now CONFIG_LINETRACE_ENABLE_STRAIGHT_BOOST

#include <algorithm>
#include <cstdio>
#include <cstring>

#include "adc_reader.hpp"
#include "button.hpp"
#include "buzzer.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "motor_driver.hpp"
#include "nvs.h"
#include "nvs_flash.h"
#include "pl9823.hpp"
#include "ssd1306.hpp"

static const char* TAG = "linetrace_car";

// ===== Pin Configuration (from Kconfig) =====
// OLED
#define I2C_SDA_PIN ((gpio_num_t)CONFIG_LINETRACE_I2C_SDA_PIN)
#define I2C_SCL_PIN ((gpio_num_t)CONFIG_LINETRACE_I2C_SCL_PIN)

// Buttons
#define BUTTON_A_PIN ((gpio_num_t)CONFIG_LINETRACE_BUTTON_A_PIN)
#define BUTTON_B_PIN ((gpio_num_t)CONFIG_LINETRACE_BUTTON_B_PIN)

// Motors (H-bridge)
#define MOTOR_L_A_PIN ((gpio_num_t)CONFIG_LINETRACE_MOTOR_L_A_PIN)
#define MOTOR_L_B_PIN ((gpio_num_t)CONFIG_LINETRACE_MOTOR_L_B_PIN)
#define MOTOR_R_A_PIN ((gpio_num_t)CONFIG_LINETRACE_MOTOR_R_A_PIN)
#define MOTOR_R_B_PIN ((gpio_num_t)CONFIG_LINETRACE_MOTOR_R_B_PIN)

// Buzzer
#define BUZZER_PIN ((gpio_num_t)CONFIG_LINETRACE_BUZZER_PIN)

// RGB LED (PL9823)
#define LED_PIN ((gpio_num_t)CONFIG_LINETRACE_LED_PIN)
#define LED_COUNT CONFIG_LINETRACE_LED_COUNT

// ===== Sensor Configuration =====
static constexpr adc_channel_t SENSOR_CHANNELS[] = {
    ADC_CHANNEL_0,  // GPIO36 - S0 (leftmost)
    ADC_CHANNEL_3,  // GPIO39 - S1
    ADC_CHANNEL_6,  // GPIO32 - S2
    ADC_CHANNEL_7,  // GPIO33 - S3
    ADC_CHANNEL_4,  // GPIO34 - S4
    ADC_CHANNEL_5,  // GPIO35 - S5 (rightmost)
};
static constexpr int NUM_SENSORS = 6;

// ===== Operating Modes =====
enum class CarMode {
  STANDBY,      // Waiting, motors off
  RUNNING,      // Line tracing active
  CALIBRATION,  // Sensor calibration mode
  SETTINGS      // Parameter adjustment
};

// ===== Line Trace Parameters (defaults from Kconfig) =====
struct LineTraceParams {
  int base_speed = CONFIG_LINETRACE_BASE_SPEED;
  int max_speed = CONFIG_LINETRACE_MAX_SPEED;
  int threshold = CONFIG_LINETRACE_THRESHOLD;
  float kp = CONFIG_LINETRACE_KP / 100.0f;
  float kd = CONFIG_LINETRACE_KD / 100.0f;
  int sharp_turn_speed = CONFIG_LINETRACE_SHARP_TURN_SPEED;
  int sharp_turn_outer = CONFIG_LINETRACE_SHARP_TURN_OUTER;
  int ramp_rate = CONFIG_LINETRACE_RAMP_RATE;
  float sharp_turn_pd_gain = CONFIG_LINETRACE_SHARP_TURN_PD_GAIN / 100.0f;
  int straight_boost_speed = CONFIG_LINETRACE_STRAIGHT_BOOST_SPEED;
  float straight_error_threshold = CONFIG_LINETRACE_STRAIGHT_ERROR_THRESHOLD / 100.0f;
};

// ===== Global State =====
static SSD1306* g_oled = nullptr;
static DifferentialDrive* g_drive = nullptr;
static Buzzer* g_buzzer = nullptr;
static PL9823* g_led = nullptr;
static volatile CarMode g_mode = CarMode::STANDBY;
static LineTraceParams g_params;

// Sensor data
static int g_sensor_values[NUM_SENSORS] = {0};
static int g_sensor_min[NUM_SENSORS] = {3300, 3300, 3300, 3300, 3300, 3300};
static int g_sensor_max[NUM_SENSORS] = {0, 0, 0, 0, 0, 0};

// NVS namespace for calibration data
static const char* NVS_NAMESPACE = "linetrace";
static const char* NVS_KEY_CALIB_MIN = "calib_min";
static const char* NVS_KEY_CALIB_MAX = "calib_max";

// Save calibration values to NVS
static esp_err_t save_calibration_to_nvs() {
  nvs_handle_t handle;
  esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(err));
    return err;
  }

  // Save min values as blob
  err = nvs_set_blob(handle, NVS_KEY_CALIB_MIN, g_sensor_min,
                     sizeof(g_sensor_min));
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to save calib_min: %s", esp_err_to_name(err));
    nvs_close(handle);
    return err;
  }

  // Save max values as blob
  err = nvs_set_blob(handle, NVS_KEY_CALIB_MAX, g_sensor_max,
                     sizeof(g_sensor_max));
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to save calib_max: %s", esp_err_to_name(err));
    nvs_close(handle);
    return err;
  }

  err = nvs_commit(handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to commit NVS: %s", esp_err_to_name(err));
  } else {
    ESP_LOGI(TAG, "Calibration saved to NVS");
  }

  nvs_close(handle);
  return err;
}

// Load calibration values from NVS
static esp_err_t load_calibration_from_nvs() {
  nvs_handle_t handle;
  esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "No calibration data in NVS (first boot?)");
    return err;
  }

  size_t required_size = sizeof(g_sensor_min);

  // Load min values
  err = nvs_get_blob(handle, NVS_KEY_CALIB_MIN, g_sensor_min, &required_size);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Failed to load calib_min: %s", esp_err_to_name(err));
    nvs_close(handle);
    return err;
  }

  // Load max values
  required_size = sizeof(g_sensor_max);
  err = nvs_get_blob(handle, NVS_KEY_CALIB_MAX, g_sensor_max, &required_size);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Failed to load calib_max: %s", esp_err_to_name(err));
    nvs_close(handle);
    return err;
  }

  ESP_LOGI(TAG, "Calibration loaded from NVS");
  nvs_close(handle);
  return ESP_OK;
}

// Line tracking state
static float g_last_error = 0;
static int g_current_left_speed = 0;   // Current left motor speed (for ramp)
static int g_current_right_speed = 0;  // Current right motor speed (for ramp)
static int g_line_position = 0;  // -250 to +250
static int g_detected_count = 0;

// Turn detection with hysteresis
enum class TurnState {
  STRAIGHT,
  SHARP_LEFT,
  SHARP_RIGHT,
  CROSSING  // Intersection detected
};
static TurnState g_turn_state = TurnState::STRAIGHT;
static int g_turn_hold_count = 0;  // Hysteresis counter

// Post-sharp-turn stabilization - suppress opposite direction turns
static int g_post_turn_stabilize_count =
    0;  // Cycles remaining for stabilization
static constexpr int POST_TURN_STABILIZE_CYCLES = CONFIG_LINETRACE_POST_TURN_STABILIZE_CYCLES;
static TurnState g_last_sharp_turn =
    TurnState::STRAIGHT;  // Remember last sharp turn direction

#ifdef CONFIG_LINETRACE_ENABLE_STRAIGHT_BOOST
// Straight acceleration mode variables
static int g_straight_count = 0;  // Consecutive cycles of straight detection
static const int STRAIGHT_BOOST_THRESHOLD = CONFIG_LINETRACE_STRAIGHT_BOOST_THRESHOLD;
static const int STRAIGHT_BOOST_MAX_COUNT = CONFIG_LINETRACE_STRAIGHT_BOOST_MAX_COUNT;
static bool g_straight_boost_active =
    false;  // Whether boost is currently active
#endif

// Settings adjustment
static int g_settings_index = 0;
static constexpr int NUM_SETTINGS = 4;

// Buzzer state
static bool g_buzzer_muted = false;

// ===== Buzzer Helper Functions =====
static void buzzer_beep_start() {
  if (g_buzzer_muted || !g_buzzer) return;
  g_buzzer->playTone(Buzzer::NOTE_C5, 80);
  g_buzzer->playTone(Buzzer::NOTE_E5, 80);
  g_buzzer->playTone(Buzzer::NOTE_G5, 120);
}

static void buzzer_beep_stop() {
  if (g_buzzer_muted || !g_buzzer) return;
  g_buzzer->playTone(Buzzer::NOTE_G5, 80);
  g_buzzer->playTone(Buzzer::NOTE_E5, 80);
  g_buzzer->playTone(Buzzer::NOTE_C5, 120);
}

static void buzzer_beep_click() {
  if (g_buzzer_muted || !g_buzzer) return;
  g_buzzer->beep(30);
}

static void buzzer_beep_error() {
  if (g_buzzer_muted || !g_buzzer) return;
  g_buzzer->playTone(200, 200);
}

static void buzzer_beep_mute_toggle() {
  if (!g_buzzer) return;
  if (g_buzzer_muted) {
    g_buzzer->playTone(400, 50);
  } else {
    g_buzzer->playTone(800, 50);
    vTaskDelay(pdMS_TO_TICKS(30));
    g_buzzer->playTone(1200, 50);
  }
}

// ===== Button Callbacks =====
void on_button_a(ButtonEvent event) {
  if (event == ButtonEvent::CLICKED) {
    switch (g_mode) {
      case CarMode::STANDBY:
        // Start running
        buzzer_beep_start();
        g_mode = CarMode::RUNNING;
        ESP_LOGI(TAG, "Mode: RUNNING");
        break;

      case CarMode::RUNNING:
        // Stop
        g_mode = CarMode::STANDBY;
        g_drive->stop();
        buzzer_beep_stop();
        ESP_LOGI(TAG, "Mode: STANDBY");
        break;

      case CarMode::SETTINGS:
        // Adjust value up
        buzzer_beep_click();
        switch (g_settings_index) {
          case 0:
            g_params.base_speed = std::min(100, g_params.base_speed + 5);
            break;
          case 1:
            g_params.max_speed = std::min(100, g_params.max_speed + 5);
            break;
          case 2:
            g_params.threshold = std::min(3300, g_params.threshold + 100);
            break;
          case 3:
            g_params.kp = std::min(2.0f, g_params.kp + 0.1f);
            break;
        }
        break;

      case CarMode::CALIBRATION:
        // Reset calibration
        buzzer_beep_click();
        for (int i = 0; i < NUM_SENSORS; i++) {
          g_sensor_min[i] = 3300;
          g_sensor_max[i] = 0;
        }
        break;
    }
  } else if (event == ButtonEvent::LONG_PRESS) {
    if (g_mode == CarMode::RUNNING) {
      g_mode = CarMode::STANDBY;
      g_drive->stop();
      buzzer_beep_stop();
    }
    // Enter settings mode
    buzzer_beep_click();
    g_mode = CarMode::SETTINGS;
    g_settings_index = 0;
    ESP_LOGI(TAG, "Mode: SETTINGS");
  }
}

void on_button_b(ButtonEvent event) {
  if (event == ButtonEvent::CLICKED) {
    switch (g_mode) {
      case CarMode::STANDBY:
        // Enter calibration mode
        buzzer_beep_click();
        g_mode = CarMode::CALIBRATION;
        ESP_LOGI(TAG, "Mode: CALIBRATION");
        break;

      case CarMode::RUNNING:
        // Emergency stop
        g_mode = CarMode::STANDBY;
        g_drive->brake();
        buzzer_beep_error();
        ESP_LOGI(TAG, "Emergency Stop!");
        break;

      case CarMode::SETTINGS:
        // Adjust value down or next setting
        buzzer_beep_click();
        switch (g_settings_index) {
          case 0:
            g_params.base_speed = std::max(10, g_params.base_speed - 5);
            break;
          case 1:
            g_params.max_speed = std::max(20, g_params.max_speed - 5);
            break;
          case 2:
            g_params.threshold = std::max(100, g_params.threshold - 100);
            break;
          case 3:
            g_params.kp = std::max(0.1f, g_params.kp - 0.1f);
            break;
        }
        break;

      case CarMode::CALIBRATION:
        // Exit calibration and save values to NVS
        buzzer_beep_click();
        save_calibration_to_nvs();
        g_mode = CarMode::STANDBY;
        ESP_LOGI(TAG, "Mode: STANDBY (calibration saved)");
        break;
    }
  } else if (event == ButtonEvent::DOUBLE_CLICK) {
    // Toggle mute (works in any mode)
    buzzer_beep_mute_toggle();  // Play before toggling
    g_buzzer_muted = !g_buzzer_muted;
    ESP_LOGI(TAG, "Buzzer %s", g_buzzer_muted ? "MUTED" : "UNMUTED");
  } else if (event == ButtonEvent::LONG_PRESS) {
    if (g_mode == CarMode::SETTINGS) {
      // Next setting or exit
      buzzer_beep_click();
      g_settings_index++;
      if (g_settings_index >= NUM_SETTINGS) {
        g_mode = CarMode::STANDBY;
        ESP_LOGI(TAG, "Mode: STANDBY");
      }
    } else if (g_mode != CarMode::RUNNING) {
      buzzer_beep_click();
      g_mode = CarMode::STANDBY;
      ESP_LOGI(TAG, "Mode: STANDBY");
    }
  }
}

// ===== Display Functions =====
void draw_standby() {
  g_oled->clear();

  // Title
  g_oled->draw_rect(0, 0, 128, 12);
  g_oled->draw_string(25, 2, "LINE TRACER");

  // Sensor visualization (compact)
  g_oled->draw_string(5, 15, "Sensors:");
  for (int i = 0; i < NUM_SENSORS; i++) {
    int x = 55 + i * 11;
    if (g_sensor_values[i] >= g_params.threshold) {
      g_oled->fill_rect(x, 14, 9, 9);
    } else {
      g_oled->draw_rect(x, 14, 9, 9);
    }
  }

  // Status
  g_oled->draw_string(5, 28, "Status: STANDBY");

  // Parameters
  char buf[30];
  snprintf(buf, sizeof(buf), "Speed:%d TH:%dmV", g_params.base_speed,
           g_params.threshold);
  g_oled->draw_string(5, 40, buf);

  // Instructions
  g_oled->draw_line(0, 52, 127, 52);
  g_oled->draw_string(5, 55, "A:Start B:Calib");

  // Mute indicator
  if (g_buzzer_muted) {
    g_oled->draw_string(110, 55, "M");
  }

  g_oled->update();
}

void draw_running() {
  g_oled->clear();

  // Title bar with mode indicator
  g_oled->fill_rect(0, 0, 128, 11);

  // Show stabilization period with indicator showing which direction is
  // suppressed
  if (g_post_turn_stabilize_count > 0) {
    if (g_last_sharp_turn == TurnState::SHARP_RIGHT) {
      g_oled->draw_string(15, 2, "NO LEFT >>>",
                          false);  // Was right, suppress left
    } else {
      g_oled->draw_string(15, 2, "<<< NO RIGHT",
                          false);  // Was left, suppress right
    }
  }
#ifdef CONFIG_LINETRACE_ENABLE_STRAIGHT_BOOST
  else if (g_straight_boost_active) {
    // Show straight boost mode
    g_oled->draw_string(25, 2, ">>> BOOST <<<", false);
  }
#endif
  else {
    switch (g_turn_state) {
      case TurnState::SHARP_LEFT:
        g_oled->draw_string(15, 2, "<<< SHARP L", false);
        break;
      case TurnState::SHARP_RIGHT:
        g_oled->draw_string(15, 2, "SHARP R >>>", false);
        break;
      case TurnState::CROSSING:
        g_oled->draw_string(25, 2, "++ CROSS ++", false);
        break;
      default:
        g_oled->draw_string(30, 2, "RUNNING", false);
        break;
    }
  }

  // Large sensor bar visualization
  const int bar_y = 15;
  const int bar_h = 25;

  for (int i = 0; i < NUM_SENSORS; i++) {
    int x = 5 + i * 20;

    // Normalize sensor value
    int normalized = 0;
    if (g_sensor_max[i] > g_sensor_min[i]) {
      normalized = ((g_sensor_values[i] - g_sensor_min[i]) * 100) /
                   (g_sensor_max[i] - g_sensor_min[i]);
    } else {
      normalized = (g_sensor_values[i] * 100) / 3300;
    }
    normalized = std::max(0, std::min(100, normalized));

    // Draw bar
    int fill_h = (normalized * (bar_h - 2)) / 100;
    g_oled->draw_rect(x, bar_y, 18, bar_h);
    if (fill_h > 0) {
      g_oled->fill_rect(x + 1, bar_y + bar_h - fill_h - 1, 16, fill_h);
    }

    // Detection indicator
    if (g_sensor_values[i] >= g_params.threshold) {
      g_oled->fill_circle(x + 9, bar_y + bar_h + 5, 3);
    }
  }

  // Line position indicator
  g_oled->draw_line(0, 48, 127, 48);
  int pos_x = 64 + (g_line_position * 50) / 250;
  pos_x = std::max(5, std::min(122, pos_x));
  g_oled->fill_rect(pos_x - 4, 49, 8, 4);

  // Motor speeds
  char buf[30];
  snprintf(buf, sizeof(buf), "L:%d R:%d", g_drive->left().get_speed(),
           g_drive->right().get_speed());
  g_oled->draw_string(35, 55, buf);

  g_oled->update();
}

void draw_calibration() {
  g_oled->clear();

  g_oled->draw_string(30, 0, "CALIBRATE");

  // Vertical bar visualization for each sensor
  const int bar_w = 14;
  const int bar_h = 35;
  const int bar_y = 12;

  for (int i = 0; i < NUM_SENSORS; i++) {
    int x = 8 + i * 20;

    // Sensor number
    char num[2] = {(char)('0' + i), '\0'};
    g_oled->draw_char(x + 4, bar_y - 1, num[0]);

    // Bar outline (represents 0-3300mV range)
    g_oled->draw_rect(x, bar_y + 8, bar_w, bar_h);

    // Current value bar
    int val_h = (g_sensor_values[i] * (bar_h - 2)) / 3300;
    if (val_h > 0) {
      g_oled->fill_rect(x + 1, bar_y + 8 + bar_h - val_h - 1, bar_w - 2, val_h);
    }

    // Min marker: > shape on left side
    int min_y = bar_y + 8 + bar_h - 1 - (g_sensor_min[i] * (bar_h - 2)) / 3300;
    g_oled->set_pixel(x - 4, min_y - 1);
    g_oled->set_pixel(x - 3, min_y);
    g_oled->set_pixel(x - 4, min_y + 1);

    // Max marker (solid line across bar)
    int max_y = bar_y + 8 + bar_h - 1 - (g_sensor_max[i] * (bar_h - 2)) / 3300;
    g_oled->draw_line(x, max_y, x + bar_w - 1, max_y);
  }

  // Footer with instructions
  g_oled->draw_string(0, 56, "A:Reset B:Done");

  g_oled->update();
}

void draw_settings() {
  g_oled->clear();

  g_oled->draw_string(30, 0, "SETTINGS");
  g_oled->draw_line(0, 10, 127, 10);

  const char* labels[] = {"Base Speed", "Max Speed", "Threshold", "Kp Gain"};

  for (int i = 0; i < NUM_SETTINGS; i++) {
    int y = 12 + i * 10;

    // Highlight current selection
    if (i == g_settings_index) {
      g_oled->fill_rect(0, y - 1, 128, 10);
      g_oled->draw_string(2, y, ">", false);
    }

    char buf[25];
    switch (i) {
      case 0:
        snprintf(buf, sizeof(buf), "%s: %d", labels[i], g_params.base_speed);
        break;
      case 1:
        snprintf(buf, sizeof(buf), "%s: %d", labels[i], g_params.max_speed);
        break;
      case 2:
        snprintf(buf, sizeof(buf), "%s: %d", labels[i], g_params.threshold);
        break;
      case 3:
        snprintf(buf, sizeof(buf), "%s: %.1f", labels[i], g_params.kp);
        break;
    }
    g_oled->draw_string(12, y, buf, i != g_settings_index);
  }

  g_oled->draw_line(0, 53, 127, 53);
  g_oled->draw_string(0, 55, "A:+ B:- LongB:Next");

  g_oled->update();
}

// ===== LED Status Indicator =====
void update_led_status() {
  if (!g_led) return;

  switch (g_mode) {
    case CarMode::STANDBY:
      // 青: 待機中
      g_led->setPixel(0, 0, 0, 64);
      break;
    case CarMode::RUNNING:
      // 走行状態に応じた色
#ifdef CONFIG_LINETRACE_ENABLE_STRAIGHT_BOOST
      if (g_straight_boost_active) {
        // 水色: 直線加速中
        g_led->setPixel(0, 0, 128, 128);
      } else
#endif
          if (g_turn_state == TurnState::SHARP_LEFT) {
        // オレンジ: 左急カーブ
        g_led->setPixel(0, 128, 64, 0);
      } else if (g_turn_state == TurnState::SHARP_RIGHT) {
        // 黄色: 右急カーブ
        g_led->setPixel(0, 128, 128, 0);
      } else if (g_turn_state == TurnState::CROSSING) {
        // 白: 交差点
        g_led->setPixel(0, 64, 64, 64);
      } else if (g_post_turn_stabilize_count > 0) {
        // 紫: スタビライズ中
        g_led->setPixel(0, 64, 0, 64);
      } else {
        // 緑: 通常走行
        g_led->setPixel(0, 0, 64, 0);
      }
      break;
    case CarMode::CALIBRATION:
      // 黄色: キャリブレーション
      g_led->setPixel(0, 64, 64, 0);
      break;
    case CarMode::SETTINGS:
      // マゼンタ: 設定モード
      g_led->setPixel(0, 64, 0, 64);
      break;
  }
  g_led->refresh();
}

void update_display() {
  // LED色を更新
  update_led_status();

  switch (g_mode) {
    case CarMode::STANDBY:
      draw_standby();
      break;
    case CarMode::RUNNING:
      draw_running();
      break;
    case CarMode::CALIBRATION:
      draw_calibration();
      break;
    case CarMode::SETTINGS:
      draw_settings();
      break;
  }
}

// ===== Line Trace Algorithm =====
void calculate_line_position() {
  // Decrement post-turn stabilization counter at start of each cycle
  if (g_post_turn_stabilize_count > 0) {
    g_post_turn_stabilize_count--;
  }

  int total_weight = 0;
  int weighted_sum = 0;
  g_detected_count = 0;

  // Sensor positions: -250, -150, -50, +50, +150, +250
  const int positions[] = {-250, -150, -50, 50, 150, 250};

  // Check which sensors are detecting line
  bool detected[NUM_SENSORS];
  int left_count = 0;   // S0, S1, S2
  int right_count = 0;  // S3, S4, S5

  for (int i = 0; i < NUM_SENSORS; i++) {
    detected[i] = (g_sensor_values[i] >= g_params.threshold);
    if (detected[i]) {
      int weight = g_sensor_values[i];
      weighted_sum += positions[i] * weight;
      total_weight += weight;
      g_detected_count++;

      if (i < 3)
        left_count++;
      else
        right_count++;
    }
  }

  // Determine turn state and confidence level
  TurnState new_state = TurnState::STRAIGHT;
  bool high_confidence = false;  // Strong pattern (edge sensors active)

  // Check if in stabilization period (more sensitive to crossings)
  bool in_stabilization = (g_post_turn_stabilize_count > 0);

  // Crossing detection: all 6 sensors or both far edges (S0 and S5) active with
  // center sensors Keep it strict - require center sensors to also be active to
  // avoid false positives
  bool is_crossing = (g_detected_count >= 6) || (detected[0] && detected[5] &&
                                                 (detected[2] || detected[3]));

  if (is_crossing) {
    new_state = TurnState::CROSSING;
    high_confidence = true;
  }
  // Sharp RIGHT - HIGH confidence (edge sensors S4+S5 active)
  else if ((detected[4] && detected[5] && left_count <= 1) ||
           (detected[5] && left_count == 0)) {
    new_state = TurnState::SHARP_RIGHT;
    high_confidence = true;
  }
  // Sharp LEFT - HIGH confidence (edge sensors S0+S1 active)
  else if ((detected[0] && detected[1] && right_count <= 1) ||
           (detected[0] && right_count == 0)) {
    new_state = TurnState::SHARP_LEFT;
    high_confidence = true;
  }
  // Sharp RIGHT - LOW confidence (4 sensors, needs more hysteresis)
  else if ((detected[2] && detected[3] && detected[4] && detected[5] &&
            !detected[0]) ||
           (detected[3] && detected[4] && detected[5] && !detected[0] &&
            !detected[1])) {
    new_state = TurnState::SHARP_RIGHT;
    high_confidence = false;
  }
  // Sharp LEFT - LOW confidence (4 sensors, needs more hysteresis)
  else if ((detected[0] && detected[1] && detected[2] && detected[3] &&
            !detected[5]) ||
           (detected[0] && detected[1] && detected[2] && !detected[4] &&
            !detected[5])) {
    new_state = TurnState::SHARP_LEFT;
    high_confidence = false;
  }
  // Moderate turn patterns (2 sensors at edge, NOT middle sensors)
  // S3+S4 only (NOT S2+S3+S4 which is center-right, use PD control)
  else if (detected[3] && detected[4] && !detected[0] && !detected[1] &&
           !detected[2]) {
    new_state = TurnState::SHARP_RIGHT;
    high_confidence = true;
  }
  // S1+S2 only (NOT S1+S2+S3 which is center-left, use PD control)
  else if (detected[1] && detected[2] && !detected[3] && !detected[4] &&
           !detected[5]) {
    new_state = TurnState::SHARP_LEFT;
    high_confidence = true;
  }

#ifdef CONFIG_LINETRACE_DISABLE_SHARP_TURN
  // 急旋回モード無効時は常にPD制御を使用
  if (new_state == TurnState::SHARP_LEFT || new_state == TurnState::SHARP_RIGHT) {
    new_state = TurnState::STRAIGHT;
  }
#endif

  // Apply hysteresis with different thresholds based on confidence (from Kconfig)
  const int HOLD_CYCLES_HIGH = CONFIG_LINETRACE_HOLD_CYCLES_HIGH;
  const int HOLD_CYCLES_LOW = CONFIG_LINETRACE_HOLD_CYCLES_LOW;
  const int HOLD_CYCLES_EXIT = CONFIG_LINETRACE_HOLD_CYCLES_EXIT;

  // ===== No Line Detected - Keep current state =====
  // When no line is detected, maintain the current turn state
  // This prevents exiting sharp turn mode during curve course-out
  if (g_detected_count == 0) {
    // Keep g_turn_state unchanged, just update line position at the end
    if (total_weight > 0) {
      g_line_position = weighted_sum / total_weight;
    }
    return;
  }

  // During post-turn stabilization period:
  // - CROSSING detection is prioritized (immediate response)
  // - OPPOSITE direction sharp turns are suppressed
  // - SAME direction sharp turns are allowed (for consecutive curves)
  if (in_stabilization) {
    // Allow crossing detection - but keep suppression active!
    if (new_state == TurnState::CROSSING) {
      g_turn_state = TurnState::CROSSING;
      g_turn_hold_count = 0;
      // DO NOT reset stabilization - keep suppressing opposite turns through
      // crossing
    }
    // Suppress OPPOSITE direction sharp turns only
    else if ((g_last_sharp_turn == TurnState::SHARP_RIGHT &&
              new_state == TurnState::SHARP_LEFT) ||
             (g_last_sharp_turn == TurnState::SHARP_LEFT &&
              new_state == TurnState::SHARP_RIGHT)) {
      // Opposite direction detected - suppress it, keep STRAIGHT
      g_turn_state = TurnState::STRAIGHT;
      g_turn_hold_count = 0;
    }
    // Allow SAME direction sharp turns (consecutive curves)
    else if (new_state == TurnState::SHARP_LEFT ||
             new_state == TurnState::SHARP_RIGHT) {
      g_turn_state = new_state;
      g_turn_hold_count = 0;
      g_post_turn_stabilize_count =
          0;  // Exit stabilization on same-direction turn
    } else if (new_state == TurnState::STRAIGHT) {
      g_turn_state = TurnState::STRAIGHT;
      g_turn_hold_count = 0;
    }
  } else {
    // Normal hysteresis logic
    int required_cycles;
    if (new_state == g_turn_state) {
      // Same state, reset counter
      g_turn_hold_count = 0;
    } else {
      // Determine required cycles based on transition type
      bool exiting_sharp_turn = (new_state == TurnState::STRAIGHT ||
                                 new_state == TurnState::CROSSING) &&
                                (g_turn_state == TurnState::SHARP_LEFT ||
                                 g_turn_state == TurnState::SHARP_RIGHT);

      if (exiting_sharp_turn) {
        // Exiting sharp turn - need more consistency
        required_cycles = HOLD_CYCLES_EXIT;
      } else if (high_confidence) {
        required_cycles = HOLD_CYCLES_HIGH;
      } else {
        required_cycles = HOLD_CYCLES_LOW;
      }

      // Different state, increment counter
      g_turn_hold_count++;

      // Only change state after enough consistent readings
      if (g_turn_hold_count >= required_cycles) {
        // Start stabilization period when exiting sharp turn
        if (exiting_sharp_turn) {
          g_post_turn_stabilize_count = POST_TURN_STABILIZE_CYCLES;
          g_last_sharp_turn =
              g_turn_state;  // Remember which direction we were turning
        }
        g_turn_state = new_state;
        g_turn_hold_count = 0;
      }
      // Otherwise keep old state (hysteresis)
    }
  }

  if (total_weight > 0) {
    g_line_position = weighted_sum / total_weight;
  }
  // If no line detected, keep last position (for recovery)
}

// Helper function: ramp current speed towards target
static int ramp_to(int current, int target, int rate) {
  if (current < target) {
    return std::min(current + rate, target);
  } else if (current > target) {
    return std::max(current - rate, target);
  }
  return current;
}

void line_trace_control() {
  int left_speed, right_speed;

  // ===== Sharp Turn Modes (also continue during no-line detection) =====
  if (g_turn_state == TurnState::SHARP_RIGHT) {
    // Base target speeds for sharp right turn
    int target_left = g_params.sharp_turn_outer;
    int target_right = g_params.sharp_turn_speed;

    // Apply PD control during sharp turn for course-following
    float error = g_line_position / 250.0f;  // Normalize to -1.0 to +1.0
    float derivative = error - g_last_error;
    float pd_correction = (g_params.kp * error + g_params.kd * derivative) 
                          * g_params.sharp_turn_pd_gain;
    
    // Adjust target speeds based on line position
    // If line is more to the right (positive), turn harder (increase outer, decrease inner)
    int correction_amount = (int)(pd_correction * CONFIG_LINETRACE_PD_CORRECTION_SCALE);
    target_left = std::min(g_params.max_speed, target_left + correction_amount);
    target_right = std::max(-g_params.max_speed, target_right - correction_amount);

    // Ramp towards target speeds (smooth acceleration)
    left_speed = ramp_to(g_current_left_speed, target_left, g_params.ramp_rate);
    right_speed = ramp_to(g_current_right_speed, target_right, g_params.ramp_rate);

#ifdef CONFIG_LINETRACE_ENABLE_STRAIGHT_BOOST
    // Reset straight boost
    g_straight_count = 0;
    g_straight_boost_active = false;
#endif
  } else if (g_turn_state == TurnState::SHARP_LEFT) {
    // Base target speeds for sharp left turn
    int target_left = g_params.sharp_turn_speed;
    int target_right = g_params.sharp_turn_outer;

    // Apply PD control during sharp turn for course-following
    float error = g_line_position / 250.0f;  // Normalize to -1.0 to +1.0
    float derivative = error - g_last_error;
    float pd_correction = (g_params.kp * error + g_params.kd * derivative) 
                          * g_params.sharp_turn_pd_gain;
    
    // Adjust target speeds based on line position
    // If line is more to the left (negative), turn harder
    int correction_amount = (int)(pd_correction * CONFIG_LINETRACE_PD_CORRECTION_SCALE);
    target_left = std::max(-g_params.max_speed, target_left - correction_amount);
    target_right = std::min(g_params.max_speed, target_right + correction_amount);

    // Ramp towards target speeds (smooth acceleration)
    left_speed = ramp_to(g_current_left_speed, target_left, g_params.ramp_rate);
    right_speed = ramp_to(g_current_right_speed, target_right, g_params.ramp_rate);

#ifdef CONFIG_LINETRACE_ENABLE_STRAIGHT_BOOST
    // Reset straight boost
    g_straight_count = 0;
    g_straight_boost_active = false;
#endif
  }
  // ===== Crossing / Intersection =====
  else if (g_turn_state == TurnState::CROSSING) {
#ifdef CONFIG_LINETRACE_ENABLE_STRAIGHT_BOOST
    // Go straight through crossing - maintain boost if active
    int crossing_speed = g_params.base_speed;
    if (g_straight_boost_active) {
      // Calculate gradual boost speed
      int boost_progress = g_straight_count - STRAIGHT_BOOST_THRESHOLD;
      int boost_range = STRAIGHT_BOOST_MAX_COUNT - STRAIGHT_BOOST_THRESHOLD;
      if (boost_progress > boost_range) boost_progress = boost_range;
      crossing_speed = g_params.base_speed +
                       (g_params.straight_boost_speed - g_params.base_speed) *
                           boost_progress / boost_range;
    }
    left_speed = crossing_speed;
    right_speed = crossing_speed;
    // Don't reset boost - continue through crossing
#else
    // Go straight through crossing
    left_speed = g_params.base_speed;
    right_speed = g_params.base_speed;
#endif
  }
  // ===== No Line Detected (not during sharp turn) =====
  else if (g_detected_count == 0) {
    // No line detected - slow down and search
    left_speed = g_params.base_speed / 2;
    right_speed = g_params.base_speed / 2;

    // Turn towards last known position
    if (g_line_position > 0) {
      left_speed += CONFIG_LINETRACE_SEARCH_TURN_SPEED;
      right_speed -= CONFIG_LINETRACE_SEARCH_TURN_SPEED;
    } else {
      left_speed -= CONFIG_LINETRACE_SEARCH_TURN_SPEED;
      right_speed += CONFIG_LINETRACE_SEARCH_TURN_SPEED;
    }
#ifdef CONFIG_LINETRACE_ENABLE_STRAIGHT_BOOST
    // Reset straight boost
    g_straight_count = 0;
    g_straight_boost_active = false;
#endif
  }
  // ===== Normal PD Control (also used during stabilization) =====
  else {
    float error = g_line_position / 250.0f;  // Normalize to -1.0 to +1.0
    float derivative = error - g_last_error;
    g_last_error = error;

    // Apply gain reduction during post-turn stabilization period
    // Gain gradually increases from 0.5 to 1.0 as stabilization count decreases
    float gain_factor = 1.0f;
    if (g_post_turn_stabilize_count > 0) {
      gain_factor = 0.5f + 0.2f * (1.0f - (float)g_post_turn_stabilize_count / POST_TURN_STABILIZE_CYCLES);
    }

    float correction = (g_params.kp * error + g_params.kd * derivative) * gain_factor;

#ifdef CONFIG_LINETRACE_ENABLE_STRAIGHT_BOOST
    // ===== Straight Acceleration Mode =====
    // Suppress straight boost during post-turn stabilization period
    if (g_post_turn_stabilize_count > 0) {
      g_straight_count = 0;
      g_straight_boost_active = false;
    }
    // Check if we're going straight (small error)
    else {
      float abs_error = (error >= 0) ? error : -error;
      if (abs_error < g_params.straight_error_threshold) {
      g_straight_count++;
      // Cap at max count to prevent overflow
      if (g_straight_count > STRAIGHT_BOOST_MAX_COUNT) {
        g_straight_count = STRAIGHT_BOOST_MAX_COUNT;
      }
      if (g_straight_count >= STRAIGHT_BOOST_THRESHOLD) {
        g_straight_boost_active = true;
      }
    } else {
      // Error is large, gradually reduce boost
      if (g_straight_count > 0) {
        g_straight_count -= 2;  // Faster decay
        if (g_straight_count < 0) g_straight_count = 0;
      }
      if (g_straight_count < STRAIGHT_BOOST_THRESHOLD / 2) {
        g_straight_boost_active = false;
      }
    }
    }  // end of else (not in stabilization period)

    // Calculate speed with gradual acceleration
    int current_base_speed = g_params.base_speed;
    if (g_straight_boost_active) {
      // Gradually increase speed from base_speed to straight_boost_speed
      int boost_progress = g_straight_count - STRAIGHT_BOOST_THRESHOLD;
      int boost_range = STRAIGHT_BOOST_MAX_COUNT - STRAIGHT_BOOST_THRESHOLD;
      current_base_speed =
          g_params.base_speed +
          (g_params.straight_boost_speed - g_params.base_speed) *
              boost_progress / boost_range;
    }
#else
    int current_base_speed = g_params.base_speed;
#endif

    // Apply correction
    int correction_amount = (int)(correction * current_base_speed);
    left_speed = current_base_speed + correction_amount;
    right_speed = current_base_speed - correction_amount;
  }

  // Clamp speeds
  left_speed =
      std::max(-g_params.max_speed, std::min(g_params.max_speed, left_speed));
  right_speed =
      std::max(-g_params.max_speed, std::min(g_params.max_speed, right_speed));

  // Update current speed tracking (for next cycle's ramp calculation)
  g_current_left_speed = left_speed;
  g_current_right_speed = right_speed;

  g_drive->set_motors(left_speed, right_speed);
}

// ===== Main =====
extern "C" void app_main(void) {
  ESP_LOGI(TAG, "Line Trace Car Starting...");

  // ===== Initialize NVS =====
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_LOGW(TAG, "Erasing NVS flash...");
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);
  ESP_LOGI(TAG, "NVS initialized");

  // Load calibration values from NVS
  load_calibration_from_nvs();

  // ===== Initialize OLED =====
  static SSD1306 oled(I2C_NUM_0, SSD1306_I2C_ADDR);
  g_oled = &oled;

  if (oled.init(I2C_SDA_PIN, I2C_SCL_PIN) != ESP_OK) {
    ESP_LOGE(TAG, "OLED initialization failed");
    return;
  }
  ESP_LOGI(TAG, "OLED initialized");

  // ===== Initialize Buttons =====
  ButtonConfig cfg_a = {BUTTON_A_PIN, true, 
                        CONFIG_LINETRACE_BUTTON_DEBOUNCE_MS,
                        CONFIG_LINETRACE_BUTTON_LONG_PRESS_MS,
                        CONFIG_LINETRACE_BUTTON_DOUBLE_CLICK_MS};
  ButtonConfig cfg_b = {BUTTON_B_PIN, true,
                        CONFIG_LINETRACE_BUTTON_DEBOUNCE_MS,
                        CONFIG_LINETRACE_BUTTON_LONG_PRESS_MS,
                        CONFIG_LINETRACE_BUTTON_DOUBLE_CLICK_MS};

  static Button btn_a(cfg_a);
  static Button btn_b(cfg_b);

  btn_a.init();
  btn_b.init();
  btn_a.set_callback(on_button_a);
  btn_b.set_callback(on_button_b);

  static ButtonManager btn_mgr;
  btn_mgr.add_button(&btn_a);
  btn_mgr.add_button(&btn_b);
  btn_mgr.start(10);
  ESP_LOGI(TAG, "Buttons initialized");

  // ===== Initialize Buzzer =====
  static Buzzer buzzer(BUZZER_PIN, LEDC_TIMER_1, LEDC_CHANNEL_4);
  buzzer.init();
  g_buzzer = &buzzer;
  ESP_LOGI(TAG, "Buzzer initialized");

  // ===== Initialize RGB LED (PL9823) =====
  static PL9823 led(LED_PIN, LED_COUNT);
  if (led.init() != ESP_OK) {
    ESP_LOGE(TAG, "PL9823 LED initialization failed");
  } else {
    g_led = &led;
    ESP_LOGI(TAG, "PL9823 LED initialized");
    led.clear();  // 初期状態で消灯
  }

  // ===== Initialize Motors =====
  static Motor motor_left(MOTOR_L_A_PIN, MOTOR_L_B_PIN, LEDC_CHANNEL_0,
                          LEDC_CHANNEL_1, LEDC_TIMER_0);
  static Motor motor_right(MOTOR_R_A_PIN, MOTOR_R_B_PIN, LEDC_CHANNEL_2,
                           LEDC_CHANNEL_3, LEDC_TIMER_0);

  if (motor_left.init() != ESP_OK || motor_right.init() != ESP_OK) {
    ESP_LOGE(TAG, "Motor initialization failed");
    oled.clear();
    oled.draw_string(10, 25, "Motor Init Failed!");
    oled.update();
    return;
  }

  static DifferentialDrive drive(motor_left, motor_right);
  g_drive = &drive;
  ESP_LOGI(TAG, "Motors initialized");

  // ===== Initialize ADC/Sensors =====
  static ADCUnit adc1(ADC_UNIT_1);
  if (adc1.init() != ESP_OK) {
    ESP_LOGE(TAG, "ADC initialization failed");
    oled.clear();
    oled.draw_string(10, 25, "ADC Init Failed!");
    oled.update();
    return;
  }

  static ADCChannel sensor0(adc1, SENSOR_CHANNELS[0]);
  static ADCChannel sensor1(adc1, SENSOR_CHANNELS[1]);
  static ADCChannel sensor2(adc1, SENSOR_CHANNELS[2]);
  static ADCChannel sensor3(adc1, SENSOR_CHANNELS[3]);
  static ADCChannel sensor4(adc1, SENSOR_CHANNELS[4]);
  static ADCChannel sensor5(adc1, SENSOR_CHANNELS[5]);

  ADCChannel* sensors[] = {&sensor0, &sensor1, &sensor2,
                           &sensor3, &sensor4, &sensor5};

  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensors[i]->init() != ESP_OK) {
      ESP_LOGE(TAG, "Sensor %d initialization failed", i);
      return;
    }
  }
  ESP_LOGI(TAG, "Sensors initialized");

  // ===== Startup Screen =====
  oled.clear();
  oled.draw_rect(0, 0, 128, 64);
  oled.draw_string(20, 8, "LINE TRACER");
  oled.draw_string(40, 22, "v1.0");
  oled.draw_line(20, 35, 108, 35);
  oled.draw_string(15, 42, "A: Start/Stop");
  oled.draw_string(15, 52, "B: Calibration");
  oled.update();

  // Startup melody
  buzzer.playStartupMelody();

  vTaskDelay(pdMS_TO_TICKS(1500));

  // ===== Main Loop =====
  ESP_LOGI(TAG, "Entering main loop");

  TickType_t last_wake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(CONFIG_LINETRACE_CONTROL_PERIOD_MS);

  while (true) {
    // Read sensors
    for (int i = 0; i < NUM_SENSORS; i++) {
      int value;
      if (sensors[i]->readVoltage(&value) == ESP_OK) {
        g_sensor_values[i] = value;

        // Update calibration values
        if (g_mode == CarMode::CALIBRATION || g_mode == CarMode::RUNNING) {
          if (value < g_sensor_min[i]) g_sensor_min[i] = value;
          if (value > g_sensor_max[i]) g_sensor_max[i] = value;
        }
      }
    }

    // Calculate line position
    calculate_line_position();

    // Control motors if running
    if (g_mode == CarMode::RUNNING) {
      line_trace_control();
    }

    // Update display (at lower rate)
    static int display_counter = 0;
    if (++display_counter >= 3) {  // ~17Hz display update
      display_counter = 0;
      update_display();
    }

    // Maintain loop timing
    vTaskDelayUntil(&last_wake, period);
  }
}
