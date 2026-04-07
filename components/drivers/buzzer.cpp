/*
   Buzzer クラス実装
   LEDC PWMを使用した電子ブザー制御
*/

#include "buzzer.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

Buzzer::Buzzer(gpio_num_t gpio_pin, ledc_timer_t timer, ledc_channel_t channel)
    : gpio_pin_(gpio_pin),
      timer_(timer),
      channel_(channel),
      initialized_(false) {}

Buzzer::~Buzzer() {
  if (initialized_) {
    noTone();
    ledc_stop(LEDC_LOW_SPEED_MODE, channel_, 0);
  }
}

void Buzzer::init() {
  // LEDCタイマー設定
  ledc_timer_config_t timer_config = {
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .duty_resolution = LEDC_TIMER_10_BIT,
      .timer_num = timer_,
      .freq_hz = 1000,
      .clk_cfg = LEDC_AUTO_CLK,
      .deconfigure = false,
  };
  ledc_timer_config(&timer_config);

  // LEDCチャンネル設定
  ledc_channel_config_t channel_config = {
      .gpio_num = gpio_pin_,
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .channel = channel_,
      .intr_type = LEDC_INTR_DISABLE,
      .timer_sel = timer_,
      .duty = 0,
      .hpoint = 0,
      .flags = {.output_invert = 0},
  };
  ledc_channel_config(&channel_config);

  initialized_ = true;
}

void Buzzer::tone(uint32_t frequency) {
  if (!initialized_ || frequency == 0) {
    noTone();
    return;
  }

  ledc_set_freq(LEDC_LOW_SPEED_MODE, timer_, frequency);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, channel_, 512);  // 50% duty (10bit)
  ledc_update_duty(LEDC_LOW_SPEED_MODE, channel_);
}

void Buzzer::noTone() {
  if (!initialized_) return;

  ledc_set_duty(LEDC_LOW_SPEED_MODE, channel_, 0);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, channel_);
}

void Buzzer::playTone(uint32_t frequency, uint32_t duration_ms) {
  tone(frequency);
  vTaskDelay(pdMS_TO_TICKS(duration_ms));
  noTone();
}

void Buzzer::beep(uint32_t duration_ms) { playTone(1000, duration_ms); }

void Buzzer::playStartupMelody() {
  // 起動メロディ (おしゃれバージョン)
  struct Note {
    uint32_t frequency;
    uint32_t duration;
    uint32_t pause;  // 音の後の間隔
  };

  const Note melody[] = {
      {NOTE_E5, 80, 20},   // 軽やかに始まる
      {NOTE_G4, 80, 20},   //
      {NOTE_C5, 120, 40},  // 少し伸ばす
      {NOTE_E5, 80, 20},   //
      {NOTE_G5, 200, 60},  // 上昇して
      {NOTE_E5, 100, 30},  //
      {NOTE_C5, 300, 0},   // 最後はゆったり
  };

  for (const auto& note : melody) {
    if (note.frequency == NOTE_REST) {
      vTaskDelay(pdMS_TO_TICKS(note.duration));
    } else {
      playTone(note.frequency, note.duration);
    }
    if (note.pause > 0) {
      vTaskDelay(pdMS_TO_TICKS(note.pause));
    }
  }
}
