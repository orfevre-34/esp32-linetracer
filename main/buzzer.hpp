/*
   Buzzer クラス定義
   LEDC PWMを使用した電子ブザー制御
*/

#pragma once

#include <cstdint>

#include "driver/ledc.h"
#include "hal/gpio_types.h"

class Buzzer {
 public:
  // 音階周波数 (Hz)
  static constexpr uint32_t NOTE_C4 = 262;
  static constexpr uint32_t NOTE_D4 = 294;
  static constexpr uint32_t NOTE_E4 = 330;
  static constexpr uint32_t NOTE_F4 = 349;
  static constexpr uint32_t NOTE_G4 = 392;
  static constexpr uint32_t NOTE_A4 = 440;
  static constexpr uint32_t NOTE_B4 = 494;
  static constexpr uint32_t NOTE_C5 = 523;
  static constexpr uint32_t NOTE_D5 = 587;
  static constexpr uint32_t NOTE_E5 = 659;
  static constexpr uint32_t NOTE_F5 = 698;
  static constexpr uint32_t NOTE_G5 = 784;
  static constexpr uint32_t NOTE_A5 = 880;
  static constexpr uint32_t NOTE_REST = 0;  // 休符

  // コンストラクタ
  explicit Buzzer(gpio_num_t gpio_pin, ledc_timer_t timer = LEDC_TIMER_0,
                  ledc_channel_t channel = LEDC_CHANNEL_0);

  // デストラクタ
  ~Buzzer();

  // コピー・ムーブ禁止
  Buzzer(const Buzzer&) = delete;
  Buzzer& operator=(const Buzzer&) = delete;
  Buzzer(Buzzer&&) = delete;
  Buzzer& operator=(Buzzer&&) = delete;

  // 初期化
  void init();

  // 指定周波数でトーン再生
  void tone(uint32_t frequency);

  // トーン停止
  void noTone();

  // 指定周波数で指定時間再生 (ms)
  void playTone(uint32_t frequency, uint32_t duration_ms);

  // 起動メロディ再生
  void playStartupMelody();

  // ビープ音
  void beep(uint32_t duration_ms = 100);

 private:
  gpio_num_t gpio_pin_;
  ledc_timer_t timer_;
  ledc_channel_t channel_;
  bool initialized_;
};
