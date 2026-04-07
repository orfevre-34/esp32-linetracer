/*
   ライントレースセンサー テストプログラム
   ADC1の6チャンネルを使用
*/

#include <cstdio>

#include "adc_reader.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {
constexpr const char* TAG = "linetrace_sensor";

// センサーチャンネル定義
constexpr adc_channel_t SENSOR_CHANNELS[] = {
    ADC_CHANNEL_0,  // GPIO36
    ADC_CHANNEL_3,  // GPIO39
    ADC_CHANNEL_6,  // GPIO32
    ADC_CHANNEL_7,  // GPIO33
    ADC_CHANNEL_4,  // GPIO34
    ADC_CHANNEL_5,  // GPIO35
};
constexpr int NUM_SENSORS = sizeof(SENSOR_CHANNELS) / sizeof(SENSOR_CHANNELS[0]);
}  // namespace

extern "C" void app_main(void) {
  // ADCユニットを1つだけ作成 (これを全チャンネルで共有)
  ADCUnit adc1(ADC_UNIT_1);
  if (adc1.init() != ESP_OK) {
    ESP_LOGE(TAG, "ADCユニット初期化失敗");
    return;
  }

  // 各チャンネルを初期化
  ADCChannel sensor0(adc1, SENSOR_CHANNELS[0]);
  ADCChannel sensor1(adc1, SENSOR_CHANNELS[1]);
  ADCChannel sensor2(adc1, SENSOR_CHANNELS[2]);
  ADCChannel sensor3(adc1, SENSOR_CHANNELS[3]);
  ADCChannel sensor4(adc1, SENSOR_CHANNELS[4]);
  ADCChannel sensor5(adc1, SENSOR_CHANNELS[5]);

  ADCChannel* sensors[] = {&sensor0, &sensor1, &sensor2,
                           &sensor3, &sensor4, &sensor5};

  for (int i = 0; i < NUM_SENSORS; ++i) {
    if (sensors[i]->init() != ESP_OK) {
      ESP_LOGE(TAG, "センサー%dの初期化失敗", i);
      return;
    }
  }
  ESP_LOGI(TAG, "全センサー初期化完了");

  int sensor_values[NUM_SENSORS];

  // センサー値読み取りループ
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(500));

    for (int i = 0; i < NUM_SENSORS; ++i) {
      if (sensors[i]->readVoltage(&sensor_values[i]) != ESP_OK) {
        ESP_LOGE(TAG, "センサー%dの読み取り失敗", i);
        sensor_values[i] = -1;
      }
    }

    ESP_LOGI(TAG,
             "センサー値: S0=%d, S1=%d, S2=%d, S3=%d, S4=%d, S5=%d mV",
             sensor_values[0], sensor_values[1], sensor_values[2],
             sensor_values[3], sensor_values[4], sensor_values[5]);
  }
}
