/*
  ADC サンプルクラス定義
  複数チャンネル対応版
*/

#pragma once

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"

// ADCユニット管理クラス (ユニットごとに1つ作成)
class ADCUnit {
 public:
  explicit ADCUnit(adc_unit_t unit);
  ~ADCUnit();

  // コピー・ムーブ禁止
  ADCUnit(const ADCUnit&) = delete;
  ADCUnit& operator=(const ADCUnit&) = delete;
  ADCUnit(ADCUnit&&) = delete;
  ADCUnit& operator=(ADCUnit&&) = delete;

  esp_err_t init();
  adc_oneshot_unit_handle_t getHandle() const { return handle_; }
  adc_unit_t getUnit() const { return unit_; }
  bool isInitialized() const { return initialized_; }

 private:
  adc_unit_t unit_;
  adc_oneshot_unit_handle_t handle_;
  bool initialized_;
};

// ADCチャンネルクラス (チャンネルごとに作成、ADCUnitを共有)
class ADCChannel {
 public:
  ADCChannel(ADCUnit& unit, adc_channel_t channel);
  ~ADCChannel();

  // コピー・ムーブ禁止
  ADCChannel(const ADCChannel&) = delete;
  ADCChannel& operator=(const ADCChannel&) = delete;
  ADCChannel(ADCChannel&&) = delete;
  ADCChannel& operator=(ADCChannel&&) = delete;

  esp_err_t init();

  // 生データ読み取り (0-4095)
  esp_err_t readRaw(int* raw);

  // 電圧読み取り (mV)
  esp_err_t readVoltage(int* voltage_mv);

 private:
  ADCUnit& unit_;
  adc_channel_t channel_;
  adc_cali_handle_t cali_handle_;
  bool initialized_;
};
