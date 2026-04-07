/*
  ADC サンプルクラス実装
  複数チャンネル対応版
*/

#include "adc_reader.hpp"

#include "esp_log.h"

static const char* TAG = "ADC";

// ============================================================
// ADCUnit 実装
// ============================================================

ADCUnit::ADCUnit(adc_unit_t unit)
    : unit_(unit), handle_(nullptr), initialized_(false) {}

ADCUnit::~ADCUnit() {
  if (handle_ != nullptr) {
    adc_oneshot_del_unit(handle_);
    handle_ = nullptr;
  }
  initialized_ = false;
}

esp_err_t ADCUnit::init() {
  if (initialized_) {
    return ESP_OK;  // 既に初期化済み
  }

  adc_oneshot_unit_init_cfg_t init_config = {
      .unit_id = unit_,
      .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
      .ulp_mode = ADC_ULP_MODE_DISABLE,
  };

  esp_err_t ret = adc_oneshot_new_unit(&init_config, &handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "ADC unit %d init failed: %s", unit_, esp_err_to_name(ret));
    return ret;
  }

  initialized_ = true;
  ESP_LOGI(TAG, "ADC unit %d initialized", unit_);
  return ESP_OK;
}

// ============================================================
// ADCChannel 実装
// ============================================================

ADCChannel::ADCChannel(ADCUnit& unit, adc_channel_t channel)
    : unit_(unit), channel_(channel), cali_handle_(nullptr), initialized_(false) {}

ADCChannel::~ADCChannel() {
  if (cali_handle_ != nullptr) {
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_delete_scheme_curve_fitting(cali_handle_);
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_delete_scheme_line_fitting(cali_handle_);
#endif
    cali_handle_ = nullptr;
  }
  initialized_ = false;
}

esp_err_t ADCChannel::init() {
  if (!unit_.isInitialized()) {
    ESP_LOGE(TAG, "ADC unit not initialized");
    return ESP_ERR_INVALID_STATE;
  }

  // チャンネル設定
  adc_oneshot_chan_cfg_t chan_config = {
      .atten = ADC_ATTEN_DB_12,     // 0 ~ 3.3V 測定可能
      .bitwidth = ADC_BITWIDTH_12,  // 12bit解像度 (0-4095)
  };

  esp_err_t ret =
      adc_oneshot_config_channel(unit_.getHandle(), channel_, &chan_config);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "ADC channel %d config failed: %s", channel_,
             esp_err_to_name(ret));
    return ret;
  }

  // キャリブレーション設定
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
  adc_cali_curve_fitting_config_t cali_config = {
      .unit_id = unit_.getUnit(),
      .chan = channel_,
      .atten = ADC_ATTEN_DB_12,
      .bitwidth = ADC_BITWIDTH_12,
  };
  ret = adc_cali_create_scheme_curve_fitting(&cali_config, &cali_handle_);
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
  adc_cali_line_fitting_config_t cali_config = {
      .unit_id = unit_.getUnit(),
      .atten = ADC_ATTEN_DB_12,
      .bitwidth = ADC_BITWIDTH_12,
      .default_vref = 1100,
  };
  ret = adc_cali_create_scheme_line_fitting(&cali_config, &cali_handle_);
#else
  ret = ESP_ERR_NOT_SUPPORTED;
#endif

  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "ADC channel %d calibration not available", channel_);
    cali_handle_ = nullptr;
  }

  initialized_ = true;
  ESP_LOGI(TAG, "ADC channel %d initialized", channel_);
  return ESP_OK;
}

esp_err_t ADCChannel::readRaw(int* raw) {
  if (!initialized_) {
    return ESP_ERR_INVALID_STATE;
  }
  return adc_oneshot_read(unit_.getHandle(), channel_, raw);
}

esp_err_t ADCChannel::readVoltage(int* voltage_mv) {
  if (!initialized_) {
    return ESP_ERR_INVALID_STATE;
  }

  int raw;
  esp_err_t ret = adc_oneshot_read(unit_.getHandle(), channel_, &raw);
  if (ret != ESP_OK) {
    return ret;
  }

  if (cali_handle_ != nullptr) {
    return adc_cali_raw_to_voltage(cali_handle_, raw, voltage_mv);
  } else {
    // 簡易計算: 3.3V / 4095 * raw
    *voltage_mv = (raw * 3300) / 4095;
    return ESP_OK;
  }
}
