/*
 * PL9823 RGB LED制御クラス - ESP-IDF実装
 */

#include "pl9823.hpp"

#include <cstdlib>
#include <cstring>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "PL9823";

// RMT解像度: 10MHz (0.1us精度)
static constexpr uint32_t RMT_RESOLUTION_HZ = 10000000;

// PL9823のタイミング（単位: ns）
static constexpr uint32_t T0H_NS = 350;
static constexpr uint32_t T0L_NS = 1360;
static constexpr uint32_t T1H_NS = 1360;
static constexpr uint32_t T1L_NS = 350;
static constexpr uint32_t RESET_US = 50;

// RMTエンコーダ用のデータ
struct Pl9823Encoder {
    rmt_encoder_t base;
    rmt_encoder_t* bytes_encoder;
    rmt_encoder_t* copy_encoder;
    int state;
    rmt_symbol_word_t reset_code;
};

// エンコーダコールバック
static size_t pl9823_encode(rmt_encoder_t* encoder,
                            rmt_channel_handle_t channel,
                            const void* primary_data, size_t data_size,
                            rmt_encode_state_t* ret_state) {
    Pl9823Encoder* enc = __containerof(encoder, Pl9823Encoder, base);
    rmt_encode_state_t session_state = RMT_ENCODING_RESET;
    int state = RMT_ENCODING_RESET;
    size_t encoded_symbols = 0;

    switch (enc->state) {
        case 0:
            encoded_symbols += enc->bytes_encoder->encode(
                enc->bytes_encoder, channel, primary_data, data_size, &session_state);
            if (session_state & RMT_ENCODING_COMPLETE) {
                enc->state = 1;
            }
            if (session_state & RMT_ENCODING_MEM_FULL) {
                state |= RMT_ENCODING_MEM_FULL;
                goto out;
            }
            // fall-through
        case 1:
            encoded_symbols += enc->copy_encoder->encode(
                enc->copy_encoder, channel, &enc->reset_code,
                sizeof(enc->reset_code), &session_state);
            if (session_state & RMT_ENCODING_COMPLETE) {
                enc->state = RMT_ENCODING_RESET;
                state |= RMT_ENCODING_COMPLETE;
            }
            if (session_state & RMT_ENCODING_MEM_FULL) {
                state |= RMT_ENCODING_MEM_FULL;
                goto out;
            }
    }
out:
    *ret_state = static_cast<rmt_encode_state_t>(state);
    return encoded_symbols;
}

static esp_err_t pl9823_encoder_del(rmt_encoder_t* encoder) {
    Pl9823Encoder* enc = __containerof(encoder, Pl9823Encoder, base);
    rmt_del_encoder(enc->bytes_encoder);
    rmt_del_encoder(enc->copy_encoder);
    free(enc);
    return ESP_OK;
}

static esp_err_t pl9823_encoder_reset(rmt_encoder_t* encoder) {
    Pl9823Encoder* enc = __containerof(encoder, Pl9823Encoder, base);
    rmt_encoder_reset(enc->bytes_encoder);
    rmt_encoder_reset(enc->copy_encoder);
    enc->state = RMT_ENCODING_RESET;
    return ESP_OK;
}

// ===== PL9823 Class Implementation =====

PL9823::PL9823(gpio_num_t gpio_num, uint32_t led_count)
    : m_gpio_num(gpio_num)
    , m_led_count(led_count)
    , m_initialized(false)
    , m_channel(nullptr)
    , m_encoder(nullptr)
    , m_pixels(nullptr)
{
}

PL9823::~PL9823() {
    deinit();
}

esp_err_t PL9823::init() {
    if (m_initialized) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "初期化開始 - GPIO: %d, LED数: %lu", m_gpio_num, m_led_count);

    // ピクセルバッファ確保
    m_pixels = static_cast<uint8_t*>(calloc(m_led_count * 3, sizeof(uint8_t)));
    if (!m_pixels) {
        ESP_LOGE(TAG, "ピクセルバッファ確保失敗");
        return ESP_ERR_NO_MEM;
    }

    // RMT TXチャンネル設定
    rmt_tx_channel_config_t tx_config = {};
    tx_config.clk_src = RMT_CLK_SRC_DEFAULT;
    tx_config.gpio_num = m_gpio_num;
    tx_config.mem_block_symbols = 64;
    tx_config.resolution_hz = RMT_RESOLUTION_HZ;
    tx_config.trans_queue_depth = 4;

    esp_err_t ret = rmt_new_tx_channel(&tx_config, &m_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "RMTチャンネル作成失敗");
        free(m_pixels);
        m_pixels = nullptr;
        return ret;
    }

    // エンコーダ作成
    ret = createEncoder();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "エンコーダ作成失敗");
        rmt_del_channel(m_channel);
        m_channel = nullptr;
        free(m_pixels);
        m_pixels = nullptr;
        return ret;
    }

    // チャンネル有効化
    ret = rmt_enable(m_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "RMTチャンネル有効化失敗");
        rmt_del_encoder(m_encoder);
        m_encoder = nullptr;
        rmt_del_channel(m_channel);
        m_channel = nullptr;
        free(m_pixels);
        m_pixels = nullptr;
        return ret;
    }

    m_initialized = true;
    ESP_LOGI(TAG, "初期化完了");
    return ESP_OK;
}

esp_err_t PL9823::createEncoder() {
    Pl9823Encoder* enc = static_cast<Pl9823Encoder*>(calloc(1, sizeof(Pl9823Encoder)));
    if (!enc) {
        return ESP_ERR_NO_MEM;
    }

    enc->base.encode = pl9823_encode;
    enc->base.del = pl9823_encoder_del;
    enc->base.reset = pl9823_encoder_reset;

    // ビットタイミング設定 (10MHz = 100ns周期)
    rmt_bytes_encoder_config_t bytes_config = {};
    bytes_config.bit0.duration0 = T0H_NS / 100;
    bytes_config.bit0.level0 = 1;
    bytes_config.bit0.duration1 = T0L_NS / 100;
    bytes_config.bit0.level1 = 0;
    bytes_config.bit1.duration0 = T1H_NS / 100;
    bytes_config.bit1.level0 = 1;
    bytes_config.bit1.duration1 = T1L_NS / 100;
    bytes_config.bit1.level1 = 0;
    bytes_config.flags.msb_first = 1;

    esp_err_t ret = rmt_new_bytes_encoder(&bytes_config, &enc->bytes_encoder);
    if (ret != ESP_OK) {
        free(enc);
        return ret;
    }

    rmt_copy_encoder_config_t copy_config = {};
    ret = rmt_new_copy_encoder(&copy_config, &enc->copy_encoder);
    if (ret != ESP_OK) {
        rmt_del_encoder(enc->bytes_encoder);
        free(enc);
        return ret;
    }

    // リセット信号
    uint16_t reset_ticks = static_cast<uint16_t>(RESET_US * RMT_RESOLUTION_HZ / 1000000);
    enc->reset_code.level0 = 0;
    enc->reset_code.duration0 = reset_ticks;
    enc->reset_code.level1 = 0;
    enc->reset_code.duration1 = reset_ticks;

    m_encoder = &enc->base;
    return ESP_OK;
}

void PL9823::deinit() {
    if (m_channel) {
        rmt_disable(m_channel);
        rmt_del_channel(m_channel);
        m_channel = nullptr;
    }
    if (m_encoder) {
        rmt_del_encoder(m_encoder);
        m_encoder = nullptr;
    }
    if (m_pixels) {
        free(m_pixels);
        m_pixels = nullptr;
    }
    m_initialized = false;
    ESP_LOGI(TAG, "リソース解放完了");
}

void PL9823::setPixel(uint32_t index, uint8_t r, uint8_t g, uint8_t b) {
    if (index < m_led_count && m_pixels) {
        // PL9823はRGB順
        m_pixels[index * 3 + 0] = r;
        m_pixels[index * 3 + 1] = g;
        m_pixels[index * 3 + 2] = b;
    }
}

void PL9823::setPixelHSV(uint32_t index, uint16_t h, uint8_t s, uint8_t v) {
    if (index >= m_led_count || !m_pixels) {
        return;
    }

    uint8_t r, g, b;

    if (s == 0) {
        r = g = b = (v * 255) / 100;
    } else {
        uint16_t sector = (h / 60) % 6;
        uint16_t f = (h % 60) * 255 / 60;
        uint16_t vv = (v * 255) / 100;
        uint16_t p = (vv * (100 - s)) / 100;
        uint16_t q = (vv * (100 - (s * f) / 255)) / 100;
        uint16_t t = (vv * (100 - (s * (255 - f)) / 255)) / 100;

        switch (sector) {
            case 0: r = vv; g = t;  b = p;  break;
            case 1: r = q;  g = vv; b = p;  break;
            case 2: r = p;  g = vv; b = t;  break;
            case 3: r = p;  g = q;  b = vv; break;
            case 4: r = t;  g = p;  b = vv; break;
            default: r = vv; g = p;  b = q;  break;
        }
    }

    setPixel(index, r, g, b);
}

esp_err_t PL9823::refresh() {
    if (!m_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    rmt_transmit_config_t tx_config = {};
    tx_config.loop_count = 0;
    return rmt_transmit(m_channel, m_encoder, m_pixels, m_led_count * 3, &tx_config);
}

void PL9823::clear() {
    if (m_pixels) {
        memset(m_pixels, 0, m_led_count * 3);
        refresh();
        waitDone(100);
    }
}

esp_err_t PL9823::waitDone(uint32_t timeout_ms) {
    if (!m_channel) {
        return ESP_ERR_INVALID_STATE;
    }
    return rmt_tx_wait_all_done(m_channel, pdMS_TO_TICKS(timeout_ms));
}
