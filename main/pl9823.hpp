/*
 * PL9823 RGB LED制御クラス - ESP-IDF
 *
 * PL9823はWS2812系と似たプロトコルですが、
 * - データ順序: RGB（WS2812BはGRB）
 * - 通信速度: 約400kHz（WS2812Bは800kHz）
 */

#ifndef PL9823_HPP
#define PL9823_HPP

#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "esp_err.h"
#include <cstdint>

class PL9823 {
public:
    /**
     * @brief コンストラクタ
     * @param gpio_num データ出力ピン
     * @param led_count LED個数
     */
    PL9823(gpio_num_t gpio_num, uint32_t led_count);
    
    /**
     * @brief デストラクタ
     */
    ~PL9823();
    
    // コピー禁止
    PL9823(const PL9823&) = delete;
    PL9823& operator=(const PL9823&) = delete;
    
    /**
     * @brief 初期化
     * @return ESP_OK: 成功, その他: エラー
     */
    esp_err_t init();
    
    /**
     * @brief LEDの色を設定（送信はしない）
     * @param index LED番号（0から）
     * @param r 赤 (0-255)
     * @param g 緑 (0-255)
     * @param b 青 (0-255)
     */
    void setPixel(uint32_t index, uint8_t r, uint8_t g, uint8_t b);
    
    /**
     * @brief HSV色空間でLEDの色を設定
     * @param index LED番号（0から）
     * @param h 色相 (0-360)
     * @param s 彩度 (0-100)
     * @param v 明度 (0-100)
     */
    void setPixelHSV(uint32_t index, uint16_t h, uint8_t s, uint8_t v);
    
    /**
     * @brief LEDデータを送信
     * @return ESP_OK: 成功, その他: エラー
     */
    esp_err_t refresh();
    
    /**
     * @brief 全LEDを消灯
     */
    void clear();
    
    /**
     * @brief 送信完了を待機
     * @param timeout_ms タイムアウト時間(ms)
     * @return ESP_OK: 成功, その他: エラー
     */
    esp_err_t waitDone(uint32_t timeout_ms = 100);
    
    /**
     * @brief LED個数を取得
     */
    uint32_t getLedCount() const { return m_led_count; }
    
    /**
     * @brief 初期化済みかどうか
     */
    bool isInitialized() const { return m_initialized; }

private:
    gpio_num_t m_gpio_num;
    uint32_t m_led_count;
    bool m_initialized;
    
    rmt_channel_handle_t m_channel;
    rmt_encoder_handle_t m_encoder;
    uint8_t* m_pixels;
    
    esp_err_t createEncoder();
    void deinit();
};

#endif /* PL9823_HPP */
