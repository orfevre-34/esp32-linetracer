#pragma once

#include <cstdint>
#include <cstring>
#include "driver/i2c.h"
#include "esp_log.h"

// SSD1306 I2C Address
#define SSD1306_I2C_ADDR 0x3C

// SSD1306 Display dimensions
#define SSD1306_WIDTH  128
#define SSD1306_HEIGHT 64
#define SSD1306_PAGES  (SSD1306_HEIGHT / 8)

// SSD1306 Commands
#define SSD1306_CMD_DISPLAY_OFF          0xAE
#define SSD1306_CMD_DISPLAY_ON           0xAF
#define SSD1306_CMD_SET_DISPLAY_CLK_DIV  0xD5
#define SSD1306_CMD_SET_MULTIPLEX        0xA8
#define SSD1306_CMD_SET_DISPLAY_OFFSET   0xD3
#define SSD1306_CMD_SET_START_LINE       0x40
#define SSD1306_CMD_CHARGE_PUMP          0x8D
#define SSD1306_CMD_MEMORY_MODE          0x20
#define SSD1306_CMD_SEG_REMAP            0xA1
#define SSD1306_CMD_COM_SCAN_DEC         0xC8
#define SSD1306_CMD_SET_COM_PINS         0xDA
#define SSD1306_CMD_SET_CONTRAST         0x81
#define SSD1306_CMD_SET_PRECHARGE        0xD9
#define SSD1306_CMD_SET_VCOM_DETECT      0xDB
#define SSD1306_CMD_DISPLAY_ALL_ON_RESUME 0xA4
#define SSD1306_CMD_NORMAL_DISPLAY       0xA6
#define SSD1306_CMD_INVERT_DISPLAY       0xA7
#define SSD1306_CMD_SET_COLUMN_ADDR      0x21
#define SSD1306_CMD_SET_PAGE_ADDR        0x22

// Control bytes
#define SSD1306_CONTROL_CMD_SINGLE  0x80
#define SSD1306_CONTROL_CMD_STREAM  0x00
#define SSD1306_CONTROL_DATA_STREAM 0x40

class SSD1306 {
public:
    SSD1306(i2c_port_t i2c_port = I2C_NUM_0, uint8_t addr = SSD1306_I2C_ADDR);
    ~SSD1306();

    // Initialize I2C and display
    esp_err_t init(gpio_num_t sda_pin, gpio_num_t scl_pin, uint32_t freq_hz = 400000);
    
    // Display control
    void display_on();
    void display_off();
    void clear();
    void update();  // Send buffer to display
    void invert(bool enable);
    void set_contrast(uint8_t contrast);

    // Drawing functions
    void set_pixel(int16_t x, int16_t y, bool color = true);
    void draw_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1, bool color = true);
    void draw_rect(int16_t x, int16_t y, int16_t w, int16_t h, bool color = true);
    void fill_rect(int16_t x, int16_t y, int16_t w, int16_t h, bool color = true);
    void draw_circle(int16_t x0, int16_t y0, int16_t r, bool color = true);
    void fill_circle(int16_t x0, int16_t y0, int16_t r, bool color = true);

    // Text functions (using built-in 5x7 font)
    void draw_char(int16_t x, int16_t y, char c, bool color = true);
    void draw_string(int16_t x, int16_t y, const char* str, bool color = true);
    
    // Bitmap functions
    void draw_bitmap(int16_t x, int16_t y, const uint8_t* bitmap, int16_t w, int16_t h, bool color = true);

    // Get display buffer (for advanced usage)
    uint8_t* get_buffer() { return buffer_; }
    
private:
    void send_command(uint8_t cmd);
    void send_commands(const uint8_t* cmds, size_t len);
    void send_data(const uint8_t* data, size_t len);
    
    i2c_port_t i2c_port_;
    uint8_t i2c_addr_;
    uint8_t buffer_[SSD1306_WIDTH * SSD1306_PAGES];
    bool initialized_;
    
    static const char* TAG;
};
