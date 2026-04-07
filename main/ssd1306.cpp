#include "ssd1306.hpp"
#include <cstdlib>

const char* SSD1306::TAG = "SSD1306";

// 5x7 ASCII Font (characters 32-127)
static const uint8_t font5x7[][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, // Space
    {0x00, 0x00, 0x5F, 0x00, 0x00}, // !
    {0x00, 0x07, 0x00, 0x07, 0x00}, // "
    {0x14, 0x7F, 0x14, 0x7F, 0x14}, // #
    {0x24, 0x2A, 0x7F, 0x2A, 0x12}, // $
    {0x23, 0x13, 0x08, 0x64, 0x62}, // %
    {0x36, 0x49, 0x55, 0x22, 0x50}, // &
    {0x00, 0x05, 0x03, 0x00, 0x00}, // '
    {0x00, 0x1C, 0x22, 0x41, 0x00}, // (
    {0x00, 0x41, 0x22, 0x1C, 0x00}, // )
    {0x08, 0x2A, 0x1C, 0x2A, 0x08}, // *
    {0x08, 0x08, 0x3E, 0x08, 0x08}, // +
    {0x00, 0x50, 0x30, 0x00, 0x00}, // ,
    {0x08, 0x08, 0x08, 0x08, 0x08}, // -
    {0x00, 0x60, 0x60, 0x00, 0x00}, // .
    {0x20, 0x10, 0x08, 0x04, 0x02}, // /
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, // 0
    {0x00, 0x42, 0x7F, 0x40, 0x00}, // 1
    {0x42, 0x61, 0x51, 0x49, 0x46}, // 2
    {0x21, 0x41, 0x45, 0x4B, 0x31}, // 3
    {0x18, 0x14, 0x12, 0x7F, 0x10}, // 4
    {0x27, 0x45, 0x45, 0x45, 0x39}, // 5
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, // 6
    {0x01, 0x71, 0x09, 0x05, 0x03}, // 7
    {0x36, 0x49, 0x49, 0x49, 0x36}, // 8
    {0x06, 0x49, 0x49, 0x29, 0x1E}, // 9
    {0x00, 0x36, 0x36, 0x00, 0x00}, // :
    {0x00, 0x56, 0x36, 0x00, 0x00}, // ;
    {0x00, 0x08, 0x14, 0x22, 0x41}, // <
    {0x14, 0x14, 0x14, 0x14, 0x14}, // =
    {0x41, 0x22, 0x14, 0x08, 0x00}, // >
    {0x02, 0x01, 0x51, 0x09, 0x06}, // ?
    {0x32, 0x49, 0x79, 0x41, 0x3E}, // @
    {0x7E, 0x11, 0x11, 0x11, 0x7E}, // A
    {0x7F, 0x49, 0x49, 0x49, 0x36}, // B
    {0x3E, 0x41, 0x41, 0x41, 0x22}, // C
    {0x7F, 0x41, 0x41, 0x22, 0x1C}, // D
    {0x7F, 0x49, 0x49, 0x49, 0x41}, // E
    {0x7F, 0x09, 0x09, 0x01, 0x01}, // F
    {0x3E, 0x41, 0x41, 0x51, 0x32}, // G
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, // H
    {0x00, 0x41, 0x7F, 0x41, 0x00}, // I
    {0x20, 0x40, 0x41, 0x3F, 0x01}, // J
    {0x7F, 0x08, 0x14, 0x22, 0x41}, // K
    {0x7F, 0x40, 0x40, 0x40, 0x40}, // L
    {0x7F, 0x02, 0x04, 0x02, 0x7F}, // M
    {0x7F, 0x04, 0x08, 0x10, 0x7F}, // N
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, // O
    {0x7F, 0x09, 0x09, 0x09, 0x06}, // P
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, // Q
    {0x7F, 0x09, 0x19, 0x29, 0x46}, // R
    {0x46, 0x49, 0x49, 0x49, 0x31}, // S
    {0x01, 0x01, 0x7F, 0x01, 0x01}, // T
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, // U
    {0x1F, 0x20, 0x40, 0x20, 0x1F}, // V
    {0x7F, 0x20, 0x18, 0x20, 0x7F}, // W
    {0x63, 0x14, 0x08, 0x14, 0x63}, // X
    {0x03, 0x04, 0x78, 0x04, 0x03}, // Y
    {0x61, 0x51, 0x49, 0x45, 0x43}, // Z
    {0x00, 0x00, 0x7F, 0x41, 0x41}, // [
    {0x02, 0x04, 0x08, 0x10, 0x20}, // backslash
    {0x41, 0x41, 0x7F, 0x00, 0x00}, // ]
    {0x04, 0x02, 0x01, 0x02, 0x04}, // ^
    {0x40, 0x40, 0x40, 0x40, 0x40}, // _
    {0x00, 0x01, 0x02, 0x04, 0x00}, // `
    {0x20, 0x54, 0x54, 0x54, 0x78}, // a
    {0x7F, 0x48, 0x44, 0x44, 0x38}, // b
    {0x38, 0x44, 0x44, 0x44, 0x20}, // c
    {0x38, 0x44, 0x44, 0x48, 0x7F}, // d
    {0x38, 0x54, 0x54, 0x54, 0x18}, // e
    {0x08, 0x7E, 0x09, 0x01, 0x02}, // f
    {0x08, 0x14, 0x54, 0x54, 0x3C}, // g
    {0x7F, 0x08, 0x04, 0x04, 0x78}, // h
    {0x00, 0x44, 0x7D, 0x40, 0x00}, // i
    {0x20, 0x40, 0x44, 0x3D, 0x00}, // j
    {0x00, 0x7F, 0x10, 0x28, 0x44}, // k
    {0x00, 0x41, 0x7F, 0x40, 0x00}, // l
    {0x7C, 0x04, 0x18, 0x04, 0x78}, // m
    {0x7C, 0x08, 0x04, 0x04, 0x78}, // n
    {0x38, 0x44, 0x44, 0x44, 0x38}, // o
    {0x7C, 0x14, 0x14, 0x14, 0x08}, // p
    {0x08, 0x14, 0x14, 0x18, 0x7C}, // q
    {0x7C, 0x08, 0x04, 0x04, 0x08}, // r
    {0x48, 0x54, 0x54, 0x54, 0x20}, // s
    {0x04, 0x3F, 0x44, 0x40, 0x20}, // t
    {0x3C, 0x40, 0x40, 0x20, 0x7C}, // u
    {0x1C, 0x20, 0x40, 0x20, 0x1C}, // v
    {0x3C, 0x40, 0x30, 0x40, 0x3C}, // w
    {0x44, 0x28, 0x10, 0x28, 0x44}, // x
    {0x0C, 0x50, 0x50, 0x50, 0x3C}, // y
    {0x44, 0x64, 0x54, 0x4C, 0x44}, // z
    {0x00, 0x08, 0x36, 0x41, 0x00}, // {
    {0x00, 0x00, 0x7F, 0x00, 0x00}, // |
    {0x00, 0x41, 0x36, 0x08, 0x00}, // }
    {0x08, 0x08, 0x2A, 0x1C, 0x08}, // ->
    {0x08, 0x1C, 0x2A, 0x08, 0x08}, // <-
};

SSD1306::SSD1306(i2c_port_t i2c_port, uint8_t addr)
    : i2c_port_(i2c_port), i2c_addr_(addr), initialized_(false)
{
    memset(buffer_, 0, sizeof(buffer_));
}

SSD1306::~SSD1306()
{
    if (initialized_) {
        display_off();
        i2c_driver_delete(i2c_port_);
    }
}

esp_err_t SSD1306::init(gpio_num_t sda_pin, gpio_num_t scl_pin, uint32_t freq_hz)
{
    // Configure I2C
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = sda_pin;
    conf.scl_io_num = scl_pin;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = freq_hz;
    
    esp_err_t ret = i2c_param_config(i2c_port_, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C param config failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = i2c_driver_install(i2c_port_, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize display with recommended sequence
    const uint8_t init_cmds[] = {
        SSD1306_CMD_DISPLAY_OFF,
        SSD1306_CMD_SET_DISPLAY_CLK_DIV, 0x80,
        SSD1306_CMD_SET_MULTIPLEX, 0x3F,              // 64 lines
        SSD1306_CMD_SET_DISPLAY_OFFSET, 0x00,
        SSD1306_CMD_SET_START_LINE | 0x00,
        SSD1306_CMD_CHARGE_PUMP, 0x14,                // Enable charge pump
        SSD1306_CMD_MEMORY_MODE, 0x00,                // Horizontal addressing mode
        SSD1306_CMD_SEG_REMAP,                        // Segment re-map
        SSD1306_CMD_COM_SCAN_DEC,                     // COM output scan direction
        SSD1306_CMD_SET_COM_PINS, 0x12,               // COM pins hardware config
        SSD1306_CMD_SET_CONTRAST, 0xCF,
        SSD1306_CMD_SET_PRECHARGE, 0xF1,
        SSD1306_CMD_SET_VCOM_DETECT, 0x40,
        SSD1306_CMD_DISPLAY_ALL_ON_RESUME,
        SSD1306_CMD_NORMAL_DISPLAY,
        SSD1306_CMD_DISPLAY_ON
    };
    
    send_commands(init_cmds, sizeof(init_cmds));
    
    initialized_ = true;
    clear();
    update();
    
    ESP_LOGI(TAG, "SSD1306 initialized successfully");
    return ESP_OK;
}

void SSD1306::send_command(uint8_t cmd)
{
    i2c_cmd_handle_t handle = i2c_cmd_link_create();
    i2c_master_start(handle);
    i2c_master_write_byte(handle, (i2c_addr_ << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(handle, SSD1306_CONTROL_CMD_SINGLE, true);
    i2c_master_write_byte(handle, cmd, true);
    i2c_master_stop(handle);
    i2c_master_cmd_begin(i2c_port_, handle, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(handle);
}

void SSD1306::send_commands(const uint8_t* cmds, size_t len)
{
    i2c_cmd_handle_t handle = i2c_cmd_link_create();
    i2c_master_start(handle);
    i2c_master_write_byte(handle, (i2c_addr_ << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(handle, SSD1306_CONTROL_CMD_STREAM, true);
    i2c_master_write(handle, cmds, len, true);
    i2c_master_stop(handle);
    i2c_master_cmd_begin(i2c_port_, handle, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(handle);
}

void SSD1306::send_data(const uint8_t* data, size_t len)
{
    i2c_cmd_handle_t handle = i2c_cmd_link_create();
    i2c_master_start(handle);
    i2c_master_write_byte(handle, (i2c_addr_ << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(handle, SSD1306_CONTROL_DATA_STREAM, true);
    i2c_master_write(handle, data, len, true);
    i2c_master_stop(handle);
    i2c_master_cmd_begin(i2c_port_, handle, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(handle);
}

void SSD1306::display_on()
{
    send_command(SSD1306_CMD_DISPLAY_ON);
}

void SSD1306::display_off()
{
    send_command(SSD1306_CMD_DISPLAY_OFF);
}

void SSD1306::clear()
{
    memset(buffer_, 0, sizeof(buffer_));
}

void SSD1306::update()
{
    // Set column address
    uint8_t col_cmd[] = {SSD1306_CMD_SET_COLUMN_ADDR, 0, SSD1306_WIDTH - 1};
    send_commands(col_cmd, sizeof(col_cmd));
    
    // Set page address
    uint8_t page_cmd[] = {SSD1306_CMD_SET_PAGE_ADDR, 0, SSD1306_PAGES - 1};
    send_commands(page_cmd, sizeof(page_cmd));
    
    // Send buffer data
    send_data(buffer_, sizeof(buffer_));
}

void SSD1306::invert(bool enable)
{
    send_command(enable ? SSD1306_CMD_INVERT_DISPLAY : SSD1306_CMD_NORMAL_DISPLAY);
}

void SSD1306::set_contrast(uint8_t contrast)
{
    uint8_t cmds[] = {SSD1306_CMD_SET_CONTRAST, contrast};
    send_commands(cmds, sizeof(cmds));
}

void SSD1306::set_pixel(int16_t x, int16_t y, bool color)
{
    if (x < 0 || x >= SSD1306_WIDTH || y < 0 || y >= SSD1306_HEIGHT) {
        return;
    }
    
    if (color) {
        buffer_[x + (y / 8) * SSD1306_WIDTH] |= (1 << (y & 7));
    } else {
        buffer_[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y & 7));
    }
}

void SSD1306::draw_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1, bool color)
{
    int16_t dx = abs(x1 - x0);
    int16_t dy = -abs(y1 - y0);
    int16_t sx = x0 < x1 ? 1 : -1;
    int16_t sy = y0 < y1 ? 1 : -1;
    int16_t err = dx + dy;
    
    while (true) {
        set_pixel(x0, y0, color);
        if (x0 == x1 && y0 == y1) break;
        int16_t e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y0 += sy;
        }
    }
}

void SSD1306::draw_rect(int16_t x, int16_t y, int16_t w, int16_t h, bool color)
{
    draw_line(x, y, x + w - 1, y, color);
    draw_line(x, y + h - 1, x + w - 1, y + h - 1, color);
    draw_line(x, y, x, y + h - 1, color);
    draw_line(x + w - 1, y, x + w - 1, y + h - 1, color);
}

void SSD1306::fill_rect(int16_t x, int16_t y, int16_t w, int16_t h, bool color)
{
    for (int16_t i = x; i < x + w; i++) {
        for (int16_t j = y; j < y + h; j++) {
            set_pixel(i, j, color);
        }
    }
}

void SSD1306::draw_circle(int16_t x0, int16_t y0, int16_t r, bool color)
{
    int16_t x = r;
    int16_t y = 0;
    int16_t err = 0;
    
    while (x >= y) {
        set_pixel(x0 + x, y0 + y, color);
        set_pixel(x0 + y, y0 + x, color);
        set_pixel(x0 - y, y0 + x, color);
        set_pixel(x0 - x, y0 + y, color);
        set_pixel(x0 - x, y0 - y, color);
        set_pixel(x0 - y, y0 - x, color);
        set_pixel(x0 + y, y0 - x, color);
        set_pixel(x0 + x, y0 - y, color);
        
        y++;
        err += 1 + 2 * y;
        if (2 * (err - x) + 1 > 0) {
            x--;
            err += 1 - 2 * x;
        }
    }
}

void SSD1306::fill_circle(int16_t x0, int16_t y0, int16_t r, bool color)
{
    for (int16_t y = -r; y <= r; y++) {
        for (int16_t x = -r; x <= r; x++) {
            if (x * x + y * y <= r * r) {
                set_pixel(x0 + x, y0 + y, color);
            }
        }
    }
}

void SSD1306::draw_char(int16_t x, int16_t y, char c, bool color)
{
    if (c < 32 || c > 127) c = '?';
    
    const uint8_t* glyph = font5x7[c - 32];
    for (int8_t i = 0; i < 5; i++) {
        uint8_t line = glyph[i];
        for (int8_t j = 0; j < 7; j++) {
            if (line & (1 << j)) {
                set_pixel(x + i, y + j, color);
            }
        }
    }
}

void SSD1306::draw_string(int16_t x, int16_t y, const char* str, bool color)
{
    while (*str) {
        draw_char(x, y, *str++, color);
        x += 6;  // 5 pixels for char + 1 pixel spacing
    }
}

void SSD1306::draw_bitmap(int16_t x, int16_t y, const uint8_t* bitmap, int16_t w, int16_t h, bool color)
{
    int16_t byte_width = (w + 7) / 8;
    
    for (int16_t j = 0; j < h; j++) {
        for (int16_t i = 0; i < w; i++) {
            if (bitmap[j * byte_width + i / 8] & (128 >> (i & 7))) {
                set_pixel(x + i, y + j, color);
            }
        }
    }
}
