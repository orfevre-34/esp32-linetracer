/*
   ライントレースセンサー OLEDディスプレイ表示プログラム
   6チャンネルのセンサー値をリアルタイムでインジケータ表示
*/

#include <cstdio>
#include <cstring>
#include <algorithm>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "ssd1306.hpp"
#include "button.hpp"
#include "adc_reader.hpp"

static const char* TAG = "linetrace_display";

// Pin configuration
#define I2C_SDA_PIN     GPIO_NUM_25
#define I2C_SCL_PIN     GPIO_NUM_26
#define BUTTON_A_PIN    GPIO_NUM_4
#define BUTTON_B_PIN    GPIO_NUM_5

// Sensor channels (GPIO36, 39, 32, 33, 34, 35)
static constexpr adc_channel_t SENSOR_CHANNELS[] = {
    ADC_CHANNEL_0,  // GPIO36 - S0
    ADC_CHANNEL_3,  // GPIO39 - S1
    ADC_CHANNEL_6,  // GPIO32 - S2
    ADC_CHANNEL_7,  // GPIO33 - S3
    ADC_CHANNEL_4,  // GPIO34 - S4
    ADC_CHANNEL_5,  // GPIO35 - S5
};
static constexpr int NUM_SENSORS = sizeof(SENSOR_CHANNELS) / sizeof(SENSOR_CHANNELS[0]);

// Display modes
enum class DisplayMode {
    BAR_GRAPH,      // Horizontal bar graph
    BAR_VERTICAL,   // Vertical bar graph (like actual sensor layout)
    NUMERIC,        // Numeric values
    LINE_POSITION,  // Calculated line position
    MODE_COUNT
};

// Global state
static SSD1306* g_oled = nullptr;
static volatile DisplayMode g_display_mode = DisplayMode::BAR_VERTICAL;
static volatile int g_threshold = 1500;  // mV threshold for line detection
static volatile bool g_threshold_adjust = false;

// Sensor data
static int g_sensor_values[NUM_SENSORS] = {0};
static int g_sensor_min[NUM_SENSORS] = {3300, 3300, 3300, 3300, 3300, 3300};
static int g_sensor_max[NUM_SENSORS] = {0, 0, 0, 0, 0, 0};

// Button callbacks
void on_button_a(ButtonEvent event)
{
    if (event == ButtonEvent::CLICKED) {
        if (g_threshold_adjust) {
            g_threshold = std::min(3300, g_threshold + 100);
        } else {
            int mode = static_cast<int>(g_display_mode);
            mode = (mode + 1) % static_cast<int>(DisplayMode::MODE_COUNT);
            g_display_mode = static_cast<DisplayMode>(mode);
        }
    } else if (event == ButtonEvent::LONG_PRESS) {
        // Reset min/max calibration
        for (int i = 0; i < NUM_SENSORS; i++) {
            g_sensor_min[i] = 3300;
            g_sensor_max[i] = 0;
        }
    }
}

void on_button_b(ButtonEvent event)
{
    if (event == ButtonEvent::CLICKED) {
        if (g_threshold_adjust) {
            g_threshold = std::max(0, g_threshold - 100);
        } else {
            g_threshold_adjust = !g_threshold_adjust;
        }
    } else if (event == ButtonEvent::LONG_PRESS) {
        g_threshold_adjust = false;
    }
}

// Draw horizontal bar graph
void draw_bar_graph()
{
    g_oled->clear();
    g_oled->draw_string(25, 0, "Line Sensor");
    
    const int bar_x = 25;
    const int bar_width = 95;
    const int bar_height = 7;
    
    for (int i = 0; i < NUM_SENSORS; i++) {
        int y = 10 + i * 9;
        
        // Label
        char label[4];
        snprintf(label, sizeof(label), "S%d", i);
        g_oled->draw_string(0, y, label);
        
        // Bar outline
        g_oled->draw_rect(bar_x, y, bar_width, bar_height);
        
        // Bar fill (map 0-3300mV to bar width)
        int fill = (g_sensor_values[i] * (bar_width - 2)) / 3300;
        fill = std::max(0, std::min(bar_width - 2, fill));
        if (fill > 0) {
            g_oled->fill_rect(bar_x + 1, y + 1, fill, bar_height - 2);
        }
        
        // Threshold marker
        int thresh_x = bar_x + (g_threshold * (bar_width - 2)) / 3300;
        g_oled->draw_line(thresh_x, y, thresh_x, y + bar_height - 1);
    }
    
    // Footer
    char footer[25];
    snprintf(footer, sizeof(footer), "TH:%dmV", g_threshold);
    g_oled->draw_string(0, 56, footer);
    
    g_oled->update();
}

// Draw vertical bar graph (matches physical sensor layout)
void draw_bar_vertical()
{
    g_oled->clear();
    
    // Title
    g_oled->draw_string(15, 0, "Sensor Array");
    
    const int bar_width = 18;
    const int bar_max_height = 40;
    const int bar_y_bottom = 52;
    const int start_x = 5;
    
    for (int i = 0; i < NUM_SENSORS; i++) {
        int x = start_x + i * (bar_width + 2);
        
        // Bar outline
        g_oled->draw_rect(x, bar_y_bottom - bar_max_height, bar_width, bar_max_height);
        
        // Bar fill
        int fill_height = (g_sensor_values[i] * bar_max_height) / 3300;
        fill_height = std::max(0, std::min(bar_max_height - 2, fill_height));
        if (fill_height > 0) {
            g_oled->fill_rect(x + 1, bar_y_bottom - fill_height - 1, 
                             bar_width - 2, fill_height);
        }
        
        // Threshold line
        int thresh_y = bar_y_bottom - (g_threshold * bar_max_height) / 3300;
        g_oled->set_pixel(x, thresh_y);
        g_oled->set_pixel(x + bar_width - 1, thresh_y);
        
        // Detection indicator (filled dot if above threshold)
        if (g_sensor_values[i] >= g_threshold) {
            g_oled->fill_circle(x + bar_width / 2, bar_y_bottom + 6, 3);
        } else {
            g_oled->draw_circle(x + bar_width / 2, bar_y_bottom + 6, 3);
        }
        
        // Sensor number
        char num[2] = {(char)('0' + i), '\0'};
        g_oled->draw_char(x + 6, bar_y_bottom - bar_max_height - 8, num[0]);
    }
    
    g_oled->update();
}

// Draw numeric values
void draw_numeric()
{
    g_oled->clear();
    g_oled->draw_string(20, 0, "Sensor Values");
    g_oled->draw_line(0, 10, 127, 10);
    
    for (int i = 0; i < NUM_SENSORS; i++) {
        int x = (i % 2) * 64;
        int y = 14 + (i / 2) * 16;
        
        char buf[20];
        snprintf(buf, sizeof(buf), "S%d:%4dmV", i, g_sensor_values[i]);
        g_oled->draw_string(x, y, buf);
        
        // Detection indicator
        if (g_sensor_values[i] >= g_threshold) {
            g_oled->fill_rect(x + 54, y + 2, 5, 5);
        }
    }
    
    // Threshold display
    g_oled->draw_line(0, 54, 127, 54);
    char footer[25];
    snprintf(footer, sizeof(footer), "Threshold: %dmV", g_threshold);
    g_oled->draw_string(10, 56, footer);
    
    g_oled->update();
}

// Calculate and display line position
void draw_line_position()
{
    g_oled->clear();
    g_oled->draw_string(15, 0, "Line Position");
    
    // Calculate weighted average position
    int total_weight = 0;
    int weighted_sum = 0;
    int detected_count = 0;
    
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (g_sensor_values[i] >= g_threshold) {
            // Weight by position (-2.5 to +2.5) and sensor value
            int weight = g_sensor_values[i];
            int position = (i * 100) - 250;  // -250 to +250
            weighted_sum += position * weight;
            total_weight += weight;
            detected_count++;
        }
    }
    
    // Draw sensor array visualization
    const int viz_y = 15;
    const int viz_width = 100;
    const int viz_height = 20;
    const int viz_x = 14;
    
    g_oled->draw_rect(viz_x, viz_y, viz_width, viz_height);
    
    // Draw sensor positions and detection status
    for (int i = 0; i < NUM_SENSORS; i++) {
        int x = viz_x + 8 + i * 15;
        
        if (g_sensor_values[i] >= g_threshold) {
            g_oled->fill_rect(x, viz_y + 2, 10, viz_height - 4);
        } else {
            g_oled->draw_rect(x, viz_y + 5, 10, viz_height - 10);
        }
    }
    
    // Draw calculated line position
    int line_pos = 0;
    if (total_weight > 0) {
        line_pos = weighted_sum / total_weight;  // -250 to +250
    }
    
    // Map to display coordinates
    int indicator_x = viz_x + viz_width / 2 + (line_pos * (viz_width / 2 - 5)) / 250;
    indicator_x = std::max(viz_x + 2, std::min(viz_x + viz_width - 4, indicator_x));
    
    // Draw position indicator (triangle)
    g_oled->draw_line(indicator_x, viz_y + viz_height + 2, indicator_x - 4, viz_y + viz_height + 8);
    g_oled->draw_line(indicator_x, viz_y + viz_height + 2, indicator_x + 4, viz_y + viz_height + 8);
    g_oled->draw_line(indicator_x - 4, viz_y + viz_height + 8, indicator_x + 4, viz_y + viz_height + 8);
    
    // Display values
    char buf[30];
    snprintf(buf, sizeof(buf), "Pos: %+d", line_pos);
    g_oled->draw_string(35, 45, buf);
    
    snprintf(buf, sizeof(buf), "Detected: %d/%d", detected_count, NUM_SENSORS);
    g_oled->draw_string(20, 56, buf);
    
    g_oled->update();
}

// Update display based on current mode
void update_display()
{
    // Draw threshold adjustment overlay if active
    if (g_threshold_adjust) {
        g_oled->clear();
        g_oled->draw_string(15, 0, "Adjust Threshold");
        g_oled->draw_line(0, 12, 127, 12);
        
        // Large threshold value
        char buf[20];
        snprintf(buf, sizeof(buf), "%d mV", g_threshold);
        int len = strlen(buf);
        int x = 64 - (len * 6) / 2;
        g_oled->draw_string(x, 25, buf);
        
        // Bar visualization
        g_oled->draw_rect(10, 42, 108, 10);
        int fill = (g_threshold * 104) / 3300;
        g_oled->fill_rect(12, 44, fill, 6);
        
        g_oled->draw_string(10, 55, "A:+ B:- LongB:Done");
        g_oled->update();
        return;
    }
    
    switch (g_display_mode) {
        case DisplayMode::BAR_GRAPH:
            draw_bar_graph();
            break;
        case DisplayMode::BAR_VERTICAL:
            draw_bar_vertical();
            break;
        case DisplayMode::NUMERIC:
            draw_numeric();
            break;
        case DisplayMode::LINE_POSITION:
            draw_line_position();
            break;
        default:
            break;
    }
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Line Trace Sensor Display Starting...");
    
    // Initialize OLED
    static SSD1306 oled(I2C_NUM_0, SSD1306_I2C_ADDR);
    g_oled = &oled;
    
    esp_err_t ret = oled.init(I2C_SDA_PIN, I2C_SCL_PIN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SSD1306");
        return;
    }
    ESP_LOGI(TAG, "OLED initialized");
    
    // Initialize buttons
    ButtonConfig cfg_a = {BUTTON_A_PIN, true, 20, 800, 300};
    ButtonConfig cfg_b = {BUTTON_B_PIN, true, 20, 800, 300};
    
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
    
    // Initialize ADC
    static ADCUnit adc1(ADC_UNIT_1);
    if (adc1.init() != ESP_OK) {
        ESP_LOGE(TAG, "ADC unit initialization failed");
        oled.clear();
        oled.draw_string(10, 25, "ADC Init Failed!");
        oled.update();
        return;
    }
    
    // Initialize sensor channels
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
            oled.clear();
            char buf[25];
            snprintf(buf, sizeof(buf), "Sensor %d Failed!", i);
            oled.draw_string(10, 25, buf);
            oled.update();
            return;
        }
    }
    ESP_LOGI(TAG, "All sensors initialized");
    
    // Show startup screen
    oled.clear();
    oled.draw_rect(0, 0, 128, 64);
    oled.draw_string(15, 10, "Line Tracer");
    oled.draw_string(25, 22, "Sensor Monitor");
    oled.draw_string(5, 40, "A:Mode B:Threshold");
    oled.draw_string(5, 52, "LongA:Calibrate");
    oled.update();
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Main loop
    ESP_LOGI(TAG, "Entering main loop");
    
    while (true) {
        // Read all sensors
        for (int i = 0; i < NUM_SENSORS; i++) {
            int value;
            if (sensors[i]->readVoltage(&value) == ESP_OK) {
                g_sensor_values[i] = value;
                
                // Update min/max for calibration
                if (value < g_sensor_min[i]) g_sensor_min[i] = value;
                if (value > g_sensor_max[i]) g_sensor_max[i] = value;
            } else {
                g_sensor_values[i] = 0;
            }
        }
        
        // Update display
        update_display();
        
        // Small delay
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
