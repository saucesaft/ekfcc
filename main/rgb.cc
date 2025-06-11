// rgb.cc
#include "rgb.h"
#include <algorithm>

const char* RGB::TAG = "RMTT_RGB";

esp_err_t RGB::Init(void) {
    esp_err_t ret = ESP_OK;
    
    // Configure LEDC timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_TIMER_RESOLUTION,
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = LEDC_BASE_FREQ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ret = ledc_timer_config(&ledc_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LEDC timer: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Configure LEDC channel for RED
    ledc_channel_config_t ledc_channel_r = {
        .gpio_num       = LED_PIN_R,
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_R,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER_0,
        .duty           = 0,
        .hpoint         = 0
    };
    ret = ledc_channel_config(&ledc_channel_r);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure RED channel: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Configure LEDC channel for GREEN
    ledc_channel_config_t ledc_channel_g = {
        .gpio_num       = LED_PIN_G,
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_G,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER_0,
        .duty           = 0,
        .hpoint         = 0
    };
    ret = ledc_channel_config(&ledc_channel_g);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GREEN channel: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Configure LEDC channel for BLUE
    ledc_channel_config_t ledc_channel_b = {
        .gpio_num       = LED_PIN_B,
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_B,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER_0,
        .duty           = 0,
        .hpoint         = 0
    };
    ret = ledc_channel_config(&ledc_channel_b);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure BLUE channel: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "LEDC Init Successful!");
    return ESP_OK;
}

void RGB::SetRed(uint32_t val, uint32_t valueMax) {
    LEDCAnalogWrite(LEDC_CHANNEL_R, val, valueMax);
    LEDCAnalogWrite(LEDC_CHANNEL_G, 0, 255);
    LEDCAnalogWrite(LEDC_CHANNEL_B, 0, 255);
}

void RGB::SetGreen(uint32_t val, uint32_t valueMax) {
    LEDCAnalogWrite(LEDC_CHANNEL_G, val, valueMax);
    LEDCAnalogWrite(LEDC_CHANNEL_R, 0, 255);
    LEDCAnalogWrite(LEDC_CHANNEL_B, 0, 255);
}

void RGB::SetBlue(uint32_t val, uint32_t valueMax) {
    LEDCAnalogWrite(LEDC_CHANNEL_B, val, valueMax);
    LEDCAnalogWrite(LEDC_CHANNEL_R, 0, 255);
    LEDCAnalogWrite(LEDC_CHANNEL_G, 0, 255);
}

void RGB::SetRGB(uint32_t R, uint32_t G, uint32_t B, uint32_t valueMax) {
    LEDCAnalogWrite(LEDC_CHANNEL_R, R, valueMax);
    LEDCAnalogWrite(LEDC_CHANNEL_G, G, valueMax);
    LEDCAnalogWrite(LEDC_CHANNEL_B, B, valueMax);
}

void RGB::LEDCAnalogWrite(ledc_channel_t channel, uint32_t value, uint32_t valueMax) {
    // Calculate duty cycle, 8191 from 2^13 - 1
    uint32_t duty = (8191 * std::min(value, valueMax)) / valueMax;
    
    // Set duty cycle
    esp_err_t ret = ledc_set_duty(LEDC_MODE, channel, duty);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set duty for channel %d: %s", channel, esp_err_to_name(ret));
        return;
    }
    
    // Update duty to apply the setting
    ret = ledc_update_duty(LEDC_MODE, channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update duty for channel %d: %s", channel, esp_err_to_name(ret));
    }
}