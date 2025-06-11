// RGB.h
#ifndef RGB_H
#define RGB_H

#include "driver/ledc.h"
#include "esp_log.h"

// Pin definitions (adjust for your hardware)
#define LED_PIN_R    GPIO_NUM_32
#define LED_PIN_G    GPIO_NUM_33
#define LED_PIN_B    GPIO_NUM_25

// LEDC configuration
#define LEDC_TIMER_RESOLUTION   LEDC_TIMER_13_BIT
#define LEDC_BASE_FREQ          5000
#define LEDC_MODE               LEDC_LOW_SPEED_MODE

// Channel assignments
#define LEDC_CHANNEL_R          LEDC_CHANNEL_0
#define LEDC_CHANNEL_G          LEDC_CHANNEL_1
#define LEDC_CHANNEL_B          LEDC_CHANNEL_2

class RGB {
private:
    static const char* TAG;
    void LEDCAnalogWrite(ledc_channel_t channel, uint32_t value, uint32_t valueMax);
    
public:
    esp_err_t Init(void);
    void SetRed(uint32_t val, uint32_t valueMax = 255);
    void SetGreen(uint32_t val, uint32_t valueMax = 255);
    void SetBlue(uint32_t val, uint32_t valueMax = 255);
    void SetRGB(uint32_t R, uint32_t G, uint32_t B, uint32_t valueMax = 255);
};

#endif // RGB_H