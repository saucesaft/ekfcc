// protocol.h
#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <string>
#include <map>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

// UART Configuration
#define TELLO_UART_NUM          UART_NUM_1
#define TELLO_UART_BAUD_RATE    1000000
#define TELLO_UART_TX_PIN       GPIO_NUM_18
#define TELLO_UART_RX_PIN       GPIO_NUM_23
#define TELLO_UART_BUF_SIZE     (1024)

// Button pin
#define RMTT_KEY_PIN            GPIO_NUM_34

class PROTOCOL {
private:
    static const char* TAG;
    uint16_t sdk_time;
    std::map<std::string, float> status_data;
    
    // Static variables for message tracking
    static uint8_t re_tag;
    static uint8_t re_cnt;
    
    void SendCMD(const char* cmd);
    void Split(const std::string& body, std::string data[], int len, char separator);
    void initStatus();
    int64_t millis_idf();
    
public:
    PROTOCOL();
    PROTOCOL(uint16_t time);
    ~PROTOCOL();
    
    esp_err_t Init();
    
    // Basic commands
    void SDKOn();
    void SDKOff();
    void TakeOff();
    void Land();
    void Emergency();
    void Stop();
    
    // Movement commands
    void Up(int16_t x);
    void Down(int16_t x);
    void Left(int16_t x);
    void Right(int16_t x);
    void Forward(int16_t x);
    void Back(int16_t x);
    void CW(uint16_t x);
    void CCW(uint16_t x);
    void Flip(char x);
    void Go(int16_t x, int16_t y, int16_t z, uint16_t speed);
    void Go(int16_t x, int16_t y, int16_t z, uint16_t speed, const char* mid);
    void Curve(int16_t x1, int16_t y1, int16_t z1, int16_t x2, int16_t y2, int16_t z2, uint16_t speed);
    void Curve(int16_t x1, int16_t y1, int16_t z1, int16_t x2, int16_t y2, int16_t z2, uint16_t speed, const char* mid);
    void Jump(int16_t x, int16_t y, int16_t z, uint16_t speed, int16_t yaw, const char* mid1, const char* mid2);
    
    // Configuration commands
    void SetSpeed(int16_t x);
    void SetRC(int16_t a, int16_t b, int16_t c, int16_t d);
    void SetMon();
    void SetMoff();
    void SetMdirection(uint8_t x);
    
    // Read commands
    void ReadSpeed();
    void ReadBattery();
    void ReadTime();
    void ReadSN();
    void ReadSDKVersion();
    
    // Communication functions
    std::string getTelloMsgString(const char* cmd, uint32_t timeout);
    int getTelloMsgInt(const char* cmd, uint32_t timeout);
    std::string getTelloResponseString(uint32_t timeout);
    int getTelloResponseInt(uint32_t timeout);
    void getTelloStatus(uint32_t timeout);
    int getTelloStatusWithNameInt(const char* name);
    float getTelloStatusWithNameFloat(const char* name);
    int sendTelloCtrlMsg(const char* cmd_str);
    
    // Control function (requires RGB LED integration)
    void startUntilControl();
};

#endif // PROTOCOL_H