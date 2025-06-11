// protocol.cc
#include "protocol.h"
#include "rgb.h"
#include <cstring>
#include <sstream>

const char* PROTOCOL::TAG = "RMTT_PROTOCOL";
uint8_t PROTOCOL::re_tag = 0;
uint8_t PROTOCOL::re_cnt = 0;

PROTOCOL::PROTOCOL() : sdk_time(0) {
    initStatus();
}

PROTOCOL::PROTOCOL(uint16_t time) : sdk_time(time) {
    initStatus();
}

PROTOCOL::~PROTOCOL() {
    uart_driver_delete(TELLO_UART_NUM);
}

esp_err_t PROTOCOL::Init() {
    // Configure UART parameters
    uart_config_t uart_config = {
        .baud_rate = TELLO_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    // Install UART driver
    esp_err_t ret = uart_driver_install(TELLO_UART_NUM, TELLO_UART_BUF_SIZE * 2, 0, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install UART driver: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = uart_param_config(TELLO_UART_NUM, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure UART: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = uart_set_pin(TELLO_UART_NUM, TELLO_UART_TX_PIN, TELLO_UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set UART pins: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Configure button pin
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << RMTT_KEY_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure button GPIO: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "UART initialized successfully at %d baud", TELLO_UART_BAUD_RATE);
    return ESP_OK;
}

int64_t PROTOCOL::millis_idf() {
    return esp_timer_get_time() / 1000;
}

void PROTOCOL::initStatus() {
    status_data["mid"] = -2;
    status_data["x"] = -200;
    status_data["y"] = -200;
    status_data["z"] = -200;
    status_data["mpryP"] = 0;
    status_data["mpryY"] = 0;
    status_data["mpryR"] = 0;
    status_data["pitch"] = 0;
    status_data["roll"] = 0;
    status_data["yaw"] = 0;
    status_data["vgx"] = 0;
    status_data["vgy"] = 0;
    status_data["vgz"] = 0;
    status_data["templ"] = 0;
    status_data["temph"] = 0;
    status_data["tof"] = 0;
    status_data["h"] = 0;
    status_data["bat"] = 0;
    status_data["baro"] = 0;
    status_data["time"] = 0;
    status_data["agx"] = 0;
    status_data["agy"] = 0;
    status_data["agz"] = 0;
}

void PROTOCOL::Split(const std::string& body, std::string data[], int len, char separator) {
    std::stringstream ss(body);
    std::string item;
    int i = 0;
    
    while (std::getline(ss, item, separator) && i < len) {
        data[i++] = item;
    }
}

void PROTOCOL::SendCMD(const char* cmd) {
    std::string full_cmd = "[TELLO] " + std::string(cmd);
    uart_write_bytes(TELLO_UART_NUM, full_cmd.c_str(), full_cmd.length());
    ESP_LOGI(TAG, "Sent: %s", full_cmd.c_str());
    
    if (sdk_time) {
        vTaskDelay(pdMS_TO_TICKS(sdk_time));
    }
}

// Basic commands
void PROTOCOL::SDKOn() { SendCMD("command"); }
void PROTOCOL::SDKOff() { }
void PROTOCOL::TakeOff() { SendCMD("takeoff"); }
void PROTOCOL::Land() { SendCMD("land"); }
void PROTOCOL::Emergency() { SendCMD("emergency"); }
void PROTOCOL::Stop() { SendCMD("stop"); }

// Movement commands
void PROTOCOL::Up(int16_t x) {
    char cmd[20];
    snprintf(cmd, sizeof(cmd), "up %d", x);
    SendCMD(cmd);
}

void PROTOCOL::Down(int16_t x) {
    char cmd[20];
    snprintf(cmd, sizeof(cmd), "down %d", x);
    SendCMD(cmd);
}

void PROTOCOL::Left(int16_t x) {
    char cmd[20];
    snprintf(cmd, sizeof(cmd), "left %d", x);
    SendCMD(cmd);
}

void PROTOCOL::Right(int16_t x) {
    char cmd[20];
    snprintf(cmd, sizeof(cmd), "right %d", x);
    SendCMD(cmd);
}

void PROTOCOL::Forward(int16_t x) {
    char cmd[20];
    snprintf(cmd, sizeof(cmd), "forward %d", x);
    SendCMD(cmd);
}

void PROTOCOL::Back(int16_t x) {
    char cmd[20];
    snprintf(cmd, sizeof(cmd), "back %d", x);
    SendCMD(cmd);
}

void PROTOCOL::CW(uint16_t x) {
    char cmd[20];
    snprintf(cmd, sizeof(cmd), "cw %d", x);
    SendCMD(cmd);
}

void PROTOCOL::CCW(uint16_t x) {
    char cmd[20];
    snprintf(cmd, sizeof(cmd), "ccw %d", x);
    SendCMD(cmd);
}

void PROTOCOL::Flip(char x) {
    char cmd[20];
    snprintf(cmd, sizeof(cmd), "flip %c", x);
    SendCMD(cmd);
}

void PROTOCOL::Go(int16_t x, int16_t y, int16_t z, uint16_t speed) {
    char cmd[40];
    snprintf(cmd, sizeof(cmd), "go %d %d %d %d", x, y, z, speed);
    SendCMD(cmd);
}

void PROTOCOL::Go(int16_t x, int16_t y, int16_t z, uint16_t speed, const char* mid) {
    char cmd[40];
    snprintf(cmd, sizeof(cmd), "go %d %d %d %d %s", x, y, z, speed, mid);
    SendCMD(cmd);
}

void PROTOCOL::Curve(int16_t x1, int16_t y1, int16_t z1, int16_t x2, int16_t y2, int16_t z2, uint16_t speed) {
    char cmd[60];
    snprintf(cmd, sizeof(cmd), "curve %d %d %d %d %d %d %d", x1, y1, z1, x2, y2, z2, speed);
    SendCMD(cmd);
}

void PROTOCOL::Curve(int16_t x1, int16_t y1, int16_t z1, int16_t x2, int16_t y2, int16_t z2, uint16_t speed, const char* mid) {
    char cmd[60];
    snprintf(cmd, sizeof(cmd), "curve %d %d %d %d %d %d %d %s", x1, y1, z1, x2, y2, z2, speed, mid);
    SendCMD(cmd);
}

void PROTOCOL::Jump(int16_t x, int16_t y, int16_t z, uint16_t speed, int16_t yaw, const char* mid1, const char* mid2) {
    char cmd[60];
    snprintf(cmd, sizeof(cmd), "jump %d %d %d %d %d %s %s", x, y, z, speed, yaw, mid1, mid2);
    SendCMD(cmd);
}

// Configuration commands
void PROTOCOL::SetSpeed(int16_t x) {
    char cmd[20];
    snprintf(cmd, sizeof(cmd), "speed %d", x);
    SendCMD(cmd);
}

void PROTOCOL::SetRC(int16_t a, int16_t b, int16_t c, int16_t d) {
    char cmd[40];
    snprintf(cmd, sizeof(cmd), "rc %d %d %d %d", a, b, c, d);
    SendCMD(cmd);
}

void PROTOCOL::SetMon() { SendCMD("mon"); }
void PROTOCOL::SetMoff() { SendCMD("moff"); }

void PROTOCOL::SetMdirection(uint8_t x) {
    char cmd[20];
    snprintf(cmd, sizeof(cmd), "mdirection %d", x);
    SendCMD(cmd);
}

// Read commands
void PROTOCOL::ReadSpeed() { SendCMD("speed?"); }
void PROTOCOL::ReadBattery() { SendCMD("battery?"); }
void PROTOCOL::ReadTime() { SendCMD("time?"); }
void PROTOCOL::ReadSN() { SendCMD("sn?"); }
void PROTOCOL::ReadSDKVersion() { SendCMD("sdk?"); }

std::string PROTOCOL::getTelloMsgString(const char* cmd, uint32_t timeout) {
    // Clear UART buffer
    uart_flush(TELLO_UART_NUM);
    
    // Send command
    uart_write_bytes(TELLO_UART_NUM, cmd, strlen(cmd));
    
    int64_t start_time = millis_idf();
    std::string response;
    uint8_t data[128];
    
    while ((millis_idf() - start_time) < timeout) {
        int len = uart_read_bytes(TELLO_UART_NUM, data, sizeof(data) - 1, pdMS_TO_TICKS(10));
        if (len > 0) {
            data[len] = '\0';
            response += reinterpret_cast<char*>(data);
            
            // Check if we have a complete response
            if (response.find("\r\n") != std::string::npos) {
                break;
            }
        }
    }
    
    if (response.empty()) {
        return "timeout";
    }
    
    // Remove \r\n if present
    size_t pos = response.find("\r\n");
    if (pos != std::string::npos) {
        response = response.substr(0, pos);
    }
    
    return response;
}

int PROTOCOL::getTelloMsgInt(const char* cmd, uint32_t timeout) {
    std::string response = getTelloMsgString(cmd, timeout);
    if (response == "timeout") {
        return -1;
    }
    
    // Extract integer from response
    size_t space_pos = response.find(' ');
    if (space_pos != std::string::npos) {
        return std::stoi(response.substr(space_pos + 1));
    }
    
    return -1;
}

std::string PROTOCOL::getTelloResponseString(uint32_t timeout) {
    int64_t start_time = millis_idf();
    std::string response;
    uint8_t data[128];
    
    while ((millis_idf() - start_time) < timeout) {
        int len = uart_read_bytes(TELLO_UART_NUM, data, sizeof(data) - 1, pdMS_TO_TICKS(10));
        if (len > 0) {
            data[len] = '\0';
            response += reinterpret_cast<char*>(data);
            
            if (response.find("\r\n") != std::string::npos) {
                break;
            }
        }
    }
    
    if (response.empty()) {
        return "timeout";
    }
    
    size_t pos = response.find("\r\n");
    if (pos != std::string::npos) {
        response = response.substr(0, pos);
    }
    
    return response;
}

int PROTOCOL::getTelloResponseInt(uint32_t timeout) {
    std::string response = getTelloResponseString(timeout);
    if (response == "timeout") {
        return -1;
    }
    
    size_t space_pos = response.find(' ');
    if (space_pos != std::string::npos) {
        return std::stoi(response.substr(space_pos + 1));
    }
    
    return -1;
}

int PROTOCOL::getTelloStatusWithNameInt(const char* name) {
    auto it = status_data.find(name);
    if (it != status_data.end()) {
        return static_cast<int>(it->second);
    }
    return 0;
}

float PROTOCOL::getTelloStatusWithNameFloat(const char* name) {
    auto it = status_data.find(name);
    if (it != status_data.end()) {
        return it->second;
    }
    return 0.0f;
}

void PROTOCOL::getTelloStatus(uint32_t timeout) {
    std::string response = getTelloMsgString("[TELLO] status?", timeout);
    
    // Remove "ETT " prefix if present
    if (response.substr(0, 4) == "ETT ") {
        response = response.substr(4);
    }
    
    if (response != "timeout") {
        std::string data[21];
        Split(response, data, 21, ';');
        
        for (int i = 0; i < 21; i++) {
            if (data[i].empty()) continue;
            
            size_t colon_pos = data[i].find(':');
            if (colon_pos != std::string::npos) {
                std::string name = data[i].substr(0, colon_pos);
                std::string value = data[i].substr(colon_pos + 1);
                
                if (name == "baro" || name == "agx" || name == "agy" || name == "agz") {
                    status_data[name] = std::stof(value);
                } else if (name == "mpry") {
                    std::string mpry[3];
                    Split(value, mpry, 3, ',');
                    status_data["mpryP"] = std::stoi(mpry[0]);
                    status_data["mpryY"] = std::stoi(mpry[1]);
                    status_data["mpryR"] = std::stoi(mpry[2]);
                } else {
                    status_data[name] = std::stoi(value);
                }
            }
        }
    } else {
        initStatus();
    }
}

int PROTOCOL::sendTelloCtrlMsg(const char* cmd_str) {
    re_cnt = 0;
    
    while (true) {
        // Clear UART buffer
        uart_flush(TELLO_UART_NUM);
        
        // Send command with retry tag
        char full_cmd[256];
        snprintf(full_cmd, sizeof(full_cmd), "[TELLO] Re%02x%02x %s", re_tag, re_cnt++, cmd_str);
        uart_write_bytes(TELLO_UART_NUM, full_cmd, strlen(full_cmd));
        
        int64_t start_time = millis_idf();
        std::string response;
        uint8_t data[128];
        
        while ((millis_idf() - start_time) < 1000) {
            int len = uart_read_bytes(TELLO_UART_NUM, data, sizeof(data) - 1, pdMS_TO_TICKS(10));
            if (len > 0) {
                data[len] = '\0';
                response += reinterpret_cast<char*>(data);
                
                if (response.find("\r\n") != std::string::npos) {
                    break;
                }
            }
            
            // Resend if timeout
            if ((millis_idf() - start_time) >= 1000) {
                snprintf(full_cmd, sizeof(full_cmd), "[TELLO] Re%02x%02x %s", re_tag, re_cnt++, cmd_str);
                uart_write_bytes(TELLO_UART_NUM, full_cmd, strlen(full_cmd));
                start_time = millis_idf();
            }
        }
        
        // Check for successful response (ETT Rexxxx ok)
        if (response.length() >= 12) {
            if (response.find("ok") != std::string::npos) {
                break;
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
    
    re_tag++;
    return 0;
}


// TODO lights when ready to control
void PROTOCOL::startUntilControl() {
    // This function requires RGB LED integration
    // You would need to include RMTT_RGB_IDF instance here
    ESP_LOGI(TAG, "Starting control sequence...");
    
    // Wait for SDK command response
    while (getTelloMsgString("[TELLO] command", 1000) != "ETT ok") {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    // Wait for button press
    while (gpio_get_level(RMTT_KEY_PIN) != 0) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    ESP_LOGI(TAG, "Control sequence complete");
}