// wifi_ap.h - Fixed WiFi Access Point Configuration
#ifndef WIFI_AP_H
#define WIFI_AP_H

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "nvs_flash.h"

// Access Point Configuration
#define AP_WIFI_SSID      "Tello-Bridge"
#define AP_WIFI_PASS      "tellobridge123"
#define AP_WIFI_CHANNEL   1
#define AP_MAX_STA_CONN   4

// IP Configuration
#define AP_IP_ADDR        "192.168.4.1"
#define AP_GATEWAY_ADDR   "192.168.4.1"
#define AP_NETMASK_ADDR   "255.255.255.0"

class WiFi_AP {
private:
    static const char* TAG;
    static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                   int32_t event_id, void* event_data);
    
public:
    esp_err_t Init();
    void GetIPInfo();
};

#endif // WIFI_AP_H