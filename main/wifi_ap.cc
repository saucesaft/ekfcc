// wifi_ap.cc - Simplified version (alternative)
#include "wifi_ap.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include <string.h>

const char* WiFi_AP::TAG = "WIFI_AP";

void WiFi_AP::wifi_event_handler(void* arg, esp_event_base_t event_base,
                                 int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        ESP_LOGI(TAG, "Station connected");
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        ESP_LOGI(TAG, "Station disconnected");
    }
}

esp_err_t WiFi_AP::Init() {
    ESP_LOGI(TAG, "Initializing WiFi Access Point...");
    
    // Initialize NVS (required for WiFi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Initialize network interface
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    // Create AP network interface
    esp_netif_t *ap = esp_netif_create_default_wifi_ap();

    // Configure static IP for the AP
    esp_netif_ip_info_t ip_info;
    inet_pton(AF_INET, AP_IP_ADDR, &ip_info.ip);
    inet_pton(AF_INET, AP_GATEWAY_ADDR, &ip_info.gw);
    inet_pton(AF_INET, AP_NETMASK_ADDR, &ip_info.netmask);
    // ESP_ERROR_CHECK(esp_netif_set_ip_info(netif, &ip_info));
    esp_netif_set_ip_info(ap, &ip_info);
    
    // Initialize WiFi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Register event handler
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    // Configure Access Point - initialize to zero first
    wifi_config_t wifi_config = {};
    
    // Set SSID
    strncpy((char*)wifi_config.ap.ssid, AP_WIFI_SSID, sizeof(wifi_config.ap.ssid) - 1);
    wifi_config.ap.ssid_len = strlen(AP_WIFI_SSID);
    
    // Set Password
    strncpy((char*)wifi_config.ap.password, AP_WIFI_PASS, sizeof(wifi_config.ap.password) - 1);
    
    // Set other parameters
    wifi_config.ap.channel = AP_WIFI_CHANNEL;
    wifi_config.ap.max_connection = AP_MAX_STA_CONN;
    wifi_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
    
    // Set to open if no password
    if (strlen(AP_WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi AP started: %s", AP_WIFI_SSID);
    return ESP_OK;
}

void WiFi_AP::GetIPInfo() {
    ESP_LOGI(TAG, "AP IP: %s", AP_IP_ADDR);
}