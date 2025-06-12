idf_component_register(SRCS "rgb.cc" "protocol.cc" "embedded.cc" "wifi_ap.cc"
                    PRIV_REQUIRES spi_flash
                    REQUIRES
                        driver
                        esp_system
                        freertos
                        esp_common
                        esp_timer
                        esp_wifi
                        esp_netif
                        esp_event
                        lwip
                        nvs_flash
                        micro_ros_espidf_component
                    INCLUDE_DIRS "."
                    )
