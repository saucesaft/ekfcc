// embedded.cc - Main with WiFi Access Point and cmd_vel control
#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "rgb.h"
#include "protocol.h"
#include "wifi_ap.h"

// micro-ROS includes
#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <geometry_msgs/msg/twist.h>

#include <rclc/rclc.h>
#include <rclc/executor.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

// Define missing constants
// #define RMTT_KEY_PIN GPIO_NUM_0  // Boot button on most ESP32 boards
// #define AP_WIFI_SSID "TelloController"
// #define AP_WIFI_PASS "tello123"

static const char *TAG = "MAIN";

// Hardware objects
RGB rgb_led;
PROTOCOL tello_protocol;
WiFi_AP wifi_ap;

// Flight control variables
bool flight_ready = false;
bool flight_in_progress = false;
bool landed = true;  // Start landed
bool button_flight_mode = true;  // Enable button-controlled flight

// Command timeout for auto-landing (15 seconds like Python version)
const int64_t CMD_TIMEOUT_US = 15 * 1000000;  // 15 seconds in microseconds
int64_t last_cmd_time = 0;

// Latest command storage (thread-safe with mutex protection)
typedef struct {
    float linear_x;   // forward/back
    float linear_y;   // left/right  
    float linear_z;   // up/down
    float angular_z;  // yaw
} cmd_vel_t;

cmd_vel_t latest_cmd = {0, 0, 0, 0};
portMUX_TYPE cmd_mutex = portMUX_INITIALIZER_UNLOCKED;

// micro-ROS objects
rcl_publisher_t sensors_publisher;
rcl_subscription_t cmd_vel_subscription;
std_msgs__msg__Float32MultiArray sensors_msg;
geometry_msgs__msg__Twist cmd_vel_msg;

// micro-ROS macros
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ESP_LOGE(TAG, "Failed status on line %d: %d", __LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ESP_LOGE(TAG, "Failed status on line %d: %d", __LINE__,(int)temp_rc);}}

void init_sensors_message() {
    // Initialize combined sensors array (gyro + velocity + accel = 9 elements)
    sensors_msg.data.capacity = 9;
    sensors_msg.data.size = 9;
    sensors_msg.data.data = (float*)malloc(9 * sizeof(float));
    
    // Initialize all values to 0
    for (int i = 0; i < 9; i++) {
        sensors_msg.data.data[i] = 0.0f;
    }
}

// Convert float to RC value (-100 to 100 range) like Python version
int to_rc(float v) {
    // Scale to RC range and apply floor/ceil like Python
    float scaled = v * 100.0f;  // Adjust scaling as needed
    if (scaled < 0) {
        return (int)floor(scaled);
    } else {
        return (int)ceil(scaled);
    }
}

// cmd_vel callback - stores latest command and updates timestamp
void cmd_vel_callback(const void * msgin) {
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
    
    // Update latest command with mutex protection
    portENTER_CRITICAL(&cmd_mutex);
    latest_cmd.linear_x = msg->linear.x;
    latest_cmd.linear_y = msg->linear.y;
    latest_cmd.linear_z = msg->linear.z;
    latest_cmd.angular_z = msg->angular.z;
    portEXIT_CRITICAL(&cmd_mutex);
    
    // Reset timeout
    last_cmd_time = esp_timer_get_time();
    
    // If landed and we receive a command, take off again (only if not in button mode)
    if (landed && flight_ready && !button_flight_mode) {
        ESP_LOGI(TAG, "Received cmd_vel while landed - taking off");
        tello_protocol.TakeOff();
        landed = false;
        vTaskDelay(pdMS_TO_TICKS(100));  // Brief delay for takeoff command
    }
    
    ESP_LOGD(TAG, "cmd_vel: [%.2f, %.2f, %.2f] yaw: %.2f", 
             msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.z);
}

// High-frequency timer for sensor publishing (20Hz)
void sensor_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        // Get Tello status (non-blocking call)
        tello_protocol.getTelloStatus(50);  // Very short timeout
        
        // Get gyro data (roll, pitch, yaw) - indices 0,1,2
        float roll = tello_protocol.getTelloStatusWithNameFloat("roll");
        float pitch = tello_protocol.getTelloStatusWithNameFloat("pitch");
        float yaw = tello_protocol.getTelloStatusWithNameFloat("yaw");
        
        // Get velocity data (vgx, vgy, vgz) - indices 3,4,5
        float vgx = tello_protocol.getTelloStatusWithNameFloat("vgx");
        float vgy = tello_protocol.getTelloStatusWithNameFloat("vgy");
        float vgz = tello_protocol.getTelloStatusWithNameFloat("vgz");
        
        // Get accel data (agx, agy, agz) - indices 6,7,8
        float agx = tello_protocol.getTelloStatusWithNameFloat("agx");
        float agy = tello_protocol.getTelloStatusWithNameFloat("agy");
        float agz = tello_protocol.getTelloStatusWithNameFloat("agz");
        
        // Populate combined sensors array: [gyro(3), vel(3), accel(3)]
        sensors_msg.data.data[0] = roll;   // Gyro
        sensors_msg.data.data[1] = pitch;
        sensors_msg.data.data[2] = yaw;
        sensors_msg.data.data[3] = vgx;    // Velocity
        sensors_msg.data.data[4] = vgy;
        sensors_msg.data.data[5] = vgz;
        sensors_msg.data.data[6] = agx;    // Acceleration
        sensors_msg.data.data[7] = agy;
        sensors_msg.data.data[8] = agz;
        
        // Publish combined sensors message
        RCSOFTCHECK(rcl_publish(&sensors_publisher, &sensors_msg, NULL));
        
        // LED indication based on flight state
        if (landed) {
            rgb_led.SetRGB(255, 0, 0);  // Red when landed
        } else if (flight_in_progress) {
            rgb_led.SetRGB(255, 0, 255);  // Magenta during button flight
        } else if (flight_ready) {
            rgb_led.SetRGB(0, 255, 0);  // Green when ready for cmd_vel
        } else {
            rgb_led.SetRGB(255, 255, 0);  // Yellow during initialization
        }
    }
}

// High-frequency timer for cmd_vel control (30Hz like Python)
void cmd_vel_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL && flight_ready) {
        int64_t current_time = esp_timer_get_time();
        
        // Check for timeout and auto-land if needed
        if ((current_time - last_cmd_time) > CMD_TIMEOUT_US && !landed) {
            ESP_LOGW(TAG, "No cmd_vel for >15s, auto-landing");
            tello_protocol.Land();
            landed = true;
            return;
        }
        
        // Skip sending commands if landed (unless button flight mode allows it)
        if (landed && button_flight_mode) {
            return;
        }
        
        // Read latest command with mutex protection
        cmd_vel_t cmd;
        portENTER_CRITICAL(&cmd_mutex);
        cmd = latest_cmd;
        portEXIT_CRITICAL(&cmd_mutex);
        
        // Convert to RC values (-100 to 100)
        int lr = to_rc(cmd.linear_y);    // left/right
        int fb = to_rc(cmd.linear_x);    // forward/back  
        int ud = to_rc(cmd.linear_z);    // up/down
        int yaw = to_rc(cmd.angular_z);  // yaw rotation
        
        // Clamp to valid RC range
        lr = (lr > 100) ? 100 : (lr < -100) ? -100 : lr;
        fb = (fb > 100) ? 100 : (fb < -100) ? -100 : fb;
        ud = (ud > 100) ? 100 : (ud < -100) ? -100 : ud;
        yaw = (yaw > 100) ? 100 : (yaw < -100) ? -100 : yaw;
        
        // Send RC control command
        tello_protocol.SetRC(lr, fb, ud, yaw);
        
        ESP_LOGV(TAG, "RC: lr=%d fb=%d ud=%d yaw=%d", lr, fb, ud, yaw);
    }
}

void button_flight_task(void * arg)
{
    ESP_LOGI(TAG, "Button flight control task started");
    
    // Configure button pin
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << RMTT_KEY_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);
    
    // Wait for flight system to be ready
    while (!flight_ready) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    ESP_LOGI(TAG, "Button flight control ready! Press button to toggle flight mode.");
    
    while (1) {
        // Check for button press (button is active low)
        if (gpio_get_level(RMTT_KEY_PIN) == 0) {
            // Debounce
            vTaskDelay(pdMS_TO_TICKS(200));
            
            if (button_flight_mode) {
                // Switch to cmd_vel mode
                if (flight_in_progress) {
                    ESP_LOGI(TAG, "Landing and switching to cmd_vel control mode...");
                    tello_protocol.Land();
                    flight_in_progress = false;
                    landed = true;
                    vTaskDelay(pdMS_TO_TICKS(3000));  // Wait for landing
                }
                
                button_flight_mode = false;
                ESP_LOGI(TAG, "🎮 CMD_VEL CONTROL MODE - Send /cmd_vel messages to control drone");
                ESP_LOGI(TAG, "Press button again to return to button flight mode");
                
            } else {
                // Switch to button flight mode
                ESP_LOGI(TAG, "Switching to button flight mode...");
                button_flight_mode = true;
                
                if (!landed) {
                    ESP_LOGI(TAG, "Landing to reset for button flight...");
                    tello_protocol.Land();
                    landed = true;
                    vTaskDelay(pdMS_TO_TICKS(3000));
                }
                
                ESP_LOGI(TAG, "🔘 BUTTON FLIGHT MODE - Press button to start square pattern");
            }
            
            // Wait for button release
            while (gpio_get_level(RMTT_KEY_PIN) == 0) {
                vTaskDelay(pdMS_TO_TICKS(50));
            }
            vTaskDelay(pdMS_TO_TICKS(200));  // Additional debounce
            
        } else if (button_flight_mode && !flight_in_progress && landed) {
            // In button mode, waiting for flight command
            vTaskDelay(pdMS_TO_TICKS(100));
            
        } else if (button_flight_mode && gpio_get_level(RMTT_KEY_PIN) == 0) {
            // Button pressed in button flight mode - start square pattern
            ESP_LOGI(TAG, "Button pressed! Starting square flight pattern...");
            flight_in_progress = true;
            
            // Debounce
            vTaskDelay(pdMS_TO_TICKS(500));
            
            // Take off
            ESP_LOGI(TAG, "Taking off...");
            tello_protocol.TakeOff();
            landed = false;
            
            // Wait for takeoff with shorter delays to keep publishing active
            for (int i = 0; i < 50; i++) {  // 5 seconds total (50 * 100ms)
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            
            // Square pattern: Forward -> Right -> Back -> Left
            const int square_distance = 100;  // 100cm sides (smaller for indoor use)
            
            // Move forward
            ESP_LOGI(TAG, "Moving forward %dcm...", square_distance);
            tello_protocol.Forward(square_distance);
            for (int i = 0; i < 30; i++) {  // 3 seconds total
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            
            // Move right
            ESP_LOGI(TAG, "Moving right %dcm...", square_distance);
            tello_protocol.Right(square_distance);
            for (int i = 0; i < 30; i++) {  // 3 seconds total
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            
            // Move back
            ESP_LOGI(TAG, "Moving back %dcm...", square_distance);
            tello_protocol.Back(square_distance);
            for (int i = 0; i < 30; i++) {  // 3 seconds total
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            
            // Move left (back to start)
            ESP_LOGI(TAG, "Moving left %dcm...", square_distance);
            tello_protocol.Left(square_distance);
            for (int i = 0; i < 30; i++) {  // 3 seconds total
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            
            // Land
            ESP_LOGI(TAG, "Landing...");
            tello_protocol.Land();
            for (int i = 0; i < 50; i++) {  // 5 seconds total
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            
            flight_in_progress = false;
            landed = true;
            ESP_LOGI(TAG, "Square pattern complete! Press button again for another flight.");
        
        } else {
            vTaskDelay(pdMS_TO_TICKS(100));  // Check button every 100ms
        }
    }
}

void flight_initialization_task(void * arg)
{
    ESP_LOGI(TAG, "Flight initialization task started");
    
    // Wait for micro-ROS to be ready
    vTaskDelay(pdMS_TO_TICKS(10000));
    
    // Initialize Tello SDK
    ESP_LOGI(TAG, "Initializing Tello SDK...");
    tello_protocol.SDKOn();
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    // Enable motor monitoring for better control
    tello_protocol.SetMon();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    flight_ready = true;
    last_cmd_time = esp_timer_get_time();  // Initialize timeout
    
    ESP_LOGI(TAG, "🚁 Tello ready for dual control mode!");
    ESP_LOGI(TAG, "📋 MODES:");
    ESP_LOGI(TAG, "  🔘 Button Mode: Press button for square flight pattern");
    ESP_LOGI(TAG, "  🎮 cmd_vel Mode: Send ROS messages for manual control");
    ESP_LOGI(TAG, "Press button to toggle between modes");
    
    // Start button flight control task
    xTaskCreate(button_flight_task,
            "button_flight_task",
            8192,
            NULL,
            3,      // Lower priority than micro-ROS
            NULL);
    
    // Task complete - delete self
    vTaskDelete(NULL);
}

void micro_ros_task(void * arg)
{
    ESP_LOGI(TAG, "Starting micro-ROS task...");
    
    // Wait for WiFi AP to be stable
    rgb_led.SetRGB(255, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(15000));
    rgb_led.SetRGB(0, 255, 0);

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

    ESP_LOGI(TAG, "Attempting to discover micro-ROS agent...");
    
    // Try discovery with timeout and retries
    rcl_ret_t discovery_ret = RCL_RET_ERROR;
    int discovery_attempts = 0;
    const int max_attempts = 5;
    
    while (discovery_ret != RCL_RET_OK && discovery_attempts < max_attempts) {
        discovery_attempts++;
        ESP_LOGI(TAG, "Discovery attempt %d/%d", discovery_attempts, max_attempts);
        
        discovery_ret = rmw_uros_discover_agent(rmw_options);
        if (discovery_ret != RCL_RET_OK) {
            ESP_LOGW(TAG, "Discovery attempt %d failed, retrying in 2 seconds...", discovery_attempts);
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    }
    
    if (discovery_ret != RCL_RET_OK) {
        ESP_LOGE(TAG, "All discovery attempts failed. Falling back to manual IP detection...");
        
        // Fallback: try common DHCP IPs
        const char* fallback_ips[] = {"192.168.4.2", "192.168.4.3", "192.168.4.100", "192.168.4.101"};
        bool connected = false;
        
        for (int i = 0; i < 4; i++) {
            ESP_LOGI(TAG, "Trying to connect to agent at %s:8888", fallback_ips[i]);
            rcl_ret_t ret = rmw_uros_options_set_udp_address(fallback_ips[i], "8888", rmw_options);
            if (ret == RCL_RET_OK) {
                ESP_LOGI(TAG, "Configured to try agent at %s:8888", fallback_ips[i]);
                connected = true;
                break;
            }
        }
        
        if (!connected) {
            ESP_LOGE(TAG, "Failed to configure any fallback IP");
            vTaskDelete(NULL);
            return;
        }
    } else {
        ESP_LOGI(TAG, "micro-ROS agent discovered successfully!");
    }
#endif

    // create init_options
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    // create node
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "tello_cmdvel_bridge", "", &support));

    // Create sensors publisher
    RCCHECK(rclc_publisher_init_default(
        &sensors_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "tello/sensors"));

    // Create cmd_vel subscriber
    RCCHECK(rclc_subscription_init_default(
        &cmd_vel_subscription,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"));

    // Initialize messages
    init_sensors_message();

    // Create timers
    rcl_timer_t sensor_timer;
    const unsigned int sensor_timer_timeout = 50;  // 50ms = 20Hz for sensors
    RCCHECK(rclc_timer_init_default(
        &sensor_timer,
        &support,
        RCL_MS_TO_NS(sensor_timer_timeout),
        sensor_timer_callback));

    rcl_timer_t cmd_timer;
    const unsigned int cmd_timer_timeout = 33;  // 33ms ≈ 30Hz for control
    RCCHECK(rclc_timer_init_default(
        &cmd_timer,
        &support,
        RCL_MS_TO_NS(cmd_timer_timeout),
        cmd_vel_timer_callback));

    // create executor with capacity for subscription + 2 timers
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_subscription, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &sensor_timer));
    RCCHECK(rclc_executor_add_timer(&executor, &cmd_timer));

    ESP_LOGI(TAG, "micro-ROS initialized successfully!");
    ESP_LOGI(TAG, "Subscribing to /cmd_vel for drone control");
    ESP_LOGI(TAG, "Publishing sensors on /tello/sensors");
    ESP_LOGI(TAG, "Data format: [roll, pitch, yaw, vgx, vgy, vgz, agx, agy, agz]");
    
    // Signal successful initialization
    rgb_led.SetRGB(0, 255, 0);  // Green
    vTaskDelay(pdMS_TO_TICKS(1000));
    rgb_led.SetRGB(0, 0, 0);    // Off

    // Start flight initialization task
    xTaskCreate(flight_initialization_task,
            "flight_init_task",
            4096,
            NULL,
            4,
            NULL);

    // Main executor loop - very tight loop for responsiveness
    while(1){
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));  // 1ms spin timeout
        // No additional delay - keep it as responsive as possible
    }

    // Cleanup (unreachable in normal operation)
    RCCHECK(rcl_publisher_fini(&sensors_publisher, &node));
    RCCHECK(rcl_subscription_fini(&cmd_vel_subscription, &node));
    
    if (sensors_msg.data.data != NULL) free(sensors_msg.data.data);
    
    RCCHECK(rcl_node_fini(&node));

    vTaskDelete(NULL);
}

extern "C" void app_main(void)
{
    printf("RMTT Protocol with micro-ROS cmd_vel Control Demo Starting!\n");

    /* Print chip information */
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), %s%s%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
           (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
           (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    printf("silicon revision v%d.%d, ", major_rev, minor_rev);
    if(esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        printf("Get flash size failed");
        return;
    }

    printf("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());

    // Initialize RGB LED
    ESP_LOGI(TAG, "Initializing RGB LED...");
    esp_err_t ret = rgb_led.Init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize RGB LED: %s", esp_err_to_name(ret));
        return;
    }

    // Initialize UART for Tello communication
    ESP_LOGI(TAG, "Initializing Tello Protocol (UART at 1000000 baud, TX=18, RX=23)...");
    ret = tello_protocol.Init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize Tello Protocol: %s", esp_err_to_name(ret));
        return;
    }

    // Initialize WiFi Access Point
    ESP_LOGI(TAG, "Starting WiFi Access Point...");
    ret = wifi_ap.Init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize WiFi AP: %s", esp_err_to_name(ret));
        return;
    }

    // Startup LED sequence
    ESP_LOGI(TAG, "System initialized successfully!");
    rgb_led.SetRGB(255, 0, 0);  // Red
    vTaskDelay(pdMS_TO_TICKS(500));
    rgb_led.SetRGB(0, 255, 0);  // Green
    vTaskDelay(pdMS_TO_TICKS(500));
    rgb_led.SetRGB(0, 0, 255);  // Blue
    vTaskDelay(pdMS_TO_TICKS(500));
    rgb_led.SetRGB(0, 0, 0);    // Off

    // Show network information
    wifi_ap.GetIPInfo();

    // Create micro-ROS task with highest priority for maximum responsiveness
    xTaskCreate(micro_ros_task,
            "uros_task",
            16384,
            NULL,
            7,      // Highest priority for micro-ROS
            NULL);
    
    ESP_LOGI(TAG, "Application started successfully!");
    ESP_LOGI(TAG, "Connect to WiFi: %s (password: %s)", AP_WIFI_SSID, AP_WIFI_PASS);
    ESP_LOGI(TAG, "Then run: ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "🎮 DUAL CONTROL MODE:");
    ESP_LOGI(TAG, "  🔘 Button Mode: Press button for autonomous square pattern");
    ESP_LOGI(TAG, "  🎮 cmd_vel Mode: Manual control via ROS messages");
    ESP_LOGI(TAG, "  📱 Toggle: Press button to switch between modes");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Auto-landing after 15s without cmd_vel messages (cmd_vel mode only)");

    // Main loop - heartbeat LED every 10 seconds
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        ESP_LOGI(TAG, "System heartbeat - AP running, Flight ready: %s, Mode: %s, In-flight: %s, Landed: %s", 
                 flight_ready ? "Yes" : "No", 
                 button_flight_mode ? "Button" : "cmd_vel",
                 flight_in_progress ? "Yes" : "No",
                 landed ? "Yes" : "No");
    }
}