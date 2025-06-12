// embedded_barometer.cc - EKF with Barometer Integration and Different Update Rates
#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <assert.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/inet.h"
#include "lwip/ip_addr.h"

#include "rgb.h"
#include "protocol.h"
#include "config.h"

// TinyEKF includes - MUST come after config.h
#include "tinyekf.h"
#include "tinyekf_custom.h"

// micro-ROS includes
#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/pose_stamped.h>
#include <builtin_interfaces/msg/time.h>

#include <rclc/rclc.h>
#include <rclc/executor.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

// Network Configuration
#define WIFI_SSID      "Lab_Robotica"
#define WIFI_PASS      "coke.fanta.sprite"

// Static IP Configuration  
#define STATIC_IP_ADDR      "10.15.232.33"
#define STATIC_GATEWAY_ADDR "10.15.232.1"
#define STATIC_NETMASK_ADDR "255.255.255.128"  // /25 subnet

// WiFi event handling
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
static int s_retry_num = 0;
#define WIFI_MAXIMUM_RETRY  5

static const char *TAG = "MAIN";

// Hardware objects
RGB rgb_led;
PROTOCOL tello_protocol;

// Flight control variables
bool flight_ready = false;
bool flight_in_progress = false;
bool landed = true;
bool button_flight_mode = true;

// Command timeout for auto-landing
const int64_t CMD_TIMEOUT_US = 15 * 1000000;
int64_t last_cmd_time = 0;

// Latest command storage
typedef struct {
    float linear_x;
    float linear_y;
    float linear_z;
    float angular_z;
} cmd_vel_t;

cmd_vel_t latest_cmd = {0, 0, 0, 0};
portMUX_TYPE cmd_mutex = portMUX_INITIALIZER_UNLOCKED;

// EKF variables
ekf_t ekf;
bool ekf_initialized = false;
bool ekf_resetting = false;
float current_attitude[3] = {0.0f, 0.0f, 0.0f}; // roll, pitch, yaw
int64_t last_prediction_time = 0;
int64_t last_button_press = 0;
const int64_t BUTTON_DEBOUNCE_US = 500000; // 500ms debounce

// Sensor update timing
int64_t last_velocity_update = 0;
int64_t last_barometer_update = 0;
const int64_t VELOCITY_UPDATE_PERIOD_US = (int64_t)(1000000.0f / VELOCITY_UPDATE_RATE_HZ);
const int64_t BAROMETER_UPDATE_PERIOD_US = (int64_t)(1000000.0f / BAROMETER_UPDATE_RATE_HZ);

// Performance tracking
uint32_t velocity_update_count = 0;
uint32_t barometer_update_count = 0;
uint32_t prediction_count = 0;

// Process and measurement noise matrices
const float Q[EKF_N*EKF_N] = {
    // Position process noise (3x3 diagonal block)
    POS_PROCESS_STD*POS_PROCESS_STD, 0, 0, 0, 0, 0, 0, 0, 0,
    0, POS_PROCESS_STD*POS_PROCESS_STD, 0, 0, 0, 0, 0, 0, 0,
    0, 0, POS_PROCESS_STD*POS_PROCESS_STD, 0, 0, 0, 0, 0, 0,
    // Velocity process noise (3x3 diagonal block)
    0, 0, 0, VEL_PROCESS_STD*VEL_PROCESS_STD, 0, 0, 0, 0, 0,
    0, 0, 0, 0, VEL_PROCESS_STD*VEL_PROCESS_STD, 0, 0, 0, 0,
    0, 0, 0, 0, 0, VEL_PROCESS_STD*VEL_PROCESS_STD, 0, 0, 0,
    // Bias process noise (3x3 diagonal block)
    0, 0, 0, 0, 0, 0, BIAS_PROCESS_STD*BIAS_PROCESS_STD, 0, 0,
    0, 0, 0, 0, 0, 0, 0, BIAS_PROCESS_STD*BIAS_PROCESS_STD, 0,
    0, 0, 0, 0, 0, 0, 0, 0, BIAS_PROCESS_STD*BIAS_PROCESS_STD
};

const float R_vel[EKF_M*EKF_M] = {
    VEL_NOISE_STD*VEL_NOISE_STD, 0, 0,
    0, VEL_NOISE_STD*VEL_NOISE_STD, 0,
    0, 0, VEL_NOISE_STD*VEL_NOISE_STD
};

// Barometer measurement noise (scalar)
const float R_baro = BARO_NOISE_STD * BARO_NOISE_STD;

// micro-ROS objects
rcl_publisher_t sensors_publisher;
rcl_publisher_t pose_publisher;
rcl_subscription_t cmd_vel_subscription;
std_msgs__msg__Float32MultiArray sensors_msg;
geometry_msgs__msg__PoseStamped pose_msg;
geometry_msgs__msg__Twist cmd_vel_msg;

// micro-ROS macros
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ESP_LOGE(TAG, "Failed status on line %d: %d", __LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ESP_LOGE(TAG, "Failed status on line %d: %d", __LINE__,(int)temp_rc);}}

// WiFi event handler
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                              int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < WIFI_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

bool wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);

    // Configure static IP
    esp_netif_dhcpc_stop(sta_netif);
    
    esp_netif_ip_info_t ip_info;
    IP4_ADDR(&ip_info.ip, 10, 15, 232, 33);        // 10.15.232.124
    IP4_ADDR(&ip_info.gw, 10, 15, 232, 1);          // 10.15.232.1  
    IP4_ADDR(&ip_info.netmask, 255, 255, 255, 128); // /25 subnet
    
    esp_netif_set_ip_info(sta_netif, &ip_info);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {};
    strcpy((char*)wifi_config.sta.ssid, WIFI_SSID);
    strcpy((char*)wifi_config.sta.password, WIFI_PASS);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "✅ Connected to AP SSID:%s with IP: %s", WIFI_SSID, STATIC_IP_ADDR);
        return true;  // Success
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "❌ Failed to connect to SSID:%s", WIFI_SSID);
        return false; // Failure
    } else {
        ESP_LOGE(TAG, "❌ UNEXPECTED WIFI EVENT");
        return false; // Failure
    }
}

// Coordinate transformation functions
void body_to_world(const float accel_body[3], const float attitude[3], float accel_world[3]) {
    float roll = attitude[0] * M_PI / 180.0f;  // Convert to radians
    float pitch = attitude[1] * M_PI / 180.0f;
    float yaw = attitude[2] * M_PI / 180.0f;
    
    float cos_r = cosf(roll), sin_r = sinf(roll);
    float cos_p = cosf(pitch), sin_p = sinf(pitch);
    float cos_y = cosf(yaw), sin_y = sinf(yaw);
    
    // Rotation matrix (body to world)
    float R[3][3] = {
        {cos_y*cos_p, cos_y*sin_p*sin_r - sin_y*cos_r, cos_y*sin_p*cos_r + sin_y*sin_r},
        {sin_y*cos_p, sin_y*sin_p*sin_r + cos_y*cos_r, sin_y*sin_p*cos_r - cos_y*sin_r},
        {-sin_p,      cos_p*sin_r,                      cos_p*cos_r}
    };
    
    // Apply rotation
    accel_world[0] = R[0][0]*accel_body[0] + R[0][1]*accel_body[1] + R[0][2]*accel_body[2];
    accel_world[1] = R[1][0]*accel_body[0] + R[1][1]*accel_body[1] + R[1][2]*accel_body[2];
    accel_world[2] = R[2][0]*accel_body[0] + R[2][1]*accel_body[1] + R[2][2]*accel_body[2];
}

void rotate_position_90_left(float pos_in[3], float pos_out[3]) {
    // new_x = -old_y
    // new_y = old_x
    // new_z = old_z (unchanged)
    pos_out[0] = -pos_in[1];  // new X = -old Y
    pos_out[1] =  pos_in[0];  // new Y = old X
    pos_out[2] =  pos_in[2];  // new Z = old Z
}

void quaternion_multiply(float q1[4], float q2[4], float result[4]) {
    // Quaternion multiplication: result = q1 * q2
    // Format: [x, y, z, w]
    result[0] = q1[3]*q2[0] + q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1];  // x
    result[1] = q1[3]*q2[1] - q1[0]*q2[2] + q1[1]*q2[3] + q1[2]*q2[0];  // y
    result[2] = q1[3]*q2[2] + q1[0]*q2[1] - q1[1]*q2[0] + q1[2]*q2[3];  // z
    result[3] = q1[3]*q2[3] - q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2];  // w
}

void rotate_quaternion_90_left(float quat_in[4], float quat_out[4]) {
    float rot_quat[4] = {0.0f, 0.0f, 0.7071067812f, 0.7071067812f};  // [x, y, z, w]
    
    // Apply rotation: quat_out = rot_quat * quat_in
    quaternion_multiply(rot_quat, quat_in, quat_out);
}

void euler_to_quaternion(float roll, float pitch, float yaw, float q[4]) {
    // Convert degrees to radians
    roll = roll * M_PI / 180.0f;
    pitch = pitch * M_PI / 180.0f;
    yaw = yaw * M_PI / 180.0f;
    
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);
    
    q[3] = cr * cp * cy + sr * sp * sy;  // w
    q[0] = sr * cp * cy - cr * sp * sy;  // x
    q[1] = cr * sp * cy + sr * cp * sy;  // y
    q[2] = cr * cp * sy - sr * sp * cy;  // z
}

void reset_ekf() {
    if (!ekf_initialized) return;
    
    ESP_LOGW(TAG, "🔄 RESETTING EKF STATE FOR TESTING");
    
    // Set reset flag FIRST to stop all EKF operations
    ekf_resetting = true;
    
    // Brief delay to ensure sensor callback sees the flag
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // Log current state before reset
    ESP_LOGI(TAG, "Before Reset - Pos: [%.3f, %.3f, %.3f] Vel: [%.3f, %.3f, %.3f] Bias: [%.4f, %.4f, %.4f]", 
             ekf.x[0], ekf.x[1], ekf.x[2], ekf.x[3], ekf.x[4], ekf.x[5], ekf.x[6], ekf.x[7], ekf.x[8]);
    
    // Reset all states to zero except bias
    ekf.x[0] = 0.0f; ekf.x[1] = 0.0f; ekf.x[2] = 0.0f; // Position
    ekf.x[3] = 0.0f; ekf.x[4] = 0.0f; ekf.x[5] = 0.0f; // Velocity
    
    // Reset bias to initial known values
    ekf.x[6] = ACCEL_BIAS_X_MS2;
    ekf.x[7] = ACCEL_BIAS_Y_MS2;
    ekf.x[8] = ACCEL_BIAS_Z_MS2;
    
    // Reset covariance matrix to initial uncertainty
    const float P_diag[EKF_N] = {
        POS_INIT_STD*POS_INIT_STD, POS_INIT_STD*POS_INIT_STD, POS_INIT_STD*POS_INIT_STD,
        VEL_INIT_STD*VEL_INIT_STD, VEL_INIT_STD*VEL_INIT_STD, VEL_INIT_STD*VEL_INIT_STD,
        BIAS_INIT_STD*BIAS_INIT_STD, BIAS_INIT_STD*BIAS_INIT_STD, BIAS_INIT_STD*BIAS_INIT_STD
    };
    
    // Reset covariance to diagonal
    for (int i = 0; i < EKF_N; i++) {
        for (int j = 0; j < EKF_N; j++) {
            ekf.P[i*EKF_N + j] = (i == j) ? P_diag[i] : 0.0f;
        }
    }
    
    // Reset timing and counters
    last_prediction_time = esp_timer_get_time();
    last_velocity_update = 0;
    last_barometer_update = 0;
    velocity_update_count = 0;
    barometer_update_count = 0;
    prediction_count = 0;
    
    ESP_LOGI(TAG, "After Reset - Pos: [%.3f, %.3f, %.3f] Vel: [%.3f, %.3f, %.3f] Bias: [%.4f, %.4f, %.4f]", 
             ekf.x[0], ekf.x[1], ekf.x[2], ekf.x[3], ekf.x[4], ekf.x[5], ekf.x[6], ekf.x[7], ekf.x[8]);
    
    // Clear reset flag LAST to allow EKF operations to resume
    ekf_resetting = false;
    
    ESP_LOGI(TAG, "✅ EKF Reset Complete - Ready for testing!");
    
    // Do LED feedback AFTER clearing the reset flag (non-blocking)
    for (int i = 0; i < 3; i++) {
        rgb_led.SetRGB(0, 0, 255);  // Blue flash
        vTaskDelay(pdMS_TO_TICKS(100));
        rgb_led.SetRGB(0, 0, 0);    // Off
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void init_ekf() {
    if (ekf_initialized) return;
    
    // Initial covariance diagonal
    const float P_diag[EKF_N] = {
        POS_INIT_STD*POS_INIT_STD, POS_INIT_STD*POS_INIT_STD, POS_INIT_STD*POS_INIT_STD,
        VEL_INIT_STD*VEL_INIT_STD, VEL_INIT_STD*VEL_INIT_STD, VEL_INIT_STD*VEL_INIT_STD,
        BIAS_INIT_STD*BIAS_INIT_STD, BIAS_INIT_STD*BIAS_INIT_STD, BIAS_INIT_STD*BIAS_INIT_STD
    };
    
    ekf_initialize(&ekf, P_diag);
    
    // Initialize bias with known Tello characteristics
    ekf.x[6] = ACCEL_BIAS_X_MS2;
    ekf.x[7] = ACCEL_BIAS_Y_MS2;
    ekf.x[8] = ACCEL_BIAS_Z_MS2;
    
    ekf_initialized = true;
    last_prediction_time = esp_timer_get_time();
    last_velocity_update = 0;
    last_barometer_update = 0;
    
    ESP_LOGI(TAG, "EKF initialized with bias: [%.4f, %.4f, %.4f] m/s²", 
             ekf.x[6], ekf.x[7], ekf.x[8]);
    ESP_LOGI(TAG, "Velocity updates: %.1f Hz, Barometer updates: %.1f Hz", 
             VELOCITY_UPDATE_RATE_HZ, BAROMETER_UPDATE_RATE_HZ);
}

void run_ekf_prediction(float accel_body[3]) {
    if (!ekf_initialized || ekf_resetting) return;
    
    int64_t current_time = esp_timer_get_time();
    float dt = (current_time - last_prediction_time) / 1000000.0f; // Convert to seconds
    
    if (dt > 0.1f) dt = 0.1f; // Limit dt to prevent instability
    if (dt < 0.001f) return;  // Skip if dt too small
    
    // Correct accelerometer measurements by removing estimated bias
    float accel_corrected[3] = {
        accel_body[0] - ekf.x[6],
        accel_body[1] - ekf.x[7], 
        accel_body[2] - ekf.x[8]
    };
    
    // Transform to world frame
    float accel_world[3];
    body_to_world(accel_corrected, current_attitude, accel_world);
    
    // Remove gravity (assuming Z is up)
    accel_world[2] -= GRAVITY_MS2;
    
    // State transition function fx (prediction)
    float fx[EKF_N];
    // Position: p(k+1) = p(k) + v(k)*dt + 0.5*a(k)*dt²
    fx[0] = ekf.x[0] + ekf.x[3] * dt + 0.5f * accel_world[0] * dt * dt;
    fx[1] = ekf.x[1] + ekf.x[4] * dt + 0.5f * accel_world[1] * dt * dt;
    fx[2] = ekf.x[2] + ekf.x[5] * dt + 0.5f * accel_world[2] * dt * dt;
    
    // Velocity: v(k+1) = v(k) + a(k)*dt
    fx[3] = ekf.x[3] + accel_world[0] * dt;
    fx[4] = ekf.x[4] + accel_world[1] * dt;
    fx[5] = ekf.x[5] + accel_world[2] * dt;
    
    // Bias: bias(k+1) = bias(k) (constant)
    fx[6] = ekf.x[6];
    fx[7] = ekf.x[7];
    fx[8] = ekf.x[8];
    
    // State transition matrix F
    float F[EKF_N*EKF_N] = {0};
    // Identity matrix
    for (int i = 0; i < EKF_N; i++) {
        F[i*EKF_N + i] = 1.0f;
    }
    // Position depends on velocity: F[0:3, 3:6] = I*dt
    F[0*EKF_N + 3] = dt; F[1*EKF_N + 4] = dt; F[2*EKF_N + 5] = dt;
    // Velocity depends on bias: F[3:6, 6:9] = -I*dt
    F[3*EKF_N + 6] = -dt; F[4*EKF_N + 7] = -dt; F[5*EKF_N + 8] = -dt;
    
    // Scale process noise by dt
    float Q_scaled[EKF_N*EKF_N];
    for (int i = 0; i < EKF_N*EKF_N; i++) {
        Q_scaled[i] = Q[i] * dt;
    }
    
    ekf_predict(&ekf, fx, F, Q_scaled);
    last_prediction_time = current_time;
    prediction_count++;
}

void run_ekf_velocity_update(float vel_meas[3]) {
    if (!ekf_initialized || ekf_resetting) return;
    
    // Measurement matrix H for velocity (measures states 3,4,5)
    float H[EKF_M*EKF_N] = {0};
    H[0*EKF_N + 3] = 1.0f; // vx measurement
    H[1*EKF_N + 4] = 1.0f; // vy measurement  
    H[2*EKF_N + 5] = 1.0f; // vz measurement
    
    // Predicted measurements (just the velocity states)
    float hx[EKF_M] = {ekf.x[3], ekf.x[4], ekf.x[5]};
    
    // Convert velocity from cm/s to m/s
    float vel_ms[3] = {
        vel_meas[0] * 0.01f,
        vel_meas[1] * 0.01f,
        vel_meas[2] * 0.01f
    };
    
    ekf_update(&ekf, vel_ms, hx, H, R_vel);
    velocity_update_count++;
}

void run_ekf_barometer_update(float baro_meas) {
    if (!ekf_initialized || ekf_resetting) return;
    
    // Simple outlier detection for barometer
    float predicted_altitude = ekf.x[2];  // Z position
    float innovation = baro_meas - predicted_altitude;
    
    if (fabsf(innovation) > BARO_OUTLIER_THRESHOLD_M) {
        ESP_LOGW(TAG, "Barometer outlier detected: meas=%.2f, pred=%.2f, diff=%.2f", 
                 baro_meas, predicted_altitude, innovation);
        return;  // Skip this measurement
    }
    
    // Observation vector h for barometer (measures Z position, state index 2)
    float h[EKF_N] = {0};
    h[2] = 1.0f;  // Observes pz (Z position)
    
    // Predicted measurement
    float hx = ekf.x[2];  // Current Z position estimate
    
    // Use scalar update for efficiency (barometer measures single scalar)
    ekf_custom_scalar_update(&ekf, baro_meas, hx, h, R_baro);
    
    barometer_update_count++;
    
    ESP_LOGD(TAG, "Baro update: meas=%.3f, pred=%.3f, innovation=%.3f", 
             baro_meas, hx, innovation);
}

void init_sensors_message() {
    sensors_msg.data.capacity = 11;  // Increased to include barometer and reset flag
    sensors_msg.data.size = 11;
    sensors_msg.data.data = (float*)malloc(11 * sizeof(float));
    
    for (int i = 0; i < 11; i++) {
        sensors_msg.data.data[i] = 0.0f;
    }
}

void init_pose_message() {
    // Initialize header
    pose_msg.header.frame_id.data = (char*)malloc(32);
    pose_msg.header.frame_id.size = 0;
    pose_msg.header.frame_id.capacity = 32;
    
    strcpy(pose_msg.header.frame_id.data, "vicon/world");
    pose_msg.header.frame_id.size = strlen("vicon/world");
    
    // Initialize pose
    pose_msg.pose.position.x = 0.0;
    pose_msg.pose.position.y = 0.0;
    pose_msg.pose.position.z = 0.0;
    pose_msg.pose.orientation.x = 0.0;
    pose_msg.pose.orientation.y = 0.0;
    pose_msg.pose.orientation.z = 0.0;
    pose_msg.pose.orientation.w = 1.0;
}

int to_rc(float v) {
    float scaled = v * 100.0f;
    if (scaled < 0) {
        return (int)floor(scaled);
    } else {
        return (int)ceil(scaled);
    }
}

void cmd_vel_callback(const void * msgin) {
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
    
    portENTER_CRITICAL(&cmd_mutex);
    latest_cmd.linear_x = msg->linear.x;
    latest_cmd.linear_y = msg->linear.y;
    latest_cmd.linear_z = msg->linear.z;
    latest_cmd.angular_z = msg->angular.z;
    portEXIT_CRITICAL(&cmd_mutex);
    
    last_cmd_time = esp_timer_get_time();
    
    if (landed && flight_ready && !button_flight_mode) {
        ESP_LOGI(TAG, "Received cmd_vel while landed - taking off");
        tello_protocol.TakeOff();
        landed = false;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    ESP_LOGD(TAG, "cmd_vel: [%.2f, %.2f, %.2f] yaw: %.2f", 
             msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.z);
}

void sensor_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        int64_t current_time = esp_timer_get_time();
        
        // Check for button press to reset EKF (with debouncing)
        if (gpio_get_level(RMTT_KEY_PIN) == 0) { // Button pressed (active low)
            if ((current_time - last_button_press) > BUTTON_DEBOUNCE_US) {
                last_button_press = current_time;
                
                // Reset EKF if initialized
                if (ekf_initialized) {
                    ESP_LOGI(TAG, "🔘 Button pressed - Resetting EKF for testing");
                    reset_ekf();
                } else {
                    ESP_LOGI(TAG, "🔘 Button pressed - EKF not yet initialized");
                }
            }
        }
        
        // Get Tello status
        tello_protocol.getTelloStatus(50);
        
        // Get sensor data
        float roll = tello_protocol.getTelloStatusWithNameFloat("roll");
        float pitch = tello_protocol.getTelloStatusWithNameFloat("pitch");
        float yaw = tello_protocol.getTelloStatusWithNameFloat("yaw");
        
        float vgx = tello_protocol.getTelloStatusWithNameFloat("vgx");
        float vgy = tello_protocol.getTelloStatusWithNameFloat("vgy");
        float vgz = tello_protocol.getTelloStatusWithNameFloat("vgz");
        
        float agx = tello_protocol.getTelloStatusWithNameFloat("agx");
        float agy = tello_protocol.getTelloStatusWithNameFloat("agy");
        float agz = tello_protocol.getTelloStatusWithNameFloat("agz");
        
        float baro = tello_protocol.getTelloStatusWithNameFloat("baro");  // Barometer in meters
        
        // Update current attitude for EKF
        current_attitude[0] = roll;
        current_attitude[1] = pitch;
        current_attitude[2] = yaw;
        
        // Initialize EKF if needed
        if (!ekf_initialized) {
            init_ekf();
        }
        
        // ONLY run EKF operations if NOT resetting
        if (ekf_initialized && !ekf_resetting) {
            // Run EKF prediction with accelerometer data (every cycle)
            float accel_body[3] = {agx * 0.01f, agy * 0.01f, agz * 0.01f}; // Convert cm/s² to m/s²
            run_ekf_prediction(accel_body);
            
            // Run EKF velocity update at specified frequency (20 Hz)
            if ((current_time - last_velocity_update) >= VELOCITY_UPDATE_PERIOD_US) {
                float vel_meas[3] = {vgx, vgy, vgz}; // Already in cm/s
                run_ekf_velocity_update(vel_meas);
                last_velocity_update = current_time;
            }
            
            // Run EKF barometer update at specified frequency (5 Hz)
            if ((current_time - last_barometer_update) >= BAROMETER_UPDATE_PERIOD_US) {
                run_ekf_barometer_update(baro);  // Barometer already in meters
                last_barometer_update = current_time;
            }
        }
        
        // Always populate and publish raw sensors message: [gyro(3), vel(3), accel(3), baro(1), reset_flag(1)]
        sensors_msg.data.data[0] = roll;
        sensors_msg.data.data[1] = pitch;
        sensors_msg.data.data[2] = yaw;
        sensors_msg.data.data[3] = vgx;
        sensors_msg.data.data[4] = vgy;
        sensors_msg.data.data[5] = vgz;
        sensors_msg.data.data[6] = agx;
        sensors_msg.data.data[7] = agy;
        sensors_msg.data.data[8] = agz;
        sensors_msg.data.data[9] = baro;  // Barometer altitude in meters
        sensors_msg.data.data[10] = ekf_resetting ? 1.0f : 0.0f;  // Reset flag
        
        // Publish raw sensors
        RCSOFTCHECK(rcl_publish(&sensors_publisher, &sensors_msg, NULL));
        
        // ONLY populate and publish EKF pose if NOT resetting
        if (ekf_initialized && !ekf_resetting) {
            // Set timestamp
            int64_t now = esp_timer_get_time();
            pose_msg.header.stamp.sec = (int32_t)(now / 1000000);
            pose_msg.header.stamp.nanosec = (uint32_t)((now % 1000000) * 1000);
            
            // Get original EKF position
            float original_pos[3] = {ekf.x[0], ekf.x[1], ekf.x[2]};
            float rotated_pos[3];
            
            // Apply 90-degree left rotation to position
            rotate_position_90_left(original_pos, rotated_pos);
            
            // Set rotated position in pose message
            pose_msg.pose.position.x = rotated_pos[0];
            pose_msg.pose.position.y = rotated_pos[1];
            pose_msg.pose.position.z = rotated_pos[2];
            
            // Convert attitude to quaternion (original orientation)
            float original_quat[4];
            euler_to_quaternion(roll, pitch, yaw, original_quat);
            
            // Apply 90-degree left rotation to quaternion
            float rotated_quat[4];
            rotate_quaternion_90_left(original_quat, rotated_quat);
            
            // Set rotated orientation in pose message
            pose_msg.pose.orientation.x = rotated_quat[0];
            pose_msg.pose.orientation.y = rotated_quat[1];
            pose_msg.pose.orientation.z = rotated_quat[2];
            pose_msg.pose.orientation.w = rotated_quat[3];
            
            RCSOFTCHECK(rcl_publish(&pose_publisher, &pose_msg, NULL));
            
            ESP_LOGD(TAG, "EKF Pose: pos=[%.3f, %.3f, %.3f] att=[%.1f, %.1f, %.1f] baro=%.3f", 
                     ekf.x[0], ekf.x[1], ekf.x[2], roll, pitch, yaw, baro);
        }
        
        // LED indication based on flight state (unless we just reset)
        if ((current_time - last_button_press) > 1000000) { // 1 second after button press
            if (landed) {
                rgb_led.SetRGB(255, 0, 0);  // Red when landed
            } else if (flight_in_progress) {
                rgb_led.SetRGB(255, 0, 255);  // Magenta during button flight
            } else if (flight_ready) {
                rgb_led.SetRGB(0, 255, 0);  // Green when ready
            } else {
                rgb_led.SetRGB(255, 255, 0);  // Yellow during initialization
            }
        }
    }
}

void cmd_vel_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL && flight_ready) {
        int64_t current_time = esp_timer_get_time();
        
        if ((current_time - last_cmd_time) > CMD_TIMEOUT_US && !landed) {
            ESP_LOGW(TAG, "No cmd_vel for >15s, auto-landing");
            tello_protocol.Land();
            landed = true;
            return;
        }
        
        if (landed && button_flight_mode) {
            return;
        }
        
        cmd_vel_t cmd;
        portENTER_CRITICAL(&cmd_mutex);
        cmd = latest_cmd;
        portEXIT_CRITICAL(&cmd_mutex);
        
        int lr = to_rc(cmd.linear_y);
        int fb = to_rc(cmd.linear_x);
        int ud = to_rc(cmd.linear_z);
        int yaw = to_rc(cmd.angular_z);
        
        lr = (lr > 100) ? 100 : (lr < -100) ? -100 : lr;
        fb = (fb > 100) ? 100 : (fb < -100) ? -100 : fb;
        ud = (ud > 100) ? 100 : (ud < -100) ? -100 : ud;
        yaw = (yaw > 100) ? 100 : (yaw < -100) ? -100 : yaw;
        
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
    
    // Wait for WiFi to stabilize
    vTaskDelay(pdMS_TO_TICKS(5000));

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    
    ESP_LOGI(TAG, "Attempting to discover micro-ROS agent...");
    
    // Try discovery first
    rcl_ret_t discovery_ret = rmw_uros_discover_agent(rmw_options);
    if (discovery_ret != RCL_RET_OK) {
        ESP_LOGW(TAG, "Discovery failed, trying common gateway IPs...");
        
        // Try common IPs in this subnet
        const char* fallback_ips[] = {
            "10.15.232.125",  // Try our own IP first
            "10.15.232.1",    // Gateway
            "10.15.232.100",  // Common static
            "10.15.232.110",  // Common static
            "10.15.232.2"     // Common DHCP
        };
        
        bool connected = false;
        for (int i = 0; i < 5; i++) {
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

    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "tello_ekf_barometer", "", &support));

    // Create sensors publisher
    RCCHECK(rclc_publisher_init_default(
        &sensors_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "tello/sensors_raw"));

    // Create pose publisher for EKF output
    RCCHECK(rclc_publisher_init_default(
        &pose_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseStamped),
        "tello/pose"));

    // Create cmd_vel subscriber
    RCCHECK(rclc_subscription_init_default(
        &cmd_vel_subscription,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"));

    // Initialize messages
    init_sensors_message();
    init_pose_message();

    // Create timers
    rcl_timer_t sensor_timer;
    const unsigned int sensor_timer_timeout = 50;  // 50ms = 20Hz
    RCCHECK(rclc_timer_init_default(
        &sensor_timer,
        &support,
        RCL_MS_TO_NS(sensor_timer_timeout),
        sensor_timer_callback));

    rcl_timer_t cmd_timer;
    const unsigned int cmd_timer_timeout = 33;  // 33ms ≈ 30Hz
    RCCHECK(rclc_timer_init_default(
        &cmd_timer,
        &support,
        RCL_MS_TO_NS(cmd_timer_timeout),
        cmd_vel_timer_callback));

    // Create executor
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_subscription, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &sensor_timer));
    RCCHECK(rclc_executor_add_timer(&executor, &cmd_timer));

    ESP_LOGI(TAG, "micro-ROS EKF bridge with barometer initialized successfully!");
    ESP_LOGI(TAG, "Publishing raw sensors on /tello/sensors_raw");
    ESP_LOGI(TAG, "Publishing EKF pose on /tello/pose");
    ESP_LOGI(TAG, "Subscribing to /cmd_vel for drone control");
    ESP_LOGI(TAG, "Sensors format: [roll, pitch, yaw, vgx, vgy, vgz, agx, agy, agz, baro, reset_flag]");
    ESP_LOGI(TAG, "Velocity updates: %.1f Hz, Barometer updates: %.1f Hz", 
             VELOCITY_UPDATE_RATE_HZ, BAROMETER_UPDATE_RATE_HZ);
    
    rgb_led.SetRGB(0, 255, 0);
    vTaskDelay(pdMS_TO_TICKS(1000));
    rgb_led.SetRGB(0, 0, 0);

    // Start flight initialization task
    xTaskCreate(flight_initialization_task,
            "flight_init_task",
            4096,
            NULL,
            4,
            NULL);

    // Main executor loop
    while(1){
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
    }

    // Cleanup
    RCCHECK(rcl_publisher_fini(&sensors_publisher, &node));
    RCCHECK(rcl_publisher_fini(&pose_publisher, &node));
    RCCHECK(rcl_subscription_fini(&cmd_vel_subscription, &node));
    
    if (sensors_msg.data.data != NULL) free(sensors_msg.data.data);
    if (pose_msg.header.frame_id.data != NULL) free(pose_msg.header.frame_id.data);
    
    RCCHECK(rcl_node_fini(&node));
    vTaskDelete(NULL);
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "🚀 Tello EKF Bridge with Barometer Integration Starting!");
    ESP_LOGI(TAG, "Target Network: %s", WIFI_SSID);
    ESP_LOGI(TAG, "Static IP: %s", STATIC_IP_ADDR);

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

    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize RGB LED
    ESP_LOGI(TAG, "Initializing RGB LED...");
    ret = rgb_led.Init();
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

    // Initialize WiFi Client
    ESP_LOGI(TAG, "📡 Connecting to WiFi...");
    bool wifi_connected = wifi_init_sta();
    
    if (!wifi_connected) {
        ESP_LOGE(TAG, "❌ WiFi connection failed! Check SSID and password.");
        
        // Blink error pattern and restart
        while (1) {
            rgb_led.SetRGB(255, 0, 0);  // Red error
            vTaskDelay(pdMS_TO_TICKS(500));
            rgb_led.SetRGB(0, 0, 0);    // Off
            vTaskDelay(pdMS_TO_TICKS(500));
            ESP_LOGE(TAG, "🔄 Restarting in 10 seconds...");
            vTaskDelay(pdMS_TO_TICKS(9000));
            esp_restart();
        }
    }

    ESP_LOGI(TAG, "✅ WiFi connected successfully! Starting micro-ROS...");

    // Startup LED sequence
    ESP_LOGI(TAG, "System initialized successfully!");
    rgb_led.SetRGB(255, 0, 0);  // Red
    vTaskDelay(pdMS_TO_TICKS(500));
    rgb_led.SetRGB(0, 255, 0);  // Green
    vTaskDelay(pdMS_TO_TICKS(500));
    rgb_led.SetRGB(0, 0, 255);  // Blue
    vTaskDelay(pdMS_TO_TICKS(500));
    rgb_led.SetRGB(0, 0, 0);    // Off

    // Create micro-ROS task with highest priority for maximum responsiveness
    xTaskCreate(micro_ros_task,
            "uros_task",
            16384,
            NULL,
            7,      // Highest priority for micro-ROS
            NULL);
    
    ESP_LOGI(TAG, "Application started successfully!");
    ESP_LOGI(TAG, "🌟 BAROMETER INTEGRATION FEATURES:");
    ESP_LOGI(TAG, "  📏 Velocity updates: %.1f Hz (high responsiveness)", VELOCITY_UPDATE_RATE_HZ);
    ESP_LOGI(TAG, "  🌤️  Barometer updates: %.1f Hz (altitude stabilization)", BAROMETER_UPDATE_RATE_HZ);
    ESP_LOGI(TAG, "  🎯 Independent altitude observations reduce Z-axis drift");
    ESP_LOGI(TAG, "  📊 Enhanced sensor fusion for improved position estimation");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "🎮 DUAL CONTROL MODE:");
    ESP_LOGI(TAG, "  🔘 Button Mode: Press button for autonomous square pattern");
    ESP_LOGI(TAG, "  🎮 cmd_vel Mode: Manual control via ROS messages");
    ESP_LOGI(TAG, "  📱 Toggle: Press button to switch between modes");
    ESP_LOGI(TAG, "  📢 EKF Reset: Press button to reset Kalman filter");

    // Main loop with enhanced status reporting
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        ESP_LOGI(TAG, "💚 System Status - Vel Updates: %lu (%.1f Hz), Baro Updates: %lu (%.1f Hz), Predictions: %lu", 
                 velocity_update_count, 
                 velocity_update_count / (esp_timer_get_time() / 1000000.0f),
                 barometer_update_count,
                 barometer_update_count / (esp_timer_get_time() / 1000000.0f),
                 prediction_count);
        ESP_LOGI(TAG, "Flight: %s, Mode: %s, In-flight: %s, Landed: %s", 
                 flight_ready ? "Ready" : "Init", 
                 button_flight_mode ? "Button" : "cmd_vel",
                 flight_in_progress ? "Yes" : "No",
                 landed ? "Yes" : "No");
    }
}