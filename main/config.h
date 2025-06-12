#ifndef CONFIG_H
#define CONFIG_H

// EKF Dimensions (must be defined before tinyekf.h)
#define EKF_N 9     // [px, py, pz, vx, vy, vz, bias_x, bias_y, bias_z]
#define EKF_M 3     // velocity measurements [vx, vy, vz]

// Tello API Configuration
#define TELLO_API_RATE_HZ 20.0f
#define TELLO_DT (1.0f / TELLO_API_RATE_HZ)

// Sensor Update Rates
#define VELOCITY_UPDATE_RATE_HZ  20.0f   // High frequency for responsiveness
#define BAROMETER_UPDATE_RATE_HZ  5.0f   // Lower frequency, typical for barometers

// Sensor Characteristics (from real Tello data analysis)
#define ACCEL_BIAS_X_MS2    0.088f   // 8.8 cm/s² converted to m/s²
#define ACCEL_BIAS_Y_MS2   -0.061f   // -6.1 cm/s²
#define ACCEL_BIAS_Z_MS2   -9.994f   // -999.4 cm/s² (includes gravity offset)

#define ACCEL_NOISE_X_STD   0.066f   // 6.6 cm/s² noise std
#define ACCEL_NOISE_Y_STD   0.052f   // 5.2 cm/s²
#define ACCEL_NOISE_Z_STD   0.028f   // 2.8 cm/s²

#define VEL_NOISE_STD       0.01f    // m/s velocity measurement noise
#define BARO_NOISE_STD      0.05f    // meters barometer measurement noise
#define ATT_NOISE_STD_RAD   0.00175f // 0.1 degrees in radians

// EKF Tuning Parameters
#define POS_PROCESS_STD     0.001f
#define VEL_PROCESS_STD     0.005f
#define BIAS_PROCESS_STD    0.0001f

#define POS_INIT_STD        0.5f
#define VEL_INIT_STD        0.1f
#define BIAS_INIT_STD       0.05f

// Gravity constant
#define GRAVITY_MS2         9.81f

// Outlier detection thresholds
#define ACCEL_OUTLIER_THRESHOLD_MS2  5.0f
#define VEL_OUTLIER_THRESHOLD_MS     2.0f
#define BARO_OUTLIER_THRESHOLD_M     1.0f

#endif // CONFIG_H