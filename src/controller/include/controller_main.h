#ifndef CONTROLLER_MAIN
#define CONTROLLER_MAIN

#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

// PID Controller Gains
float Kp[4] = {0.0023, 85.000, 0.0000, 0.0000};
float Kd[4] = {0.0000, 10.000, 0.0000, 0.0000};
float Ki[4] = {0.0000, 10.000, 0.0000, 0.0000};


// Control Constants
#define loop_hz 100
#define sbus_center 1024
#define sbus_Range 676 
#define kill_bound 700
#define connect_bound 700
#define leg_up_bound 700
#define leg_down_bound 1400
#define DEADZONE_SBUS 2
#define DEADZONE_INPUT 0
#define Lim_INPUT 10
#define anti_windup_gain 2

// Math Constants
#define deg2rad 0.01745329252
#define rad2deg 57.29577951
#define wheel_radius 0.0525 
#define RevPerS2RadPerS 6.283185307 


// Controller Values
float sbus_data[5] = {0.0, 0.0, 0.0, 0.0, 0.0}; // Raw signal data
float ref[5] = {0.0, 0.0, 0.0, 0.0, 0.0};       // Filtered & transferred signal data { Heading Angle Velocity | Thrust Velocity | Leg Case | Web Connect  | Kill }
float I[4] = {0.0, 0.0, 0.0, 0.0};              // Integral term

// IMU Data
float imu_theta = 0.0; 
float imu_theta_dot = 0.0; 
float imu_theta_dot_past = 0.0; 
float imu_theta_ddot = 0.0;

float imu_psi = 0.0; 
float imu_psi_dot = 0.0; 
float imu_psi_dot_past = 0.0; 
float imu_psi_ddot = 0.0; 

// Motor Commands
float Motor_L_cmd = 0.0; 
float Motor_R_cmd = 0.0; 
float balancing_CMD = 0.0; 
float heading_CMD = 0.0;
float ref_theta_dot = 0.0;

// Motor Commands
float odrive_L_pos = 0.0f;
float odrive_R_pos = 0.0f;
float odrive_L_vel = 0.0f;
float odrive_R_vel = 0.0f;

// State Flags
bool isKilled = true;
bool isConnected = true;

// GPS Data
float Long = 0.0;
float Lat = 0.0;
float Alt = 0.0;
float SIV = 0.0;
float FIX = 0.0;
float year = 0.0;
float month = 0.0;
float day = 0.0;
float hour = 0.0;
float minute = 0.0;
float second = 0.0;

// Timing
float dt = 1.0 / loop_hz;
rclcpp::Time last_time;


#endif // CONTROLLER_MAIN
