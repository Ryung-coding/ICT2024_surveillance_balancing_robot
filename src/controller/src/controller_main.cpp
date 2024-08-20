#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>


//Control Const
#define loop_hz 100
#define sbus_center 1024
#define sbus_Range 676 
#define kill_bound 700
#define leg_up_bound 700
#define leg_down_bound 1400
#define DEADZONE_SBUS 2
#define DEADZONE_INPUT 2

//Math Const
#define deg2rad 0.01745329252
#define rad2deg 57.29577951
#define wheel_radius 0.0525 
#define RevPerS2RadPerS 6.283185307 

//Controller Value
float sbus_data[4] = {0.0, 0.0, 0.0, 0.0}; //raw signal data
float ref[4] = {0.0, 0.0, 0.0, 0.0};       //filtering & Transfer signal data { Heading Angle Velocity | Thrust Velocity | Leg Case | Kill }
float I[4] = {0.0, 0.0, 0.0, 0.0}; 
float y_past[4] = {0.0, 0.0, 0.0, 0.0};
float imu_theta = 0.0; float imu_theta_past = 0.0; float imu_theta_dot = 0.0;
float imu_psi = 0.0; float imu_psi_past = 0.0; float imu_psi_dot = 0.0;
float Motor_L_cmd=0.0; float Motor_R_cmd=0.0; 
float balancing_CMD=0.0; float heading_CMD=0.0;

float dt = 1/loop_hz;
rclcpp::Time last_time;

float lowpassfilter(float filter, float data, float alpha)
{
    return data * (1 - alpha) + filter * alpha;
}

float dead_zone_calibration(float data)
{
    if (data > DEADZONE_SBUS) 
        return data - DEADZONE_SBUS;
    else if (data < -DEADZONE_SBUS) 
        return data + DEADZONE_SBUS;
    else 
        return 0.0f;
}

void sbus_callback(const sensor_msgs::msg::JointState::SharedPtr msg) 
{
    sbus_data[0] = 100.*(msg->position[0] - sbus_center)/sbus_Range;  // SBUS ch1 : Heading angle stick
    ref[0] = lowpassfilter(ref[0], sbus_data[0], 0.1);
    ref[0] = dead_zone_calibration(ref[0]);
    
    sbus_data[1] = 100.*(sbus_center - msg->position[1])/sbus_Range;  // SBUS ch2 : Thrust stick
    ref[1] = lowpassfilter(ref[1], sbus_data[1], 0.1);
    ref[1] = dead_zone_calibration(ref[1]); 

    sbus_data[2] = msg->position[2];  // SBUS ch5 : leg stick
    ref[2] = lowpassfilter(ref[2], sbus_data[2] > leg_down_bound ? -1 : (sbus_data[2] > leg_up_bound ? 0 : 1), 0.2);

    
    sbus_data[3] = msg->position[3];  // SBUS ch8 : Kill switch
    ref[3] = lowpassfilter(ref[3], sbus_data[3] > kill_bound ? 1 : 0, 0.9);

    
}


void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{   
    //update
    imu_theta_past = imu_theta;
    imu_psi_past = imu_psi;

    imu_theta = msg->angular_velocity.y * deg2rad;                                          //rad
    imu_psi = msg->angular_velocity.z * deg2rad;                                            //rad
    imu_theta_dot = lowpassfilter(imu_theta_dot, (imu_theta - imu_theta_past)/dt, 0.05);    //rad/s
    imu_psi_dot = lowpassfilter(imu_psi_dot, (imu_psi - imu_psi_past)/dt, 0.05);            //rad/s
}

float computePID(float r, float y, float dt, int i, float Kp, float Kd, float Ki)
{
    float error = r - y;

    float P = Kp * error;
    I[i] += Ki * error * dt;

    float D = Kd * -(y - y_past[i]) / dt;
    y_past[i] = y;

    float u = P + I[i] + D;
    return u;
}

void controller_main()
{
    //velocity PID | Err_velocity -> desired theta
    // float ref_theta = computePID(ref[1], wheel_radius*balancing_CMD*RevPerS2RadPerS, dt, 0, 1.0, 0.0, 0.0);

    //Attitude PID | Err_theta -> desired theta_dot 
    float ref_theta_dot = computePID(ref[2], imu_theta, dt, 1, 10.0, 0.0, 0.0); //ref_theta
    Motor_L_cmd=ref_theta_dot;
    //Attitude PID | Err_theta_dot -> motor Input CMD for balancing 
    // balancing_CMD = computePID(ref_theta_dot, imu_theta_dot, dt, 2, 1.0, 0.0, 0.0);

    //Heading PID | Err_psi_dot -> motor Input CMD for balancing 
    heading_CMD = computePID(ref[0], imu_psi_dot, dt, 3, 0.0, 0.0, 0.0);

    // if(Motor_L_cmd <  DEADZONE_INPUT && Motor_L_cmd > -DEADZONE_INPUT) Motor_L_cmd=0;
    // Motor_R_cmd = (balancing_CMD + heading_CMD) >  DEADZONE_INPUT || (balancing_CMD + heading_CMD) < -DEADZONE_INPUT ? (balancing_CMD + heading_CMD) : 0;
}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("Main_controller");

    auto sbus_subscription = node->create_subscription<sensor_msgs::msg::JointState>("sbus_data", 10, sbus_callback);

    auto imu_subscription = node->create_subscription<sensor_msgs::msg::Imu>("imu_data", 10, imu_callback);

    auto odrive_publisher_0 = node->create_publisher<std_msgs::msg::Float64MultiArray>("/joint0_velocity_controller/commands", 10);
    // auto odrive_publisher_1 = node->create_publisher<std_msgs::msg::Float64MultiArray>("/joint1_velocity_controller/commands", 10);

    std_msgs::msg::Float64MultiArray odrive_msg_0;
    std_msgs::msg::Float64MultiArray odrive_msg_1;

    odrive_msg_0.data.resize(1);;
    // odrive_msg_1.data.resize(1);

    last_time = node->now();
    rclcpp::WallRate loop_rate(loop_hz);

    while (rclcpp::ok())
    {
        // "dt" update
        rclcpp::Time current_time = node->now();
        dt = (current_time - last_time).seconds();
        last_time = current_time;

        controller_main();

        if(ref[3] > 0.4)
        {   
            odrive_msg_0.data[0] = Motor_L_cmd;
            // odrive_msg_1.data[0] = Motor_R_cmd;
        }
        else
        {
            odrive_msg_0.data[0] = 0;
            // odrive_msg_1.data[0] = 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("controller"), "heading=%f thrust=%f leg=%f kill=%f u=%f", ref[0], ref[1], ref[2], ref[3], Motor_L_cmd);

        odrive_publisher_0->publish(odrive_msg_0);
        // odrive_publisher_1->publish(odrive_msg_1);
        
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}