#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <controller_main.h>
#include <controller_main_function.h>

// 전역 변수로 dubal_vel의 velocity 값을 저장할 변수를 선언합니다.
float dubal_vel_value = 0.0f;
float dubal_pos_value = 0.0f;

void balancing_controller()
{
    // velocity PID | Err_velocity -> desired theta
    float ref_theta = computePID(0, dubal_pos_value*3.141592/180, 0, dt, 0);
    
    // Attitude PID | Err_theta -> desired theta_dot 
    ref_theta_dot = computePID(-0.0023*ref[1], imu_theta, imu_theta_dot, dt, 1);
    
    // Attitude PID | Err_theta_dot -> motor Input CMD for balancing 
    imu_theta_ddot = lowpassfilter(imu_theta_ddot, (imu_theta_dot - imu_theta_dot_past) / dt, 0.1);  
    balancing_CMD = computePID(ref_theta_dot, imu_theta_ddot, imu_theta_ddot, dt, 2);

    balancing_CMD=ref_theta_dot;

}

void heading_controller()
{
    // Heading PID | Err_psi_dot -> motor Input CMD for balancing 
    imu_psi_ddot = lowpassfilter(imu_psi_ddot, (imu_psi_dot - imu_psi_dot_past) / dt, 0.1);  
    heading_CMD = computePID(ref[0], imu_psi_dot, imu_psi_ddot, dt, 3);
    imu_psi_dot_past = imu_psi_dot;
}

// 콜백 함수: dubal_vel 토픽에서 JointState 메시지를 수신합니다.
void dubal_vel_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{

        dubal_vel_value = lowpassfilter(dubal_vel_value, msg->velocity[0], 0.99); 
        dubal_pos_value = msg->position[0];

}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("Main_controller");

    auto sbus_subscription = node->create_subscription<sensor_msgs::msg::JointState>("sbus_data", 10, sbus_callback);
    auto sgps_subscription = node->create_subscription<sensor_msgs::msg::JointState>("gps_data", 10, gps_callback);
    auto imu_subscription = node->create_subscription<sensor_msgs::msg::JointState>("imu_data", 10, imu_callback);

    // dubal_vel 토픽을 구독하여 dubal_vel_callback 함수가 호출되도록 설정합니다.
    auto dubal_vel_subscription = node->create_subscription<sensor_msgs::msg::JointState>(
        "dubal_vel", 10, dubal_vel_callback);

    // Left motor
    auto odrive_publisher_0 = node->create_publisher<std_msgs::msg::Float64MultiArray>("/joint0_velocity_controller/commands", 10);
    std_msgs::msg::Float64MultiArray odrive_msg_0;
    odrive_msg_0.data.resize(1);

    // Right motor
    // auto odrive_publisher_1 = node->create_publisher<std_msgs::msg::Float64MultiArray>("/joint1_velocity_controller/commands", 10);
    // std_msgs::msg::Float64MultiArray odrive_msg_1;
    // odrive_msg_1.data.resize(1);

    // data plot
    auto controller_input_publisher = node->create_publisher<sensor_msgs::msg::JointState>("/controller_input_data", 10); 
    sensor_msgs::msg::JointState controller_input_data; 
    
    last_time = node->now();
    rclcpp::WallRate loop_rate(loop_hz);

    while (rclcpp::ok())
    {
        // "dt" update
        rclcpp::Time current_time = node->now();
        dt = (current_time - last_time).seconds();
        last_time = current_time;

        // controller
        balancing_controller();
        heading_controller();

        Motor_L_cmd = balancing_CMD - heading_CMD;
        Motor_R_cmd = balancing_CMD + heading_CMD;

        // constrain
        if (abs(Motor_L_cmd) < DEADZONE_INPUT)  Motor_L_cmd = 0;
        if (abs(Motor_L_cmd) > Lim_INPUT)       Motor_L_cmd = Motor_L_cmd > 0 ? Lim_INPUT : -Lim_INPUT;
        if (abs(Motor_R_cmd) < DEADZONE_INPUT)  Motor_R_cmd = 0;
        if (abs(Motor_R_cmd) > Lim_INPUT)       Motor_R_cmd = Motor_R_cmd > 0 ? Lim_INPUT : -Lim_INPUT;

        // publisher
        odrive_msg_0.data[0] = Motor_L_cmd;
        odrive_publisher_0->publish(odrive_msg_0);
        // odrive_msg_1.data[0] = Motor_R_cmd;
        // odrive_publisher_1->publish(odrive_msg_1);
        
        // check data
        RCLCPP_INFO(rclcpp::get_logger("controller"), "heading=%f thrust=%f Connect=%f kill=%f u=%f",  computePID(0, dubal_pos_value, 0, dt, 0), ref[1], ref[3], ref[4], Motor_L_cmd);
        auto controller_input_msg = sensor_msgs::msg::JointState();
        controller_input_msg.header.stamp = node->now();
        controller_input_msg.name = {"pitch[rad]", "yaw[rad]"};
        controller_input_msg.position = {0.0, 0.0};
        controller_input_msg.velocity.resize(1);
        controller_input_msg.velocity[0] = dubal_vel_value;

        controller_input_publisher->publish(controller_input_msg);

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
