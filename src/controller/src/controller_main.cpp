#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <controller_main.h>
#include <controller_main_function.h>

float balancing_controller()
{
        //velocity PID | Err_velocity -> desired theta
    float ref_theta = computePID(ref[1], wheel_radius*balancing_CMD*RevPerS2RadPerS, dt, 0);

        //Attitude PID | Err_theta -> desired theta_dot 
    ref_theta=-3*3.141592/180; //example
    ref_theta_dot = computePID(ref_theta, imu_theta, dt, 1);
    
        //Attitude PID | Err_theta_dot -> motor Input CMD for balancing 
    balancing_CMD = computePID(ref_theta_dot, imu_theta_dot, dt, 2);

    return ref_theta_dot;
}

float heading_controller()
{
    //Heading PID | Err_psi_dot -> motor Input CMD for balancing 
    heading_CMD = computePID(ref[0], imu_psi_dot, dt, 3);
    
    return heading_CMD;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("Main_controller");

    auto sbus_subscription = node->create_subscription<sensor_msgs::msg::JointState>("sbus_data", 10, sbus_callback);
    auto sgps_subscription = node->create_subscription<sensor_msgs::msg::JointState>("gps_data", 10, gps_callback);
    auto imu_subscription = node->create_subscription<sensor_msgs::msg::JointState>("imu_data", 10, imu_callback);

        //Left motor
    auto odrive_publisher_0 = node->create_publisher<std_msgs::msg::Float64MultiArray>("/joint0_velocity_controller/commands", 10);
    std_msgs::msg::Float64MultiArray odrive_msg_0;
    odrive_msg_0.data.resize(1);

        //Right motor
    // auto odrive_publisher_1 = node->create_publisher<std_msgs::msg::Float64MultiArray>("/joint1_velocity_controller/commands", 10);
    // std_msgs::msg::Float64MultiArray odrive_msg_1;
    // odrive_msg_1.data.resize(1);

        //data plot
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

            //controller
        balancing_CMD = balancing_controller();
        heading_CMD = heading_controller();

        Motor_L_cmd=balancing_CMD - heading_CMD;
        Motor_R_cmd=balancing_CMD + heading_CMD;

            // constrain
        if (abs(Motor_L_cmd) < DEADZONE_INPUT)  Motor_L_cmd = 0;
        if (abs(Motor_L_cmd) > Lim_INPUT)       Motor_L_cmd = Motor_L_cmd > 0 ? Lim_INPUT : -Lim_INPUT;
        if (abs(Motor_R_cmd) < DEADZONE_INPUT)  Motor_R_cmd = 0;
        if (abs(Motor_R_cmd) > Lim_INPUT)       Motor_R_cmd = Motor_R_cmd > 0 ? Lim_INPUT : -Lim_INPUT;

            //publisher
        odrive_msg_0.data[0] = Motor_L_cmd;
        odrive_publisher_0->publish(odrive_msg_0);
        // odrive_msg_1.data[0] = Motor_R_cmd;
        // odrive_publisher_1->publish(odrive_msg_1);
        
            //cheak data
        RCLCPP_INFO(rclcpp::get_logger("controller"), "heading=%f thrust=%f Connect=%f kill=%f u=%f", ref[0], ref[1], ref[3], ref[4], Motor_L_cmd);
        auto controller_input_msg = sensor_msgs::msg::JointState();
        controller_input_msg.header.stamp = node->now();
        controller_input_msg.name = {"pitch[rad]", "yaw[rad]"};
        controller_input_msg.position = {0.0, 0.0};
        controller_input_msg.velocity = {0.0, 0.0};
        controller_input_publisher->publish(controller_input_msg);

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
