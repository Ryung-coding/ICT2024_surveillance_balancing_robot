#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <cmath>

//Control Const
#define loop_hz 100
#define sbus_center 1024
#define sbus_Range 676 
#define kill_bound 700
#define leg_up_bound 700
#define leg_down_bound 1400
#define DEADZONE_SBUS 2
#define DEADZONE_INPUT 0
#define Lim_INPUT 10

//Math Const
#define deg2rad 0.01745329252
#define rad2deg 57.29577951
#define wheel_radius 0.0525 
#define RevPerS2RadPerS 6.283185307 

float Kp0=0.0; float Ki0=0.0; float Kd0=0.0;
float Kp1=5.5; float Ki1=0.0; float Kd1=1.0;
float Kp2=11.0; float Ki2=0.0; float Kd2=2.2;
float Kp3=0.0; float Ki3=0.0; float Kd3=0.0;

//Controller Value
float sbus_data[4] = {0.0, 0.0, 0.0, 0.0}; //raw signal data
float ref[4] = {0.0, 0.0, 0.0, 0.0};       //filtering & Transfer signal data { Heading Angle Velocity | Thrust Velocity | Leg Case | Kill }
float I[4] = {0.0, 0.0, 0.0, 0.0}; 
float D[4] = {0.0, 0.0, 0.0, 0.0}; 
float y_past[4] = {0.0, 0.0, 0.0, 0.0};
float imu_theta = 0.0; float imu_theta_past = 0.0; float imu_theta_dot = 0.0; 
float imu_psi = 0.0; float imu_psi_past = 0.0; float imu_psi_dot = 0.0;
float Motor_L_cmd=0.0; float Motor_R_cmd=0.0; 
float balancing_CMD=0.0; float heading_CMD=0.0;
float ref_theta_dot=0.0;

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

    imu_theta = lowpassfilter(imu_theta, -msg->angular_velocity.y * deg2rad, 0.01);                                         //rad
    imu_psi = lowpassfilter(imu_psi, -msg->angular_velocity.z * deg2rad, 0.01);          
    
    if(abs(imu_theta-imu_theta_past)>0.01*deg2rad)
    {   
        imu_theta_dot = lowpassfilter(imu_theta_dot, (imu_theta - imu_theta_past)/dt, 0.1);    //rad/s
        // RCLCPP_INFO(rclcpp::get_logger("controller"), "", ref[0]);
    }        
    
    imu_psi_dot = lowpassfilter(imu_psi_dot, (imu_psi - imu_psi_past)/dt, 0.01);            //rad/s
    
}

float computePID(float r, float y, float dt, int i, float Kp, float Kd, float Ki)
{
    float error = r - y;

    float P = Kp * error;

    I[i] += Ki * error * dt;
    if(I[i]>2)
    {
        I[i]=2;
    } 
    if(I[i]<-2)
    {
        I[i]=-2;
    } 

    
    D[i] = lowpassfilter(D[i],-Kd*(y - y_past[i])/dt,0.1);
    y_past[i] = y;

    float u = P + I[i] + D[i];
    return u;
}

void controller_main()
{
        //velocity PID | Err_velocity -> desired theta
    // float ref_theta = computePID(ref[1], wheel_radius*balancing_CMD*RevPerS2RadPerS, dt, 0, 1.0, 0.0, 0.0);

        //Attitude PID | Err_theta -> desired theta_dot 
    ref[2]=-3*deg2rad; //example

    ref_theta_dot = computePID(ref[2], imu_theta, dt, 1, Kp1, Ki1, Kd1); //ref_theta 
    
        //Attitude PID | Err_theta_dot -> motor Input CMD for balancing 
    balancing_CMD = computePID(ref_theta_dot, imu_theta_dot, dt, 2, Kp2, Ki2, Kd2); //ref_theta_dot

        //Heading PID | Err_psi_dot -> motor Input CMD for balancing 
    heading_CMD = computePID(ref[0], imu_psi_dot, dt, 3, Kp3, Ki3, Kd3);

    Motor_L_cmd=balancing_CMD - heading_CMD;
}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("Main_controller");

    node->declare_parameter<float>("Kp0", 0.0);
    node->declare_parameter<float>("Ki0", 0.0);
    node->declare_parameter<float>("Kd0", 0.0);
    node->declare_parameter<float>("Kp1", 0.0);
    node->declare_parameter<float>("Ki1", 0.0);
    node->declare_parameter<float>("Kd1", 0.0);
    node->declare_parameter<float>("Kp2", 0.0);
    node->declare_parameter<float>("Ki2", 0.0);
    node->declare_parameter<float>("Kd2", 0.0);
    node->declare_parameter<float>("Kp3", 0.0);
    node->declare_parameter<float>("Ki3", 0.0);
    node->declare_parameter<float>("Kd3", 0.0);

    Kp0 = node->get_parameter("Kp0").as_double();
    Ki0 = node->get_parameter("Ki0").as_double();
    Kd0 = node->get_parameter("Kd0").as_double();
    Kp1 = node->get_parameter("Kp1").as_double();
    Ki1 = node->get_parameter("Ki1").as_double();
    Kd1 = node->get_parameter("Kd1").as_double();
    Kp2 = node->get_parameter("Kp2").as_double();
    Ki2 = node->get_parameter("Ki2").as_double();
    Kd2 = node->get_parameter("Kd2").as_double();
    Kp3 = node->get_parameter("Kp3").as_double();
    Ki3 = node->get_parameter("Ki3").as_double();
    Kd3 = node->get_parameter("Kd3").as_double();

    auto sbus_subscription = node->create_subscription<sensor_msgs::msg::JointState>("sbus_data", 10, sbus_callback);
    auto imu_subscription = node->create_subscription<sensor_msgs::msg::Imu>("imu_data", 10, imu_callback);

    auto odrive_publisher_0 = node->create_publisher<std_msgs::msg::Float64MultiArray>("/joint0_velocity_controller/commands", 10);

    // 추가한 퍼블리셔들 - 메시지 타입을 Float64로 수정
    auto ref_theta_dot_publisher = node->create_publisher<std_msgs::msg::Float64>("/ref_theta_dot", 10);
    auto imu_theta_publisher = node->create_publisher<std_msgs::msg::Float64>("/imu_theta", 10);
    auto imu_theta_dot_publisher = node->create_publisher<std_msgs::msg::Float64>("/imu_theta_dot", 10);

    std_msgs::msg::Float64MultiArray odrive_msg_0;

    odrive_msg_0.data.resize(1);

    last_time = node->now();
    rclcpp::WallRate loop_rate(loop_hz);

    while (rclcpp::ok())
    {
        // "dt" update
        rclcpp::Time current_time = node->now();
        dt = (current_time - last_time).seconds();
        last_time = current_time;

        controller_main();

        // constrain
        if (((Motor_L_cmd <  DEADZONE_INPUT) && (Motor_L_cmd > 0)) || 
            (Motor_L_cmd > -DEADZONE_INPUT) && (Motor_L_cmd < 0)) 
            Motor_L_cmd = 0;
        if (Motor_L_cmd >  Lim_INPUT || Motor_L_cmd <  -Lim_INPUT) 
            Motor_L_cmd = Motor_L_cmd > 0 ? Lim_INPUT : -Lim_INPUT;

        odrive_msg_0.data[0] = Motor_L_cmd;
        RCLCPP_INFO(rclcpp::get_logger("controller"), 
                    "heading=%f thrust=%f leg=%f kill=%f u=%f", 
                    ref[0], ref[1], ref[2], ref[3], Motor_L_cmd);
        
        odrive_publisher_0->publish(odrive_msg_0);

        // 새로 추가된 토픽들에 데이터를 퍼블리시 - Float64 메시지로 변경
        std_msgs::msg::Float64 msg;

        msg.data = ref_theta_dot*rad2deg;
        ref_theta_dot_publisher->publish(msg);

        msg.data = imu_theta*rad2deg;
        imu_theta_publisher->publish(msg);

        msg.data = imu_theta_dot*rad2deg;
        imu_theta_dot_publisher->publish(msg);
        
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
