#ifndef CONTROLLER_MAIN_FUNCTION
#define CONTROLLER_MAIN_FUNCTION


float lowpassfilter(float filter, float data, float past_weight)
{
    return data * (1 - past_weight) + filter * past_weight;
}

void web_ref_update()
{
    ref[0]=0;
    ref[1]=0;
    // ref[2]=??
    // ref[3]=??
    // ref[4]=??
}

void sbus_callback(const sensor_msgs::msg::JointState::SharedPtr msg) 
{
    sbus_data[0] = 100.*(msg->position[0] - sbus_center)/sbus_Range;        // SBUS ch1 : Heading angle stick
    ref[0] = lowpassfilter(ref[0], sbus_data[0], 0.1);
    if(abs(ref[0]) <= DEADZONE_SBUS) ref[0]= 0.0f;
    else ref[0] = ref[0]>=0 ? (ref[0] - DEADZONE_SBUS) : (ref[0] + DEADZONE_SBUS);
    
    sbus_data[1] = 100.*(sbus_center - msg->position[1])/sbus_Range;        // SBUS ch2 : Thrust stick
    ref[1] = lowpassfilter(ref[1], sbus_data[1], 0.1);
    if(abs(ref[1]) <= DEADZONE_SBUS) ref[1]= 0.0f;
    else ref[1] = ref[1]>=0 ? (ref[1] - DEADZONE_SBUS) : (ref[1] + DEADZONE_SBUS);

    sbus_data[2] = msg->position[2];                                        // SBUS ch5 : leg stick
    ref[2] = lowpassfilter(ref[2], sbus_data[2] > leg_down_bound ? -1 : (sbus_data[2] > leg_up_bound ? 0 : 1), 0.2);

    sbus_data[3] = msg->position[3];                                        // SBUS ch6 : Connect switch
    ref[3] = lowpassfilter(ref[3], sbus_data[3] > connect_bound ? 0 : 1, 0.0);
    if(ref[3]>0.4) isConnected=false;
    else isConnected=true;

    sbus_data[4] = msg->position[4];                                        // SBUS ch8 : Kill switch
    ref[4] = lowpassfilter(ref[4], sbus_data[4] > kill_bound ? 1 : 0, 0.0);
    if(ref[4]>0.4) isKilled=false;
    else isKilled=true;

        //cmd changing
    if(isConnected==true) web_ref_update();
}


void imu_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{   
    imu_theta = lowpassfilter(imu_theta, msg->position[1], 0.001);                             
    imu_psi = lowpassfilter(imu_psi, -msg->position[2], 0.001);     

    imu_theta_dot=lowpassfilter(imu_theta_dot, msg->velocity[1], 0.001);         
    imu_psi_dot=lowpassfilter(imu_psi_dot, -msg->velocity[2], 0.001);    
}

void gps_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{   
    Long=msg->position[0];
    Lat=msg->position[1];
    Alt=msg->position[2];
    SIV=msg->position[3];
    FIX=msg->position[4];
    year=msg->position[5];
    month=msg->position[6];
    day=msg->position[7];
    hour=msg->position[8];
    minute=msg->position[9];
    second=msg->position[10];
}

void dubal_vel_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{       
        odrive_L_pos = msg->position[0];
        odrive_L_vel = lowpassfilter(odrive_L_vel, msg->velocity[0], 0.99); 
        // odrive_R_pos = msg->position[1];
        // odrive_R_vel = lowpassfilter(odrive_R_vel, msg->velocity[1], 0.99); 
}


float computePID(float r, float y,float y_dot, float dt, int PID_case)
{
    float error = r - y;

    //P gain
    float P = Kp[PID_case] * error;

    //I gain
    I[PID_case] += Ki[PID_case] * error * dt;
    if(abs(I[PID_case])>anti_windup_gain) I[PID_case]=I[PID_case]>0 ? anti_windup_gain : -anti_windup_gain; 
    
    //D gain
    float D = Kd[PID_case]*(0-y_dot);

    float u = P + I[PID_case] + D;
    return u;
}

#endif
