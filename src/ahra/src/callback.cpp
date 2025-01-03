
#include "callback.hpp"

bool flgflg = 0;
FILE *Trajectory_all;

Callback::Callback(Trajectory *trajectoryPtr, IK_Function *IK_Ptr, Dxl *dxlPtr, Pick *pick_Ptr)
    : trajectoryPtr(trajectoryPtr),
      IK_Ptr(IK_Ptr),
      dxlPtr(dxlPtr),
      pick_Ptr(pick_Ptr),
      SPIN_RATE(100)
{
    ros::NodeHandle nh(ros::this_node::getName());
    boost::thread queue_thread = boost::thread(boost::bind(&Callback::callbackThread, this));
    boost::thread imu_thread = boost::thread(boost::bind(&Callback::IMUThread, this));
    trajectoryPtr->Ref_RL_x = MatrixXd::Zero(1, 675);
    trajectoryPtr->Ref_LL_x = MatrixXd::Zero(1, 675);
    trajectoryPtr->Ref_RL_y = -0.06 * MatrixXd::Ones(1, 675);
    trajectoryPtr->Ref_LL_y = 0.06 * MatrixXd::Ones(1, 675);
    trajectoryPtr->Ref_RL_z = MatrixXd::Zero(1, 675);
    trajectoryPtr->Ref_LL_z = MatrixXd::Zero(1, 675);
    trajectoryPtr->Turn_Trajectory = VectorXd::Zero(135);
    omega_w = sqrt(g / z_c);

    pick_Ptr->Ref_WT_th = MatrixXd::Zero(1, 675);
    pick_Ptr->Ref_RA_th = MatrixXd::Zero(4, 675);
    pick_Ptr->Ref_LA_th = MatrixXd::Zero(4, 675);
    pick_Ptr->Ref_NC_th = MatrixXd::Zero(2, 675);


    Trajectory_all = fopen("/home/ryu/Trajectory/trajectoryall.dat", "w");

}

void Callback::JointStatesCallback(const sensor_msgs::JointState::ConstPtr &joint_command)
{
    for (int i = 0; i < NUMBER_OF_DYNAMIXELS; i++)
    {
        Goal_joint_[i] = joint_command->position[i];
        dxlPtr->SetThetaRef(Goal_joint_);
    }
}

void Callback::L_FSRsensorCallback(const std_msgs::UInt8::ConstPtr &FSR)
{
    R_value = FSR->data; // Left_foot_FSR
}

void Callback::R_FSRsensorCallback(const std_msgs::UInt8::ConstPtr &FSR)
{
    L_value = FSR->data; // Right_foot_FSR
    // ROS_INFO("%d" , L_value );
}

void Callback::StartMode(const std_msgs::Bool::ConstPtr &start)
{
    if (start->data)
    {
        indext = 0;
        trajectoryPtr->Ref_RL_x = MatrixXd::Zero(1, 30);
        trajectoryPtr->Ref_LL_x = MatrixXd::Zero(1, 30);
        trajectoryPtr->Ref_RL_y = -0.06 * MatrixXd::Ones(1, 30);
        trajectoryPtr->Ref_LL_y = 0.06 * MatrixXd::Ones(1, 30);
        trajectoryPtr->Ref_RL_z = MatrixXd::Zero(1, 30);
        trajectoryPtr->Ref_LL_z = MatrixXd::Zero(1, 30);
        pick_Ptr->Ref_WT_th = MatrixXd::Zero(1, 30);
        pick_Ptr->Ref_RA_th = MatrixXd::Zero(4, 30);
        pick_Ptr->Ref_LA_th = MatrixXd::Zero(4, 30);
        pick_Ptr->Ref_NC_th = MatrixXd::Zero(2, 30);
        
        srv_SendMotion.request.ST_finish = true;
                // srv_SendMotion.response.select_motion = Motion_Index::Right_Halfstep; 

        flgflg = 1;
        srv_SendMotion.response.select_motion = Motion_Index::Shoot; 

        emergency = 0;
        srv_SendMotion.request.TA_finish = true;

    }  
    else // 여기서 else if 대신 else를 사용하여 start->data가 거짓인 경우를 처리합니다.
    {
        indext = 0;
        trajectoryPtr->Ref_RL_x = MatrixXd::Zero(1, 30);
        trajectoryPtr->Ref_LL_x = MatrixXd::Zero(1, 30);
        trajectoryPtr->Ref_RL_y = -0.06 * MatrixXd::Ones(1, 30);
        trajectoryPtr->Ref_LL_y = 0.06 * MatrixXd::Ones(1, 30);
        trajectoryPtr->Ref_RL_z = MatrixXd::Zero(1, 30);
        trajectoryPtr->Ref_LL_z = MatrixXd::Zero(1, 30);
        pick_Ptr->Ref_WT_th = MatrixXd::Zero(1, 30);
        pick_Ptr->Ref_RA_th = MatrixXd::Zero(4, 30);
        pick_Ptr->Ref_LA_th = MatrixXd::Zero(4, 30);
        pick_Ptr->Ref_NC_th = MatrixXd::Zero(2, 30);

        flgflg = 1;

        srv_SendMotion.response.select_motion = Motion_Index::Ready_to_throw; 

        // srv_SendMotion.response.turn_angle = -15;
        
        emergency = 0;
        srv_SendMotion.request.TA_finish = true;
        // ROS_INFO("Picking_Ball!");
    }
}


/////////////////////////////////////////////// About Subscribe IMUThread ///////////////////////////////////////////////

void Callback::VelocityCallback(const std_msgs::Float32::ConstPtr &IMU)
{
    vel_x = IMU->data;
    vel_y = IMU->data;
    vel_z = IMU->data;

    // ROS_ERROR("X : %f", vel_x);
    // ROS_ERROR("Y : %f", vel_y);
    // ROS_ERROR("Z : %f", vel_z);
}

void Callback::IMUThread()
{
    IMU_Velocity_Complementary_x_subscriber_ = nh.subscribe("/filtered/Velocity_Complementary/x", 1000, &Callback::VelocityCallback, this);
    IMU_Velocity_Complementary_y_subscriber_ = nh.subscribe("/filtered/Velocity_Complementary/y", 1000, &Callback::VelocityCallback, this);
    IMU_Velocity_Complementary_z_subscriber_ = nh.subscribe("/filtered/Velocity_Complementary/z", 1000, &Callback::VelocityCallback, this);

    ros::Rate loop_rate(200);
    Set_Callback();
    while (nh.ok())
    {
        // Calculate_Real_CP(indext, vel_x, vel_y);
        ros::spinOnce();
        loop_rate.sleep();

        // ROS_INFO("%lf", vel_x);
    }
}

// Function to generate a unique request ID
int Callback::generateUniqueRequestID()
{
    // Initialize a seed using srand function
    // Initializing the seed allows you to generate random numbers using rand() function
    srand(static_cast<unsigned int>(std::time(0)));

    // Generate a random request ID (e.g., a random integer between 1 and 10000)
    return (rand() % 10000) + 1;
}

void Callback::callbackThread()
{
    ros::NodeHandle nh(ros::this_node::getName());

    joint_state_publisher_ = nh.advertise<sensor_msgs::JointState>("KWJ_joint_states", 100);
    joint_state_subscriber_ = nh.subscribe("KWJ_desired_joint_states", 1000, &Callback::JointStatesCallback, this);

    FSR_L_sensor_subscriber_ = nh.subscribe("/FSR_L", 1000, &Callback::L_FSRsensorCallback, this);
    FSR_R_sensor_subscriber_ = nh.subscribe("/FSR_R", 1000, &Callback::R_FSRsensorCallback, this);
    Start_subscriber_ = nh.subscribe("/START", 1000, &Callback::StartMode, this);

    srv_SendMotion.request.SM_finish = false;
    srv_SendMotion.request.TA_finish = false;
    srv_SendMotion.request.UD_finish = false;
    srv_SendMotion.request.RL_finish = true;
    srv_SendMotion.request.EM_finish = true;
    srv_SendMotion.request.ST_finish = true;
    srv_SendMotion.request.walkcount = 0;


    ros::Rate loop_rate(SPIN_RATE);
//     while (nh.ok())
//     {
//         srv_SendMotion.request.request_id = generateUniqueRequestID(); // generate a unique request ID

//         if (flgflg) //client_SendMotion.call(srv_SendMotion)
//         {
//                         flgflg = false;
//                                     srv_SendMotion.response.success = true;
//             if (srv_SendMotion.response.success)
//             {
//                 Motion_Info();
//                 RecieveMotion();
//             }
//             else if (!error_printed)
//             {
//                 if (error_counter < 3) // Check the counter
//                 {
//                     // ROS_INFO("\n");
//                     // ROS_ERROR("Failed to call service");
//                     // ROS_INFO("\n");
//                     error_printed = true; // Set the flag to true
//                     error_counter++;      // Increase the counter
//                 }
//             }
//         }

//         ros::spinOnce();
//         loop_rate.sleep();
//         // usleep(1000);
//     }
// }
    while (nh.ok())
    {
        srv_SendMotion.request.request_id = generateUniqueRequestID(); // generate a unique request ID

        if (client_SendMotion.call(srv_SendMotion))
        {
            if (srv_SendMotion.response.success)
            {
                Motion_Info();
                RecieveMotion();
            }
            else if (!error_printed)
            {
                if (error_counter < 3) // Check the counter
                {
                    // ROS_INFO("\n");
                    // ROS_ERROR("Failed to call service");
                    // ROS_INFO("\n");
                    error_printed = true; // Set the flag to true
                    error_counter++;      // Increase the counter
                }
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
        // usleep(1000);
    }
}
/////////////////////////////////////////////// About Client Callback ///////////////////////////////////////////////

void Callback::RecieveMotion()
{

    // ROS_INFO("#[MESSAGE] SM Request : %s#", srv_SendMotion.request.SM_finish ? "true" : "false");
    // ROS_INFO("#[MESSAGE] TA Request : %s#", srv_SendMotion.request.TA_finish ? "true" : "false");
    // ROS_INFO("#[MESSAGE] UD Request : %s#", srv_SendMotion.request.UD_finish ? "true" : "false");

    // ROS_INFO("#[MESSAGE] RL Request : %s#", srv_SendMotion.request.RL_finish ? "true" : "false");
    // ROS_INFO("#[MESSAGE] EM Request : %s#", srv_SendMotion.request.EM_finish ? "true" : "false");

    // ROS_WARN("RL_NECK : %f", rl_neckangle);
    // ROS_WARN("UD_NECK : %f", -ud_neckangle+90);
    // ROS_WARN("TURN_ANGLE : %f", turn_angle * RAD2DEG); 
    // //ROS_ERROR("EMERGENCY : %s", emergency_ ? "True" : "False");

    SelectMotion();
    TATA();
    Move_UD_NeckAngle();
    // Move_RL_NeckAngle();
    // Emergency();
}

void Callback::Move_RL_NeckAngle()
{

    double res_rl_neck = srv_SendMotion.response.rl_neckangle;
    rl_neckangle = res_rl_neck;
    pick_Ptr->NC_th[2] = rl_neckangle * DEG2RAD;
    // All_Theta[2] = rl_neckangle * DEG2RAD;
    // ROS_WARN("RL_NECK : %f", rl_neckangle);
    // ROS_INFO("------------------------- RL NECK Angle ----------------------------");
}

// 90[deg] : Initial Propose (Look straight)
// CCW (+)
void Callback::Move_UD_NeckAngle()
{
    double res_ud_neck = srv_SendMotion.response.ud_neckangle;
    ud_neckangle = 90 - res_ud_neck;
    pick_Ptr->NC_th[1] = ud_neckangle * DEG2RAD;
    srv_SendMotion.request.UD_finish = true;
    // ROS_WARN("UD_NECK : %f", res_ud_neck);
    // ROS_INFO("------------------------- UD NECK Angle ----------------------------");
}

void Callback::TATA()
{
    double res_turn_angle = srv_SendMotion.response.turn_angle;

    if (res_turn_angle != 0)
    {
        turn_angle = res_turn_angle * DEG2RAD;
        trajectoryPtr->Make_turn_trajectory(turn_angle);
        index_angle = 0;
    }
    ROS_WARN("TURN_ANGLE : %f", res_turn_angle);
    ROS_INFO("------------------------- TURN_ANGLE ----------------------------");
}

/// 재민이형 긴급정지에 대한 코드 여기다가 넣으면 됨 ///
void Callback::Emergency()
{
    bool res_emergency = srv_SendMotion.response.emergency;
    emergency_ = res_emergency;
    if (emergency_ == false)
    {
        on_emergency = false;
        stop_indext = 0;
        // mode = 0;
    }
    else if (emergency_ == true)
    {
        on_emergency = true;
        stop_indext = 0;
        // mode = 0;
    }
    // ROS_ERROR("EMERGENCY : %s", emergency_ ? "True" : "False");
    // ROS_INFO("------------------------- EMERGENCY ----------------------------");
}

void Callback::Check_FSR()
{
    if (check_indext == 9 && R_value == 0) // 왼발 들때 오른발 착지 확인
    {
        indext -= 1;
    }
    else if (check_indext == 77 && L_value == 0) // 오른발 들때 왼발 착지 확인
    {
        indext -= 1;
    }
}

void Callback::Motion_Info()
{
    int8_t res_mode = srv_SendMotion.response.select_motion;
    float res_distance = srv_SendMotion.response.distance;
    string tmp_motion;
    switch (res_mode)
    {
    case Motion_Index::InitPose:
        tmp_motion = Str_InitPose;
        break;

    case Motion_Index::Forward_2step:
        tmp_motion = Str_Forward_2step;
        break;

    case Motion_Index::Left_2step:
        tmp_motion = Str_Left_2step;
        break;

    case Motion_Index::Step_in_place:
        tmp_motion = Str_Step_in_place;
        break;

    case Motion_Index::Right_2step:
        tmp_motion = Str_Right_2step;
        break;

    case Motion_Index::ForWard_fast4step:
        tmp_motion = Str_ForWard_fast4step;
        break;

    case Motion_Index::Forward_Nstep:
        tmp_motion = Str_Forward_Nstep;
        break;

    case Motion_Index::Shoot:
        tmp_motion = Str_Shoot;
        break;

    case Motion_Index::Forward_Halfstep:
        tmp_motion = Str_Forward_Halfstep;
        break;

    case Motion_Index::Left_Halfstep:
        tmp_motion = Str_Left_Halfstep;
        break;

    case Motion_Index::Right_Halfstep:
        tmp_motion = Str_Right_Halfstep;
        break;

    case Motion_Index::Back_Halfstep:
        tmp_motion = Str_Back_Halfstep;
        break;

    case Motion_Index::Right_1step:
        tmp_motion = Str_Right_1step;
        break;

    case Motion_Index::Left_1step:
        tmp_motion = Str_Left_1step;
        break;

    case Motion_Index::FWD_UP:
        tmp_motion = Str_FWD_UP;
        break;

    case Motion_Index::BWD_UP:
        tmp_motion = Str_BWD_UP;
        break;

    case Motion_Index::Forward_1step:
        tmp_motion = Str_Forward_1step;
        break;

    case Motion_Index::Right_6step:
        tmp_motion = Right_6step;
        break;

    case Motion_Index::Left_6step:
        tmp_motion = Left_6step;
        break;

    case Motion_Index::Ready_to_throw:
        tmp_motion = Str_Ready_to_throw;
        break;

    case Motion_Index::NONE:
        tmp_motion = Str_NONE;
        break;

    case Motion_Index::Gen2_Walking:
        tmp_motion = Str_Gen2_Walking;
    break;

    case Motion_Index::Huddle_Jump_V2:
        tmp_motion = Str_Huddle_Jump;
    break;

    case Motion_Index::Picking_Ball:
        tmp_motion = Str_Picking_Ball;
    break;

    case Motion_Index::Back_2step:
        tmp_motion = Str_Back_2step;
    break;

    case Motion_Index::FINISH:
        tmp_motion = Str_FINISH;
    break;
    
    }



    if (res_mode == Motion_Index::Forward_Nstep)
    {
        ROS_WARN("Motion_Index : %s", tmp_motion.c_str());
        ROS_WARN("Distance : %f", res_distance);
        ROS_INFO("------------------------- Select Motion ----------------------------");
    }

    else if (res_mode == Motion_Index::InitPose)
    {
        ROS_ERROR("Motion_Index : %s", tmp_motion.c_str());
        ROS_INFO("------------------------- Select Motion ----------------------------");
    }

    else
    {
        ROS_WARN("Motion_Index : %s", tmp_motion.c_str());
        ROS_INFO("------------------------- Select Motion ----------------------------");
    }
    // if (mode = Motion_Index::START)
    // {
    //     // ROS_ERROR(Str_START.c_str());
    //     // ROS_INFO("------------------------- Select Motion ----------------------------");
    // }
}

void Callback::SelectMotion()
{
    int8_t res_mode = srv_SendMotion.response.select_motion;
    float res_distance = srv_SendMotion.response.distance;
    if (res_mode == Motion_Index::InitPose)
    {
        indext = 0;
    
                trajectoryPtr->Change_Freq(2);

        mode = Motion_Index::InitPose;
        trajectoryPtr->Ref_RL_x = MatrixXd::Zero(1, 30);
        trajectoryPtr->Ref_LL_x = MatrixXd::Zero(1, 30);
        trajectoryPtr->Ref_RL_y = -0.06 * MatrixXd::Ones(1, 30);
        trajectoryPtr->Ref_LL_y = 0.06 * MatrixXd::Ones(1, 30);
        trajectoryPtr->Ref_RL_z = MatrixXd::Zero(1, 30);
        trajectoryPtr->Ref_LL_z = MatrixXd::Zero(1, 30);
        pick_Ptr->Ref_WT_th = MatrixXd::Zero(1, 30);
        pick_Ptr->Ref_RA_th = MatrixXd::Zero(4, 30);
        pick_Ptr->Ref_LA_th = MatrixXd::Zero(4, 30);
        pick_Ptr->Ref_NC_th = MatrixXd::Zero(2, 30);

        // trajectoryPtr->Ref_RL_x = MatrixXd::Zero(1, 675);
        // trajectoryPtr->Ref_LL_x = MatrixXd::Zero(1, 675);
        // trajectoryPtr->Ref_RL_y = -0.06 * MatrixXd::Ones(1, 675);
        // trajectoryPtr->Ref_LL_y = 0.06 * MatrixXd::Ones(1, 675);
        // trajectoryPtr->Ref_RL_z = MatrixXd::Zero(1, 675);
        // trajectoryPtr->Ref_LL_z = MatrixXd::Zero(1, 675);

    }

    else if (res_mode == Motion_Index::Forward_2step)
    {
        indext = 0;
        trajectoryPtr->Change_Freq(2);

        mode = Motion_Index::Forward_2step;
        IK_Ptr->Change_Com_Height(30);
        trajectoryPtr->Go_Straight_start(0.05, 0.25, 0.05);
        IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
        IK_Ptr->Change_Angle_Compensation(3, 4, 2, 2, 2, 2, 2, -2); 
        IK_Ptr->Set_Angle_Compensation(135);
        trajectoryPtr->Stop_Trajectory_straightwalk(0.05);

    }

    else if (res_mode == Motion_Index::Forward_1step)
    {
        indext = 0;

        trajectoryPtr->Change_Freq(2);

        mode = Motion_Index::Forward_1step;
        IK_Ptr->Change_Com_Height(30);
        trajectoryPtr->Go_Straight(0.05, 0.15, 0.05);
        IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
        IK_Ptr->Change_Angle_Compensation(3, 4, 2, 2, 2, 2, 2, -2); 
        IK_Ptr->Set_Angle_Compensation(135);
        trajectoryPtr->Stop_Trajectory_straightwalk(0.05);
    }

    else if (res_mode == Motion_Index::Left_2step)
    {
        indext = 0;
        trajectoryPtr->Change_Freq(2);

        mode = Motion_Index::Left_2step;
        IK_Ptr->Change_Com_Height(30);
        trajectoryPtr->Side_Left2();
        IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
        IK_Ptr->Change_Angle_Compensation(3, 2, 0, 0, 1.5, 3.5, 0, 0);
        IK_Ptr->Set_Angle_Compensation(135);
    }

    else if (res_mode == Motion_Index::Step_in_place)
    {
        indext = 0;

        trajectoryPtr->Change_Freq(2);

        mode = Motion_Index::Step_in_place;
        IK_Ptr->Change_Com_Height(30);
        trajectoryPtr->Step_in_place(0.02, 0.1, 0.025);
        IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
        IK_Ptr->Change_Angle_Compensation(3, 4, 2, 2, 2, 2, 2, -2); 
        IK_Ptr->Set_Angle_Compensation(135);
    }

    else if (res_mode == Motion_Index::Right_2step)
    {
        indext = 0;

                trajectoryPtr->Change_Freq(2);

        mode = Motion_Index::Right_2step;
        IK_Ptr->Change_Com_Height(30);
        trajectoryPtr->Side_Right2();
        IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
        IK_Ptr->Change_Angle_Compensation(2, 4, 1, 0, 2, 4, -1, 0);
        IK_Ptr->Set_Angle_Compensation(135);

        
    }

    else if (res_mode == Motion_Index::Forward_Nstep)
    {
        // mode = Motion_Index::Forward_Nstep;
        // IK_Ptr->Change_Com_Height(30);
        // trajectoryPtr->Go_Straight(0.05, res_distance, 0.05);
        // IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
        // IK_Ptr->Change_Angle_Compensation(3.5, 3.5, 0, 3.5, 3.5, 3.5, 0, -3.5);
        // IK_Ptr->Set_Angle_Compensation(135);
        // indext = 0;
        indext = 0;
                trajectoryPtr->Change_Freq(2);
        mode = Motion_Index::Forward_Nstep;
        IK_Ptr->Change_Com_Height(30);
        trajectoryPtr->Go_Straight_start(0.05, 2.2, 0.05);
        IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
        IK_Ptr->Change_Angle_Compensation(3, 3, 2, 2, 2, 2, 2, -2); 
        IK_Ptr->Set_Angle_Compensation(135);
        trajectoryPtr->Stop_Trajectory_straightwalk(0.05);

    }

    else if (res_mode == Motion_Index::Huddle_Jump)
    {
        indext = 0;

                trajectoryPtr->Change_Freq(2);

        mode = Motion_Index::Huddle_Jump;
        IK_Ptr->Change_Com_Height(30);
        trajectoryPtr->Huddle_Motion(0.22, 0.14, 0.05);
        IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
        IK_Ptr->Change_Angle_Compensation(6.5, 3.5, 0, 3.5, 6.5, 3.5, 0, -3.5);
        IK_Ptr->Set_Angle_Compensation(135);

    }

    else if (res_mode == Motion_Index::Huddle_Jump_V2)
    {
        indext = 0;

                trajectoryPtr->Change_Freq(2);

        mode = Motion_Index::Huddle_Jump_V2;
        IK_Ptr->Change_Com_Height(30);
        trajectoryPtr->Huddle_Motion_V2(0.22, 0.1, 0);

    }

    else if (res_mode == Motion_Index::ForWard_fast4step) // 2step
    {
        indext = 0;

        mode = Motion_Index::ForWard_fast4step;
        IK_Ptr->Change_Com_Height(30);
        trajectoryPtr->Freq_Change_Straight(0.05, 0.2, 0.05, 1);
        IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
        IK_Ptr->Change_Angle_Compensation(3, 3, 2, 2, 3, 2, 2, -2);   
        IK_Ptr->Set_Angle_Compensation(67);

    }
    
    else if (res_mode == Motion_Index::Right_1step)
    {
        indext = 0;

                trajectoryPtr->Change_Freq(2);

        mode = Motion_Index::Right_1step;
        IK_Ptr->Change_Com_Height(30);
        trajectoryPtr->Side_Right1(0.06);
        IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
        IK_Ptr->Change_Angle_Compensation(2, 4, 0, 0, 2, 4, 0, 0);
        IK_Ptr->Set_Angle_Compensation(135);

    }

    else if (res_mode == Motion_Index::Left_1step)
    {
        indext = 0;

                trajectoryPtr->Change_Freq(2);

        mode = Motion_Index::Left_1step;
        IK_Ptr->Change_Com_Height(30);
        trajectoryPtr->Side_Left1(0.06);
        IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
        IK_Ptr->Change_Angle_Compensation(2, 4, 0, 0, 2, 4, 0, 0);
        IK_Ptr->Set_Angle_Compensation(135);

    }

    else if (res_mode == Motion_Index::Forward_Halfstep)
    {
        indext = 0;
        trajectoryPtr->Change_Freq(2);

        mode = Motion_Index::Forward_Halfstep;
        IK_Ptr->Change_Com_Height(30);
        trajectoryPtr->Go_Straight(0.01, 0.03, 0.025);
        IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
        IK_Ptr->Change_Angle_Compensation(3, 4, 2, 2, 2, 2, 2, -2); 
        IK_Ptr->Set_Angle_Compensation(135);
    }

    else if (res_mode == Motion_Index::Right_Halfstep)
    {
        indext = 0;
        trajectoryPtr->Change_Freq(2);

        mode = Motion_Index::Right_Halfstep;
        IK_Ptr->Change_Com_Height(30);
        trajectoryPtr->Side_Right1(0.03);
        IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
        IK_Ptr->Change_Angle_Compensation(2, 7, 1, 1, 2, 4, 1, -1);
        IK_Ptr->Set_Angle_Compensation(135);
    }

    else if (res_mode == Motion_Index::Left_Halfstep)
    {
        indext = 0;

                trajectoryPtr->Change_Freq(2);

        mode = Motion_Index::Left_Halfstep;
        IK_Ptr->Change_Com_Height(30);
        trajectoryPtr->Side_Left1(0.03);
        IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
        IK_Ptr->Change_Angle_Compensation(2, 4, 1, 1, 2, 7, 1, -1);
        IK_Ptr->Set_Angle_Compensation(135);

    }

    else if (res_mode == Motion_Index::Right_6step)
    {
        indext = 0;
        trajectoryPtr->Change_Freq(2);

        mode = Motion_Index::Right_6step;
        IK_Ptr->Change_Com_Height(30);
        trajectoryPtr->Side_Right6();
        IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
        IK_Ptr->Change_Angle_Compensation(3.5, 3.5, 0, 0, 3.5, 3.5, 0, 0);
        IK_Ptr->Set_Angle_Compensation(135);
    }

    else if (res_mode == Motion_Index::Left_6step)
    {
        indext = 0;

                trajectoryPtr->Change_Freq(2);

        mode = Motion_Index::Left_6step;
        IK_Ptr->Change_Com_Height(30);
        trajectoryPtr->Side_Left6();
        IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
        IK_Ptr->Change_Angle_Compensation(3.5, 3.5, 0, 0, 3.5, 3.5, 0, 0);
        IK_Ptr->Set_Angle_Compensation(135);
    }

    else if (res_mode == Motion_Index::Ready_to_throw)
    {
        indext = 0;

                trajectoryPtr->Change_Freq(2);

        mode = Motion_Index::Ready_to_throw;
        IK_Ptr->Change_Com_Height(30);
        trajectoryPtr->Ref_RL_x = MatrixXd::Zero(1, 500);
        trajectoryPtr->Ref_LL_x = MatrixXd::Zero(1, 500);
        trajectoryPtr->Ref_RL_y = -0.06 * MatrixXd::Ones(1, 500);
        trajectoryPtr->Ref_LL_y = 0.06 * MatrixXd::Ones(1, 500);
        trajectoryPtr->Ref_RL_z = trajectoryPtr->RF_zsimulation_sitdown3(-0.025);
        trajectoryPtr->Ref_LL_z = trajectoryPtr->LF_zsimulation_sitdown3(-0.025);

        pick_Ptr->WT_Trajectory(10,trajectoryPtr->Ref_RL_x.cols());
        pick_Ptr->RA_Trajectory(180,22,-50,50,trajectoryPtr->Ref_RL_x.cols());
        pick_Ptr->LA_Trajectory(0,0,0,0,trajectoryPtr->Ref_RL_x.cols());
        pick_Ptr->NC_Trajectory(0,0,trajectoryPtr->Ref_RL_x.cols());

    }

    else if (res_mode == Motion_Index::Shoot)
    {
        indext = 0;

                trajectoryPtr->Change_Freq(2);

        mode = Motion_Index::Shoot;
        IK_Ptr->Change_Com_Height(5);
        trajectoryPtr->Ref_RL_x = MatrixXd::Zero(1, 500);
        trajectoryPtr->Ref_LL_x = MatrixXd::Zero(1, 500);
        trajectoryPtr->Ref_RL_y = -0.06 * MatrixXd::Ones(1, 500);
        trajectoryPtr->Ref_LL_y = 0.06 * MatrixXd::Ones(1, 500);
        trajectoryPtr->Ref_RL_z = MatrixXd::Zero(1, 500);;
        trajectoryPtr->Ref_LL_z = MatrixXd::Zero(1, 500);;
    }

    else if (res_mode == Motion_Index::FINISH)
    {
        indext = 0;

                trajectoryPtr->Change_Freq(2);

        mode = Motion_Index::FINISH;
        IK_Ptr->Change_Com_Height(30);
        trajectoryPtr->Ref_RL_x = MatrixXd::Zero(1, 500);
        trajectoryPtr->Ref_LL_x = MatrixXd::Zero(1, 500);
        trajectoryPtr->Ref_RL_y = -0.06 * MatrixXd::Ones(1, 500);
        trajectoryPtr->Ref_LL_y = 0.06 * MatrixXd::Ones(1, 500);
        trajectoryPtr->Ref_RL_z = trajectoryPtr->RF_zsimulation_standup3(-0.025);
        trajectoryPtr->Ref_LL_z = trajectoryPtr->LF_zsimulation_standup3(-0.025);

        pick_Ptr->WT_Trajectory(-20,trajectoryPtr->Ref_RL_x.cols());
        pick_Ptr->RA_Trajectory(-150,-22,110,-50,trajectoryPtr->Ref_RL_x.cols());
        pick_Ptr->LA_Trajectory(0,0,0,0,trajectoryPtr->Ref_RL_x.cols());
        pick_Ptr->NC_Trajectory(0,0,trajectoryPtr->Ref_RL_x.cols());

    }

    else if (res_mode == Motion_Index::Gen2_Walking)
    {

        indext = 0;
        mode = Motion_Index::Gen2_Walking;
        
        IK_Ptr->Change_Com_Height(30);
        trajectoryPtr->Freq_Change_Straight(0.05, 0.5, 0.05, 1); //
        IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
        IK_Ptr->Change_Angle_Compensation(3.5, 3.5, 0, 0, 3.5, 3.5, 0, 0);
        IK_Ptr->Set_Angle_Compensation(67);


        // pick_Ptr->WT_Trajectory(10,trajectoryPtr->Ref_RL_x.cols());
        // pick_Ptr->RA_Trajectory(45,15,-30,10,trajectoryPtr->Ref_RL_x.cols());
        // pick_Ptr->LA_Trajectory(30,0,-30,0,trajectoryPtr->Ref_RL_x.cols());
        // pick_Ptr->NC_Trajectory(-10,0,trajectoryPtr->Ref_RL_x.cols());
    }

    else if (res_mode == Motion_Index::Picking_Ball)
    {

        indext = 0;
        mode = Motion_Index::Picking_Ball;
        trajectoryPtr -> Picking_Motion(300, 150, 0.165);

    }


    else if (res_mode == Motion_Index::Back_Halfstep)
    {

        indext = 0;
        trajectoryPtr->Change_Freq(2);

        mode = Motion_Index::Back_Halfstep;
        IK_Ptr->Change_Com_Height(30);
        trajectoryPtr->Go_Back_Straight(-0.04, -0.12, 0.05);
        IK_Ptr->Change_Angle_Compensation(3, 4, 2, 2, 2, 2, 2, -2); 
        IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
        IK_Ptr->Set_Angle_Compensation(135);
    }
    
    else if (res_mode == Motion_Index::Back_2step)
    {

        indext = 0;
        trajectoryPtr->Change_Freq(2);

        mode = Motion_Index::Back_2step;
        IK_Ptr->Change_Com_Height(30);
        trajectoryPtr->Go_Back_Straight(-0.04, -0.2, 0.05);
        IK_Ptr->Change_Angle_Compensation(3, 4, 2, 2, 2, 2, 2, -2); 
        IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
        IK_Ptr->Set_Angle_Compensation(135);
    }

    else if (res_mode == Motion_Index::START)
    {
        //fast_20step
        indext = 0;
        mode = Motion_Index::START;
        IK_Ptr->Change_Com_Height(30);
        trajectoryPtr->Freq_Change_Straight(0.05, 1.05, 0.05, 1); 
        IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
        IK_Ptr->Change_Angle_Compensation(3, 3, 2, 2, 2, 2, 2, -2); 
        IK_Ptr->Set_Angle_Compensation(67);
        trajectoryPtr->Change_Freq(2);
    }
    // ROS_WARN("Distance(%f)", res_distance);
}

void Callback::Write_All_Theta()
{
                // ROS_INFO("%d", srv_SendMotion.request.SM_finish);

    double thro = 0;



    check_indext = indext % walktime_n;
    if (turn_angle > 0 && check_indext == 0 && indext > 0.5 * walktime_n && indext < trajectoryPtr->Ref_RL_x.cols() - walktime_n * 0.5)
    {
        turn_left = true;
    }

    if (turn_angle < 0 && check_indext == 67 && indext > 0.5 * walktime_n && indext < trajectoryPtr->Ref_RL_x.cols() - walktime_n * 0.5)
    {
        turn_right = true;
    }

    if (turn_right || turn_left)
    {
        srv_SendMotion.request.TA_finish = false;
    }

    if (emergency == 0)
    {
        indext += 1;

        if (mode == 0)
        {
            IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);

        }

        else if (mode == Motion_Index::Forward_2step || mode == Motion_Index::Step_in_place || mode == Motion_Index::Forward_Halfstep || mode == Motion_Index::Forward_1step)
        {
            IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
            IK_Ptr->Angle_Compensation(indext, trajectoryPtr->Ref_RL_x.cols());
            if( mode != Motion_Index::Forward_Halfstep ){
                if (turn_left)
                {
                    IK_Ptr->LL_th[0] = trajectoryPtr->Turn_Trajectory(index_angle);
                    step = IK_Ptr->LL_th[0];
                    index_angle += 1;
                    if (index_angle > walktime_n - 1)
                    {
                        turn_angle = 0;
                        turn_left = false;
                        srv_SendMotion.request.TA_finish = true;
                    }
                }
                if (turn_right)
                {
                    IK_Ptr->RL_th[0] = trajectoryPtr->Turn_Trajectory(index_angle);
                    step = IK_Ptr->RL_th[0];
                    index_angle += 1;
                    if (index_angle > walktime_n - 1)
                    {
                        turn_angle = 0;
                        turn_right = false;
                        srv_SendMotion.request.TA_finish = true;
                    }
                }
            }

            // Check_FSR();
        }
        else if (mode == Motion_Index::Left_2step || mode == Motion_Index::Left_Halfstep || mode == Motion_Index::Left_6step)
        {
            IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
            IK_Ptr->Angle_Compensation_Leftwalk(indext);
        }

        else if (mode == Motion_Index::Right_2step || mode == Motion_Index::Right_Halfstep || mode == Motion_Index::Right_6step)
        {
            IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
            IK_Ptr->Angle_Compensation_Rightwalk(indext);
        }

        else if (mode == Motion_Index::Huddle_Jump)
        {
            IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
            IK_Ptr->Angle_Compensation_Huddle(indext);
        }


        else if (mode == Motion_Index::Huddle_Jump_V2)
        {

            //             if(indext > 1180){
            //     indext = 1180;
            // }

            IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
            pick_Ptr->Huddle(trajectoryPtr->Ref_RL_x, indext, RL_th1, LL_th1,RL_th2, LL_th2, HS, SR);

        }

        else if (mode == Motion_Index::ForWard_fast4step)
        {

            IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
            IK_Ptr->Fast_Angle_Compensation(indext);
        }

        else if (mode == Motion_Index::Forward_Nstep)
        {

            IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
            IK_Ptr->Angle_Compensation(indext, trajectoryPtr->Ref_RL_x.cols());

            if(indext == 500){
                srv_SendMotion.request.ST_finish = false;
            }
        }


        else if (mode == Motion_Index::START)
        {
            IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
            // IK_Ptr->Angle_Compensation(indext, trajectoryPtr->Ref_RL_x.cols());
            IK_Ptr->Fast_Angle_Compensation(indext);

        }
        
        else if (mode == Motion_Index::Ready_to_throw)
        {
            IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
            pick_Ptr->UPBD_SET(pick_Ptr->Ref_WT_th, pick_Ptr->Ref_RA_th, pick_Ptr->Ref_LA_th, pick_Ptr->Ref_NC_th, indext);

        }
        
        else if (mode == Motion_Index::Shoot)
        {
            thro = -10;
            IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
            pick_Ptr->WT_th[0] = 20 * DEG2RAD;
	        pick_Ptr->RA_th[0] = 150 * DEG2RAD;
            pick_Ptr->RA_th[1] = 22 * DEG2RAD;
            pick_Ptr->RA_th[2] = -110 * DEG2RAD;
            pick_Ptr->RA_th[3]  = 50 * DEG2RAD;
        }

        else if (mode == Motion_Index::FINISH)
        {
            IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
            pick_Ptr->UPBD_SET(pick_Ptr->Ref_WT_th, pick_Ptr->Ref_RA_th, pick_Ptr->Ref_LA_th, pick_Ptr->Ref_NC_th, indext);

        }

        else if (mode == Motion_Index::Gen2_Walking)
        {
            IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
            IK_Ptr->Fast_Angle_Compensation(indext);
            pick_Ptr->Fast_Arm_Swing(indext, trajectoryPtr->Return_Walktime_n(), trajectoryPtr->Return_Sim_n());
            // pick_Ptr->UPBD_SET(pick_Ptr->Ref_WT_th, pick_Ptr->Ref_RA_th, pick_Ptr->Ref_LA_th, pick_Ptr->Ref_NC_th, indext);
            
        }

        else if (mode == Motion_Index::Picking_Ball)
        {
            IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
            pick_Ptr->Picking(trajectoryPtr->Ref_RL_x, indext, RL_th2, LL_th2);

            // if(indext > 400){
            //     indext = 400;
            // }

        }

        else if (mode == Motion_Index::Back_Halfstep)
        
        {
            IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
            IK_Ptr->Angle_Compensation(indext, trajectoryPtr->Ref_RL_x.cols());
        }

        else if (mode == Motion_Index::Back_2step)
        {
            IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
            IK_Ptr->Angle_Compensation(indext, trajectoryPtr->Ref_RL_x.cols());
        }    

        //line to hoop counter
        if((mode == Motion_Index::ForWard_fast4step || mode == Motion_Index::Forward_2step || mode == Motion_Index::Forward_1step) && indext == 100)
        {
            srv_SendMotion.request.walkcount++;
        }   



    
        if( 0 < indext){

        ROS_INFO("%d", indext);
        }

        if (indext >= trajectoryPtr->Ref_RL_x.cols() && indext != 0)
        {
            indext -= 1;
            srv_SendMotion.request.SM_finish = true;

            // ROS_INFO("xxxxxxxxxxxxxxxxxxxxxxxxx");
        }
        
        else
        {
            srv_SendMotion.request.SM_finish = false;
        }
    }



    All_Theta[0] = -IK_Ptr->RL_th[0];
    All_Theta[1] = IK_Ptr->RL_th[1] -RL_th1 * DEG2RAD - 3 * DEG2RAD;
    All_Theta[2] = IK_Ptr->RL_th[2] -RL_th2 * DEG2RAD - 17 * DEG2RAD; //10.74 right
    All_Theta[3] = -IK_Ptr->RL_th[3]  + 40 * DEG2RAD; //38.34 * DEG2RAD;   
    All_Theta[4] = -IK_Ptr->RL_th[4] + 24.5 * DEG2RAD;
    All_Theta[5] = -IK_Ptr->RL_th[5] + SR * DEG2RAD - 2* DEG2RAD;
    All_Theta[6] = -IK_Ptr->LL_th[0];
    All_Theta[7] = IK_Ptr->LL_th[1] +LL_th1 * DEG2RAD + 2 * DEG2RAD;
    All_Theta[8] = -IK_Ptr->LL_th[2] +LL_th2 * DEG2RAD + 17 * DEG2RAD; //left
    All_Theta[9] = IK_Ptr->LL_th[3] - 40 * DEG2RAD; //40.34 * DEG2RAD;  
    All_Theta[10] = IK_Ptr->LL_th[4] - HS * DEG2RAD - 21.22 * DEG2RAD;
    All_Theta[11] = -IK_Ptr->LL_th[5] + SR * DEG2RAD; - 2 * DEG2RAD;


    //upper_body
    All_Theta[12] = pick_Ptr->WT_th[0] + step + 0 * DEG2RAD;  //waist
    All_Theta[13] = pick_Ptr->LA_th[0] + 90 * DEG2RAD; //L_arm
    All_Theta[14] = pick_Ptr->RA_th[0] - 90 * DEG2RAD; //R_arm
    All_Theta[15] = pick_Ptr->LA_th[1] - 60 * DEG2RAD;//L_arm
    All_Theta[16] = pick_Ptr->RA_th[1] + 60 * DEG2RAD; //R_arm
    All_Theta[17] = pick_Ptr->LA_th[2] - 90 * DEG2RAD;//L_elbow
    All_Theta[18] = pick_Ptr->RA_th[2] + 90 * DEG2RAD; //R_elbow
    All_Theta[19] = pick_Ptr->LA_th[3] - 45 * DEG2RAD; //L_hand
    All_Theta[20] = pick_Ptr->RA_th[3] + 45 * DEG2RAD; //R_hand
    All_Theta[21] = pick_Ptr->NC_th[0] + 0 * DEG2RAD; //neck_RL
    All_Theta[22] = pick_Ptr->NC_th[1] + 6 * DEG2RAD; //neck_UD

    // ROS_INFO("%f", All_Theta[14]);

    //     All_Theta[13] = -90 * DEG2RAD;
    // All_Theta[14] = 90 * DEG2RAD;
    // All_Theta[15] = -30 * DEG2RAD;
    // All_Theta[16] = 60 * DEG2RAD;
    // All_Theta[17] = -90 * DEG2RAD;
    // All_Theta[18] = 90 * DEG2RAD;
    // All_Theta[19] = 0 * DEG2RAD;
    // All_Theta[20] = 0 * DEG2RAD;
    

}


void Callback::Set_Callback()
{
    trajectoryPtr->Go_Straight(0.05, 0.5, 0.05);
    trajectoryPtr->Ref_RL_x = MatrixXd::Zero(1, 10);
    trajectoryPtr->Ref_LL_x = MatrixXd::Zero(1, 10);
    trajectoryPtr->Ref_RL_y = -0.06 * MatrixXd::Ones(1, 10);
    trajectoryPtr->Ref_LL_y = 0.06 * MatrixXd::Ones(1, 10);
    trajectoryPtr->Ref_RL_z = MatrixXd::Zero(1, 10);
    trajectoryPtr->Ref_LL_z = MatrixXd::Zero(1, 10);
}