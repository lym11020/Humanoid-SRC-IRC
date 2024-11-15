#include "Move_decision.hpp"
#include <numeric>
#include <cmath> // M_PI를 사용하기 위함
#define DEG2RAD (M_PI / 180.0)


//
Move_Decision::Move_Decision(Img_proc *img_procPtr)
    : img_procPtr(img_procPtr),
      FALL_FORWARD_LIMIT(60), //전방 넘어지는 각도 한계
      FALL_BACK_LIMIT(-60), //후방
      SPIN_RATE(30),
      stand_status_(Stand_Status::Stand), //초기자세
      motion_index_(Motion_Index::NONE), //아무 동작도 하지 않는 상태
      stop_fallen_check_(false), //넘어짐검사 비활성화
      Emergency_(false), 
      turn_angle_(0), 
      straightLine(true),
      ProcessON_(true),
      UD_NeckAngle_(UD_CENTER)
{
    // Init ROS
    ros::NodeHandle nh(ros::this_node::getName());

    boost::thread process_thread = boost::thread(boost::bind(&Move_Decision::processThread, this));
    // boost::thread web_process_thread = boost::thread(boost::bind(&Img_proc::webcam_thread, img_procPtr));
    boost::thread depth_process_thread = boost::thread(boost::bind(&Img_proc::realsense_thread, img_procPtr));
    boost::thread queue_thread = boost::thread(boost::bind(&Move_Decision::callbackThread, this));
}

Move_Decision::~Move_Decision()
{
}

// ********************************************** PROCESS THREAD************************************************** //
void Move_Decision::process(bool Switch_ON)
{
    if (Switch_ON)
    {
        tmp_img_proc_line_det_flg_ = img_procPtr->Get_img_proc_line_det();
        tmp_img_proc_no_line_det_flg_ = img_procPtr->Get_img_proc_no_line_det();
        tmp_img_proc_huddle_det_flg_2d_ = img_procPtr->Get_img_proc_huddle_det_2d();
        tmp_img_proc_ball_det_flg_ = img_procPtr->Get_img_proc_ball_det_();
        tmp_img_proc_hoop_det_flg_ = img_procPtr->Get_img_proc_hoop_det_();
        

        ROS_INFO("-------------------------PROCESSTHREAD----------------------------");
        // ROS_WARN("tmp_img_proc_line_det_flg_ : %d", tmp_img_proc_line_det_flg_);
        // ROS_WARN("tmp_img_proc_no_line_det_flg_ : %d", tmp_img_proc_no_line_det_flg_);
        // ROS_WARN("tmp_img_proc_huddle_det_flg_2d_ : %d", tmp_img_proc_huddle_det_flg_2d_);
        // ROS_WARN("tmp_img_proc_ball_det_flg_ : %d", tmp_img_proc_ball_det_flg_);
        // ROS_WARN("is_in_pick_mode_ : %d", is_in_pick_mode_);
        // ROS_WARN("is_in_huddle_mode_ : %d", is_in_huddle_mode_);
        // ROS_WARN("tmp_pick_seq : %d", tmp_pick_seq);
        // ROS_WARN("tmp_huddle_seq : %d", tmp_huddle_seq);
        // ROS_WARN("is_in_shoot_mode_ : %d", is_in_shoot_mode_);

        // ROS_WARN("START? START? : %d", Get_ST_req_finish());
        ROS_WARN("count? count? count? : %d", Get_walkcount_req_());
        ROS_WARN("NCangle? NCangle? NCangle? : %f", Get_UD_NeckAngle());
        
        
    }
    // if(Get_ST_req_finish() == true){
    //     ROS_INFO("Entering START MODE");
    //     Set_running_mode_(Running_Mode::START_MODE);
    //     return;
    // }
    // PICK mode
    if (is_in_pick_mode_ || tmp_img_proc_ball_det_flg_)
    {
        if (!is_in_pick_mode_)
        {
            ROS_INFO("Entering PICK MODE");
            tmp_pick_seq = 0;
            AllModereset(Running_Mode::PICK_MODE);
        }
        return;
    }

    // SHOOT mode
    if (is_in_shoot_mode_ || tmp_img_proc_hoop_det_flg_)
    {
        if (!is_in_shoot_mode_)
        {
            ROS_INFO("Entering PICK MODE");
            AllModereset(Running_Mode::SHOOT_MODE);
            tmp_shoot_seq = 0;
        }
        return;
    }

    // HUDDLE mode
    if (is_in_huddle_mode_ || tmp_img_proc_huddle_det_flg_2d_)
    {
        if (!is_in_huddle_mode_)
        {
            ROS_INFO("Entering HUDDLE MODE");
            AllModereset(Running_Mode::HUDDLE_MODE);
            tmp_huddle_seq = 0;
        }
        return;
    }

    // LINE mode
    if (tmp_img_proc_line_det_flg_)
    {
        ROS_INFO("Entering LINE MODE");
        AllModereset(Running_Mode::LINE_MODE);
        return;
    }

    // NO LINE mode
    if (tmp_img_proc_no_line_det_flg_)
    {
        ROS_INFO("Entering NO LINE MODE");
        AllModereset(Running_Mode::NO_LINE_MODE);
        return;
    }


}
void Move_Decision::processThread()
{
    // set node loop rate
    ros::Rate loop_rate(SPIN_RATE);

    // node loop
    while (ros::ok())
    {
        ROS_INFO("-------------------------PROCESSTHREAD----------------------------");
        ROS_INFO("Get_ProcessON : %s", Get_ProcessON() ? "TRUE" : "FALSE");
        process(Get_ProcessON());
        loop_rate.sleep();
    }
}

// ********************************************** CALLBACK THREAD ************************************************** //
void Move_Decision::Running_Mode_Decision()
{
    switch (Get_running_mode_())
    {

        // case Running_Mode::START_MODE:
        //     if(Get_ST_req_finish() == true){
        //        START_LINE_mode();
        //        img_procPtr -> Set_UD_NeckAngle_(Get_UD_NeckAngle());

        //     }
        //     else{
    
        //     }
        //     break;

        case Running_Mode::PICK_MODE:
            if (is_in_pick_mode_)
            {
                PICK_mode();
                img_procPtr -> Set_UD_NeckAngle_(Get_UD_NeckAngle());
            }
            else
            {
                tmp_pick_seq = 0;
            }
            break;

        case Running_Mode::HUDDLE_MODE:
            if (is_in_huddle_mode_)
            {
                HUDDLE_mode2();
                img_procPtr -> Set_UD_NeckAngle_(Get_UD_NeckAngle());
            }
            else
            {
                tmp_huddle_seq = 0;
            }
            break;

        case Running_Mode::SHOOT_MODE:
            if (is_in_shoot_mode_)
            {
                SHOOT_mode();
                img_procPtr -> Set_UD_NeckAngle_(Get_UD_NeckAngle());
            }
            else
            {
                tmp_shoot_seq = 0;
            }
            break;


        case Running_Mode::LINE_MODE:
            LINE_mode();
            img_procPtr -> Set_UD_NeckAngle_(Get_UD_NeckAngle());
            break;

        case Running_Mode::NO_LINE_MODE:
            NOLINE_mode();
            img_procPtr -> Set_UD_NeckAngle_(Get_UD_NeckAngle());
            break;

        default:
            // ROS_WARN("Unexpected mode, defaulting to LINE_MODE");
            // set_pick_mode(false);
            // set_huddle_mode(false);
            // AllModereset(Running_Mode::LINE_MODE);
            // LINE_mode();
            // START_LINE_mode();
            img_procPtr -> Set_UD_NeckAngle_(Get_UD_NeckAngle());
            break;
    }
}

void Move_Decision::LINE_mode()
{
    line_gradient = img_procPtr->Get_gradient();
    StraightLineDecision(line_gradient, MARGIN_GRADIENT);
    line_actual_angle = Get_turn_angle_();
    line_motion = Motion_Index::InitPose;
    line_ud_neckangle = Get_UD_NeckAngle();
   
    // Initializing
    huddle_seq_finish = false;
    Set_huddle_det_stop_flg(false);

    // If SM_req_finish = false -> InitPose
    // Straight Line
    if (straightLine == true)
    {
        if (!Get_turn_angle_on_flg())
        {
            // Left turn
            // To be zero
            if (line_actual_angle > 0)
            {
                line_actual_angle = 0;
                Set_turn_angle_(line_actual_angle);
                Set_turn_angle_on_flg(true);
            }

            // Right turn
            // To be zero
            else if (line_actual_angle < 0)
            {
                line_actual_angle = 0;
                Set_turn_angle_(line_actual_angle);
                Set_turn_angle_on_flg(true);
            }
        }

        if (!Get_select_motion_on_flg())
        {
            line_motion = Motion_Index::ForWard_fast4step;            
            Set_motion_index_(line_motion);
            Set_select_motion_on_flg(true);
        }

        if (!Get_UD_Neck_on_flg())
        {
            Set_UD_NeckAngle(UD_CENTER);
            Set_UD_Neck_on_flg(true);
        }

        ROS_ERROR("STRAIGHT LINE");
    }

    // Non Straight Line
    else if (straightLine == false)
    {
        if (!Get_turn_angle_on_flg())
        {
            // Increase Actual_angle more quickly for larger line_gradient values
            // Counter Clock wise(+) (Turn Angle sign)
            // Gradient : Angle from center of window.x to center of line.x
            // LEFT TURN
            if (line_gradient >= MARGIN_GRADIENT * 5)
            {
                increment = 4;
                ROS_WARN("LEFT_TURN");
            }
            else if (line_gradient >= MARGIN_GRADIENT * 4)
            {
                increment = 3;
                ROS_WARN("LEFT_TURN");
            }
            else if (line_gradient >= MARGIN_GRADIENT * 3)
            {
                increment = 2;
                ROS_WARN("LEFT_TURN");
            }
            else if (line_gradient >= MARGIN_GRADIENT * 2)
            {
                increment = 2;
                ROS_WARN("LEFT_TURN");
            }
            else if (line_gradient > MARGIN_GRADIENT * 1)
            {
                increment = 2;
                ROS_WARN("LEFT_TURN");
            }

            // Decrease Actual_angle relatively slowly for smaller line_gradient values
            // Right Turn
            else if (line_gradient <= -MARGIN_GRADIENT * 5)
            {
                increment = -4;
                ROS_WARN("RIGHT TURN");
            }
            else if (line_gradient <= -MARGIN_GRADIENT * 4)
            {
                increment = -3;
                ROS_WARN("RIGHT TURN");
            }
            else if (line_gradient <= -MARGIN_GRADIENT * 3)
            {
                increment = -2;
                ROS_WARN("RIGHT TURN");
            }
            else if (line_gradient <= -MARGIN_GRADIENT * 2)
            {
                increment = -2;
                ROS_WARN("RIGHT TURN");
            }
            else if (line_gradient < -MARGIN_GRADIENT * 1)
            {
                increment = -2;
                ROS_WARN("RIGHT TURN");
            }
            else
            {
                increment = 0;
            }

            line_actual_angle += increment;
            if (line_actual_angle >= 15)
            {
                line_actual_angle = LINE_TURN;
            }
            else if (line_actual_angle <= -15)
            {
                line_actual_angle = -LINE_TURN;
            }

            Set_turn_angle_(line_actual_angle);
            Set_turn_angle_on_flg(true);
        }

        if (!Get_select_motion_on_flg())
        {
            if (line_gradient > MARGIN_GRADIENT * 3 || line_gradient < -MARGIN_GRADIENT * 3)
            {
                line_motion = Motion_Index::Step_in_place;
            }
            else
            {
                line_motion = Motion_Index::ForWard_fast4step;
            }

            Set_motion_index_(line_motion);
            Set_select_motion_on_flg(true);
        }

        if (!Get_UD_Neck_on_flg())
        {
            Set_UD_NeckAngle(UD_CENTER);
            Set_UD_Neck_on_flg(true);
        }

        ROS_ERROR("NO STRAIGHT LINE");

        // TEST
        //  Set_RL_Neck_on_flg(true);
        //  Set_RL_NeckAngle(Actual_angle);
    }


    if (Get_pickmotion_det_flg_()) //
        {
            if(real_once_flg==false){
                tmp_real_walkcount = Get_walkcount_req_();
                real_once_flg = true;
            }

            if((Get_walkcount_req_() - tmp_real_walkcount)  >=4){
                if(once_flg==false){
                    tmp_walkcount = Get_walkcount_req_();
                    once_flg = true;
                    }
                if((Get_walkcount_req_() - tmp_walkcount)  == 1){
                
                    line_motion = Motion_Index::InitPose;
                    Set_motion_index_(line_motion);
    
                    Set_UD_NeckAngle(HOOP_NECK);
                    if(img_procPtr->Get_hoopcounter_det_flg_() == 0)
                    {
                        sh++;
                    }
                    if(sh == 210 )
                    {
                        Set_UD_NeckAngle(UD_CENTER);
                        sh = 0;
                        once_flg = false;
                
                    }
                }

                if(line_motion == Motion_Index::ForWard_fast4step){
                    line_motion = Motion_Index::Forward_1step;
                    Set_motion_index_(line_motion);
                }
            }
        }

}

void Move_Decision::NOLINE_mode()
{
    // Counter Clock wise(+) (Turn Angle sign)
    // delta_x > 0 : LEFT Window  ->  Left turn (-)
    // delta_x < 0 : RIGHT window ->  Right turn  (+)
    tmp_delta_x = img_procPtr->Get_delta_x();
    noline_actual_angle = Get_turn_angle_();
    noline_motion = Get_motion_index_();

    // Initializing
    huddle_seq_finish = false;
    Set_huddle_det_stop_flg(false);

    if (tmp_delta_x < 0) // Right
    {
        if (!Get_turn_angle_on_flg())
        {
            noline_actual_angle -= 3;
            if (noline_actual_angle < -Angle_ToFindLine)
            {
                noline_actual_angle = -Angle_ToFindLine;
            }
            Set_turn_angle_(noline_actual_angle);
            Set_turn_angle_on_flg(true);
        }

        if (!Get_select_motion_on_flg())
        {
            Set_motion_index_(Motion_Index::Step_in_place);
            Set_select_motion_on_flg(true);
        }

        if (!Get_UD_Neck_on_flg())
        {
            noline_neckangle = UD_CENTER;
            Set_UD_NeckAngle(noline_neckangle);
            Set_UD_Neck_on_flg(true);
        }

        ROS_WARN("RIGHT TURN");
        // ROS_INFO("turn angle : %d", Get_turn_angle_());
    }

    else if (tmp_delta_x > 0) // LEFT
    {
        if (!Get_turn_angle_on_flg())
        {
            noline_actual_angle += 3;
            if (noline_actual_angle > Angle_ToFindLine)
            {
                noline_actual_angle = Angle_ToFindLine;
            }
            Set_turn_angle_(noline_actual_angle);
            Set_turn_angle_on_flg(true);
        }

        if (!Get_select_motion_on_flg())
        {
            Set_motion_index_(Motion_Index::Step_in_place);
            Set_select_motion_on_flg(true);
        }

        ROS_WARN("LEFT_TURN");
    }
}

void Move_Decision::HUDDLE_mode2()
{
    static bool huddle_jump_done = false;

    huddle_actual_angle = Get_turn_angle_();
    huddle_motion = Get_motion_index_();
    img_proc_huddle_angle = img_procPtr->Get_huddle_angle();

    Set_UD_NeckAngle(HUDDLE_NECK);
    Set_UD_Neck_on_flg(true);

    ROS_ERROR("img_proc_huddle_angle : %lf", img_proc_huddle_angle);
    ROS_WARN("Get_foot_huddle_distance : %d", img_procPtr->Get_foot_huddle_distance());

    switch (tmp_huddle_seq)
    {
        case 0: // InitPose (Dummy)
            if (!Get_select_motion_on_flg())
            {
                huddle_motion = Motion_Index::InitPose;
                Set_motion_index_(huddle_motion);
                Set_select_motion_on_flg(true);
            }
            if (finish_past != Get_SM_req_finish())
            {
                finish_past = Get_SM_req_finish();
                if (finish_past) tmp_huddle_seq++;
            }
            break;

        case 1: // Pose Control (Posture(Gradient))
            ROS_ERROR(Str_HUDDLE2_SEQUENCE_0.c_str());
            if (!Get_select_motion_on_flg())
            {
                huddle_motion = Motion_Index::Step_in_place;
                Set_motion_index_(huddle_motion);
                Set_select_motion_on_flg(true);
            }
            if (!Get_turn_angle_on_flg())
            {
                if (std::abs(img_proc_huddle_angle) > 3)
                {
                    huddle_actual_angle = (img_proc_huddle_angle >= 0) ? HUDDLE_TURN : -HUDDLE_TURN;
                    Set_turn_angle_(huddle_actual_angle);
                    Set_turn_angle_on_flg(true);
                }
                else
                {
                    tmp_huddle_seq++;
                }
            }
            break;

        case 2: // Approach to the Huddle
            ROS_ERROR(Str_HUDDLE2_SEQUENCE_1.c_str());
            img_proc_contain_huddle_to_foot = img_procPtr->Get_contain_huddle_to_foot();
            if (!Get_select_motion_on_flg())
            {
                if (!img_proc_contain_huddle_to_foot)
                {
                    huddle_motion = Motion_Index::Forward_Halfstep;
                    Set_motion_index_(huddle_motion);
                    Set_select_motion_on_flg(true);
                    stable_count = 0;
                }
                else
                {
                    stable_count++;
                    if (stable_count >= stable_threshold)
                    {
                        tmp_huddle_seq++;
                        stable_count = 0;
                    }
                }
            }
            break;

        case 3: // Final Pose Control
            ROS_ERROR(Str_HUDDLE2_SEQUENCE_2.c_str());
            if (!Get_select_motion_on_flg())
            {
                huddle_motion = Motion_Index::Step_in_place;
                Set_motion_index_(huddle_motion);
                Set_select_motion_on_flg(true);
            }
            if (!Get_turn_angle_on_flg())
            {
                if (std::abs(img_proc_huddle_angle) > HUDDLE_TURN)
                {
                    huddle_actual_angle = (img_proc_huddle_angle >= 0) ? HUDDLE_TURN : -HUDDLE_TURN;
                    Set_turn_angle_(huddle_actual_angle);
                    Set_turn_angle_on_flg(true);
                }
                else
                {
                    tmp_huddle_seq++;
                }
            }
            break;

        case 4: // HUDDLE_JUMP
            ROS_ERROR(Str_HUDDLE2_SEQUENCE_3.c_str());
            if (!Get_select_motion_on_flg())
            {
                huddle_motion = Motion_Index::Huddle_Jump_V2;
                Set_motion_index_(huddle_motion);
                Set_select_motion_on_flg(true);
            }

            if (finish_past != Get_SM_req_finish())
            {
                finish_past = Get_SM_req_finish();
                if (finish_past)
                {
                    huddle_jump_done = true;
                    // 허들 점프 직후 huddle_done 플래그 설정
                    img_procPtr->Set_img_proc_huddle_done_(true);
                    ROS_INFO("HUDDLE_MODE: Jump completed, setting huddle_done flag");
                    tmp_huddle_seq++;
                    to_be_line_mode++;
                }
            }
            break;

        case 5: // Initializing
            ROS_ERROR(Str_HUDDLE2_SEQUENCE_4.c_str());

            if (!Get_select_motion_on_flg())
            {
                huddle_motion = Motion_Index::InitPose;
                Set_motion_index_(huddle_motion);
                Set_select_motion_on_flg(true);
            }

            if (finish_past != Get_SM_req_finish())
            {
                finish_past = Get_SM_req_finish();
                if (finish_past)
                {
                    tmp_huddle_seq = 0;
                    contain_huddle_Y = false;
                    huddle_posture = false;
                    huddle_seq_finish = true;
                    huddle_jump_done = false;
                    
                    is_in_huddle_mode_ = false;

                    AllModereset(Running_Mode::LINE_MODE);
                    ROS_INFO("HUDDLE_MODE: Completed, switching to LINE_MODE");
                }
            }
            break;
    }

    ROS_INFO("HUDDLE_MODE: Current sequence: %d, Motion index: %d", tmp_huddle_seq, Get_motion_index_());
}
void Move_Decision::ADJUST_mode()
{

}

void Move_Decision::SHOOT_mode()
{
    sh = 0;
    once_flg = false;
    static bool finish_past = false;
    Set_UD_NeckAngle(HOOP_NECK);
    Set_UD_Neck_on_flg(true);
    const double TARGET_X = -59.0;
    const double TARGET_Z_MIN = 0.55;
    const double TARGET_Z_MAX = 0.65;
    const double X_THRESHOLD = 40.0;
    const double X_ADJUST_ANGLE = 8.0;

  


    ROS_DEBUG("SHOOT_MODE: Current sequence: %d", tmp_shoot_seq);

    switch (tmp_shoot_seq)
    {

        case 0: // 공 검출
            if (img_procPtr->Get_img_proc_hoop_det_())
            {
                ROS_INFO("SHOOT_MODE: Hoop detected, moving to initial pose");
                tmp_shoot_seq++;
                Set_UD_NeckAngle(HOOP_NECK);
                Set_UD_Neck_on_flg(true);

            }
            break;

        case 1: // 초기 자세
            if (!Get_select_motion_on_flg())
            {
                ROS_INFO("SHOOT_MODE: Setting initial pose");
                Set_motion_index_(Motion_Index::InitPose);
                Set_select_motion_on_flg(true);
            }
            // Sequence++
            else if (finish_past != Get_SM_req_finish())
            {
                finish_past = Get_SM_req_finish();
                if (finish_past)
                {
                    tmp_shoot_seq++;
                    Set_select_motion_on_flg(false);  // Reset for next motion
                }
            }
            break;


        case 2: // 조정 단계 (x -> z -> x 순서로 조정)
        
            if (!Get_select_motion_on_flg())
            {
                double hoop_x = img_procPtr->Get_hoop_x();
                double hoop_z = img_procPtr->Get_hoop_z();

                bool is_x_aligned = std::abs(hoop_x - TARGET_X) <= X_THRESHOLD;
                bool is_z_aligned = hoop_z >= TARGET_Z_MIN && hoop_z <= TARGET_Z_MAX;

                ROS_INFO("SHOOT_MODE: Current hoop position - X: %f, Z: %f", hoop_x, hoop_z);
                ROS_INFO("SHOOT_MODE: Alignment status - X: %s, Z: %s", 
                         is_x_aligned ? "Yes" : "No",
                         is_z_aligned ? "Yes" : "No");

                // X축이 정렬되지 않은 경우
                if (!is_x_aligned)
                {
                    double x_diff = hoop_x - TARGET_X;
                    double turn_angle = (x_diff > 0) ? -X_ADJUST_ANGLE : X_ADJUST_ANGLE;

                    Set_turn_angle_(turn_angle);
                    Set_turn_angle_on_flg(true);
                    Set_motion_index_(Motion_Index::Step_in_place);
                    ROS_INFO("SHOOT_MODE: X alignment - Turn angle: %f", turn_angle);
                }
                // X축이 정렬되었고 Z축이 정렬되지 않은 경우
                else if (!is_z_aligned)
                {
                    if (hoop_z < TARGET_Z_MAX)
                    {
                        // Z값이 너무 큰 경우 (로봇이 너무 가까이 있는 경우)
                        Set_motion_index_(Motion_Index::Back_Halfstep);
                        ROS_INFO("SHOOT_MODE: Z adjustment - Moving back half step");
                    }
                    else if (hoop_z > TARGET_Z_MIN)
                    {
                        // Z값이 너무 작은 경우 (로봇이 너무 멀리 있는 경우)
                        if (hoop_z >= 0.7)
                        {
                            Set_motion_index_(Motion_Index::Forward_2step);
                            ROS_INFO("SHOOT_MODE: Large Z adjustment - Moving forward 2 steps");
                        }
                        else
                        {
                            Set_motion_index_(Motion_Index::Forward_Halfstep);
                            ROS_INFO("SHOOT_MODE: Fine Z adjustment - Moving forward half step");
                        }
                    }
                }
                // 모든 정렬이 완료된 경우
                else if (is_x_aligned && is_z_aligned)
                {
                    ROS_INFO("SHOOT_MODE: All alignments complete");
                    tmp_shoot_seq++; // 다음 시퀀스로 이동
                    break;
                }

                Set_select_motion_on_flg(true);
            }

            // Sequence++
            // 회전 모션 완료 확인
            else if (finish_past != Get_TA_req_finish() && Get_motion_index_() == Motion_Index::Step_in_place)
            {
                finish_past = Get_TA_req_finish();
                if (finish_past)
                {
                    double hoop_x = img_procPtr->Get_hoop_x();
                    double hoop_z = img_procPtr->Get_hoop_z();
            
                    bool is_x_aligned = std::abs(hoop_x - TARGET_X) <= X_THRESHOLD;
                    bool is_z_aligned = hoop_z >= TARGET_Z_MIN && hoop_z <= TARGET_Z_MAX;
            
                    if (is_x_aligned && is_z_aligned)
                    {
                        ROS_INFO("SHOOT_MODE: Alignment confirmed after rotation, moving to next sequence");
                        tmp_shoot_seq++;
                    }
                    else
                    {
                        ROS_INFO("SHOOT_MODE: Further adjustment needed after rotation");
                    }
                    Set_select_motion_on_flg(false);
                }
            }
            // Sequence++ for Forward/Back steps (보행 동작의 완료 확인)
            else if (finish_past != Get_SM_req_finish() && 
                    (Get_motion_index_() == Motion_Index::Forward_Halfstep || 
                        Get_motion_index_() == Motion_Index::Back_Halfstep || 
                        Get_motion_index_() == Motion_Index::Forward_2step))
            {
                finish_past = Get_SM_req_finish();
                if (finish_past)
                {
                    // 보행 동작 완료 후 다시 후프 위치 확인
                    double hoop_x = img_procPtr->Get_hoop_x();
                    double hoop_z = img_procPtr->Get_hoop_z();
            
                    bool is_x_aligned = std::abs(hoop_x - TARGET_X) <= X_THRESHOLD;
                    bool is_z_aligned = hoop_z >= TARGET_Z_MIN && hoop_z <= TARGET_Z_MAX;
            
                    if (is_x_aligned && is_z_aligned)
                    {
                        ROS_INFO("SHOOT_MODE: Alignment confirmed after step, moving to next sequence");
                        tmp_shoot_seq++;
                    }
                    else
                    {
                        ROS_INFO("SHOOT_MODE: Further adjustment needed after step");
                    }
                    Set_select_motion_on_flg(false);
                }
            }
            break;

        case 3: // Ready_to_throw
            if (!Get_select_motion_on_flg())
            {
                ROS_INFO("SHOOT_MODE: Executing Ready_to_throw motion");
                Set_motion_index_(Motion_Index::Ready_to_throw);
                Set_select_motion_on_flg(true);
                // Set_hoop_det_flg_(false);
                // img_procPtr->Set_img_proc_hoop_det_(false);
            }
            // Sequence++
            else if (finish_past != Get_SM_req_finish())
            {
                finish_past = Get_SM_req_finish();
                if (finish_past)
                {
                    tmp_shoot_seq++;
                    Set_select_motion_on_flg(false);  // Reset for next motion
                }
            }
            break;


        case 4: // SHOOTING_Ball
            if (!Get_select_motion_on_flg())
            {
                ROS_INFO("SHOOT_MODE: Executing ball shooting motion");
                Set_motion_index_(Motion_Index::Shoot);
                Set_select_motion_on_flg(true);
            }
            // Sequence++
            else if (finish_past != Get_SM_req_finish())
            {
                finish_past = Get_SM_req_finish();
                if (finish_past)
                {
                    tmp_shoot_seq++;
                    Set_select_motion_on_flg(false);  // Reset for next motion
                }
            }
            break;
    

        
        case 5: // SHOOTING_Ball
            if (!Get_select_motion_on_flg())
            {
                ROS_INFO("SHOOT_MODE: Executing ball shooting motion");
                Set_motion_index_(Motion_Index::FINISH);
                Set_select_motion_on_flg(true);
            }
            // Sequence++
            else if (finish_past != Get_SM_req_finish())
            {
                finish_past = Get_SM_req_finish();
                if (finish_past)
                {
                    tmp_shoot_seq++;
                    Set_select_motion_on_flg(false);  // Reset for next motion
                }
            }

            
            break;

        case 6: // Back_2step            
            if (!Get_select_motion_on_flg())
            {
                ROS_INFO("PICK_MODE: Executing Back_2step");
                Set_motion_index_(Motion_Index::Back_2step);
                Set_select_motion_on_flg(true);
            }
            else if (finish_past != Get_SM_req_finish())
            {
                finish_past = Get_SM_req_finish();
                if (finish_past)
                {
                    tmp_shoot_seq++;
                    Set_select_motion_on_flg(false);  // Reset for next motion
                }
            }
            break;


        case 7: // 오른쪽으로 회전하며 4번 회전
            static int rotation_count = 0;  // 회전 횟수 카운터

            if (!Get_select_motion_on_flg())
            {
                if (rotation_count >= 6)  // 4번 회전 완료
                {
                    ROS_INFO("PICK_MODE: Completed 4 rotations, moving to initialization");
                    rotation_count = 0;  // 카운터 리셋
                    tmp_shoot_seq++;      // 다음 단계로
                    break;
                }

                ROS_INFO("PICK_MODE: Executing rotation %d/4", rotation_count + 1);
                Set_motion_index_(Motion_Index::Step_in_place);
                Set_turn_angle_(-15.0);  // 10도씩 오른쪽으로 회전
                Set_turn_angle_on_flg(true);
                Set_select_motion_on_flg(true);
            }
            else if (finish_past != Get_TA_req_finish())  // SM_req_finish 대신 TA_req_finish 사용
            {
                finish_past = Get_TA_req_finish();
                if (finish_past)
                {
                    rotation_count++;  // 회전이 완료되었을 때만 카운트 증가
                    ROS_INFO("PICK_MODE: Rotation %d/4 completed", rotation_count);
                    Set_select_motion_on_flg(false);
                }
            }
            break;

        case 8: // 초기화
            if (!Get_select_motion_on_flg())
            {
                ROS_INFO("SHOOT_MODE: Executing initialization motion");
                Set_motion_index_(Motion_Index::InitPose);
                Set_select_motion_on_flg(true);
            }
            // Sequence++
            else if (finish_past != Get_SM_req_finish())
            {
                finish_past = Get_SM_req_finish();
                if (finish_past)
                {
                    tmp_shoot_seq++;
                    Set_select_motion_on_flg(false);  // Reset for next motion
                }
            }
            break;

        case 9: // LINE_MODE로 전환
            Set_UD_NeckAngle(UD_CENTER);
            Set_UD_Neck_on_flg(true);
            ROS_INFO("SHOOT_MODE: Initialization completed, switching to LINE_MODE");
            Set_pickmotion_det_flg_(false);
            tmp_shoot_seq = 0;
            Set_line_det_flg(true);
            Set_select_motion_on_flg(false);
            is_in_shoot_mode_ = false;  
            AllModereset(Running_Mode::LINE_MODE);

            break;

        default:
            Set_motion_index_(Motion_Index::InitPose);
            break;
    }

    ROS_INFO("SHOOT_MODE: Current sequence: %d, Motion index: %d", tmp_shoot_seq, Get_motion_index_());
}
void Move_Decision::PICK_mode()
{
    static bool finish_past = false;
    static int retry_count = 0;
    const int MAX_RETRIES = 3;
    real_once_flg = false;
    

    const double TARGET_X = 68;
    const double TARGET_Y = 74;
    const double TARGET_Z = 0.48;
    const double INITIAL_X_THRESHOLD = 200.0; 
    

    const double X_THRESHOLD = 8.0;
    const double Y_THRESHOLD = 15.0;
    const double Z_THRESHOLD = 0.05;
    

    Set_UD_NeckAngle(PICK_NECK);
    Set_UD_Neck_on_flg(true);

    ROS_DEBUG("PICK_MODE: Current sequence: %d", tmp_pick_seq);

    switch (tmp_pick_seq)
    {
        case 0: // 공 검출
            if (img_procPtr->Get_img_proc_ball_det_())
            {
                ROS_INFO("PICK_MODE: Ball detected, moving to initial pose");
                tmp_pick_seq++;
            }
            break;

        case 1: // 초기 자세
            if (!Get_select_motion_on_flg())
            {
                ROS_INFO("PICK_MODE: Setting initial pose");
                Set_motion_index_(Motion_Index::InitPose);
                Set_select_motion_on_flg(true);
            }
            // Sequence++
            else if (finish_past != Get_SM_req_finish())
            {
                finish_past = Get_SM_req_finish();
                if (finish_past)
                {
                    tmp_pick_seq++;
                    Set_select_motion_on_flg(false);  // Reset for next motion
                }
            }
            break;

       case 2: // 초기 X 좌표 정렬
            if (!Get_select_motion_on_flg())
            {
                bool is_real = img_procPtr->Get_img_proc_ball_det_();
                
                if (is_real)
                {
                    double ball_x = img_procPtr->Get_ball_x();
                    bool is_x_aligned = std::abs(ball_x - TARGET_X) <= INITIAL_X_THRESHOLD;
                    
                    ROS_INFO("PICK_MODE: Initial X alignment - Current X: %f, Target X: %f", ball_x, TARGET_X);
                    
                    if (!is_x_aligned)
                    {
                        if (ball_x > TARGET_X)
                        {
                            Set_motion_index_(Motion_Index::Right_Halfstep);
                            ROS_INFO("PICK_MODE: Moving left for initial X alignment");
                        }
                        else
                        {
                            Set_motion_index_(Motion_Index::Left_Halfstep);
                            ROS_INFO("PICK_MODE: Moving right for initial X alignment");
                        }
                    }
                    else
                    {
                        ROS_INFO("PICK_MODE: Initial X alignment complete");
                        tmp_pick_seq++;
                        break;
                    }
                }
                else
                {
                    ROS_INFO("PICK_MODE: Ball not detected during initial alignment");
                    Set_motion_index_(Motion_Index::Step_in_place);
                }
                
                Set_select_motion_on_flg(true);
            }
            else if (finish_past != Get_SM_req_finish())
            {
                finish_past = Get_SM_req_finish();
                if (finish_past)
                {
                    bool is_real = img_procPtr->Get_img_proc_ball_det_();
                    if (is_real)
                    {
                        double ball_x = img_procPtr->Get_ball_x();
                        bool is_x_aligned = std::abs(ball_x - TARGET_X) <= INITIAL_X_THRESHOLD;
                        
                        if (is_x_aligned)
                        {
                            ROS_INFO("PICK_MODE: Initial X alignment confirmed, moving to fine adjustment");
                            tmp_pick_seq++;
                        }
                        else
                        {
                            ROS_INFO("PICK_MODE: Further initial X adjustment needed");
                        }
                    }
                    Set_select_motion_on_flg(false);
                }
            }
            break;


        case 3: // Z 좌표 조정 (전진으로 0.48 맞추기)
            if (!Get_select_motion_on_flg())
            {
                bool is_real = img_procPtr->Get_img_proc_ball_det_();
                if(is_real)
                {
                    double ball_z = img_procPtr->Get_ball_z();
                    bool is_z_aligned = std::abs(ball_z - TARGET_Z) <= Z_THRESHOLD;
                    ROS_INFO("PICK_MODE: Z alignment - Current Z: %f, Target Z: %f", ball_z, TARGET_Z);
                    if (!is_z_aligned && ball_z < TARGET_Z)
                    {
                        Set_motion_index_(Motion_Index::Forward_Halfstep);
                        ROS_INFO("PICK_MODE: Moving forward to reach target Z");
                    }
                    else
                    {
                        ROS_INFO("PICK_MODE: Z alignment complete");
                        tmp_pick_seq++;
                        break;
                    }
                }
                else
                {
                    ROS_INFO("PICK_MODE: Ball not detected during Z alignment");
                    Set_motion_index_(Motion_Index::Step_in_place);
                }
                Set_select_motion_on_flg(true);
            }
            else if (finish_past != Get_SM_req_finish())
            {
                finish_past = Get_SM_req_finish();
                if (finish_past)
                {
                    bool is_real = img_procPtr->Get_img_proc_ball_det_();
                    if (is_real)
                    {
                        double ball_z = img_procPtr->Get_ball_z();
                        bool is_z_aligned = std::abs(ball_z - TARGET_Z) <= Z_THRESHOLD;
                        if (is_z_aligned)
                        {
                            ROS_INFO("PICK_MODE: Z alignment confirmed");
                            tmp_pick_seq++;
                        }
                    }
                    Set_select_motion_on_flg(false);
                }
            }
            break;

        case 4: // X 좌표 미세 조정 (회전으로)
            if (!Get_select_motion_on_flg())
            {
                bool is_real = img_procPtr->Get_img_proc_ball_det_();
                if(is_real)
                {
                    double ball_x = img_procPtr->Get_ball_x();
                    bool is_x_aligned = std::abs(ball_x - TARGET_X) <= X_THRESHOLD;
                    ROS_INFO("PICK_MODE: Fine X alignment - Current X: %f, Target X: %f", ball_x, TARGET_X);
                    if (!is_x_aligned)
                    {
                        double x_diff = std::abs(ball_x - TARGET_X);
                        double turn_angle = (x_diff > 50) ? 
                            ((ball_x > TARGET_X) ? -10.0 : 10.0) : 
                            ((ball_x > TARGET_X) ? -5.0 : 5.0);
                        Set_turn_angle_(turn_angle);
                        Set_turn_angle_on_flg(true);
                        Set_motion_index_(Motion_Index::Step_in_place);
                        ROS_INFO("PICK_MODE: Rotating for X alignment. Angle: %f", turn_angle);
                    }
                    else
                    {
                        ROS_INFO("PICK_MODE: X alignment complete");
                        tmp_pick_seq++;
                        break;
                    }
                }
                else
                {
                    ROS_INFO("PICK_MODE: Ball not detected during X alignment");
                    Set_motion_index_(Motion_Index::Step_in_place);
                }
                Set_select_motion_on_flg(true);
            }
            else if (finish_past != Get_TA_req_finish())
            {
                finish_past = Get_TA_req_finish();
                if (finish_past)
                {
                    bool is_real = img_procPtr->Get_img_proc_ball_det_();
                    if (is_real)
                    {
                        double ball_x = img_procPtr->Get_ball_x();
                        bool is_x_aligned = std::abs(ball_x - TARGET_X) <= X_THRESHOLD;
                        if (is_x_aligned)
                        {
                            ROS_INFO("PICK_MODE: X alignment confirmed");
                            tmp_pick_seq++;
                        }
                    }
                    Set_select_motion_on_flg(false);
                }
            }
            break;
        case 5: // Y 좌표 미세 조정
            if (!Get_select_motion_on_flg())
            {
                bool is_real = img_procPtr->Get_img_proc_ball_det_();
                if(is_real)
                {
                    double ball_y = img_procPtr->Get_ball_y();
                    bool is_y_aligned = std::abs(ball_y - TARGET_Y) <= Y_THRESHOLD;
                    ROS_INFO("PICK_MODE: Y alignment - Current Y: %f, Target Y: %f", ball_y, TARGET_Y);
                    if (!is_y_aligned)
                    {
                        if (ball_y > TARGET_Y)
                        {
                            Set_motion_index_(Motion_Index::Back_Halfstep);
                            ROS_INFO("PICK_MODE: Moving backward for Y alignment");
                        }
                        else
                        {
                            Set_motion_index_(Motion_Index::Forward_Halfstep);
                            ROS_INFO("PICK_MODE: Moving forward for Y alignment");
                        }
                    }
                    else
                    {
                        ROS_INFO("PICK_MODE: Y alignment complete, moving to ball picking");
                        tmp_pick_seq++;
                        break;
                    }
                }
                else
                {
                    ROS_INFO("PICK_MODE: Ball not detected during Y alignment");
                    Set_motion_index_(Motion_Index::Step_in_place);
                }
                Set_select_motion_on_flg(true);
            }
            else if (finish_past != Get_SM_req_finish())
            {
                finish_past = Get_SM_req_finish();
                if (finish_past)
                {
                    bool is_real = img_procPtr->Get_img_proc_ball_det_();
                    if (is_real)
                    {
                        double ball_y = img_procPtr->Get_ball_y();
                        bool is_y_aligned = std::abs(ball_y - TARGET_Y) <= Y_THRESHOLD;
                        if (is_y_aligned)
                        {
                            ROS_INFO("PICK_MODE: Y alignment confirmed, moving to ball picking");
                            tmp_pick_seq++;
                        }
                    }
                    Set_select_motion_on_flg(false);
                }
            }
            break;
        // case 3: // 조정 단계 (x, y, z를 순차적으로 조정)
        //     if (!Get_select_motion_on_flg())
        //     {
        //         bool is_real = img_procPtr->Get_img_proc_ball_det_();
        
        //         if (is_real)
        //         {
        //             double ball_x = img_procPtr->Get_ball_x();
        //             double ball_y = img_procPtr->Get_ball_y();
        //             double ball_z = img_procPtr->Get_ball_z();
        
        //             bool is_x_aligned = std::abs(ball_x - TARGET_X) <= X_THRESHOLD;
        //             bool is_y_aligned = std::abs(ball_y - TARGET_Y) <= Y_THRESHOLD;
        
        //             ROS_INFO("PICK_MODE: Current ball position - X: %f, Y: %f", ball_x, ball_y);
        //             ROS_INFO("PICK_MODE: Alignment status - X: %s, Y: %s", 
        //                      is_x_aligned ? "Yes" : "No",
        //                      is_y_aligned ? "Yes" : "No");

        //             if (ball_z < 0.425)
        //             {
        //                 Set_motion_index_(Motion_Index::Back_2step);
        //                 ROS_INFO("PICK_MODE: Z value too small (%.3f), moving back 2 steps", ball_z);
        //                 last_adjusted_x = false;
        //             }
        //             else if (is_x_aligned && is_y_aligned) 
        //             {
        //                 ROS_INFO("PICK_MODE: Both X and Y aligned, moving to next sequence");
        //                 tmp_pick_seq++;
        //             }
        //             else if (!last_adjusted_x) // Y 조정 차례
        //             {
        //                 if (!is_y_aligned)
        //                 {
        //                     if (ball_y > TARGET_Y)
        //                     {
        //                         Set_motion_index_(Motion_Index::Back_Halfstep);
        //                         ROS_INFO("PICK_MODE: Adjusting Y - moving backward");
        //                     }
        //                     else
        //                     {
        //                         Set_motion_index_(Motion_Index::Forward_Halfstep);
        //                         ROS_INFO("PICK_MODE: Adjusting Y - moving forward");
        //                     }
        //                 }
        //                 last_adjusted_x = true;  // 다음은 X 조정
        //             }
        //             else  // X 조정 차례
        //             {
        //                 if (!is_x_aligned)
        //                 {
        //                     Set_motion_index_(Motion_Index::Step_in_place);
        //                     if (ball_x > TARGET_X)
        //                     {
        //                         Set_turn_angle_(8.0);
        //                         ROS_INFO("PICK_MODE: Adjusting X - rotating left 5 degrees");
        //                     }
        //                     else
        //                     {
        //                         Set_turn_angle_(-15.0);
        //                         ROS_INFO("PICK_MODE: Adjusting X - rotating right 5 degrees");
        //                     }
        //                     Set_turn_angle_on_flg(true);
        //                 }
        //                 last_adjusted_x = false;  // 다음은 Y 조정
        //             }
        //         }
        //         else
        //         {
        //             Set_motion_index_(Motion_Index::Step_in_place);
        //             ROS_INFO("PICK_MODE: Ball not detected. Rotating in place to search for the ball.");
        //             last_adjusted_x = false;  // 공을 다시 찾으면 Y부터 시작
        //         }
        
        //         Set_select_motion_on_flg(true);
        //     }

        //     // Sequence++ for Step_in_place (회전 동작의 완료 확인)
        //     else if (finish_past != Get_TA_req_finish() && Get_motion_index_() == Motion_Index::Step_in_place)
        //     {
        //         finish_past = Get_TA_req_finish();
        //         if (finish_past)
        //         {
        //             // 회전 완료 후 다시 공 위치 확인
        //             bool is_real = img_procPtr->Get_img_proc_ball_det_();
        //             if (is_real)
        //             {
        //                 double ball_x = img_procPtr->Get_ball_x();
        //                 double ball_y = img_procPtr->Get_ball_y();
        //                 double ball_z = img_procPtr->Get_ball_z();

        //                 bool is_x_aligned = std::abs(ball_x - TARGET_X) <= X_THRESHOLD;
        //                 bool is_y_aligned = std::abs(ball_y - TARGET_Y) <= Y_THRESHOLD;

        //                 if (ball_z >= 0.42 && is_x_aligned && is_y_aligned)
        //                 {
        //                     ROS_INFO("PICK_MODE: Alignment confirmed after rotation, moving to next sequence");
        //                     tmp_pick_seq++;
        //                 }
        //                 else
        //                 {
        //                     ROS_INFO("PICK_MODE: Further adjustment needed after rotation");
        //                 }
        //             }
        //             else
        //             {
        //                 ROS_INFO("PICK_MODE: Ball not detected after rotation, continuing search");
        //             }
        //             Set_select_motion_on_flg(false);
        //         }
        //     }
        //     // Sequence++ for Forward/Back steps (보행 동작의 완료 확인)
        //     else if (finish_past != Get_SM_req_finish() && 
        //              (Get_motion_index_() == Motion_Index::Forward_Halfstep || 
        //               Get_motion_index_() == Motion_Index::Back_Halfstep || 
        //               Get_motion_index_() == Motion_Index::Back_2step))
        //     {
        //         finish_past = Get_SM_req_finish();
        //         if (finish_past)
        //         {
        //             // 보행 동작 완료 후 다시 공 위치 확인
        //             bool is_real = img_procPtr->Get_img_proc_ball_det_();
        //             if (is_real)
        //             {
        //                 double ball_x = img_procPtr->Get_ball_x();
        //                 double ball_y = img_procPtr->Get_ball_y();
        //                 double ball_z = img_procPtr->Get_ball_z();

        //                 bool is_x_aligned = std::abs(ball_x - TARGET_X) <= X_THRESHOLD;
        //                 bool is_y_aligned = std::abs(ball_y - TARGET_Y) <= Y_THRESHOLD;

        //                 if (ball_z >= 0.42 && is_x_aligned && is_y_aligned)
        //                 {
        //                     ROS_INFO("PICK_MODE: Alignment confirmed after step, moving to next sequence");
        //                     tmp_pick_seq++;
        //                 }
        //                 else
        //                 {
        //                     ROS_INFO("PICK_MODE: Further adjustment needed after step");
        //                 }
        //             }
        //             else
        //             {
        //                 ROS_INFO("PICK_MODE: Ball not detected after step, continuing search");
        //             }
        //             Set_select_motion_on_flg(false);
        //         }
        //     }
        //     break;
        // case 6: // 최종 위치 확인
        //     if (!Get_select_motion_on_flg())
        //     {
        //         bool is_real = img_procPtr->Get_img_proc_ball_det_();
        //         if(is_real)
        //         {
        //             double ball_x = img_procPtr->Get_ball_x();
        //             double ball_y = img_procPtr->Get_ball_y();
        //             bool is_x_aligned = std::abs(ball_x - TARGET_X) <= X_THRESHOLD;
        //             bool is_y_aligned = std::abs(ball_y - TARGET_Y) <= Y_THRESHOLD;
        
        //             ROS_INFO("PICK_MODE: Final position check - X: %f, Y: %f", ball_x, ball_y);
        //             ROS_INFO("PICK_MODE: Alignment status - X: %s, Y: %s", 
        //                      is_x_aligned ? "Yes" : "No",
        //                      is_y_aligned ? "Yes" : "No");
        
        //             if (is_x_aligned && is_y_aligned)
        //             {
        //                 ROS_INFO("PICK_MODE: Final position confirmed, proceeding to pick the ball");
        //                 tmp_pick_seq++;
        //                 break;
        //             }
        //             else
        //             {
        //                 ROS_INFO("PICK_MODE: Position not optimal, moving back to readjust");
        //                 Set_motion_index_(Motion_Index::Back_halfstep);
        //             }
        //         }
        //         else
        //         {
        //             ROS_INFO("PICK_MODE: Ball not detected during final check");
        //             Set_motion_index_(Motion_Index::Step_in_place);
        //         }
        //         Set_select_motion_on_flg(true);
        //     }
        //     else if (finish_past != Get_SM_req_finish())
        //     {
        //         finish_past = Get_SM_req_finish();
        //         if (finish_past)
        //         {
        //             bool is_real = img_procPtr->Get_img_proc_ball_det_();
        //             if (is_real)
        //             {
        //                 double ball_x = img_procPtr->Get_ball_x();
        //                 double ball_y = img_procPtr->Get_ball_y();
        //                 bool is_x_aligned = std::abs(ball_x - TARGET_X) <= X_THRESHOLD;
        //                 bool is_y_aligned = std::abs(ball_y - TARGET_Y) <= Y_THRESHOLD;
                        
        //                 if (!is_x_aligned || !is_y_aligned)
        //                 {
        //                     ROS_INFO("PICK_MODE: Readjustment needed, returning to fine adjustment");
        //                     tmp_pick_seq = 4;  // X 좌표 미세 조정으로 돌아가기
        //                 }
        //             }
        //             Set_select_motion_on_flg(false);
        //         }
        //     }
        //     break;

        case 6: // Picking_Ball
            if (!Get_select_motion_on_flg())
            {
                ROS_INFO("PICK_MODE: Executing ball picking motion");
                Set_motion_index_(Motion_Index::Picking_Ball);
                Set_select_motion_on_flg(true);
                // Set_ball_det_flg_(false);
                // img_procPtr->Set_img_proc_ball_det_(false);
            }
            // Sequence++
            else if (finish_past != Get_SM_req_finish())
            {
                finish_past = Get_SM_req_finish();
                if (finish_past)
                {
                    tmp_pick_seq++;
                    Set_select_motion_on_flg(false);  // Reset for next motion
                }
            }
            break;
       
        case 7: // Back_2step            
            if (!Get_select_motion_on_flg())
            {
                ROS_INFO("PICK_MODE: Executing Back_2step");
                Set_motion_index_(Motion_Index::Back_2step);
                Set_select_motion_on_flg(true);
            }
            else if (finish_past != Get_SM_req_finish())
            {
                finish_past = Get_SM_req_finish();
                if (finish_past)
                {
                    tmp_pick_seq++;
                    Set_select_motion_on_flg(false);  // Reset for next motion
                }
            }
            break;

        case 8: // 공 감지 확인 및 재시도 결정
            {
                bool ball_detected = img_procPtr->Get_img_proc_ball_det_();
                if (ball_detected)
                {
                    retry_count++;
                    if (retry_count < MAX_RETRIES)
                    {
                        ROS_WARN("PICK_MODE: Ball still detected after picking attempt %d. Retrying from the beginning.", retry_count);
                        tmp_pick_seq = 0;  // 처음부터 다시 시작
                    }
                    else
                    {
                        ROS_ERROR("PICK_MODE: Failed to pick the ball after %d attempts. Moving to next step.", MAX_RETRIES);
                        tmp_pick_seq++;  // 다음 단계로 진행
                        retry_count = 0;
                        img_procPtr->Set_img_proc_ball_done_(true);

                    }
                }
                else
                {
                    ROS_INFO("PICK_MODE: Ball successfully picked. Moving to next step.");
                    tmp_pick_seq++;
                    retry_count = 0;
                    img_procPtr->Set_img_proc_ball_done_(true);
                }
                Set_select_motion_on_flg(false);
            }
            break;

        case 9: // 오른쪽으로 회전하며 라인 찾기
            if (!Get_select_motion_on_flg())
            {
                ROS_INFO("PICK_MODE: Rotating right to find line");
                Set_motion_index_(Motion_Index::Step_in_place);
                Set_turn_angle_(-15.0);  // 10도씩 오른쪽으로 회전
                Set_turn_angle_on_flg(true);
                Set_select_motion_on_flg(true);
            }
            else if (finish_past != Get_TA_req_finish())  // SM_req_finish 대신 TA_req_finish 사용
            {
                finish_past = Get_TA_req_finish();
                if (finish_past)
                {
                    // 라인 감지 확인
                    if (img_procPtr->Get_img_proc_line_det())
                    {
                        ROS_INFO("SHOOT_MODE: Line detected, moving to initialization");
                        tmp_pick_seq++;
                    }
                    else
                    {
                        // 라인을 찾지 못했다면 다시 회전
                        Set_select_motion_on_flg(false);
                    }
                }
            }
            break;

        case 10: // 초기화
            if (!Get_select_motion_on_flg())
            {
                ROS_INFO("PICK_MODE: Executing initialization motion");
                Set_motion_index_(Motion_Index::InitPose);
                Set_select_motion_on_flg(true);
            }
            // Sequence++
            else if (finish_past != Get_SM_req_finish())
            {
                finish_past = Get_SM_req_finish();
                if (finish_past)
                {
                    tmp_pick_seq++;
                    Set_select_motion_on_flg(false);  // Reset for next motion
                }
            }
            break;

        case 11: // LINE_MODE로 전환
            ROS_INFO("PICK_MODE: Initialization completed, switching to LINE_MODE");
            Set_pickmotion_det_flg_(true);
            tmp_pick_seq = 0;
            Set_select_motion_on_flg(false);
            is_in_pick_mode_ = false;  // PICK 모드 종료
            AllModereset(Running_Mode::LINE_MODE);

            Set_UD_NeckAngle(UD_CENTER);
            Set_UD_Neck_on_flg(true);
            break;


        default:
            Set_motion_index_(Motion_Index::InitPose);
            break;
    }

    // ROS_INFO("PICK_MODE: Current sequence: %d, Motion index: %d", tmp_pick_seq, Get_motion_index_());
    ROS_INFO("PICK_MODE: Current sequence: %d, Motion index: %d, Retry count: %d", 
             tmp_pick_seq, Get_motion_index_(), retry_count);
}

void Move_Decision::START_LINE_mode()
{

    line_gradient = img_procPtr->Get_gradient();
    StraightLineDecision(line_gradient, MARGIN_GRADIENT_START);
    line_actual_angle = Get_turn_angle_();
    line_motion = Motion_Index::InitPose;
    line_ud_neckangle = Get_UD_NeckAngle();
   
    // Initializing
    huddle_seq_finish = false;
    Set_huddle_det_stop_flg(false);

    // If SM_req_finish = false -> InitPose
    // Straight Line
    if (straightLine == true)
    {
        if (!Get_turn_angle_on_flg())
        {
            // Left turn
            // To be zero
            if (line_actual_angle > 0)
            {
                line_actual_angle = 0;
                Set_turn_angle_(line_actual_angle);
                Set_turn_angle_on_flg(true);
            }

            // Right turn
            // To be zero
            else if (line_actual_angle < 0)
            {
                line_actual_angle = 0;
                Set_turn_angle_(line_actual_angle);
                Set_turn_angle_on_flg(true);
            }
        }

        if (!Get_select_motion_on_flg())
        {
            line_motion = Motion_Index::Forward_2step;            
            Set_motion_index_(line_motion);
            Set_select_motion_on_flg(true);
        }

        if (!Get_UD_Neck_on_flg())
        {
            Set_UD_NeckAngle(UD_CENTER);
            Set_UD_Neck_on_flg(true);
        }

        ROS_ERROR("STRAIGHT LINE");
    }

    // Non Straight Line
    else if (straightLine == false)
    {
        if (!Get_turn_angle_on_flg())
        {
            // Increase Actual_angle more quickly for larger line_gradient values
            // Counter Clock wise(+) (Turn Angle sign)
            // Gradient : Angle from center of window.x to center of line.x
            // LEFT TURN
            if (line_gradient >= MARGIN_GRADIENT_START * 2.5)
            {
                increment = 2;
                ROS_WARN("LEFT_TURN");
            }
            else if (line_gradient >= MARGIN_GRADIENT_START * 2)
            {
                increment = 1.5;
                ROS_WARN("LEFT_TURN");
            }
            else if (line_gradient >= MARGIN_GRADIENT_START * 1.5)
            {
                increment = 1;
                ROS_WARN("LEFT_TURN");
            }
            else if (line_gradient >= MARGIN_GRADIENT_START * 1)
            {
                increment = 1;
                ROS_WARN("LEFT_TURN");
            }

            // Decrease Actual_angle relatively slowly for smaller line_gradient values
            // Right Turn
            else if (line_gradient <= -MARGIN_GRADIENT_START * 2.5)
            {
                increment = -2;
                ROS_WARN("RIGHT TURN");
            }
            else if (line_gradient <= -MARGIN_GRADIENT_START * 2)
            {
                increment = -1.5;
                ROS_WARN("RIGHT TURN");
            }
            else if (line_gradient <= -MARGIN_GRADIENT_START * 1.5)
            {
                increment = -1;
                ROS_WARN("RIGHT TURN");
            }
            else if (line_gradient <= -MARGIN_GRADIENT_START * 1)
            {
                increment = -1;
                ROS_WARN("RIGHT TURN");
            }
            else
            {
                increment = 0;
            }

            line_actual_angle += increment;
            if (line_actual_angle >= 8)
            {
                line_actual_angle = LINE_TURN;
            }
            else if (line_actual_angle <= -8)
            {
                line_actual_angle = -LINE_TURN;
            }

            Set_turn_angle_(line_actual_angle);
            Set_turn_angle_on_flg(true);
        }

        if (!Get_select_motion_on_flg())
        {
            if (line_gradient > MARGIN_GRADIENT_START * 1.5 || line_gradient < -MARGIN_GRADIENT_START * 1.5)
            {
                line_motion = Motion_Index::Step_in_place;
            }
            else
            {
                line_motion = Motion_Index::Forward_2step;
            }

            Set_motion_index_(line_motion);
            Set_select_motion_on_flg(true);
        }

        if (!Get_UD_Neck_on_flg())
        {
            Set_UD_NeckAngle(UD_CENTER);
            Set_UD_Neck_on_flg(true);
        }

        ROS_ERROR("NO STRAIGHT LINE");

        // TEST
        //  Set_RL_Neck_on_flg(true);
        //  Set_RL_NeckAngle(Actual_angle);
    }

    if(real_once_flg==false){
        tmp_real_walkcount = Get_walkcount_req_();
        real_once_flg = true;
    }
    
    if(((Get_walkcount_req_() - tmp_real_walkcount)  >=6) && ((line_motion == Forward_2step) || (line_motion == Forward_Nstep))){
        
        line_motion = Motion_Index::Forward_Nstep;
        Set_motion_index_(line_motion);
    }

                // Set_UD_NeckAngle(10);

}

void Move_Decision::callbackThread()
{
    ros::NodeHandle nh(ros::this_node::getName());

    // Subscriber
    // IMU_sensor_x_subscriber_ = nh.subscribe("/Angle/x", 1000, &Move_Decision::IMUsensorCallback, this);
    IMU_sensor_y_subscriber_ = nh.subscribe("/Angle/y", 1000, &Move_Decision::IMUsensorCallback, this);
    // IMU_sensor_z_subscriber_ = nh.subscribe("/Angle/z", 1000, &Move_Decision::IMUsensorCallback, this);

    // Server
    SendMotion_server_ = nh.advertiseService("SendMotion", &Move_Decision::SendMotion, this);

    ros::Rate loop_rate(SPIN_RATE);
    startMode();
    while (nh.ok())
    {
        // ROS_INFO("-------------------------CALLBACKTHREAD----------------------------");
        // ROS_INFO("-------------------------------------------------------------------");
        Running_Mode_Decision();
        Running_Info();
        Motion_Info();
        // ROS_INFO("angle : %f", Get_turn_angle_());
        // ROS_INFO("RL_Neck : %f", Get_RL_NeckAngle());
        // ROS_INFO("UD_Neck : %f", Get_UD_NeckAngle());
        // ROS_INFO("Distance : %f", img_procPtr->Get_distance());
        // ROS_INFO("EMG : %s", Get_Emergency_() ? "true" : "false");
        // ROS_INFO("-------------------------------------------------------------------");
        // ROS_INFO("-------------------------CALLBACKTHREAD----------------------------");

        ros::spinOnce();
        loop_rate.sleep();
        // usleep(1000);
    }
}

//////////////////////////////////////////////////////////// Server Part ////////////////////////////////////////////////////////////////

bool Move_Decision::SendMotion(ahra::SendMotion::Request &req, ahra::SendMotion::Response &res)
{
    // Extract the unique ID included in the request
    int request_id = req.request_id;

    // Check if the request has already been processed
    if (isRequestProcessed(request_id) && !warning_printed)
    {
        if (warning_counter) // Check the counter
        {
            ROS_WARN("Duplicate service response prevented for request ID: %d", request_id);
            warning_printed = true; // Set the flag to true
            warning_counter++;      // Increase the counter
        }
        return false;
    }

    // Finish Check
    bool req_SM_finish = req.SM_finish;
    bool req_TA_finish = req.TA_finish;
    bool req_UD_finish = req.UD_finish;
    bool req_RL_finish = req.RL_finish;
    bool req_EM_finish = req.EM_finish;
    bool req_ST_finish = req.ST_finish;
    int32_t req_walkcount = req.walkcount;


    Set_SM_req_finish(req_SM_finish);
    Set_TA_req_finish(req_TA_finish);
    Set_UD_req_finish(req_UD_finish);
    Set_RL_req_finish(req_RL_finish);
    Set_EM_req_finish(req_EM_finish);
    Set_ST_req_finish(req_ST_finish);
    Set_walkcount_req(req_walkcount);


    // Response
    auto res_SM = playMotion();
    double res_TA = turn_angle();
    double res_UD = Move_UD_NeckAngle();
    double res_RL = Move_RL_NeckAngle();
    bool res_EM = Emergency();

    if (Get_SM_req_finish())
    {
        if (std::get<0>(res_SM) == Motion_Index::NONE)
        {
            std::get<0>(res_SM) = Get_motion_index_();
            Set_select_motion_on_flg(false);
        }

        res.select_motion = std::get<0>(res_SM);
        res.distance = std::get<1>(res_SM);
    }

    else if (!Get_SM_req_finish())
    {
        res.select_motion = Motion_Index::NONE;
        Set_select_motion_on_flg(false);
    }

    ROS_WARN("[MESSAGE] SM Request :   %s ", Get_SM_req_finish() ? "true" : "false");
    ROS_WARN("[MESSAGE] TA Request :   %s ", Get_TA_req_finish() ? "true" : "false");
    // ROS_WARN("[MESSAGE] RL Request :   %s ", Get_RL_req_finish() ? "true" : "false");
    // ROS_ERROR("[MESSAGE] EMG Request :   %s ", Get_EM_req_finish() ? "true" : "false");

    if (Get_TA_req_finish())
    {
        res.turn_angle = res_TA;
    }

    if (Get_UD_req_finish())
    {
        res.ud_neckangle = res_UD;
    }

    if (Get_RL_req_finish())
    {
        res.rl_neckangle = res_RL;
    }

    if (Get_EM_req_finish())
    {
        res.emergency = res_EM;
    }

    Send_Info(res.select_motion, res.turn_angle, res.ud_neckangle, res.rl_neckangle, res.emergency);

    res.success = true;
    recordProcessedRequest(request_id);

    return true;
}

std::tuple<int8_t, double> Move_Decision::playMotion()
{
    int8_t res_select_motion = 99;
    double res_distance = 0;
    // int8_t total = Get_TA_req_finish() + Get_UD_req_finish() + Get_RL_req_finish() + Get_EM_req_finish();

    if ((Get_stand_status_() == Stand_Status::Stand) && (Get_select_motion_on_flg() == true) /*&& total <=4*/)
    {
        switch (Get_motion_index_())
        {
        case Motion_Index::InitPose:
            res_select_motion = Motion_Index::InitPose;
            break;

        case Motion_Index::Forward_2step:
            res_select_motion = Motion_Index::Forward_2step;
            break;

        case Motion_Index::Left_2step:
            res_select_motion = Motion_Index::Left_2step;
            break;

        case Motion_Index::Step_in_place:
            res_select_motion = Motion_Index::Step_in_place;
            break;

        case Motion_Index::Right_2step:
            res_select_motion = Motion_Index::Right_2step;
            break;

        case Motion_Index::ForWard_fast4step:
            res_select_motion = Motion_Index::ForWard_fast4step;
            break;

        case Motion_Index::Forward_Nstep:
            res_select_motion = Motion_Index::Forward_Nstep;
            // if (Get_distance_on_flg())
            // {
            //     res_distance = Get_distance_();
            //     Set_distance_on_flg(false);
            // }
            break;

        case Motion_Index::Huddle_Jump:
            res_select_motion = Motion_Index::Huddle_Jump;
            break;

        case Motion_Index::Huddle_Jump_V2:
            res_select_motion = Motion_Index::Huddle_Jump_V2;
            break;

        case Motion_Index::Picking_Ball:
            res_select_motion = Motion_Index::Picking_Ball;
            break;

        case Motion_Index::NONE:
            res_select_motion = Motion_Index::NONE;
            break;

        case Motion_Index::Forward_Halfstep:
            res_select_motion = Motion_Index::Forward_Halfstep;
            break;

        case Motion_Index::Right_Halfstep:
            res_select_motion = Motion_Index::Right_Halfstep;
            break;

        case Motion_Index::Left_Halfstep:
            res_select_motion = Motion_Index::Left_Halfstep;
            break;

        case Motion_Index::Back_Halfstep:
            res_select_motion = Motion_Index::Back_Halfstep;
            break;

        case Motion_Index::Back_2step:
            res_select_motion = Motion_Index::Back_2step;
            break;

        case Motion_Index::FINISH:
            res_select_motion = Motion_Index::FINISH;
            break;

        case Motion_Index::Forward_1step:
            res_select_motion = Motion_Index::Forward_1step;
            break;

        case Motion_Index::Right_6step:
            res_select_motion = Motion_Index::Right_6step;
            break;

        case Motion_Index::Left_6step:
            res_select_motion = Motion_Index::Left_6step;
            break;

        case Motion_Index::Shoot:
            res_select_motion = Motion_Index::Shoot;
            break;
                
        case Motion_Index::Ready_to_throw:
            res_select_motion = Motion_Index::Ready_to_throw;
            break;

        default:
            res_select_motion = Motion_Index::InitPose;
        }

        Set_select_motion_on_flg(false);
    }

    else if (((Get_stand_status_() == Stand_Status::Fallen_Back)) || (Get_stand_status_() == Stand_Status::Fallen_Forward) && (Get_select_motion_on_flg() == true))
    {
        switch (Get_motion_index_())
        {
        case Motion_Index::FWD_UP:
            res_select_motion = Motion_Index::FWD_UP;
            break;

        case Motion_Index::BWD_UP:
            res_select_motion = Motion_Index::BWD_UP;
            break;

        default:
            res_select_motion = Motion_Index::InitPose;
        }
        Set_select_motion_on_flg(false);
    }

    // ROS_WARN("[MESSAGE] SM Request :   %s ", Get_SM_req_finish() ? "true" : "false");
    // ROS_INFO("#[MESSAGE] SM Motion :   %d#", res_select_motion);
    // ROS_INFO("#[MESSAGE] SM Distance : %f#", res_distance);

    return std::make_tuple(res_select_motion, res_distance);
}

double Move_Decision::turn_angle()
{
    double res_turn_angle = 0;

    if ((Get_stand_status_() == Stand_Status::Stand) && Get_turn_angle_on_flg())
    {
        if (Get_motion_index_() == Motion_Index::Forward_2step || Get_motion_index_() == Motion_Index::Step_in_place || Get_motion_index_() == Motion_Index::Forward_Nstep)
        {
            res_turn_angle = this->Get_turn_angle_();
            if (res_turn_angle > TURN_MAX)
                res_turn_angle = TURN_MAX;
            else if (res_turn_angle < TURN_MIN)
                res_turn_angle = TURN_MIN;
            Set_turn_angle_on_flg(false);
        }
    }

    // ROS_WARN("[MESSAGE] TA Request :   %s ", Get_TA_req_finish() ? "true" : "false");
    // ROS_INFO("#[MESSAGE] TA Response : %f#", res_turn_angle);

    return res_turn_angle;
}

double Move_Decision::Move_UD_NeckAngle()
{
    double res_ud_neckangle = UD_CENTER;

    if ((Get_stand_status_() == Stand_Status::Stand) && Get_UD_Neck_on_flg())
    {
        res_ud_neckangle = this->Get_UD_NeckAngle();
        if (res_ud_neckangle > UD_MAX)
            res_ud_neckangle = UD_MAX;
        else if (res_ud_neckangle < UD_MIN)
            res_ud_neckangle = UD_MIN;
        Set_UD_Neck_on_flg(false);
    }

    else if (!Get_UD_req_finish())
    {
        res_ud_neckangle = this->Get_UD_NeckAngle();
        Set_UD_Neck_on_flg(false);
    }

    // ROS_WARN("[MESSAGE] UD Request :   %s ", Get_UD_req_finish() ? "true" : "false");
    // ROS_INFO("#[MESSAGE] UD Response : %f#", res_ud_neckangle);

    return res_ud_neckangle;
}

double Move_Decision::Move_RL_NeckAngle()
{
    double res_rl_neckangle = 0;

    if ((Get_stand_status_() == Stand_Status::Stand) && Get_RL_Neck_on_flg())
    {
        // img_procssing
        res_rl_neckangle = this->Get_RL_NeckAngle();
        if (res_rl_neckangle > RL_MAX)
            res_rl_neckangle = RL_MAX;
        else if (res_rl_neckangle < RL_MIN)
            res_rl_neckangle = RL_MIN;
        Set_RL_Neck_on_flg(false);
    }

    // ROS_WARN("[MESSAGE] RL Request :   %s ", Get_RL_req_finish() ? "true" : "false");
    // ROS_INFO("#[MESSAGE] RL Response : %f#", res_rl_neckangle);

    return res_rl_neckangle;
}

bool Move_Decision::Emergency()
{
    bool res_emergency = false;

    if ((Get_stand_status_() == Stand_Status::Stand) && Get_emergency_on_flg())
    {
        // 1 : Stop
        // 0 : Keep Going (Option)
        res_emergency = Get_Emergency_();
        Set_emergency_on_flg(false);
    }

    // ROS_ERROR("[MESSAGE] EMG Request :   %s ", Get_EM_req_finish() ? "true" : "false");
    // ROS_ERROR("#[MESSAGE] EMG Response : %s#", res_emergency ? "true" : "false");

    return res_emergency;
}

///////////////////////////////////////// About Publish & Subscribe /////////////////////////////////////////

void Move_Decision::IMUsensorCallback(const std_msgs::Float32::ConstPtr &IMU)
{
    if (stop_fallen_check_ == true)
        return;

    // RPY(0) = IMU->data;
    RPY(1) = IMU->data;
    // RPY(2) = IMU->data;

    double pitch = RPY(1) * 57.2958;

    // Quaternino2RPY();
    // sensorPtr->RPY *= (180 / M_PI);
    // double roll = sensorPtr->RPY(0);
    // double pitch = sensorPtr->RPY(1);
    // double yaw = sensorPtr->RPY(2);

    double alpha = 0.4;
    if (present_pitch_ == 0)
        present_pitch_ = pitch;
    else
        present_pitch_ = present_pitch_ * (1 - alpha) + pitch * alpha;

    if (present_pitch_ > FALL_FORWARD_LIMIT)
        Set_stand_status_(Stand_Status::Fallen_Forward);
    else if (present_pitch_ < FALL_BACK_LIMIT)
        Set_stand_status_(Stand_Status::Fallen_Back);
    else
        Set_stand_status_(Stand_Status::Stand);

    // ROS_ERROR("STAND_STAUS : %d", Get_stand_status_());
    // ROS_ERROR("PITCH : %f", pitch);
}

// ********************************************** FUNCTION ************************************************** //

void Move_Decision::startMode()
{
    Set_motion_index_(Motion_Index::InitPose);
}

/*            -mg_gra        mg_gra              */
/*      False    |     True    |     False       */
/*  <----------------------------------------->  */
void Move_Decision::StraightLineDecision(double gra, double mg_gra)
{
    if ((gra > mg_gra) || (gra < -mg_gra)) // judged to be a straight line. If it exists between the slopes, it is a straight line.
    {
        straightLine = false;
    }

    // Straight Line Decision
    else /*if ((gradient < MARGIN_GRADIENT && (gradient > -MARGIN_GRADIENT)))*/
    {
        straightLine = true;
    }
}

void Move_Decision::Send_Motion_Info(int8_t res_motion)
{
    string tmp_motion;
    switch (res_motion)
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

    case Motion_Index::Huddle_Jump:
        tmp_motion = Str_Huddle_Jump;
        break;

    case Motion_Index::Huddle_Jump_V2:
        tmp_motion = Str_Huddle_Jump_V2;
        break;

    case Motion_Index::Picking_Ball:
        tmp_motion = Str_Picking_Ball;
        break;

    case Motion_Index::Ready_to_throw:
        tmp_motion = Str_Ready_to_throw;
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

    case Motion_Index::Back_2step:
        tmp_motion = Str_Back_2step;
        break;

    case Motion_Index::FINISH:
        tmp_motion = Str_FINISH;
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

    case Motion_Index::Left_6step:
        tmp_motion = Str_Left_6step;
        break;

    case Motion_Index::Right_6step:
        tmp_motion = Str_Right_6step;
        break;

    case Motion_Index::NONE:
        tmp_motion = Str_NONE;
        break;
    }
    ROS_ERROR("SEND Motion_Index : %s", tmp_motion.c_str());
}

void Move_Decision::Send_Info(int8_t motion_, double turn_angle_, double ud, double rl, bool emg)
{
    ROS_INFO("------------------------- SEND ----------------------------");
    Send_Motion_Info(motion_);
    ROS_ERROR("#[MESSAGE] TA Response : %f#", turn_angle_);
    ROS_ERROR("#[MESSAGE] UD Response : %f#", ud);
    ROS_ERROR("#[MESSAGE] RL Response : %f#", rl);
    ROS_ERROR("#[MESSAGE] EMG Response : %s#", emg ? "true" : "false");
    ROS_INFO("------------------------- SEND ----------------------------");
}

void Move_Decision::Motion_Info()
{
    string tmp_motion;
    switch (Get_motion_index_())
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

    case Motion_Index::Huddle_Jump:
        tmp_motion = Str_Huddle_Jump;
        break;

    case Motion_Index::Huddle_Jump_V2:
        tmp_motion = Str_Huddle_Jump_V2;
        break;
    
    case Motion_Index::Picking_Ball:
        tmp_motion = Str_Picking_Ball;
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

    case Motion_Index::Back_2step:
        tmp_motion = Str_Back_2step;
        break;

    case Motion_Index::FINISH:
        tmp_motion = Str_FINISH;
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
        tmp_motion = Str_Right_6step;
        break;

    case Motion_Index::Left_6step:
        tmp_motion = Str_Left_6step;
        break;

    case Motion_Index::Ready_to_throw:
        tmp_motion = Str_Ready_to_throw;
        break;    

    case Motion_Index::Shoot:
        tmp_motion = Str_Shoot;
        break;   

    case Motion_Index::NONE:
        tmp_motion = Str_NONE;
        break;
    }
    ROS_INFO("Motion_Index : %s", tmp_motion.c_str());
}

void Move_Decision::Running_Info()
{
    string tmp_running;
    switch (Get_running_mode_())
    {
    case Running_Mode::LINE_MODE:
        tmp_running = Str_LINE_MODE;
        break;

    case Running_Mode::NO_LINE_MODE:
        tmp_running = Str_NO_LINE_MODE;
        break;

    case Running_Mode::HUDDLE_MODE:
        tmp_running = Str_HUDDLE_MODE;
        break;

    case Running_Mode::PICK_MODE:
        tmp_running = Str_PICK_MODE;
        break;

    case Running_Mode::SHOOT_MODE:
        tmp_running = Str_Shoot;
        break;

    case Running_Mode::START_MODE:
        tmp_running = Str_START;
        break;



    }
    ROS_INFO("------------------------- RUNNING ----------------------------");
    ROS_INFO("Running_Mode : %s", tmp_running.c_str());
}

void Move_Decision::AllModereset(int8_t mode)
{
    set_pick_mode(false);
    set_shoot_mode(false);
    set_huddle_mode(false);
    Set_line_det_flg(false);
    Set_no_line_det_flg(false);

    switch (mode)
    {
    case Running_Mode::LINE_MODE:
        Set_line_det_flg(true);
        break;
    case Running_Mode::NO_LINE_MODE:
        Set_no_line_det_flg(true);
        break;
    case Running_Mode::HUDDLE_MODE:
        set_huddle_mode(true);
        break;
    case Running_Mode::PICK_MODE:
        set_pick_mode(true);
        break;
    case Running_Mode::SHOOT_MODE:
        set_shoot_mode(true);
        break;
    default:
        Set_motion_index_(Motion_Index::InitPose);
        break;
    }
    
    Set_running_mode_(mode);
    ROS_INFO("Mode changed to: %d", mode);
}

double Move_Decision::Relax_huddle_to_foot(double huddle_distance_)
{
    huddle_distance_save.push_back(huddle_distance_);
    ROS_WARN("huddle_distance_save SIZE : %lu", huddle_distance_save.size());
    if (huddle_distance_save.size() == SPIN_RATE * 2) // 3sec Mean Distance Value
    {
        // 거리 값 필터링
        float threshold = 0.1f; // 필터링 임계값 설정
        if (std::abs(huddle_distance_ - huddle_distance_save.back()) < threshold)
        {
            // 현재 값이 이전 값과 비교하여 임계값 이내인 경우
            huddle_distance_save.push_back(huddle_distance_); // 현재 값을 벡터에 추가
        }
        else
        {
            // 튀는 값인 경우, 이전 값으로 대체
            huddle_distance_ = huddle_distance_save.back();
        }

        // 벡터의 평균 계산double Move_Decision::Relax_Distance(double distance_)
{
    huddle_distance_save.push_back(distance_);
    ROS_WARN("huddle_distance_save SIZE : %lu", huddle_distance_save.size());
    if (huddle_distance_save.size() == SPIN_RATE * 2) // 3sec Mean Distance Value
    {
        // 거리 값 필터링
        float threshold = 0.1f; // 필터링 임계값 설정
        if (std::abs(distance_ - huddle_distance_save.back()) < threshold)
        {
            // 현재 값이 이전 값과 비교하여 임계값 이내인 경우
            huddle_distance_save.push_back(distance_); // 현재 값을 벡터에 추가
        }
        else
        {
            // 튀는 값인 경우, 이전 값으로 대체
            distance_ = huddle_distance_save.back();
        }

        // 벡터의 평균 계산
        float sum = 0.0f;
        for (const float &distance : huddle_distance_save)
        {
            sum += distance;
        }
        distance_ = sum / huddle_distance_save.size();
        distance_ = std::floor(huddle_distance * 1000.0) / 1000.0;
        huddle_distance_save.clear();
    }

    return distance_;
}

        float sum = 0.0f;
        for (const float &distance : huddle_distance_save)
        {
            sum += distance;
        }
        huddle_distance_ = sum / huddle_distance_save.size();
        huddle_distance_ = std::floor(huddle_distance_ * 1000.0) / 1000.0;
        huddle_distance_save.clear();
    }

    return huddle_distance_;
}

double Move_Decision::Relax_X(double x)
{
    x_save.push_back(x);
    ROS_WARN("x_save SIZE : %lu", x_save.size());
    if (x_save.size() == SPIN_RATE * 2) // 3sec Mean X Value
    {
        // X 값 필터링
        float threshold = 0.1f; // 필터링 임계값 설정
        if (std::abs(x - x_save.back()) < threshold)
        {
            // 현재 값이 이전 값과 비교하여 임계값 이내인 경우
            x_save.push_back(x); // 현재 값을 벡터에 추가
        }
        else
        {
            // 튀는 값인 경우, 이전 값으로 대체
            x = x_save.back();
        }

        // 벡터의 평균 계산
        float sum = 0.0f;
        for (const float &x_value : x_save)
        {
            sum += x_value;
        }
        x = sum / x_save.size();
        x = std::floor(x * 1000.0) / 1000.0;
        x_save.clear();
    }
    return x;
}

double Move_Decision::Relax_Y(double y)
{
    y_save.push_back(y);
    ROS_WARN("y_save SIZE : %lu", y_save.size());
    if (y_save.size() == SPIN_RATE * 2) // 3sec Mean Y Value
    {
        // Y 값 필터링
        float threshold = 0.1f; // 필터링 임계값 설정
        if (std::abs(y - y_save.back()) < threshold)
        {
            // 현재 값이 이전 값과 비교하여 임계값 이내인 경우
            y_save.push_back(y); // 현재 값을 벡터에 추가
        }
        else
        {
            // 튀는 값인 경우, 이전 값으로 대체
            y = y_save.back();
        }

        // 벡터의 평균 계산
        float sum = 0.0f;
        for (const float &y_value : y_save)
        {
            sum += y_value;
        }
        y = sum / y_save.size();
        y = std::floor(y * 1000.0) / 1000.0;
        y_save.clear();
    }
    return y;
}

double Move_Decision::Relax_Z(double z)
{
    z_save.push_back(z);
    ROS_WARN("z_save SIZE : %lu", z_save.size());
    if (z_save.size() == SPIN_RATE * 2) // 3sec Mean Z Value
    {
        // Z 값 필터링
        float threshold = 0.1f; // 필터링 임계값 설정
        if (std::abs(z - z_save.back()) < threshold)
        {
            // 현재 값이 이전 값과 비교하여 임계값 이내인 경우
            z_save.push_back(z); // 현재 값을 벡터에 추가
        }
        else
        {
            // 튀는 값인 경우, 이전 값으로 대체
            z = z_save.back();
        }

        // 벡터의 평균 계산
        float sum = 0.0f;
        for (const float &z_value : z_save)
        {
            sum += z_value;
        }
        z = sum / z_save.size();
        z = std::floor(z * 1000.0) / 1000.0;
        z_save.clear();
    }
    return z;
}


void Move_Decision::DistanceDecision(double distance_)
{
    if (distance_ > Green_area_dis) // judged to be a straight line. If it exists between the slopes, it is a straight line.
    {
        robot_forward = true;
    }
    else if (distance_ < Green_area_dis)
    {
        robot_forward = false;
    }
}

// ********************************************** GETTERS ************************************************** //

bool Move_Decision::Get_Emergency_() const
{
    std::lock_guard<std::mutex> lock(mtx_Emergency_);
    return Emergency_;
}

bool Move_Decision::Get_emergency_on_flg() const
{
    std::lock_guard<std::mutex> lock(mtx_emergency_on_flg_);
    return emergency_on_flg_;
}

int8_t Move_Decision::Get_motion_index_() const
{
    std::lock_guard<std::mutex> lock(mtx_motion_index_);
    return motion_index_;
}

int8_t Move_Decision::Get_stand_status_() const
{
    std::lock_guard<std::mutex> lock(mtx_stand_status_);
    return stand_status_;
}

int8_t Move_Decision::Get_running_mode_() const
{
    std::lock_guard<std::mutex> lock(mtx_running_mode_);
    return running_mode_;
}

double Move_Decision::Get_turn_angle_() const
{
    std::lock_guard<std::mutex> lock(mtx_turn_angle_);
    return turn_angle_;
}

double Move_Decision::Get_distance_() const
{
    std::lock_guard<std::mutex> lock(mtx_distance_);
    return distance_;
}

bool Move_Decision::Get_ProcessON() const
{
    std::lock_guard<std::mutex> lock(mtx_ProcessON_);
    return ProcessON_;
}

bool Move_Decision::Get_MoveDecisionON() const
{
    std::lock_guard<std::mutex> lock(mtx_MoveDecisionON_);
    return MoveDecisionON_;
}

bool Move_Decision::Get_CallbackON() const
{
    std::lock_guard<std::mutex> lock(mtx_CallbackON_);
    return CallbackON_;
}

bool Move_Decision::Get_response_sent_() const
{
    std::lock_guard<std::mutex> lock(mtx_response_sent_);
    return response_sent_;
}

bool Move_Decision::Get_line_det_flg() const
{
    std::lock_guard<std::mutex> lock(mtx_line_det_flg);
    return line_det_flg_;
}

bool Move_Decision::Get_no_line_det_flg() const
{
    std::lock_guard<std::mutex> lock(mtx_no_line_det_flg);
    return no_line_det_flg_;
}

bool Move_Decision::Get_huddle_det_flg_2d() const
{
    std::lock_guard<std::mutex> lock(mtx_huddle_det_flg_2d);
    return huddle_det_flg_2d_;
}

bool Move_Decision::Get_huddle_det_flg_3d() const
{
    std::lock_guard<std::mutex> lock(mtx_huddle_det_flg_3d);
    return huddle_det_flg_3d_;
}

bool Move_Decision::Get_huddle_det_stop_flg() const
{
    std::lock_guard<std::mutex> lock(mtx_huddle_det_stop_flg);
    return huddle_det_stop_flg_;
}

bool Move_Decision::Get_ball_det_flg_() const
{
    std::lock_guard<std::mutex> lock(mtx_ball_det_flg_);
    return ball_det_flg_;
}

bool Move_Decision::Get_stop_det_flg() const
{
    std::lock_guard<std::mutex> lock(mtx_stop_det_flg);
    return stop_det_flg_;
}

bool Move_Decision::Get_select_motion_on_flg() const
{
    std::lock_guard<std::mutex> lock(mtx_select_motion_on_flg_);
    return select_motion_on_flg_;
}

bool Move_Decision::Get_turn_angle_on_flg() const
{
    std::lock_guard<std::mutex> lock(mtx_turn_angle_on_flg_);
    return turn_angle_on_flg_;
}

bool Move_Decision::Get_distance_on_flg() const
{
    std::lock_guard<std::mutex> lock(mtx_distance_on_flg_);
    return distance_on_flg_;
}

double Move_Decision::Get_UD_NeckAngle() const
{
    std::lock_guard<std::mutex> lock(mtx_UD_NeckAngle_);
    return UD_NeckAngle_;
}

double Move_Decision::Get_RL_NeckAngle() const
{
    std::lock_guard<std::mutex> lock(mtx_RL_NeckAngle_);
    return RL_NeckAngle_;
}

bool Move_Decision::Get_UD_Neck_on_flg() const
{
    std::lock_guard<std::mutex> lock(mtx_UD_Neck_on_flg);
    return UD_Neck_on_flg_;
}

bool Move_Decision::Get_RL_Neck_on_flg() const
{
    std::lock_guard<std::mutex> lock(mtx_RL_Neck_on_flg);
    return RL_Neck_on_flg_;
}

bool Move_Decision::Get_SM_req_finish() const
{
    std::lock_guard<std::mutex> lock(mtx_SM_req_finish_);
    return SM_req_finish_;
}

bool Move_Decision::Get_TA_req_finish() const
{
    std::lock_guard<std::mutex> lock(mtx_TA_req_finish_);
    return TA_req_finish_;
}

bool Move_Decision::Get_UD_req_finish() const
{
    std::lock_guard<std::mutex> lock(mtx_UD_req_finish_);
    return UD_req_finish_;
}

bool Move_Decision::Get_RL_req_finish() const
{
    std::lock_guard<std::mutex> lock(mtx_RL_req_finish_);
    return RL_req_finish_;
}

bool Move_Decision::Get_EM_req_finish() const
{
    std::lock_guard<std::mutex> lock(mtx_EM_req_finish_);
    return EM_req_finish_;
}

bool Move_Decision::Get_ST_req_finish() const
{
    std::lock_guard<std::mutex> lock(mtx_ST_req_finish_);
    return ST_req_finish_;
}

int32_t Move_Decision::Get_walkcount_req_() const
{
    std::lock_guard<std::mutex> lock(mtx_walkcount_req_);
    return walkcount_req_;
}
//농구코드 추가

bool Move_Decision::Get_Adjust_flg() const
{
    std::lock_guard<std::mutex> lock(mtx_adjust_flg);
    return adjust_flg_;
}

bool Move_Decision::Get_Shoot_flg() const
{
    std::lock_guard<std::mutex> lock(mtx_shoot_flg);
    return shoot_flg_;
}

bool Move_Decision::Get_hoop_det_flg_() const
{
    std::lock_guard<std::mutex> lock(mtx_hoop_det_flg_);
    return hoop_det_flg_;
}
bool Move_Decision::Get_pickmotion_det_flg_() const
{
    std::lock_guard<std::mutex> lock(mtx_pickmotion_det_flg_);
    return pickmotion_det_flg_;
}
// ********************************************** SETTERS ************************************************** //

void Move_Decision::Set_Emergency_(bool Emergency)
{
    std::lock_guard<std::mutex> lock(mtx_Emergency_);
    this->Emergency_ = Emergency;
}

void Move_Decision::Set_emergency_on_flg(bool emergency_on_flg)
{
    std::lock_guard<std::mutex> lock(mtx_emergency_on_flg_);
    this->emergency_on_flg_ = emergency_on_flg;
}

void Move_Decision::Set_motion_index_(int8_t motion_index)
{
    std::lock_guard<std::mutex> lock(mtx_motion_index_);
    this->motion_index_ = motion_index;

    Set_select_motion_on_flg(true);
}

void Move_Decision::Set_stand_status_(int8_t stand_status)
{
    std::lock_guard<std::mutex> lock(mtx_stand_status_);
    this->stand_status_ = stand_status;
}

void Move_Decision::Set_running_mode_(int8_t running_mode)
{
    std::lock_guard<std::mutex> lock(mtx_running_mode_);
    this->running_mode_ = running_mode;
}

void Move_Decision::Set_turn_angle_(double turn_angle)
{
    std::lock_guard<std::mutex> lock(mtx_turn_angle_);
    this->turn_angle_ = turn_angle;
}

void Move_Decision::Set_distance_(double distance)
{
    std::lock_guard<std::mutex> lock(mtx_distance_);
    this->distance_ = distance;
}

void Move_Decision::Set_ProcessON(bool ProcessON)
{
    std::lock_guard<std::mutex> lock(mtx_ProcessON_);
    this->ProcessON_ = ProcessON;
}

void Move_Decision::Set_MoveDecisionON(bool MoveDecisionON)
{
    std::lock_guard<std::mutex> lock(mtx_MoveDecisionON_);
    this->MoveDecisionON_ = MoveDecisionON;
}

void Move_Decision::Set_CallbackON(bool CallbackON)
{
    std::lock_guard<std::mutex> lock(mtx_CallbackON_);
    this->CallbackON_ = CallbackON;
}

void Move_Decision::Set_response_sent_(bool response_sent)
{
    std::lock_guard<std::mutex> lock(mtx_response_sent_);
    this->response_sent_ = response_sent;
}

void Move_Decision::Set_line_det_flg(bool line_det_flg)
{
    std::lock_guard<std::mutex> lock(mtx_CallbackON_);
    this->line_det_flg_ = line_det_flg;
}

void Move_Decision::Set_no_line_det_flg(bool no_line_det_flg)
{
    std::lock_guard<std::mutex> lock(mtx_no_line_det_flg);
    this->no_line_det_flg_ = no_line_det_flg;
}

void Move_Decision::Set_huddle_det_flg_2d(bool huddle_det_flg_2d)
{
    std::lock_guard<std::mutex> lock(mtx_huddle_det_flg_2d);
    this->huddle_det_flg_2d_ = huddle_det_flg_2d;
}

void Move_Decision::Set_ball_det_flg_(bool ball_det_flg_)
{
    std::lock_guard<std::mutex> lock(mtx_ball_det_flg_);
    this->ball_det_flg_ = ball_det_flg_;
}


void Move_Decision::Set_huddle_det_flg_3d(bool huddle_det_flg_3d)
{
    std::lock_guard<std::mutex> lock(mtx_huddle_det_flg_3d);
    this->huddle_det_flg_3d_ = huddle_det_flg_3d;
}

void Move_Decision::Set_stop_det_flg(bool stop_det_flg)
{
    std::lock_guard<std::mutex> lock(mtx_stop_det_flg);
    this->stop_det_flg_ = stop_det_flg;
}

void Move_Decision::Set_huddle_det_stop_flg(bool huddle_det_stop_flg)
{
    std::lock_guard<std::mutex> lock(mtx_huddle_det_stop_flg);
    this->huddle_det_stop_flg_ = huddle_det_stop_flg;
}

void Move_Decision::Set_select_motion_on_flg(bool select_motion_on_flg)
{
    std::lock_guard<std::mutex> lock(mtx_select_motion_on_flg_);
    this->select_motion_on_flg_ = select_motion_on_flg;
}

void Move_Decision::Set_turn_angle_on_flg(bool turn_angle_on_flg)
{
    std::lock_guard<std::mutex> lock(mtx_turn_angle_on_flg_);
    this->turn_angle_on_flg_ = turn_angle_on_flg;
}

void Move_Decision::Set_distance_on_flg(bool distance_on_flg)
{
    std::lock_guard<std::mutex> lock(mtx_distance_on_flg_);
    this->distance_on_flg_ = distance_on_flg;
}

void Move_Decision::Set_RL_NeckAngle(double RL_NeckAngle)
{
    std::lock_guard<std::mutex> lock(mtx_RL_NeckAngle_);
    this->RL_NeckAngle_ = RL_NeckAngle;
}

void Move_Decision::Set_UD_NeckAngle(double UD_NeckAngle)
{
    std::lock_guard<std::mutex> lock(mtx_UD_NeckAngle_);
    this->UD_NeckAngle_ = UD_NeckAngle;
}

void Move_Decision::Set_RL_Neck_on_flg(bool RL_Neck_on_flg)
{
    std::lock_guard<std::mutex> lock(mtx_RL_Neck_on_flg);
    this->RL_Neck_on_flg_ = RL_Neck_on_flg;
}

void Move_Decision::Set_UD_Neck_on_flg(bool UD_Neck_on_flg)
{
    std::lock_guard<std::mutex> lock(mtx_UD_Neck_on_flg);
    this->UD_Neck_on_flg_ = UD_Neck_on_flg;
}

void Move_Decision::Set_SM_req_finish(bool SM_req_finish)
{
    std::lock_guard<std::mutex> lock(mtx_SM_req_finish_);
    this->SM_req_finish_ = SM_req_finish;
}

void Move_Decision::Set_TA_req_finish(bool TA_req_finish)
{
    std::lock_guard<std::mutex> lock(mtx_TA_req_finish_);
    this->TA_req_finish_ = TA_req_finish;
}

void Move_Decision::Set_UD_req_finish(bool UD_req_finish)
{
    std::lock_guard<std::mutex> lock(mtx_UD_req_finish_);
    this->UD_req_finish_ = UD_req_finish;
}

void Move_Decision::Set_RL_req_finish(bool RL_req_finish)
{
    std::lock_guard<std::mutex> lock(mtx_RL_req_finish_);
    this->RL_req_finish_ = RL_req_finish;
}

void Move_Decision::Set_EM_req_finish(bool EM_req_finish)
{
    std::lock_guard<std::mutex> lock(mtx_EM_req_finish_);
    this->EM_req_finish_ = EM_req_finish;
}

void Move_Decision::Set_ST_req_finish(bool ST_req_finish)
{
    std::lock_guard<std::mutex> lock(mtx_ST_req_finish_);
    this->ST_req_finish_ = ST_req_finish;
}

void Move_Decision::Set_walkcount_req(int32_t walkcount_req)
{
    std::lock_guard<std::mutex> lock(mtx_walkcount_req_);
    this->walkcount_req_ = walkcount_req;
}

//농구코드 추가

void Move_Decision::Set_Adjust_flg(bool adjust_flg)
{
    std::lock_guard<std::mutex> lock(mtx_adjust_flg);
    this->adjust_flg_ = adjust_flg;
}

void Move_Decision::Set_Shoot_flg(bool shoot_flg)
{
    std::lock_guard<std::mutex> lock(mtx_shoot_flg);
    this->shoot_flg_ = shoot_flg;
}

void Move_Decision::Set_hoop_det_flg_(bool hoop_det_flg_)
{
    std::lock_guard<std::mutex> lock(mtx_hoop_det_flg_);
    this->hoop_det_flg_ = hoop_det_flg_;
}

void Move_Decision::Set_pickmotion_det_flg_(bool pickmotion_det_flg_)
{
    std::lock_guard<std::mutex> lock(mtx_pickmotion_det_flg_);
    this->pickmotion_det_flg_= pickmotion_det_flg_;
}