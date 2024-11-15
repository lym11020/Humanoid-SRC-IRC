#ifndef MOVE_DECISION_H
#define MOVE_DECISION_H

#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>
#include <ros/ros.h>
#include <boost/thread.hpp>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
#include <opencv2/opencv.hpp>
#include <string.h>
#include <tf/tf.h>

#include "ahra/SendMotion.h"

#include "img_proc.hpp"

#define UD_MAX 90
#define UD_MIN 0
#define UD_CENTER 84
#define HOOP_NECK 10
#define PICK_NECK 84
#define HUDDLE_NECK 91
#define RL_MAX 90
#define RL_MIN -90
#define RL_CENTER 0

#define TURN_MAX 10
#define TURN_MIN -10

#define LINE_TURN 10
#define START_TURN 5

#define HUDDLE_TURN 5

#define MARGIN_GRADIENT 10 // margin of straight line
#define MARGIN_GRADIENT_START 5
//농구코드 추가
#define ADJUST_TURN 5
#define ADJUST_Y_MARGIN 125
#define NOLINE_ANGLE 15
 
using namespace std;

class Move_Decision
{
public:
    enum Motion_Index
    {
        InitPose = 0,
        Forward_2step = 1,
        Left_2step = 2,
        Step_in_place = 3,
        Right_2step = 4,
        Forward_Nstep = 5,
        Huddle_Jump = 6,
        ForWard_fast4step = 7,
        FWD_UP = 8,
        BWD_UP = 9,
        Forward_Halfstep = 10,
        Left_Halfstep = 11,
        Right_Halfstep = 12,
        Back_Halfstep = 13,
        Back_2step = 33,
        Forward_1step = 14,
        Left_6step = 15,
        Right_6step = 16,
        NONE = 99,
        Test = 56,

        Huddle_Jump_V2 = 55,
        //농구코드 추가
        Shoot = 17,
        Ready_to_throw = 18,
        Right_1step = 19,
        Left_1step = 20,
        Grab = 30,
        START = 50,
        FINISH = 100,

        Gen2_Walking = 111,
        Picking_Ball = 112,
    };

    enum Running_Mode
    {
        LINE_MODE = 0,
        NO_LINE_MODE = 1,
        STOP_MODE = 2,
        WAKEUP_MODE = 3,
        GOAL_MODE = 4,
        HUDDLE_MODE = 5,

        //농구코드 추가
        FAR_HOOP_MODE = 10,
        ADJUST_MODE = 11,
        SHOOT_MODE = 14,
        NO_HOOP_MODE = 15,
        PICK_MODE = 17,
        START_MODE = 18
    };

    enum Stand_Status
    {
        Stand = 0,
        Fallen_Forward = 1,
        Fallen_Back = 2,
    };

    bool START_FLG = true;

    string Str_InitPose = "InitPose";
    string Str_Forward_2step = "Forward_2step";
    string Str_Forward_1step = "Forward_1step";
    string Str_Left_2step = "Left_2step";
    string Str_Step_in_place = "Step_in_place";
    string Str_Right_2step = "Right_2step";
    string Str_ForWard_fast4step = "ForWard_fast4step";
    string Str_Forward_Nstep = "Forward_Nstep";
    string Str_Huddle_Jump = "Huddle_Jump";
    string Str_Huddle_Jump_V2 = "Huddle_Jump_V2";
    string Str_Forward_Halfstep = "Forward_Halfstep";
    string Str_Left_Halfstep = "Left_Halfstep";
    string Str_Right_Halfstep = "Right_Halfstep";
    string Str_Back_Halfstep = "Back_Halfstep";
    string Str_Back_2step = "Back_2step";
    string Str_Left_6step = "Left_6step";
    string Str_Right_6step = "Right_6step";
    string Str_FWD_UP = "FWD_UP";
    string Str_BWD_UP = "BWD_UP";
    string Str_NONE = "NONE";
    string Str_START = "START";

    //농구코드 추가
    string Str_Ready_to_throw = "Ready_to_throw_MODE";
    string Str_Shoot = "SHOOT_MODE";
    string Str_LINE_MODE = "LINE_MODE";
    string Str_NO_LINE_MODE = "NO_LINE_MODE";
    string Str_STOP_MODE = "STOP_MODE";
    string Str_WAKEUP_MODE = "WAKEUP_MODE";
    string Str_GOAL_MODE = "GOAL_MODE";
    string Str_HUDDLE_MODE = "HUDDLE_MODE";

    //농구코드 추가
    string Str_FAR_HOOP_MODE = "FAR_HOOP_MODE";
    string Str_ADJUST_MODE = "ADJUST_MODE";
    string Str_NO_HOOP_MODE = "NO_HOOP_MODE";
    string Str_PICK_MODE = "PICK_MODE";

    string Str_TEST_MODE = "TEST_MODE";
    string Str_Gen2_Walking = "Gen2_Walking";
    string Str_Picking_Ball = "Picking_Ball";
    string Str_FINISH = "FINISH";

    // Constructor
    Move_Decision(Img_proc *img_procPtr);
    Img_proc *img_procPtr;

    // Destructor
    ~Move_Decision();

    // ********************************************** PROCESS THREAD************************************************** //

    void process(bool Switch_ON); 
    void processThread();
    void LINE_mode();
    void NOLINE_mode();
    void HUDDLE_mode2(); // using webcam


    //농구코드 추가
    void ADJUST_mode();
    void SHOOT_mode();
    void PICK_mode();
    void START_LINE_mode();


    bool tmp_img_proc_line_det_flg_ = false;
    bool tmp_img_proc_no_line_det_flg_ = false;
    bool tmp_img_proc_huddle_det_flg_2d_ = false;
    bool tmp_img_proc_huddle_det_flg_3d_ = false;
    bool tmp_img_proc_goal_det_flg_ = false;

    //농구코드 추가
    bool tmp_img_proc_far_hoop_flg_ = false;
    bool tmp_img_proc_adjust_flg_ = false;
    bool tmp_img_proc_shoot_flg_ = false;
    bool tmp_img_proc_no_hoop_flg_ = false;
    bool tmp_img_proc_ball_det_flg_ = false;
    bool tmp_img_proc_hoop_det_flg_ = false;

    void set_pick_mode(bool is_in_pick_mode) { is_in_pick_mode_ = is_in_pick_mode; }
    bool is_in_pick_mode() const { return is_in_pick_mode_; }


    void set_huddle_mode(bool is_in_huddle_mode) { is_in_huddle_mode_ = is_in_huddle_mode; }
    bool is_in_huddle_mode() const { return is_in_huddle_mode_; }


    void set_shoot_mode(bool is_in_shoot_mode) { is_in_shoot_mode_ = is_in_shoot_mode; }
    bool is_in_shoot_mode() const { return is_in_shoot_mode_; }

    // ********************************************** CALLBACK THREAD ************************************************** //

    void Running_Mode_Decision();
    void callbackThread();
    void startMode();

    bool SendMotion(ahra::SendMotion::Request &req, ahra::SendMotion::Response &res);

    std::tuple<int8_t, double> playMotion();
    double turn_angle();
    double Move_UD_NeckAngle();
    double Move_RL_NeckAngle();
    bool Emergency();
    

    // Publish & Subscribe
    // ros::Publisher Emergency_pub_;
    // IMU
    void IMUsensorCallback(const std_msgs::Float32::ConstPtr &IMU);
    ros::Subscriber IMU_sensor_x_subscriber_; ///< Gets IMU Sensor data from Sensor_node
    ros::Subscriber IMU_sensor_y_subscriber_; ///< Gets IMU Sensor data from Sensor_node
    ros::Subscriber IMU_sensor_z_subscriber_; ///< Gets IMU Sensor data from Sensor_node

    bool stop_fallen_check_;
    double present_pitch_;
    double present_roll_;
    Eigen::VectorXd RPY = Eigen::VectorXd::Zero(3); // Roll Pitch Yaw

    // Server && Client
    ros::ServiceServer SendMotion_server_;

    // ********************************************** FUNCTION ************************************************** //
    //농구코드 추가
    Eigen::Vector3d convertRotationToRPY(const Eigen::Matrix3d &rotation);
    Eigen::Vector3d convertQuaternionToRPY(const Eigen::Quaterniond &quaternion);

    void Motion_Info();
    void Send_Motion_Info(int8_t res_motion);
    void Send_Info(int8_t motion_, double turn_angle_, double ud, double rl, bool emg);
    void Running_Info();
    void AllModereset(int8_t mode);
    double Relax_huddle_to_foot(double distance_);
    double Relax_X(double x);
    double Relax_Y(double y);
    double Relax_Z(double z);
    //농구코드 추가
    void DistanceDecision(double distance_);

    /////////////////////// Sequence++ ///////////////////////
    bool finish_past = false;
    int8_t req_finish_count = 0;


    // void CalculateQuotientAndRemainder(int dividend, int divisor, int &quotient, int &remainder);

    // ********************************************** GETTERS ************************************************** //

    bool Get_Emergency_() const;
    int8_t Get_motion_index_() const;
    int8_t Get_stand_status_() const;
    int8_t Get_running_mode_() const;
    double Get_turn_angle_() const;
    double Get_distance_() const;

    bool Get_ProcessON() const;
    bool Get_MoveDecisionON() const;
    bool Get_CallbackON() const;

    //농구코드 추가
    bool Get_Adjust_flg() const;
    bool Get_Shoot_flg() const;


    // RUNNING MODE
    bool Get_goal_line_det_flg() const;
    bool Get_line_det_flg() const;
    bool Get_no_line_det_flg() const;
    bool Get_huddle_det_flg_3d() const;
    bool Get_huddle_det_flg_2d() const;
    bool Get_stop_det_flg() const;
    bool Get_huddle_det_stop_flg() const;

    bool Get_ball_det_flg_() const;
    bool Get_hoop_det_flg_() const;
    bool Get_pickmotion_det_flg_() const;

    bool Get_select_motion_on_flg() const;
    bool Get_turn_angle_on_flg() const;
    bool Get_emergency_on_flg() const;
    bool Get_distance_on_flg() const;

    bool Get_response_sent_() const;

    double Get_RL_NeckAngle() const;
    double Get_UD_NeckAngle() const;
    bool Get_RL_Neck_on_flg() const;
    bool Get_UD_Neck_on_flg() const;

    bool Get_SM_req_finish() const;
    bool Get_TA_req_finish() const;
    bool Get_UD_req_finish() const;
    bool Get_RL_req_finish() const;
    bool Get_EM_req_finish() const;
    bool Get_ST_req_finish() const;
    int32_t Get_walkcount_req_() const;


    // ********************************************** SETTERS ************************************************** //

    void Set_Emergency_(bool Emergency);
    void Set_motion_index_(int8_t motion_index);
    void Set_stand_status_(int8_t stand_status);
    void Set_running_mode_(int8_t running_mode);
    void Set_turn_angle_(double turn_angle);
    void Set_distance_(double distance);

    void Set_ProcessON(bool ProcessON);
    void Set_MoveDecisionON(bool MoveDecisionON);
    void Set_CallbackON(bool CallbackON);

    void Set_response_sent_(bool response_sent);

    void Set_line_det_flg(bool line_det_flg);
    void Set_no_line_det_flg(bool no_line_det_flg);
    void Set_huddle_det_flg_2d(bool huddle_det_flg_2d);
    void Set_huddle_det_flg_3d(bool huddle_det_flg_3d);
    void Set_stop_det_flg(bool stop_det_flg);
    void Set_huddle_det_stop_flg(bool huddle_det_stop_flg);
    
    void Set_select_motion_on_flg(bool select_motion_on_flg);
    void Set_turn_angle_on_flg(bool turn_angle_on_flg);
    void Set_emergency_on_flg(bool emergency_on_flg);
    void Set_distance_on_flg(bool distance_on_flg);

    void Set_RL_NeckAngle(double RL_NeckAngle);
    void Set_UD_NeckAngle(double UD_NeckAngle);
    void Set_RL_Neck_on_flg(bool RL_Neck_on_flg);
    void Set_UD_Neck_on_flg(bool UD_Neck_on_flg);

    void Set_SM_req_finish(bool SM_req_finish);
    void Set_TA_req_finish(bool TA_req_finish);
    void Set_UD_req_finish(bool UD_req_finish);
    void Set_RL_req_finish(bool RL_req_finish);
    void Set_EM_req_finish(bool EM_req_finish);
    void Set_ST_req_finish(bool ST_req_finish);
    void Set_walkcount_req(int32_t walkcount_req);



    //농구코드 추가
    void Set_Adjust_flg(bool adjust_flg);
    void Set_Shoot_flg(bool shoot_flg);
    void Set_hoop_det_flg_(bool hoop_det_flg_);


    void Set_ball_det_flg_(bool ball_det_flg_);
    void Set_pickmotion_det_flg_(bool pickmotion_det_flg_);

    // ********************************************** IMG_PROC ************************************************** //

    /////////////////////// Far Hoop Mode /////////////////zzz//////농구
    bool robot_forward = false;
    double Green_area_dis = 1;
    int8_t hoop_distance = 0;
    int8_t far_hoop_motion = 0;

    /////////////////////// Adjust Mode ///////////////////////농구
    // 0 : Approach to the Adjust --> Motion : Motion_Index::Forward_Halfstep (Until adjust center)
    // 1 : Pose Control (Posture(Gradient))
    // 2 : Motion : SHOOT
    // 3 : Initializing
    int8_t tmp_adjust_seq = 0;
    int8_t img_proc_adjust_delta_x = 0;
    int8_t adjust_motion = 0;
    int img_proc_contain_adjust_to_foot = 0; // adjust Y Point
    bool contain_adjust_X = false;                // adjust X Point
    bool contain_adjust_Y = false;                // adjust Y Point
    bool adjust_posture = false;                  // adjust gradient
    bool adjust_seq_finish = false;
    double img_proc_adjust_angle = 0;
    double adjust_actual_angle = 0;
    int stable_count = 0;  // 안정적인 거리를 확인하는 카운터
    const int stable_threshold = 5;  // 이 카운트를 넘으면 안정적이라고 판단
    int count_hoop=0;
    string Str_ADJUST_SEQUENCE_0 = "ADJUST_SEQUENCE_0 : POSTURE CONTROL";
    string Str_ADJUST_SEQUENCE_1 = "ADJUST_SEQUENCE_1 : POSITION CONTROL";
    string Str_ADJUST_SEQUENCE_2 = "ADJUST_SEQUENCE_2 : POSTURE CONTROL ONE MORE TIME";
    string Str_ADJUST_SEQUENCE_3 = "ADJUST_SEQUENCE_3 : Ready_to_throw";
    string Str_ADJUST_SEQUENCE_4 = "ADJUST_SEQUENCE_4 : SHOOT";
    string Str_ADJUST_SEQUENCE_5 = "ADJUST_SEQUENCE_5 : INITIALIZING";

    /////////////////////// No Hoop Mode ///////////////////////농구
    int8_t nohoop_motion = 0;
    double nohoop_actual_angle = 0;

    /////////////////////// Line Mode ///////////////////////
    // StraightLine
    bool straightLine;
    double margin_gradient = 20; // margin of straight line
    void StraightLineDecision(double gra, double mg_gra);
    double Angle_toBeStraight = 40; // max or min
    int8_t line_gradient = 0;
    double line_actual_angle = 0;
    int8_t line_motion = 0;
    double line_ud_neckangle = 0;
    double hoop_ud_neckangle = 0;
    
    int sh =0;
    bool once_flg = false;
    bool real_once_flg = false;
    double tmp_walkcount = 0;
    double tmp_real_walkcount = 0;

    /////////////////////// No Line Mode ///////////////////////
    // If no find line (NO_LINE_MODE)
    // delta_x : Center of window.x - Center of last captured line.x
    // delta_x > 0 : LEFT
    // delta_x < 0 : RIGHT
    // Out of Range -> A straight trun walking
    int8_t tmp_delta_x = 0;
    int8_t noline_motion = 0;
    double noline_actual_angle = 0;
    double Angle_ToFindLine = 15; // max or min
    double noline_neckangle = 0;

    // Actural send turn angle
    double Actual_angle = 0;
    double increment = 0;

    /////////////////////// WAKEUP_MODE ///////////////////////
    // WakeUp_seq = 0 : Initial
    // WakeUp_seq = 1 : FWD_UP or BWD_UP
    // WakeUp_seq = 2 : Motion_Index : Initial_pose
    // WakeUp_seq = 3 : Line_mode()
    int8_t WakeUp_seq = 0;
    int8_t tmp_stand_status = 0;
    int8_t wakeup_motion = 0;
    int8_t wakeup_running = 0;

    /////////////////////// Huddle Mode ///////////////////////

    // 0 : Pose Control (Posture(Gradient))
    // --> Motion : Motion_Index::Step_In_Place && Turn Angle(Gradient)

    // 1 : Approach to the Huddle
    // --> Motion : Motion_Index::Forward_Halfstep (Until Huddle center) : About Y diff

    // 2 : Pose Control (Posture(Gradient))
    // --> Motion : Motion_Index::Step_In_Place && Turn Angle(Gradient)

    // 3 : Motion : HUDDLE_JUMP

    // 4 : Initializing
    int8_t tmp_huddle_seq = 0;
    double huddle_distance = 0;
    double huddle_actual_angle = 0;
    int8_t huddle_motion = 0;
    double huddle_ud_neck_angle = 0;
    std::vector<double> huddle_distance_save;
    bool contain_huddle_to_foot = false;
    int8_t to_be_line_mode = 0;

    std::vector<double> x_save;
    std::vector<double> y_save;
    std::vector<double> z_save;

    bool img_proc_contain_huddle_to_foot = false; // huddle Y Point
    int8_t img_proc_huddle_delta_x = 0;
    double img_proc_huddle_angle = 0;

    bool contain_huddle_X = false; // huddle X Point
    bool contain_huddle_Y = false; // huddle Y Point
    bool huddle_posture = false;   // huddle gradient

    bool huddle_seq_finish = false;

    string Str_HUDDLE2_SEQUENCE_0 = "HUDDLE_SEQUENCE_0 : POSTURE CONTROL";
    string Str_HUDDLE2_SEQUENCE_1 = "HUDDLE_SEQUENCE_1 : POSITION CONTROL";
    string Str_HUDDLE2_SEQUENCE_2 = "HUDDLE_SEQUENCE_2 : POSTURE CONTROL ONE MORE TIME";
    string Str_HUDDLE2_SEQUENCE_3 = "HUDDLE_SEQUENCE_3 : HUDDLE JUMP";
    string Str_HUDDLE2_SEQUENCE_4 = "HUDDLE_SEQUENCE_4 : INITIALIZING";


    /////////PICK_MODE////
    int8_t tmp_pick_seq = 0;

    ////////SHOOT_MODE????
    int8_t tmp_shoot_seq = 0;

    /////////////////////// WAKEUP_MODE ///////////////////////
    // WakeUp_seq = 0 : Initial
    // WakeUp_seq = 1 : FWD_UP or BWD_UP
    // WakeUp_seq = 2 : Motion_Index : Initial_pose
    // WakeUp_seq = 3 : Line_mode()
    int warning_counter = 0;
    bool warning_printed = false;

private:
    ros::NodeHandle nh;
    ros::Publisher pub;

    bool response_sent_ = false;
    std::set<int> processed_requests_;

    void recordProcessedRequest(int request_id)
    {
        processed_requests_.insert(request_id);
    }

    // Check if the request ID has already been processed
    bool isRequestProcessed(int request_id)
    {
        return processed_requests_.find(request_id) != processed_requests_.end();
    }

    const double FALL_FORWARD_LIMIT;
    const double FALL_BACK_LIMIT;
    const int SPIN_RATE;

    int8_t motion_index_ = 99;
    int8_t stand_status_;
    int8_t running_mode_;


    // Body Angle
    // Counter Clock Wise(+)
    // LEFT(+) / RIGHT(-)
    double turn_angle_ = 0;

    // Distance
    double distance_ = 0;

    // Neck
    // Counter Clock Wise(+)
    // LEFT(+) / RIGHT(-)
    double RL_NeckAngle_ = 0;
    bool RL_Neck_on_flg_ = false;
    // Counter Clock Wise(+)
    // UP(+) / DOWN(-)
    double UD_NeckAngle_ = 0;
    bool UD_Neck_on_flg_ = false;

    // Running mode
    bool goal_line_det_flg_ = false;
    bool line_det_flg_ = false;
    bool no_line_det_flg_ = false;
    bool huddle_det_flg_3d_ = false;
    bool huddle_det_flg_2d_ = false;
    bool stop_det_flg_ = false;

    // True : unEnable to write the value
    // False : Enable to write the value
    bool select_motion_on_flg_ = false;
    bool turn_angle_on_flg_ = false;
    bool emergency_on_flg_ = false;
    bool distance_on_flg_ = false;

    bool huddle_det_stop_flg_ = false;

    bool Emergency_;

    //농구코드 추가
    bool far_hoop_flg_ = false;
    bool adjust_flg_ = false;
    bool shoot_flg_ = false;
    bool no_hoop_flg_ = false;
    bool hoop_det_flg_ = false;
    bool pickmotion_det_flg_ = false;
    bool ball_det_flg_ = false;


    /// Thread switch ///
    bool ProcessON_;
    bool MoveDecisionON_;
    bool CallbackON_;

    // true -> req.finish is true
    // false -> req.finish is false
    bool SM_req_finish_ = false;
    bool TA_req_finish_ = false;
    bool UD_req_finish_ = false;
    bool RL_req_finish_ = false;
    bool EM_req_finish_ = false;
    bool ST_req_finish_ = false;
    int32_t walkcount_req_ = 0;

    bool is_in_pick_mode_ = false;
    int pick_mode_step_ = 0;
    bool is_in_huddle_mode_ = false;
    bool is_in_shoot_mode_ = false;
    int huddle_mode_step_ = 0;
    int shoot_mode_step_ = 0;

    // ********************************************** MUTEX ************************************************** //
    mutable std::mutex mtx_goal_line_det_flg;
    mutable std::mutex mtx_line_det_flg;
    mutable std::mutex mtx_no_line_det_flg;
    mutable std::mutex mtx_huddle_det_flg_3d;
    mutable std::mutex mtx_huddle_det_flg_2d;
    mutable std::mutex mtx_stop_det_flg;
    mutable std::mutex mtx_huddle_det_stop_flg;

    mutable std::mutex mtx_RL_NeckAngle_;
    mutable std::mutex mtx_UD_NeckAngle_;

    mutable std::mutex mtx_RL_Neck_on_flg;
    mutable std::mutex mtx_UD_Neck_on_flg;

    mutable std::mutex mtx_turn_angle_;
    mutable std::mutex mtx_distance_;

    mutable std::mutex mtx_motion_index_;
    mutable std::mutex mtx_stand_status_;
    mutable std::mutex mtx_running_mode_;
    mutable std::mutex mtx_response_sent_;

    mutable std::mutex mtx_select_motion_on_flg_;
    mutable std::mutex mtx_turn_angle_on_flg_;
    mutable std::mutex mtx_emergency_on_flg_;
    mutable std::mutex mtx_distance_on_flg_;

    mutable std::mutex mtx_Emergency_;
    mutable std::mutex mtx_ProcessON_;
    mutable std::mutex mtx_MoveDecisionON_;
    mutable std::mutex mtx_CallbackON_;

    mutable std::mutex mtx_SM_req_finish_;
    mutable std::mutex mtx_TA_req_finish_;
    mutable std::mutex mtx_UD_req_finish_;
    mutable std::mutex mtx_RL_req_finish_;
    mutable std::mutex mtx_EM_req_finish_;
    mutable std::mutex mtx_ST_req_finish_;
    mutable std::mutex mtx_walkcount_req_;

    //농구코드 추가
    mutable std::mutex mtx_far_hoop_flg;
    mutable std::mutex mtx_adjust_flg;
    mutable std::mutex mtx_shoot_flg;
    mutable std::mutex mtx_no_hoop_flg;

    mutable std::mutex mtx_ball_det_flg_;
    mutable std::mutex mtx_hoop_det_flg_;
    mutable std::mutex mtx_pickmotion_det_flg_;
    // mutable std::mutex mtx_huddle_det_flg_;
};

#endif // MOVE_DECISION_H