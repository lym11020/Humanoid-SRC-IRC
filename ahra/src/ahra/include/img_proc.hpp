#ifndef IMG_PROC_HPP
#define IMG_PROC_HPP

#include <opencv2/opencv.hpp>
#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <thread>
#include <librealsense2/rs.hpp>
#include <vector>
#include <mutex>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <cuda_runtime.h>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudaarithm.hpp>    
#include <opencv2/cudafilters.hpp>   



// 필요한 매크로 정의 및 네임스페이스 설정
#define TOP_BORDER_LINE 240    // == (IMG_H / 2)
#define BOTTOM_BORDER_LINE 460 // == (IMG_H - 20)
#define LEFT_BORDER_LINE 300   // == (IMG_W/2 - 20)
#define RIGHT_BORDER_LINE 260  // == (IMG_W/2 + 20)

#define CIRCLE_RADIUS 100 // 50 -> 60 -> 100

#define LEFT_EDGE_BORDER_LINE 0 + 15
#define RIGHT_EDGE_BORDER_LINE 640 - 15

#define RR_LINE_CURVATURE 0.004 // 0.005 -> 0.004
#define Y_VERTEX 90

#define NOISE_DELETE_DELTA_X 120
#define CONTOUR_AREA 200
#define MIN_CONTOUR_AREA 50
#define NO_LINE_DETECT_DX 160

#define IMG_W 640
#define IMG_H 480

#define PROP_EXPOSURE -6
#define PROP_GAIN 128
#define PROP_TEMPERATURE 4985
#define PROP_BRIGHTNESS 128
#define PROP_CONTRAST 128
#define PROP_SATURATION 128

#define LINE_AREA 500
#define HUDDLE_AREA 200

#define HUDDLE_X_MARGIN 50
#define HUDDLE_Y_MARGIN 13
//____________//  
#define ADJUST_X_MARGIN 40
#define CREEK 30
#define HOOP_NECK 10
#define PICK_NECK 84
#define HUDDLE_NECK 91

//-------------------------------//

using namespace cv;
using namespace std;

class Img_proc
{
public:

    Img_proc();
    ~Img_proc();

    // ********************************************** 2D THREAD************************************************** //

    const double Robot_Height_Cam = 0.52;


    int threshold_value_white = 180;
    int threshold_value_yellow = 130;
    int threshold_value_green = 50;
    int threshold_value_black = 50;

    const int max_value = 255;
    int hue_lower = 0;
    int hue_upper = 179;
    int saturation_lower = 0;
    int saturation_upper = 255;
    int value_lower = 0;
    int value_upper = 255;
    // 변수 선언 (전역 변수로 트랙바 값들을 관리)
    int lowerH = 0, lowerS = 0, lowerV = 0;
    int upperH = 179, upperS = 255, upperV = 255;

    bool has_white_prev = false;
    bool has_yellow_prev = false;
    cv::Point center_now_white = cv::Point(474, 240);
    cv::Point center_now_yellow = cv::Point(474, 240);
    cv::Point center_huddle;
    cv::Point green_center;
    cv::Point topmost_point;
    cv::Point bottommost_point;


    cv::Scalar blue_color = {255, 0, 0};
    cv::Scalar green_color = {0, 255, 0};
    cv::Scalar red_color = {0, 0, 255};
    cv::Scalar yellow_color = {0, 255, 255};
    cv::Scalar white_color = {200, 200, 200};

    cv::Scalar lower_bound_yellow = {22, 59, 115}; // HSV에서 노란색의 하한값
    cv::Scalar upper_bound_yellow = {38, 255, 255};

    cv::Scalar lower_bound_white = {45, 0, 0};
    cv::Scalar upper_bound_white = {91, 255, 255};

    cv::Scalar lower_bound_blue = {101, 223, 108};
    cv::Scalar upper_bound_blue = {109, 255, 255};

    cv::Scalar lower_bound_green = {101, 223, 108};
    cv::Scalar upper_bound_green = {109, 255, 255};

    cv::Scalar lower_bound_red = { 0,50,20 };
    cv::Scalar upper_bound_red = { 20, 255, 255 };

    cv::Scalar lower_bound_ball_black = {0, 0, 0};
    cv::Scalar upper_bound_ball_black = {180, 255, 50};

    cv::Scalar lower_bound_ball_red = { 10,100,100 };
    cv::Scalar upper_bound_ball_red = { 40,255,255 };


    int line_condition_count = 0;
    int huddle_condition_count = 0;

    int top_contour_area = 0;

    bool a = 0;

    bool left = false;
    bool right = false;
    bool plane_direction = true;

    static void on_trackbar(int, void *);

    std::tuple<cv::Mat, cv::Mat> ROI_Line(const cv::Mat& input_frame, const cv::Mat& ori_frame);
    cv::Mat ROI_Rectangle(const cv::Mat &input_frame, int y_start, int y_end, int x_start, int x_end);
    std::tuple<cv::Mat, cv::Mat> extract_color(cv::Mat &input_frame, const cv::Scalar &lower_bound, const cv::Scalar &upper_bound);
    void create_color_range_trackbar(const std::string &window_name);
    


    // CUDA를 사용하는 함수는 별도로 선언
    std::tuple<cv::Mat, float, int, double>
    detect_Line_areas(const cv::Mat &input_frame, const cv::Mat &origin_frame, const cv::Scalar &contour_color, 
                            int threshold_value);
    std::tuple<cv::Mat, int, cv::Point, cv::Point, float, cv::Point, std::vector<cv::Point>, int>
    detect_Huddle_areas(const cv::Mat &input_frame, const cv::Mat &origin_frame, const cv::Scalar &contour_color, int threshold_value);
    cv::Point detect_green_point(const cv::Mat &input_frame, int threshold_value);
    std::tuple<cv::Mat, cv::Point3f> Hoop_Detect(cv::Mat color, cv::Mat depth_dist, int threshold_value);
    std::tuple<cv::Mat, cv::Point3f> Ball_Detect(cv::Mat color, cv::Mat depth_dist, int threshold_value);

    void realsense_thread();
    int8_t Athletics_FLAG = 0;

    // Cam set
    const int realsense_width = 848;
    const int realsense_height = 480;
    const int realsense_color_fps = 30;
    const int realsense_depth_fps = 30;
    const int realsense_fps = 30;

    double huddle_distance = 0;

    enum Interest_Object
    {
        None = 0,
        Ball = 1,
        Hoop = 2,
        Huddle = 3,
    };

Interest_Object img_proc_mode = Huddle;


    // ********************************************** GETTERS ************************************************** //

    bool Get_img_proc_line_det() const;
    bool Get_img_proc_no_line_det() const;
    bool Get_img_proc_huddle_det_3d() const;
    bool Get_img_proc_huddle_det_2d() const;
    bool Get_img_proc_stop_det() const;
    bool Get_img_proc_ball_det_() const;
    bool Get_img_proc_hoop_det_() const;
    bool Get_img_proc_huddle_det_() const;


    bool Get_img_proc_ball_done_() const;
    bool Get_img_proc_hoop_done_() const;
    bool Get_img_proc_huddle_done_() const;

    double Get_gradient() const;
    double Get_delta_x() const;
    double Get_huddle_distance() const;
    double Get_huddle_angle() const;


    bool Get_contain_huddle_to_foot() const;
    int Get_foot_huddle_distance() const;

    bool Get_img_proc_Far_Hoop_det() const;
    bool Get_img_proc_Adjust_det() const;
    bool Get_img_proc_Shoot_det() const;
    bool Get_img_proc_No_Hoop_det() const;
    int8_t Get_img_proc_Adjust_number() const;

    double Get_distance() const;
    double Get_adjust_angle() const;
    int Get_contain_adjust_to_foot() const;

    double Get_ball_x() const;
    double Get_ball_y() const;
    double Get_ball_z() const;

    double Get_hoop_x() const;
    double Get_hoop_y() const;
    double Get_hoop_z() const;

    int Get_hoopcounter_det_flg_() const;
    int Get_UD_NeckAngle_() const;


    // ********************************************** SETTERS ************************************************** //

    void Set_img_proc_line_det(bool img_proc_line_det);
    void Set_img_proc_no_line_det(bool img_proc_no_line_det);
    void Set_img_proc_goal_line_det(bool img_proc_goal_line_det);
    void Set_img_proc_huddle_det_2d(bool img_proc_huddle_det_2d);
    void Set_img_proc_stop_det(bool img_proc_stop_det);
    void Set_img_proc_ball_det_(bool img_proc_ball_det_);
    void Set_img_proc_hoop_det_(bool img_proc_hoop_det_);
    void Set_img_proc_huddle_det_(bool img_proc_huddle_det_);


    void Set_img_proc_ball_done_(bool img_proc_ball_done_);
    void Set_img_proc_hoop_done_(bool img_proc_hoop_done_);
    void Set_img_proc_huddle_done_(bool img_proc_huddle_done_);



    void Set_gradient(double gradient);
    void Set_delta_x(double delta_x);
    //void Set_plane_mode(bool plane_mode);


    void Set_huddle_distance(double huddle_distance);
    void Set_huddle_angle(double huddle_angle);

    void Set_contain_huddle_to_foot(bool contain_huddle_to_foot);
    void Set_foot_huddle_distance(int foot_huddle_distance);

    void Set_img_proc_Far_Hoop_det(bool img_proc_far_hoop_det);
    void Set_img_proc_Adjust_det(bool img_proc_adjust_det);
    void Set_img_proc_Shoot_det(bool img_proc_adjust_det);
    void Set_img_proc_No_Hoop_det(bool img_proc_no_hoop_det);

    void Set_img_proc_adjust_number(int8_t img_proc_adjust_number);

    void Set_distance(double set_distance);
    void Set_adjust_angle(double adjust_angle);
    void Set_contain_adjust_to_foot(int contain_adjust_to_foot);

    void Set_ball_x(double set_ball_x);
    void Set_ball_y(double set_ball_y);
    void Set_ball_z(double set_ball_z);

    void Set_hoop_x(double set_hoop_x);
    void Set_hoop_y(double set_hoop_y);
    void Set_hoop_z(double set_hoop_z);

    void Set_hoopcounter_det_flg_(int hoop_counter);
    void Set_UD_NeckAngle_(int UD_NeckAngle);


    // ********************************************** running ************************************************** //

    string Str_LINE_MODE = "LINE_MODE";
    string Str_NO_LINE_MODE = "NO_LINE_MODE";
    string Str_STOP_MODE = "STOP_MODE";
    string Str_WAKEUP_MODE = "WAKEUP_MODE";
    string Str_GOAL_MODE = "GOAL_MODE";
    string Str_HUDDLE_MODE = "HUDDLE_MODE";

    cv::VideoCapture vcap;
    Mat Origin_img;

    /////////////////////LINE////////////////////
    Point tmp_point_target = Point(IMG_W / 2, IMG_H / 2);
    Point point_target = Point(IMG_W / 2, IMG_H);
    std::vector<std::vector<cv::Point>> contours_;
    bool roi_line_flg = true;
    double delta_x_list[3] = {0.f, 0.f, 0.f};
    double delta_x_ = 0;
    // cv::Mat final_binary_mask;
    cv::Mat final_binary_mask = cv::Mat::zeros(IMG_H, IMG_W, CV_8UC1);

private:

    #ifdef __CUDACC__
        cudaStream_t stream;   // CUDA 스트림
    #endif
    ros::NodeHandle nh;
    ros::Publisher pub;
    const int SPIN_RATE;

    rs2::colorizer color_map;
    rs2::pipeline pipe;    // RealSense 파이프라인
    rs2::config cfg;       // 스트림 설정
    // cudaStream_t stream;   // CUDA 스트림

    // HSV and LAB parameter values
    int h_min, h_max, s_min, s_max, v_min, v_max;
    int l_min, l_max, a_min, a_max, b_min, b_max;

    // LINE Determine flg from img_proc
    bool img_proc_line_det_ = false;
    bool img_proc_no_line_det_ = false;
    bool img_proc_huddle_det_3d_ = false;
    bool img_proc_huddle_det_2d_ = false;
    bool img_proc_stop_det_ = false;

    bool img_proc_ball_det_ = false;
    bool img_proc_hoop_det_ = false;
    bool img_proc_huddle_det_ = false;

    bool img_proc_ball_done_ = false;
    bool img_proc_hoop_done_ = false;
    bool img_proc_huddle_done_ = false;


    // Determine flg from img_proc
    bool img_proc_far_hoop_det_ = false;
    bool img_proc_adjust_det_ = false;
    bool img_proc_shoot_det_ = false;
    bool img_proc_no_hoop_det_ = false;


    int8_t img_proc_adjust_number_ = 0;

    // Hoop mode
    double gradient_ = 0; // Line_angle
    double distance_ = 0; // huddle 

    double ball_x_ = 0;
    double ball_y_ = 0;
    double ball_z_ = 0;

    double hoop_x_ = 0;
    double hoop_y_ = 0;
    double hoop_z_ = 0;

    int hoop_counter_ = 0;
    int UD_NeckAngle_ = 0;


    double huddle_angle_ = 0; // huddle angle
    double huddle_distance_ = 0; // huddle distance

    //Adjust mode
    double adjust_angle_ = 0;
    int contain_adjust_to_foot_ = 0;

    // Huddle mode
    bool contain_huddle_to_foot_ = false;
    int foot_huddle_distance_ = 0;


    /////////////////////////////////////////// Mutex ///////////////////////////////////////////
    // LINE Determine flg from img_proc
    mutable std::mutex mtx_img_proc_line_det_;
    mutable std::mutex mtx_img_proc_no_line_det_;
    mutable std::mutex mtx_img_proc_huddle_det_3d; // realsense
    mutable std::mutex mtx_img_proc_huddle_det_2d; // webcam
    mutable std::mutex mtx_img_proc_stop_det_;

    mutable std::mutex mtx_img_proc_ball_det_;
    mutable std::mutex mtx_img_proc_hoop_det_;
    mutable std::mutex mtx_img_proc_huddle_det_;

    mutable std::mutex mtx_img_proc_ball_done_;
    mutable std::mutex mtx_img_proc_hoop_done_;
    mutable std::mutex mtx_img_proc_huddle_done_;
    // Line Mode

    mutable std::mutex mtx_gradient;
    
    // No Line Mode
    mutable std::mutex mtx_delta_x;


    // Huddle Mode
    mutable std::mutex mtx_contain_huddle_to_foot;
    mutable std::mutex mtx_huddle_angle_;
    mutable std::mutex mtx_huddle_distance_;
    mutable std::mutex mtx_foot_huddle_distance_;


    /////////////////////////////////////////// Mutex ///////////////////////////////////////////
   // LINE Determine flg from img_proc
    mutable std::mutex mtx_img_proc_far_hoop_det_;
    mutable std::mutex mtx_img_proc_adjust_det_;
    mutable std::mutex mtx_img_proc_shoot_det_;
    mutable std::mutex mtx_img_proc_no_hoop_det_;
 
    mutable std::mutex mtx_img_proc_adjust_number_;

    // Far_Hoop Mode

    mutable std::mutex mtx_distance;
    // No Hoop Mode
    //Adjust Mode
    mutable std::mutex mtx_adjust_angle;
    mutable std::mutex mtx_contain_adjust_to_foot;

    // Ball Mode
    mutable std::mutex mtx_ball_x;
    mutable std::mutex mtx_ball_y;
    mutable std::mutex mtx_ball_z;

    // Hoop Mode
    mutable std::mutex mtx_hoop_x;
    mutable std::mutex mtx_hoop_y;
    mutable std::mutex mtx_hoop_z;
    mutable std::mutex mtx_hoop_counter;
    mutable std::mutex mtx_UD_NeckAngle;
};

#endif // IMG_PROC_HPP