
#include <opencv2/opencv.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
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
// #include <cmath.hpp>
#include <cmath>

#define TOP_BORDER_LINE 240    // == (IMG_H / 2)
#define BOTTOM_BORDER_LINE 460 // == (IMG_H - 20) change this value when Up-Down Neck Angle changed or to adjust with RV value
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

#define LINE_AREA 200
#define HUDDLE_AREA 200

#define CORNER_X_MARGIN 150
#define CORNER_Y_MARGIN 180

#define HUDDLE_X_MARGIN 50
#define HUDDLE_Y_MARGIN 50

using namespace cv;
using namespace std;

class Img_proc
{
public:
    Img_proc();
    ~Img_proc(){};

    // ********************************************** 2D THREAD************************************************** //

    void webcam_thread();
    void webcam_thread2();

    void topic_publish(const std::string &topic_name)
    {
        pub = nh.advertise<std_msgs::Float32>(topic_name, 1000);
    }

    void publish_message(float msg_data)
    {
        std_msgs::Float32 msg;
        msg.data = msg_data;
        pub.publish(msg);
    }

    const double Robot_Height_Cam = 0.52;

    // Cam set
    const int webcam_width = 640;
    const int webcam_height = 480;

    // const int webcam_width = 320;
    // const int webcam_height = 240;

    const int webcam_fps = 30;
    const int webcam_id = 0;

    int threshold_value_white = 110;
    int threshold_value_yellow = 130;
    int threshold_value_blue = 0;
    const int max_value = 255;
    int hue_lower = 0;
    int hue_upper = 179;
    int saturation_lower = 0;
    int saturation_upper = 255;
    int value_lower = 0;
    int value_upper = 255;
    int corner_count = 0;

    bool has_white_prev = false;
    bool has_yellow_prev = false;
    bool Corner = false;
    cv::Point center_now_white = cv::Point(320, 240);
    cv::Point center_now_yellow = cv::Point(320, 240);
    cv::Point center_huddle;
    cv::Point blue_center;
    cv::Point topmost_point;
    cv::Point bottommost_point;
    // cv::Point corner_center;
    // cv::Point huddle_center;

    cv::Scalar blue_color = {255, 0, 0};
    cv::Scalar green_color = {0, 255, 0};
    cv::Scalar red_color = {0, 0, 255};
    cv::Scalar yellow_color = {0, 255, 255};
    cv::Scalar white_color = {200, 200, 200};

    cv::Scalar lower_bound_yellow = {10, 50, 100}; // HSV에서 노란색의 하한값
    cv::Scalar upper_bound_yellow = {40, 255, 255};

    cv::Scalar lower_bound_white = {0, 0, 100};
    cv::Scalar upper_bound_white = {255, 50, 255};

    cv::Scalar lower_bound_blue = {90, 170, 100};
    cv::Scalar upper_bound_blue = {125, 255, 255};

    int corner_condition_count = 0;
    int line_condition_count = 0;
    int huddle_condition_count = 0;

    int top_contour_area = 0;

    bool a = 0;

    bool left = false;
    bool right = false;
    bool plane_direction = true;

    static void on_trackbar(int, void *);
    void create_threshold_trackbar_W(const std::string &window_name);
    void create_threshold_trackbar_Y(const std::string &window_name);
    void create_threshold_trackbar_B(const std::string &window_name);
    void create_color_range_trackbar(const std::string &window_name);
    std::tuple<cv::Mat, cv::Mat> ROI_Line(const cv::Mat& input_frame, const cv::Mat& ori_frame);
    cv::Mat ROI_Circle(const cv::Mat &input_frame);
    cv::Mat ROI_Rectangle(const cv::Mat &input_frame, int y_start, int y_end, int x_start, int x_end);
    std::tuple<cv::Mat, cv::Mat> extract_color(const cv::Mat &input_frame, const cv::Scalar &lower_bound, const cv::Scalar &upper_bound);
    std::tuple<cv::Mat, bool, int, int, bool, int8_t, cv::Point, cv::Point, cv::Point, int, int, cv::Point, int, std::vector<cv::Point>, int> detect_Line_areas(const cv::Mat &input_frame, const cv::Mat &origin_frame, const cv::Scalar &contour_color, int threshold_value, bool check_disappearance = false, bool is_white_line = false);
    double Distance_Point(const rs2::depth_frame &depth, cv::Point center);

    // ********************************************** 3D THREAD************************************************** //

    Eigen::Vector3f calculateNormalVector(const std::vector<Eigen::Vector3f>& points);
    float calculateYRotationBetweenVectors(const Eigen::Vector3f& vec1, const Eigen::Vector3f& vec2);
    float calculateDistanceFromPlaneToCamera(const Eigen::Vector3f& normal, const Eigen::Vector3f& centroid);
    std::tuple<bool, float, float> applyPCA(cv::Mat &colorMat, cv::Mat depthMat, float depth_scale, cv::Mat depth_dist, rs2::depth_frame depth_frame, rs2_intrinsics intr, int startX, int startY, int endX, int endY);
    void realsense_thread();
    int8_t Athletics_FLAG = 0;

    // Cam set
    const int realsense_width = 848;
    const int realsense_height = 480;
    const int realsense_color_fps = 15;
    const int realsense_depth_fps = 15;

    double huddle_distance = 0;

    // ********************************************** GETTERS ************************************************** //
    bool Get_img_proc_Far_Hoop_det() const;
    bool Get_img_proc_Adjust_det() const;
    bool Get_img_proc_shoot_det() const;
    bool Get_img_proc_No_Hoop_det() const;
    bool Get_img_proc_line_det() const;
    bool Get_img_proc_no_line_det() const;
    bool Get_img_proc_corner_det_3d() const;
    bool Get_img_proc_corner_det_2d() const;
    bool Get_img_proc_goal_line_det() const;
    bool Get_img_proc_huddle_det_3d() const;
    bool Get_img_proc_huddle_det_2d() const;
    bool Get_img_proc_wall_det() const;
    bool Get_img_proc_stop_det() const;
    int8_t Get_img_proc_wall_number() const;
    int8_t Get_img_proc_corner_number() const;

    double Get_gradient() const;
    double Get_delta_x() const;
    double Get_wall_angle() const;
    double Get_wall_distance() const;
    double Get_huddle_distance() const;
    double Get_huddle_angle() const;
    double Get_corner_angle() const;
    bool Get_plane_mode() const;
    double Get_distance() const;

    bool Get_contain_huddle_to_foot() const;
    bool Get_contain_corner_to_foot() const;
    int Get_foot_huddle_distance() const;

    // ********************************************** SETTERS ************************************************** //
    void Set_img_proc_Far_Hoop_det(bool img_proc_far_hoop_det);
    void Set_img_proc_Adjust_det(bool img_proc_adjust_det);
    void Set_img_proc_No_Hoop_det(bool img_proc_no_hoop_det);
    void Set_img_proc_shoot_det(bool img_proc_shoot_det);
    void Set_img_proc_line_det(bool img_proc_line_det);
    void Set_img_proc_no_line_det(bool img_proc_no_line_det);
    void Set_img_proc_corner_det_3d(bool img_proc_corner_det_3d);
    void Set_img_proc_corner_det_2d(bool img_proc_corner_det_2d);
    void Set_img_proc_goal_line_det(bool img_proc_goal_line_det);
    void Set_img_proc_huddle_det_3d(bool img_proc_huddle_det_3d);
    void Set_img_proc_huddle_det_2d(bool img_proc_huddle_det_2d);
    void Set_img_proc_stop_det(bool img_proc_stop_det);
    void Set_img_proc_wall_det(bool img_proc_wall_det);
    void Set_img_proc_wall_number(int8_t img_proc_wall_number);
    void Set_img_proc_corner_number(int8_t img_proc_corner_number);

    void Set_gradient(double gradient);
    void Set_delta_x(double delta_x);
    void Set_wall_angle(double wall_angle);
    void Set_wall_distance(double wall_distance);
    void Set_plane_mode(bool plane_mode);


    void Set_huddle_distance(double huddle_distance);
    void Set_huddle_angle(double huddle_angle);
    void Set_corner_angle(double corner_angle);

    void Set_contain_huddle_to_foot(bool contain_huddle_to_foot);
    void Set_contain_corner_to_foot(bool contain_corner_to_foot);
    void Set_foot_huddle_distance(int foot_huddle_distance);

    // ********************************************** running ************************************************** //

    string Str_LINE_MODE = "LINE_MODE";
    string Str_NO_LINE_MODE = "NO_LINE_MODE";
    string Str_STOP_MODE = "STOP_MODE";
    string Str_WAKEUP_MODE = "WAKEUP_MODE";
    string Str_GOAL_MODE = "GOAL_MODE";
    string Str_HUDDLE_MODE = "HUDDLE_MODE";
    string Str_WALL_MODE = "WALL_MODE";
    string Str_CORNER_MODE = "CORNER_MODE";

    cv::VideoCapture vcap;
    Mat Origin_img;

    void RGB2HSV(const cv::Mat &rgb_image, cv::Mat &hsv_image);
    void RGB2LAB(const cv::Mat &rgb_image, cv::Mat &lab_image);
    void saveParameters(const std::string &filename);
    void loadParameters(const std::string &filename);

    void running_process();
    // void extractAndDisplayObject2(cv::VideoCapture& cap, const cv::Scalar& hsv_lower, const cv::Scalar& hsv_upper, const cv::Scalar& lab_lower, const cv::Scalar& lab_upper);

    void init();
    void LINE_imgprocessing();
    void GOAL_LINE_recognition();

    /////////////////////LINE////////////////////
    Point tmp_point_target = Point(IMG_W / 2, IMG_H / 2);
    Point point_target = Point(IMG_W / 2, IMG_H);
    std::vector<std::vector<cv::Point>> contours_;
    bool roi_line_flg = true;
    double delta_x_list[3] = {0.f, 0.f, 0.f};
    double delta_x_ = 0;
    // cv::Mat final_binary_mask;
    cv::Mat final_binary_mask = cv::Mat::zeros(IMG_H, IMG_W, CV_8UC1);
    double Calc_angle(double _x, Point _pt);

private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    const int SPIN_RATE;

    // HSV and LAB parameter values
    int h_min, h_max, s_min, s_max, v_min, v_max;
    int l_min, l_max, a_min, a_max, b_min, b_max;

    // LINE Determine flg from img_proc
    bool img_proc_far_hoop_det_ = false;
    bool img_proc_adjust_det_ = false;
    bool img_proc_no_hoop_det_ = false;
    bool img_proc_shoot_det_ = false; // Initialize it according to your needs

    bool img_proc_line_det_ = false;
    bool img_proc_no_line_det_ = false;
    bool img_proc_corner_det_3d_ = false;
    bool img_proc_corner_det_2d_ = false;
    bool img_proc_goal_det_ = false;
    bool img_proc_huddle_det_3d_ = false;
    bool img_proc_huddle_det_2d_ = false;
    bool img_proc_wall_det_ = false;
    bool img_proc_stop_det_ = false;
    int8_t img_proc_wall_number_ = 0;
    int8_t img_proc_corner_number_ = 0; // 1번 ㅓ(좌90) 2번 ㅜ(우90)

    double gradient_ = 0;     // Line_angle
    double wall_angle_ = 0;   // wall angle
    double huddle_angle_ = 0; // huddle angle
    double huddle_distance_ = 0; // huddle distance
    double corner_angle_ = 0; //corner angle

    //  wall mode
    double wall_distance_ = 0;
    bool prev_plane_direction = false;
    int consecutive_changes = 0;
    const int CHANGE_THRESHOLD = 10;
    bool plane_mode_ = 0;
    float previous_real_distance = 0.0f;
    int change_counter = 0;
    const int FRAME_THRESHOLD = 200;
    const float DISTANCE_THRESHOLD = 0.5f;
    float previous_angle = 0.0f;

    // No Line mode
    // delta_x : Center of window.x - Center of last captured line.x
    // delta_x > 0 : LEFT
    // delta_x < 0 : RIGHT

    // Huddle mode
    bool contain_huddle_to_foot_ = false;
    int foot_huddle_distance_ = 0;

    // Corner mode
    bool contain_corner_to_foot_ = false;

    /////////////////////////////////////////// Mutex ///////////////////////////////////////////
    // LINE Determine flg from img_proc
    mutable std::mutex mtx_img_proc_far_hoop_det_;
    mutable std::mutex mtx_img_proc_adjust_det_;
    mutable std::mutex mtx_img_proc_shoot_det_;
    mutable std::mutex mtx_img_proc_no_hoop_det_;
    mutable std::mutex mtx_img_proc_line_det_;
    mutable std::mutex mtx_img_proc_no_line_det_;
    mutable std::mutex mtx_img_proc_corner_det_3d; // realsense
    mutable std::mutex mtx_img_proc_corner_det_2d; // webcam
    mutable std::mutex mtx_img_proc_goal_det_;
    mutable std::mutex mtx_img_proc_huddle_det_3d; // realsense
    mutable std::mutex mtx_img_proc_huddle_det_2d; // webcam
    mutable std::mutex mtx_img_proc_wall_det_;
    mutable std::mutex mtx_img_proc_stop_det_;
    mutable std::mutex mtx_img_proc_wall_number_;
    mutable std::mutex mtx_img_proc_corner_number_;

    // Line Mode

    mutable std::mutex mtx_gradient;
    
    // No Line Mode
    mutable std::mutex mtx_delta_x;
    
    // Wall Mode
    mutable std::mutex mtx_wall_angle;
    mutable std::mutex mtx_wall_distance_;
    mutable std::mutex mtx_plane_mode_;

    // Huddle Mode
    mutable std::mutex mtx_contain_huddle_to_foot;
    mutable std::mutex mtx_huddle_angle_;
    mutable std::mutex mtx_huddle_distance_;
    mutable std::mutex mtx_foot_huddle_distance_;

    // Corner mode
    mutable std::mutex mtx_contain_corner_to_foot;
    mutable std::mutex mtx_corner_angle_;
 
    mutable std::mutex mtx_distance;
    mutable std::mutex mtx_adjust_angle;
    mutable std::mutex mtx_contain_adjust_to_foot;
};