#include "img_proc.hpp"

// CUDA 커널 함수
__global__ void color_to_hsv(const unsigned char* bgr, unsigned char* hsv, int width, int height)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if (x < width && y < height) {
        int index = (y * width + x) * 3;
        float b = bgr[index + 0] / 255.0f;
        float g = bgr[index + 1] / 255.0f;
        float r = bgr[index + 2] / 255.0f;

        float max_val = fmaxf(fmaxf(r, g), b);
        float min_val = fminf(fminf(r, g), b);
        float delta = max_val - min_val;

        float h = 0.0f, s = 0.0f, v = max_val;

        if (delta > 0.00001f) {
            s = delta / max_val;

            if (r >= max_val)
                h = (g - b) / delta;
            else if (g >= max_val)
                h = 2.0f + (b - r) / delta;
            else
                h = 4.0f + (r - g) / delta;

            h *= 60.0f;
            if (h < 0.0f) h += 360.0f;
        }

        int hsv_index = (y * width + x) * 3;
        hsv[hsv_index + 0] = static_cast<unsigned char>(h / 2);       // H value (0-180)
        hsv[hsv_index + 1] = static_cast<unsigned char>(s * 255);     // S value (0-255)
        hsv[hsv_index + 2] = static_cast<unsigned char>(v * 255);     // V value (0-255)
    }
}

__global__ void filter_red_pixels(const unsigned char* hsv, unsigned char* red_mask, int width, int height)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if (x < width && y < height) {
        int index = (y * width + x) * 3;
        unsigned char h = hsv[index + 0];
        unsigned char s = hsv[index + 1];
        unsigned char v = hsv[index + 2];

        if (((h <= 10 || h >= 160) && s >= 100 && v >= 100)) {
            red_mask[y * width + x] = 255;
        } else {
            red_mask[y * width + x] = 0;
        }
    }
}

// Constructor
Img_proc::Img_proc()
    : SPIN_RATE(100),
      img_proc_line_det_(false),
      gradient_(0)
{
    // RealSense 파이프라인 초기화
    cfg.enable_stream(RS2_STREAM_COLOR, realsense_width, realsense_height, RS2_FORMAT_BGR8, realsense_color_fps);
    cfg.enable_stream(RS2_STREAM_DEPTH, realsense_width, realsense_height, RS2_FORMAT_Z16, realsense_depth_fps);

    cudaStreamCreate(&stream);
}

// 소멸자
Img_proc::~Img_proc()
{
    // CUDA 스트림 파괴
    cudaStreamDestroy(stream);
}

void Img_proc::on_trackbar(int, void *)
{
    // Function body if required.
}

void Img_proc::create_color_range_trackbar(const std::string &window_name)
{
    cv::createTrackbar("Hue Lower", window_name, &lowerH, 179, on_trackbar);
    cv::createTrackbar("Hue Upper", window_name, &upperH, 179, on_trackbar);
    cv::createTrackbar("Saturation Lower", window_name, &lowerS, 255, on_trackbar);
    cv::createTrackbar("Saturation Upper", window_name, &upperS, 255, on_trackbar);
    cv::createTrackbar("Value Lower", window_name, &lowerV, 255, on_trackbar);
    cv::createTrackbar("Value Upper", window_name, &upperV, 255, on_trackbar);
}

std::tuple<cv::Mat, cv::Mat> Img_proc::ROI_Line(const cv::Mat &input_frame, const cv::Mat &ori_frame) // 관심영역 설정하는 코드 -- 육상
{
    cv::Mat draw_frame = ori_frame.clone();
    cv::Mat mask = cv::Mat::zeros(input_frame.size(), CV_8UC1);

    cv::rectangle(mask, cv::Point(250, 480), cv::Point(698, 0), cv::Scalar(255), -1);

    cv::Mat circleMask = cv::Mat::zeros(input_frame.size(), CV_8UC1);

    cv::circle(circleMask, cv::Point(490, 500), 500, cv::Scalar(255), -1);
    cv::bitwise_and(mask, circleMask, mask);

    int circle_center_x = 490;
    int circle_center_y = 600;
    int radius = 500;

    for (int x = 250; x <= 698; x++)
    {

        double y_positive = circle_center_y + std::sqrt(radius * radius - (x - circle_center_x) * (x - circle_center_x));
        double y_negative = circle_center_y - std::sqrt(radius * radius - (x - circle_center_x) * (x - circle_center_x));

    }

    cv::Mat roi;
    ori_frame.copyTo(roi, mask);

    return {roi, draw_frame};
}

cv::Mat Img_proc::ROI_Rectangle(const cv::Mat &input_frame, int y_start, int y_end, int x_start, int x_end)
{
    cv::Mat result = input_frame.clone();
    
    // 좌측 영역을 하얗게 채움
    cv::rectangle(result, cv::Point(0, 0), cv::Point(x_start, result.rows), cv::Scalar(255, 255, 255), -1);
    
    // 우측 영역을 하얗게 채움
    cv::rectangle(result, cv::Point(x_end, 0), cv::Point(result.cols, result.rows), cv::Scalar(255, 255, 255), -1);
    
    // ROI 영역의 경계를 빨간색으로 표시 (선택사항)
    cv::rectangle(result, cv::Point(x_start, y_start), cv::Point(x_end, y_end), cv::Scalar(0, 0, 255), 2);

    return result;
}

std::tuple<cv::Mat, cv::Mat> Img_proc::extract_color(cv::Mat &input_frame, const cv::Scalar &lower_bound, const cv::Scalar &upper_bound)
{
    cv::Mat hsv;
    cv::cvtColor(input_frame, hsv, cv::COLOR_BGR2HSV);

    cv::Mat mask;
    cv::inRange(hsv, lower_bound, upper_bound, mask);

    cv::Mat color_extracted;
    cv::bitwise_and(input_frame, input_frame, color_extracted, mask);

    return {color_extracted, input_frame}; 
}

std::tuple<cv::Mat, float, int, double>
Img_proc::detect_Line_areas(const cv::Mat &input_frame, const cv::Mat &origin_frame, const cv::Scalar &contour_color, int threshold_value)
{
    // Clone the origin frame to draw results
    cv::Mat ori_frame = origin_frame.clone();

    // Upload the input frame to GPU
    cv::cuda::GpuMat d_frame(input_frame);

    // Convert to grayscale on GPU
    cv::cuda::GpuMat d_gray;
    cv::cuda::cvtColor(d_frame, d_gray, cv::COLOR_BGR2GRAY);

    // Apply threshold on GPU
    cv::cuda::GpuMat d_binary;
    cv::cuda::threshold(d_gray, d_binary, threshold_value, max_value, cv::THRESH_BINARY);  // 추가된 인자


    // Download the binary image from GPU to CPU for further processing
    cv::Mat binary;
    d_binary.download(binary);

    // Find contours on the CPU (currently OpenCV does not support contour finding on GPU)
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<cv::Point> top_contour;

    bool foundLargeContour = false;
    double topmost_y = std::numeric_limits<double>::max();
    bool has_white_now = false;

    float SM_angle = 0;
    float Rnd_angle = 0;
    float Line_Angle = 0;
    int line_area = 0;

    cv::Point top_center, bottom_center, left_center, right_center;

    bool &has_prev = has_white_prev;
    cv::Point &center_now = center_now_white;

    for (const auto &contour : contours)
    {
        line_area = cv::contourArea(contour);

        if (line_area < 2500 && line_area > LINE_AREA)
        {
            cv::Moments m = cv::moments(contour);
            foundLargeContour = true;
            line_condition_count = 0;
            if (m.m00 == 0)
                continue;

            cv::Point center(m.m10 / m.m00, m.m01 / m.m00);
            if (center.y < topmost_y)
            {
                topmost_y = center.y;
                top_contour = contour;
                center_now = center;
            }
            has_white_now = true;
        }
        else if (line_area < LINE_AREA)
        {
            line_condition_count++;
            if (line_condition_count >= 15)
            {
                foundLargeContour = false;
                has_white_now = false;
            }
        }
    }

    if (!top_contour.empty())
    {
        top_contour_area = cv::contourArea(top_contour);
        cv::line(ori_frame, center_now, cv::Point(490, 480), contour_color, 2);

        float deltaY = center_now.y - 480;   
        float deltaX = center_now.x - 490;    

        float radians = atan2(deltaY, deltaX);

        float adjustedAngle = radians * (180.0 / CV_PI);

        Rnd_angle = -90 - adjustedAngle;
    }

    if (has_prev && !has_white_now && center_now.x < 490)
    {
        int8_t tmp_delta_x = 1;
        delta_x_ = tmp_delta_x;
        tmp_delta_x = 0;
        std::cout << "Line area disappeared to the left\n";
    }
    else if (has_prev && !has_white_now && center_now.x > 490)
    {
        int8_t tmp_delta_x = -1;
        delta_x_ = tmp_delta_x;
        tmp_delta_x = 0;
        std::cout << "Line area disappeared to the right\n";
    }

    has_prev = has_white_now;

    if (!top_contour.empty())
    {
        std::vector<cv::Point> approx;
        double epsilon = 0.02 * cv::arcLength(top_contour, true);
        cv::approxPolyDP(top_contour, approx, epsilon, true);

        int numVertices = approx.size(); // 근사화된 컨투어의 꼭지점 수를 얻음

        cv::RotatedRect min_area_rect = cv::minAreaRect(top_contour);

        float width = min_area_rect.size.width;
        float height = min_area_rect.size.height;

        float long_len = (width > height) ? width : height;
        float short_len = (width > height) ? height : width;

        cv::Point2f vertices[4];
        min_area_rect.points(vertices);


                    for (int i = 0; i < 4; ++i)
            cv::line(ori_frame, vertices[i], vertices[(i + 1) % 4], contour_color, 3);

        // Line angle
        if (  short_len * 1.2 < long_len && short_len * 5 > long_len && numVertices > 3 && numVertices < 5)
        {
            if (min_area_rect.size.width < min_area_rect.size.height)
            {
                SM_angle = -min_area_rect.angle;
            }
            else
            {
                SM_angle = -min_area_rect.angle + 90;
            }



            // cv::imshow("white binary", binary);
            // cv::moveWindow("white binary", 700, 540);
        }

        if (Rnd_angle > 0)
        {
            Line_Angle = (SM_angle + Rnd_angle) * 0.5;
        }
        else if (Rnd_angle <= 0)
        {
            Line_Angle = (SM_angle + (3 * Rnd_angle));
        }

        //--------------------------------------------------------------------- frame interface -------------------------------------------------------------------------

        if (has_white_now)
        {
            cv::putText(ori_frame, "MODE : " + Str_LINE_MODE, cv::Point(490 + 50, 25), cv::FONT_HERSHEY_SIMPLEX, 0.7, contour_color, 2);

        }
        if (!has_white_now)
        {
            cv::putText(ori_frame, "MODE : " + Str_NO_LINE_MODE, cv::Point(490 + 50, 25), cv::FONT_HERSHEY_SIMPLEX, 0.7, contour_color, 2);
        }

        cv::putText(ori_frame, "Rnd_Angle : " + std::to_string(Rnd_angle), cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.6, contour_color, 2);
        cv::putText(ori_frame, "SM_angle : " + std::to_string(SM_angle), cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 0.6, contour_color, 2);
        cv::putText(ori_frame, "Line angle : " + std::to_string(SM_angle + Rnd_angle), cv::Point(10, 75), cv::FONT_HERSHEY_SIMPLEX, 0.6, contour_color, 2);
        cv::putText(ori_frame, "Vertice : " + std::to_string(numVertices), cv::Point(10, 300), cv::FONT_HERSHEY_SIMPLEX, 0.6, contour_color, 2);
        cv::putText(ori_frame, "Area : " + std::to_string(top_contour_area), cv::Point(10, 455), cv::FONT_HERSHEY_SIMPLEX, 0.7, contour_color, 2);

        topmost_point = *std::min_element(top_contour.begin(), top_contour.end(),
                                          [](const cv::Point &a, const cv::Point &b)
                                          {
                                              return a.y < b.y;
                                          });

        bottommost_point = *std::max_element(top_contour.begin(), top_contour.end(),
                                             [](const cv::Point &a, const cv::Point &b)
                                             {
                                                 return a.y < b.y;
                                             });
    }

    return std::make_tuple(ori_frame, Line_Angle, line_area, delta_x_);
}

std::tuple<cv::Mat, int, cv::Point, cv::Point, float, cv::Point, std::vector<cv::Point>, int>
Img_proc::detect_Huddle_areas(const cv::Mat &input_frame, const cv::Mat &origin_frame, const cv::Scalar &contour_color, int threshold_value)
{
    // Clone the origin frame to draw results
    cv::Mat ori_frame = origin_frame.clone();

    // Upload the input frame to GPU
    cv::cuda::GpuMat d_frame(input_frame);

    // Convert to grayscale on GPU
    cv::cuda::GpuMat d_gray;
    cv::cuda::cvtColor(d_frame, d_gray, cv::COLOR_BGR2GRAY);

    // Apply threshold on GPU
    cv::cuda::GpuMat d_binary;
    cv::cuda::threshold(d_gray, d_binary, threshold_value, max_value, cv::THRESH_BINARY);  // 추가된 인자


    // Download the binary image from GPU to CPU for further processing
    cv::Mat binary;
    d_binary.download(binary);

    // Find contours on the CPU (currently OpenCV does not support contour finding on GPU)
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<cv::Point> top_contour;

    bool foundLargeContour = false;
    double topmost_y = std::numeric_limits<double>::max();
    double distance_huddle = 0;
    bool has_yellow_now = false;

    float SM_angle = 0;
    float Rnd_angle = 0;
    float huddle_angle = 0;
    float Line_Angle = 0;
    float corner_angle = 0;
    int huddle_area = 0;

    cv::Point huddle_center = cv::Point(0, 0);

    cv::Point top_center, bottom_center, left_center, right_center;
    cv::Point &center_now = center_now_yellow;

    for (const auto &contour : contours)
    {
        /// Huddle
        huddle_area = cv::contourArea(contour);

        if (huddle_area > HUDDLE_AREA)
        {
            cv::Moments m = cv::moments(contour);
            foundLargeContour = true;
            line_condition_count = 0;
            if (m.m00 == 0)
                continue;

            cv::Point center(m.m10 / m.m00, m.m01 / m.m00);
            if (center.y < topmost_y)
            {
                topmost_y = center.y;
                top_contour = contour;
                center_now = center;
                distance_huddle = 480 - topmost_y;
            }
            has_yellow_now = true;
        }
    }

    if (!top_contour.empty())
    {
        top_contour_area = cv::contourArea(top_contour);
        cv::line(ori_frame, center_now, cv::Point(424, 480), contour_color, 2);

        float deltaY = center_now.y - 480;
        float deltaX = center_now.x - 424;

        float radians = atan2(deltaY, deltaX);

        float adjustedAngle = radians * (180.0 / CV_PI);

        Rnd_angle = -90 - adjustedAngle;
    }

    if (!top_contour.empty())
    {
        std::vector<cv::Point> approx;
        double epsilon = 0.02 * cv::arcLength(top_contour, true);
        cv::approxPolyDP(top_contour, approx, epsilon, true);

        int numVertices = approx.size();

        cv::RotatedRect min_area_rect = cv::minAreaRect(top_contour);

        float width = min_area_rect.size.width;
        float height = min_area_rect.size.height;

        float long_len = (width > height) ? width : height;
        float short_len = (width > height) ? height : width;

        cv::Point2f vertices[4];
        min_area_rect.points(vertices);

        for (int i = 0; i < 4; ++i)
            cv::line(ori_frame, vertices[i], vertices[(i + 1) % 4], contour_color, 3);


        if (short_len * 1.5 < long_len)
        {
            if (min_area_rect.size.width < min_area_rect.size.height)
            {
                huddle_angle = -min_area_rect.angle + 90;
            }
            else
            {
                huddle_angle = -min_area_rect.angle;
            }
            huddle_center = min_area_rect.center;
            cv::circle(ori_frame, huddle_center, 2, contour_color, -1, 8);
        }

        if (Rnd_angle > 0)
        {
            Line_Angle = Rnd_angle * 0.5;
        }
        else if (Rnd_angle <= 0)
        {
            Line_Angle = (3 * Rnd_angle);
        }

        //--------------------------------------------------------------------- frame interface -------------------------------------------------------------------------

        cv::putText(ori_frame, "Rnd_Angle : " + std::to_string(Rnd_angle), cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.6, contour_color, 2);
        cv::putText(ori_frame, "Hurdle Angle : " + std::to_string(huddle_angle), cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 0.6, contour_color, 2);
        cv::putText(ori_frame, "Line angle : " + std::to_string(Line_Angle), cv::Point(10, 75), cv::FONT_HERSHEY_SIMPLEX, 0.6, contour_color, 2);
        cv::putText(ori_frame, "Vertice : " + std::to_string(numVertices), cv::Point(10, 300), cv::FONT_HERSHEY_SIMPLEX, 0.6, contour_color, 2);
        cv::putText(ori_frame, "Area : " + std::to_string(top_contour_area), cv::Point(10, 455), cv::FONT_HERSHEY_SIMPLEX, 0.7, contour_color, 2);

        topmost_point = *std::min_element(top_contour.begin(), top_contour.end(),
                                          [](const cv::Point &a, const cv::Point &b)
                                          {
                                              return a.y < b.y;
                                          });

        bottommost_point = *std::max_element(top_contour.begin(), top_contour.end(),
                                             [](const cv::Point &a, const cv::Point &b)
                                             {
                                                 return a.y < b.y;
                                             });
    }

    return std::make_tuple(ori_frame, Line_Angle, topmost_point, bottommost_point, huddle_angle, huddle_center, top_contour, huddle_area);
}

cv::Point Img_proc::detect_green_point(const cv::Mat &input_frame, int threshold_value)
{
    // Upload the input frame to GPU
    cv::cuda::GpuMat d_frame(input_frame);

    // Convert to grayscale on GPU
    cv::cuda::GpuMat d_gray;
    cv::cuda::cvtColor(d_frame, d_gray, cv::COLOR_BGR2GRAY);

    // Apply threshold on GPU
    cv::cuda::GpuMat d_binary;
    cv::cuda::threshold(d_gray, d_binary, threshold_value, max_value, cv::THRESH_BINARY);  // 추가된 인자


    // Download the binary image from GPU to CPU for further processing
    cv::Mat binary;
    d_binary.download(binary);

    // Find contours on the CPU (currently OpenCV does not support contour finding on GPU)
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // Initialize variables
    std::vector<cv::Point> top_contour;
    double topmost_y = std::numeric_limits<double>::max();

    cv::Point topmost_point;

    for (const auto &contour : contours)
    {
            cv::Moments m = cv::moments(contour);

            if (m.m00 == 0)
                continue;

            cv::Point center(m.m10 / m.m00, m.m01 / m.m00);

            if (center.y < topmost_y)
            {
                topmost_y = center.y;
                top_contour = contour;
            }        
    }

    if (!top_contour.empty())
    {
        topmost_point = *std::min_element(top_contour.begin(), top_contour.end(),
                                          [](const cv::Point &a, const cv::Point &b) { return a.y < b.y; });
    }

    return topmost_point;
}

std::tuple<cv::Mat, cv::Point3f> Img_proc::Ball_Detect(cv::Mat color, cv::Mat depth_dist, int threshold_value) {
    // 깊이 값이 1000mm 이하인 영역만 마스크로 적용
    cv::Mat mask = depth_dist < 1000;

    // 원본 color 이미지를 mask를 사용하여 필터링
    cv::Mat output = color.clone();  // color.copyTo(output, mask);를 대체
    output.setTo(cv::Scalar(255, 255, 255), ~mask);

    // 화면 중앙 좌표 계산
    int center_x = output.cols / 2;
    int center_y = output.rows / 2;

    cv::Point3f ball_center(0, 0, 0); // 결과값으로 반환될 공의 좌표

    // HSV 색상 공간 변환
    cv::Mat hsv;
    cv::cvtColor(output, hsv, cv::COLOR_BGR2HSV);

    // 농구공 색상 범위 정의 (주황색)
    const cv::Scalar lower_orange(8, 60, 0);
    const cv::Scalar upper_orange(60, 255, 255);
    // 주황색 마스크 생성
    cv::Mat color_mask;
    cv::inRange(hsv, lower_orange, upper_orange, color_mask);

    // 노이즈 제거
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::morphologyEx(color_mask, color_mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(color_mask, color_mask, cv::MORPH_CLOSE, kernel);

    // 윤곽선 검출
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(color_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // 최소 원형도 및 면적 조건
    const double min_circularity = 0.7;
    const double min_area = 1000;
    const double max_area = 50000;

    double max_valid_area = 0;
    int max_valid_index = -1;

    for (int i = 0; i < contours.size(); i++) {
        double area = cv::contourArea(contours[i]);
        if (area < min_area || area > max_area) continue;

        cv::Point2f center;
        float radius;
        cv::minEnclosingCircle(contours[i], center, radius);

        // 원형도 계산
        double perimeter = cv::arcLength(contours[i], true);
        double circularity = 4 * CV_PI * area / (perimeter * perimeter);

        if (circularity > min_circularity && area > max_valid_area) {
            max_valid_area = area;
            max_valid_index = i;
        }
    }

    if (max_valid_index >= 0) {
        cv::Point2f center;
        float radius;
        cv::minEnclosingCircle(contours[max_valid_index], center, radius);

        // 원형 객체 그리기
        cv::circle(output, cv::Point(center.x, center.y), radius, cv::Scalar(0, 255, 0), 2);

        // 좌표 범위 확인 후 깊이 값 가져오기
        if (center.x >= 0 && center.x < depth_dist.cols && center.y >= 0 && center.y < depth_dist.rows) {
            float cz = depth_dist.at<uint16_t>(cv::Point(center.x, center.y)) * 0.001f;

            // 화면 중앙 기준 좌표 변환
            float cx = center.x - center_x;
            float cy = center.y - center_y;

            ball_center = cv::Point3f(cx, cy, cz);

            // 검출된 공의 중심 표시
            cv::circle(output, cv::Point(center.x, center.y), 5, cv::Scalar(0, 0, 255), -1);

            // 객체 정보 표시
            std::string info = "Ball: (" + std::to_string(int(ball_center.x)) + ", "
                + std::to_string(int(ball_center.y)) + ", "
                + std::to_string(ball_center.z) + "m)";
            cv::putText(output, info, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
        }
    }

    // ROI 영역 표시 (선택사항)
    cv::Rect roi(0, 0, output.cols, output.rows);
    cv::rectangle(output, roi, cv::Scalar(0, 255, 0), 2);

    return std::make_tuple(output, ball_center);
}

std::tuple<cv::Mat, cv::Point3f> Img_proc::Hoop_Detect(cv::Mat color, cv::Mat depth_dist, int threshold_value)
{

    int width = color.cols;
    int height = color.rows;

    unsigned char *d_color, *d_hsv, *d_red_mask;

    size_t color_size = width * height * 3 * sizeof(unsigned char);
    size_t mask_size = width * height * sizeof(unsigned char);

    // GPU 메모리 할당
    cudaMalloc(&d_color, color_size);
    cudaMalloc(&d_hsv, color_size);
    cudaMalloc(&d_red_mask, mask_size);

    // 호스트에서 디바이스로 데이터 복사
    cudaMemcpyAsync(d_color, color.data, color_size, cudaMemcpyHostToDevice, stream);

    // CUDA 커널 실행 설정
    dim3 blockSize(16, 16);
    dim3 gridSize((width + blockSize.x - 1) / blockSize.x,
                  (height + blockSize.y - 1) / blockSize.y);

    // CUDA 커널 호출
    color_to_hsv<<<gridSize, blockSize, 0, stream>>>(d_color, d_hsv, width, height);
    
    // 필수적인 동기화 추가
    cudaStreamSynchronize(stream);

    filter_red_pixels<<<gridSize, blockSize, 0, stream>>>(d_hsv, d_red_mask, width, height);

    // 또다시 동기화 추가
    cudaStreamSynchronize(stream);

    // 결과를 호스트로 복사
    std::vector<unsigned char> red_mask_host(mask_size);
    cudaMemcpyAsync(red_mask_host.data(), d_red_mask, mask_size, cudaMemcpyDeviceToHost, stream);

    // CUDA 스트림 동기화
    cudaStreamSynchronize(stream);

    // OpenCV Mat으로 변환
    cv::Mat red_mask_mat(height, width, CV_8UC1, red_mask_host.data());

    // 윤곽선 검출
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(red_mask_mat, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // 변수 초기화
    double max_area = 0;
    cv::RotatedRect largest_rect;
    bool detection_made = false;
    cv::Point3f hoop_center(0, 0, 0);

    for (const auto& contour : contours) {
        cv::RotatedRect min_area_rect = cv::minAreaRect(contour);
        float width_rect = min_area_rect.size.width;
        float height_rect = min_area_rect.size.height;

        double area = width_rect * height_rect;
        if (area > 5000 && 45000 > area) {
            float long_len = std::max(width_rect, height_rect);
            float short_len = std::min(width_rect, height_rect);
            float aspect_ratio = long_len / short_len;

            if (aspect_ratio >= 1.0 && aspect_ratio <= 2.5) {
                cv::Rect bbox = min_area_rect.boundingRect();
                bbox &= cv::Rect(0, 0, color.cols, color.rows);

                cv::Mat cropped = color(bbox);
                cv::Mat hsv_cropped;
                cv::cvtColor(cropped, hsv_cropped, cv::COLOR_BGR2HSV);

                cv::Mat mask1, mask2, red_mask;
                cv::inRange(hsv_cropped, cv::Scalar(159, 145, 0), cv::Scalar(180, 255, 255), mask1);
                //cv::inRange(hsv_cropped, cv::Scalar(160, 100, 100), cv::Scalar(180, 255, 255), mask2);
                red_mask = mask1;

                double red_pixels = cv::countNonZero(red_mask);
                double total_pixels = bbox.area();
                double red_ratio = red_pixels / total_pixels;

                if (0.1 < red_ratio && red_ratio < 0.4) {
                    if (area > max_area) {
                        max_area = area;
                        largest_rect = min_area_rect;
                        detection_made = true;
                    }
                }
            }
        }
    }

    cv::Mat output = color.clone();

    if (detection_made) {
        cv::Point2f vertices[4];
        largest_rect.points(vertices);
        for (int i = 0; i < 4; ++i)
            cv::line(output, vertices[i], vertices[(i + 1) % 4], cv::Scalar(0, 255, 0), 2);

        cv::Point2f center = largest_rect.center;

        // 깊이 값 추출
        float depth_value = depth_dist.at<uint16_t>(cv::Point(center.x, center.y)) * 0.001f;

        // 화면 중앙 기준 좌표 변환
        int center_x = output.cols / 2;
        int center_y = output.rows / 2;

        float cx = center.x - center_x;
        float cy = center_y - center.y;

        hoop_center = cv::Point3f(cx, cy, depth_value);

        // 중심점 표시
        cv::circle(output, center, 5, cv::Scalar(0, 0, 255), -1);

        // 정보 출력
        std::string info = "Hoop: (" + std::to_string(int(hoop_center.x)) + ", "
            + std::to_string(int(hoop_center.y)) + ", "
            + std::to_string(hoop_center.z) + "m)";
        cv::putText(output, info, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
    }

    // 메모리 해제
    cudaFree(d_color);
    cudaFree(d_hsv);
    cudaFree(d_red_mask);
    // cudaStreamDestroy(stream);

    return std::make_tuple(output, hoop_center);
}

void Img_proc::realsense_thread() {

    try {
        pipe.start(cfg);
    } catch (const rs2::error& e) {
        std::cerr << "Failed to open the RealSense camera: " << e.what() << std::endl;
        return;
    }

    const auto window_name = "RealSense Ball Frame";
    const auto window_name_hoop = "RealSense Hoop Frame";
    cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(window_name_hoop, cv::WINDOW_AUTOSIZE);
    const auto window_name_hsv = "HSV Binary Image";
    cv::namedWindow(window_name_hsv, cv::WINDOW_AUTOSIZE);
    create_color_range_trackbar(window_name_hsv);

    rs2::align align_to(RS2_STREAM_COLOR);

    int White_count = 0;
    int Yellow_count = 0;
    int noline_count = 0;
    int ball_detected_count = 0;
    int hoop_detected_count = 0;
    const int detection_threshold = 3;

    cv::Mat colorMat, depthMat, Line_frame;

    int frameCounter = 0;
    // img_proc_mode = Interest_Object::Ball;

    while (ros::ok() && cv::waitKey(1) < 0) {
        frameCounter++;

        // Retrieve and align frames
        rs2::frameset data = pipe.wait_for_frames();
        data = align_to.process(data);

        rs2::frame color_frame = data.get_color_frame();
        rs2::depth_frame depth_frame = data.get_depth_frame();

        if (!color_frame || !depth_frame) {
            continue;
        }

        // Create color and depth matrices
        colorMat = cv::Mat(cv::Size(realsense_width, realsense_height), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
        depthMat = cv::Mat(cv::Size(realsense_width, realsense_height), CV_16UC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);

        // cv::Rect roi(100, 0, realsense_width - 100, realsense_height);
        // roi_line_frame = colorMat(roi);
        // roi_depth_mat = depthMat(roi);

        bool object_detected = false;
        // Update detection mode based on previous detections
        if (this->Get_img_proc_ball_done_()) {
            img_proc_mode = Interest_Object::Hoop;
            this->Set_img_proc_ball_done_(false);
            ball_detected_count = 0;
        } else if (this->Get_img_proc_hoop_done_()) {
            img_proc_mode = Interest_Object::Huddle;
            this->Set_img_proc_hoop_done_(false);
            hoop_detected_count = 0;
        } else if (this->Get_img_proc_huddle_done_()) {
            img_proc_mode = Interest_Object::Ball;
            this->Set_img_proc_huddle_done_(false);
        }

        // Object detection based on mode
        switch (img_proc_mode) {
            case Interest_Object::Ball:
                if (frameCounter % 3 == 0 || Get_UD_NeckAngle_() == PICK_NECK) {
                    auto Ball = Ball_Detect(colorMat, depthMat, threshold_value_black);
                    cv::Point3f Ball_center = std::get<1>(Ball);

                    if ((Ball_center.x != 0 || Ball_center.y != 0 || Ball_center.z != 0)) {
                        ball_detected_count++;
                        if (ball_detected_count >= detection_threshold) {
                            this->Set_img_proc_ball_det_(true);
                            this->Set_ball_x(Ball_center.x);
                            this->Set_ball_y(Ball_center.y);
                            this->Set_ball_z(Ball_center.z);
                            object_detected = true;

                            
                            cv::imshow(window_name, std::get<0>(Ball));
                            cv::moveWindow(window_name, 900, 700);
                        }
                    } else {
                        ball_detected_count = 0;
                        this->Set_img_proc_ball_det_(false);
                    }
                }
                break;

            case Interest_Object::Hoop: 
                if (Get_UD_NeckAngle_() == HOOP_NECK) {
                // if (0) {
                    auto Hoop = Hoop_Detect(colorMat, depthMat, threshold_value_black);
                    cv::Point3f Hoop_center = std::get<1>(Hoop);

                    if ((Hoop_center.x != 0 || Hoop_center.y != 0 || Hoop_center.z != 0)) {
                        hoop_detected_count++;
                        if (hoop_detected_count >= detection_threshold) {
                            this->Set_img_proc_hoop_det_(true);
                            this->Set_hoop_x(Hoop_center.x);
                            this->Set_hoop_y(Hoop_center.y);
                            this->Set_hoop_z(Hoop_center.z);
                            object_detected = true;


                            cv::imshow(window_name_hoop, std::get<0>(Hoop));
                            cv::moveWindow(window_name_hoop, 900, 700);
                        }
                    } else {
                        hoop_detected_count = 0;
                        this->Set_img_proc_hoop_det_(false);
                    }
                    Set_hoopcounter_det_flg_(hoop_detected_count);
                }
                break;

            case Interest_Object::Huddle:
                if (frameCounter % 3 == 0 || Get_UD_NeckAngle_() == HUDDLE_NECK) {
                // if (0) {
                    cv::Mat Roi_huddle = ROI_Rectangle(colorMat, 0, realsense_height, realsense_width / 2 - 100, realsense_width / 2 + 100);
            
                    // 불필요한 복사 제거
                    auto hsv_frame_yellow = extract_color(Roi_huddle, lower_bound_yellow, upper_bound_yellow);
                    auto hsv_frame_green = extract_color(colorMat, lower_bound_blue, upper_bound_blue);
            
                    // Detection
                    auto thresh_frame_yellow = detect_Huddle_areas(std::get<0>(hsv_frame_yellow), colorMat, yellow_color, threshold_value_yellow);
                    auto thresh_frame_green = detect_green_point(std::get<0>(hsv_frame_green), threshold_value_green);
                        
                    int YellowColorDetected = std::get<7>(thresh_frame_yellow);

                    if (YellowColorDetected > HUDDLE_AREA) {
                        Yellow_count++;
            
                        if (Yellow_count > 0) {
                            
                            noline_count = 0;
                            this->Set_img_proc_huddle_det_2d(true);
        
                            double gradient = std::get<1>(thresh_frame_yellow);
                            this->Set_gradient(gradient);
            
                            double huddle_angle_ = std::get<4>(thresh_frame_yellow);
                            Set_huddle_angle(huddle_angle_);

            
                            cv::Point huddle_center = std::get<5>(thresh_frame_yellow);
                            cv::circle(std::get<0>(thresh_frame_yellow), huddle_center, 2, CV_RGB(0, 255, 255), -1);
            
                            cv::Point foot_top_point = thresh_frame_green; 
                            cv::Point huddle_bottom_point = std::get<3>(thresh_frame_yellow);
                            cv::circle(std::get<0>(thresh_frame_yellow), huddle_bottom_point, 5, cv::Scalar(0, 0, 255), -1);
            
                            int minDistance = std::numeric_limits<int>::max();
                            cv::Point2f closestPoint;
            
                            for (const auto &point : std::get<6>(thresh_frame_yellow)) {
                                int distance = static_cast<int>(cv::norm(foot_top_point - point));
                                if (distance < minDistance) {
                                    minDistance = distance;
                                    closestPoint = point;
                                }
                            }
            
                            int _foot_huddle_distance = std::abs(foot_top_point.y - huddle_bottom_point.y);
                            Set_foot_huddle_distance(_foot_huddle_distance);
            
                            if (_foot_huddle_distance < HUDDLE_Y_MARGIN)
                            {
                                Set_contain_huddle_to_foot(true);
                            }
                            else
                            {
                                Set_contain_huddle_to_foot(false);
                            }

                            cv::line(std::get<0>(thresh_frame_yellow), foot_top_point, closestPoint, cv::Scalar(0, 0, 255), 3);
                            cv::putText(std::get<0>(thresh_frame_yellow), "Distance: " + std::to_string(minDistance), 
                            cv::Point(10, 175), cv::FONT_HERSHEY_SIMPLEX, 0.7, yellow_color, 2);
                            cv::putText(std::get<0>(thresh_frame_yellow), "Angle: " + std::to_string(huddle_angle_) + "deg", 
                            cv::Point(10, 200), cv::FONT_HERSHEY_SIMPLEX, 0.7, yellow_color, 2);
            
                            cv::imshow("hsv Frame_yellow", std::get<0>(thresh_frame_yellow));
                            cv::moveWindow("hsv Frame_yellow", 700, 0);
                        }
                    } else {
                        this->Set_img_proc_huddle_det_2d(false);
                        Yellow_count = 0; 
                
                    }
                }
                break;
        }

        // Line detection if no object detected
        if (!object_detected && (Get_UD_NeckAngle_() == 84)) {
        // if (1) {
            Line_frame = colorMat.clone();
            auto Roi_Line = ROI_Line(Line_frame, colorMat);
            auto hsv_frame_white = extract_color(std::get<0>(Roi_Line), lower_bound_white, upper_bound_white);
            auto thresh_frame_white = detect_Line_areas(std::get<0>(hsv_frame_white), colorMat, white_color, threshold_value_white);
            int WhiteColorDetected = std::get<2>(thresh_frame_white);

            auto hsv_frame_hsv = extract_color(std::get<0>(Roi_Line), {lowerH, lowerS, lowerV}, {upperH, upperS, upperV});
            cv::imshow(window_name_hsv, std::get<0>(hsv_frame_hsv));

            if (WhiteColorDetected > LINE_AREA) {
                White_count++;
                if (White_count > 30) {
                    noline_count = 0;
                    this->Set_img_proc_line_det(true);
                    this->Set_img_proc_no_line_det(false);
                    this->Set_gradient(std::get<1>(thresh_frame_white));
                    Yellow_count = 0;

                
                    cv::imshow("hsv Frame_white", std::get<0>(thresh_frame_white));
                    cv::moveWindow("hsv Frame_white", 0, 0);
                }
            }
            else if(WhiteColorDetected < LINE_AREA)
            {
                noline_count++;
                if(noline_count > 15)
                {
                    double gradient = std::get<1>(thresh_frame_white);
                    double tmp_delta_x = std::get<3>(thresh_frame_white);
                    
                    this->Set_img_proc_no_line_det(true);
                    this->Set_img_proc_line_det(false);
                    if (this->Get_img_proc_line_det() == false)
                    {
                        this->Set_gradient(gradient);
                        this->Set_delta_x(tmp_delta_x);
                    }
                }
            }
            
        }
    }
}
    
// ********************************************** GETTERS ************************************************** //

bool Img_proc::Get_img_proc_line_det() const
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_line_det_);
    return img_proc_line_det_;
}

bool Img_proc::Get_img_proc_no_line_det() const
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_no_line_det_);
    return img_proc_no_line_det_;
}

bool Img_proc::Get_img_proc_huddle_det_2d() const
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_huddle_det_2d);    
    return img_proc_huddle_det_2d_;
}

bool Img_proc::Get_img_proc_stop_det() const
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_stop_det_);
    return img_proc_stop_det_;
}

double Img_proc::Get_gradient() const
{
    std::lock_guard<std::mutex> lock(mtx_gradient);
    return gradient_;
}

double Img_proc::Get_delta_x() const
{
    std::lock_guard<std::mutex> lock(mtx_delta_x);
    return delta_x_;
}


double Img_proc::Get_huddle_distance() const
{
    std::lock_guard<std::mutex> lock(mtx_huddle_distance_);
    return huddle_distance_;
}

bool Img_proc::Get_contain_huddle_to_foot() const
{
    std::lock_guard<std::mutex> lock(mtx_contain_huddle_to_foot);
    return contain_huddle_to_foot_;
}

int Img_proc::Get_foot_huddle_distance() const
{
    std::lock_guard<std::mutex> lock(mtx_foot_huddle_distance_);
    return foot_huddle_distance_;
}

double Img_proc::Get_huddle_angle() const
{
    std::lock_guard<std::mutex> lock(mtx_huddle_angle_);
    return huddle_angle_;
}

bool Img_proc::Get_img_proc_Far_Hoop_det() const
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_far_hoop_det_);
    return img_proc_far_hoop_det_;
}

bool Img_proc::Get_img_proc_Adjust_det() const
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_adjust_det_);
    return img_proc_adjust_det_;
}

bool Img_proc::Get_img_proc_Shoot_det() const
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_shoot_det_);
    return img_proc_shoot_det_;
}

bool Img_proc::Get_img_proc_No_Hoop_det() const
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_no_hoop_det_);
    return img_proc_no_hoop_det_;
}

double Img_proc::Get_distance() const
{
    std::lock_guard<std::mutex> lock(mtx_distance);
    return distance_;
}

int8_t Img_proc::Get_img_proc_Adjust_number() const
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_adjust_number_);
    return img_proc_adjust_number_;
}

//double Img_proc::Get_gradient() const
//{
//    std::lock_guard<std::mutex> lock(mtx_gradient);
//    return gradient_;
//}

double Img_proc::Get_adjust_angle() const
{
    std::lock_guard<std::mutex> lock(mtx_adjust_angle);
    return adjust_angle_;
}

int Img_proc::Get_contain_adjust_to_foot() const
{
    std::lock_guard<std::mutex> lock(mtx_contain_adjust_to_foot);
    return contain_adjust_to_foot_;
}

double Img_proc::Get_ball_x() const
{
    std::lock_guard<std::mutex> lock(mtx_ball_x);
    return ball_x_;
}

double Img_proc::Get_ball_y() const
{
    std::lock_guard<std::mutex> lock(mtx_ball_y);
    return ball_y_;
}

double Img_proc::Get_ball_z() const
{
    std::lock_guard<std::mutex> lock(mtx_ball_z);
    return ball_z_;
}

double Img_proc::Get_hoop_x() const
{
    std::lock_guard<std::mutex> lock(mtx_hoop_x);
    return hoop_x_;
}

double Img_proc::Get_hoop_y() const
{
    std::lock_guard<std::mutex> lock(mtx_hoop_y);
    return hoop_y_;
}

double Img_proc::Get_hoop_z() const
{
    std::lock_guard<std::mutex> lock(mtx_hoop_z);
    return hoop_z_;
}

bool Img_proc::Get_img_proc_ball_det_() const
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_ball_det_);
    return img_proc_ball_det_;
}
bool Img_proc::Get_img_proc_hoop_det_() const
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_hoop_det_);
    return img_proc_hoop_det_;
}
bool Img_proc::Get_img_proc_huddle_det_() const
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_huddle_det_);
    return img_proc_huddle_det_;
}

bool Img_proc::Get_img_proc_ball_done_() const
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_ball_done_);
    return img_proc_ball_done_;
}
bool Img_proc::Get_img_proc_hoop_done_() const
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_hoop_done_);
    return img_proc_hoop_done_;
}

bool Img_proc::Get_img_proc_huddle_done_() const
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_huddle_done_);
    return img_proc_huddle_done_;
}

int Img_proc::Get_hoopcounter_det_flg_() const
{
    std::lock_guard<std::mutex> lock(mtx_hoop_counter);
    return hoop_counter_;
}


int Img_proc::Get_UD_NeckAngle_() const
{
    std::lock_guard<std::mutex> lock(mtx_UD_NeckAngle);
    return UD_NeckAngle_;
}


// ********************************************** SETTERS ************************************************** //

void Img_proc::Set_img_proc_line_det(bool img_proc_line_det)
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_line_det_);
    this->img_proc_line_det_ = img_proc_line_det;
}

void Img_proc::Set_img_proc_no_line_det(bool img_proc_no_line_det)
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_no_line_det_);
    this->img_proc_no_line_det_ = img_proc_no_line_det;
}

void Img_proc::Set_img_proc_huddle_det_2d(bool img_proc_huddle_det_2d)
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_huddle_det_2d);
    this->img_proc_huddle_det_2d_ = img_proc_huddle_det_2d;
}

void Img_proc::Set_img_proc_stop_det(bool img_proc_stop_det)
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_stop_det_);
    this->img_proc_stop_det_ = img_proc_stop_det;
}

void Img_proc::Set_gradient(double gradient)
{
    std::lock_guard<std::mutex> lock(mtx_gradient);
    this->gradient_ = gradient;
}

void Img_proc::Set_delta_x(double delta_x)
{
    std::lock_guard<std::mutex> lock(mtx_delta_x);
    this->delta_x_ = delta_x;
}

void Img_proc::Set_huddle_distance(double huddle_distance)
{
    std::lock_guard<std::mutex> lock(mtx_huddle_distance_);
    this->huddle_distance_ = huddle_distance;
}

void Img_proc::Set_contain_huddle_to_foot(bool contain_huddle_to_foot)
{
    std::lock_guard<std::mutex> lock(mtx_contain_huddle_to_foot);
    this->contain_huddle_to_foot_ = contain_huddle_to_foot;
}

void Img_proc::Set_foot_huddle_distance(int foot_huddle_distance)
{
    std::lock_guard<std::mutex> lock(mtx_foot_huddle_distance_);
    this->foot_huddle_distance_ = foot_huddle_distance;
}

void Img_proc::Set_huddle_angle(double huddle_angle)
{
    std::lock_guard<std::mutex> lock(mtx_huddle_angle_);
    this->huddle_angle_ = huddle_angle;
}

void Img_proc::Set_img_proc_Far_Hoop_det(bool img_proc_far_hoop_det)
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_far_hoop_det_);
    this->img_proc_far_hoop_det_ = img_proc_far_hoop_det;
}

void Img_proc::Set_img_proc_Adjust_det(bool img_proc_adjust_det)
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_adjust_det_);
    this->img_proc_adjust_det_ = img_proc_adjust_det;
}

void Img_proc::Set_img_proc_Shoot_det(bool img_proc_shoot_det)
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_shoot_det_);
    this->img_proc_shoot_det_ = img_proc_shoot_det;
}

void Img_proc::Set_img_proc_No_Hoop_det(bool img_proc_no_hoop_det)
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_no_hoop_det_);
    this->img_proc_no_hoop_det_ = img_proc_no_hoop_det;
}

//void Img_proc::Set_img_proc_stop_det(bool img_proc_stop_det)
//{
//    std::lock_guard<std::mutex> lock(mtx_img_proc_stop_det_);
//    this->img_proc_stop_det_ = img_proc_stop_det;
//}

//void Img_proc::Set_delta_x(double delta_x)
//{
//    std::lock_guard<std::mutex> lock(mtx_delta_x);
//    this->delta_x_ = delta_x;
//}

void Img_proc::Set_distance(double distance)
{
    std::lock_guard<std::mutex> lock(mtx_distance);
    this->distance_ = distance;
}

void Img_proc::Set_img_proc_adjust_number(int8_t img_proc_adjust_number)
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_adjust_number_);
    this->img_proc_adjust_number_ = img_proc_adjust_number;
}

//void Img_proc::Set_gradient(double gradient)
//{
//    std::lock_guard<std::mutex> lock(mtx_gradient);
//    this->gradient_ = gradient;
//}

void Img_proc::Set_adjust_angle(double adjust_angle)
{
    std::lock_guard<std::mutex> lock(mtx_adjust_angle);
    this->adjust_angle_ = adjust_angle;
}

void Img_proc::Set_contain_adjust_to_foot(int contain_adjust_to_foot)
{
    std::lock_guard<std::mutex> lock(mtx_contain_adjust_to_foot);
    this->contain_adjust_to_foot_ = contain_adjust_to_foot;
}

void Img_proc::Set_ball_x(double ball_x)
{
    std::lock_guard<std::mutex> lock(mtx_ball_x);
    this->ball_x_ = ball_x;
}

void Img_proc::Set_ball_y(double ball_y)
{
    std::lock_guard<std::mutex> lock(mtx_ball_y);
    this->ball_y_ = ball_y;
}

void Img_proc::Set_ball_z(double ball_z)
{
    std::lock_guard<std::mutex> lock(mtx_ball_z);
    this->ball_z_ = ball_z;
}

void Img_proc::Set_img_proc_ball_det_(bool img_proc_ball_det_)
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_ball_det_);
    this->img_proc_ball_det_ = img_proc_ball_det_;
}

void Img_proc::Set_img_proc_hoop_det_(bool img_proc_hoop_det_)
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_hoop_det_);
    this->img_proc_hoop_det_ = img_proc_hoop_det_;
}

void Img_proc::Set_img_proc_huddle_det_(bool img_proc_huddle_det_)
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_huddle_det_);
    this->img_proc_huddle_det_ = img_proc_huddle_det_;
}


void Img_proc::Set_img_proc_ball_done_(bool img_proc_ball_done_)
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_ball_done_);
    this->img_proc_ball_done_ = img_proc_ball_done_;
}

void Img_proc::Set_img_proc_hoop_done_(bool img_proc_hoop_done_)
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_hoop_done_);
    this->img_proc_hoop_done_ = img_proc_hoop_done_;
}
void Img_proc::Set_img_proc_huddle_done_(bool img_proc_huddle_done_)
{
    std::lock_guard<std::mutex> lock(mtx_img_proc_huddle_done_);
    this->img_proc_huddle_done_ = img_proc_huddle_done_;
}

void Img_proc::Set_hoop_x(double hoop_x)
{
    std::lock_guard<std::mutex> lock(mtx_hoop_x);
    this->hoop_x_ = hoop_x;
}

void Img_proc::Set_hoop_y(double hoop_y)
{
    std::lock_guard<std::mutex> lock(mtx_hoop_y);
    this->hoop_y_ = hoop_y;
}

void Img_proc::Set_hoop_z(double hoop_z)
{
    std::lock_guard<std::mutex> lock(mtx_hoop_z);
    this->hoop_z_ = hoop_z;
}

void Img_proc::Set_hoopcounter_det_flg_(int hoop_counter)
{
    std::lock_guard<std::mutex> lock(mtx_hoop_counter);
    this->hoop_counter_= hoop_counter;
}

void Img_proc::Set_UD_NeckAngle_(int UD_NeckAngle)
{
    std::lock_guard<std::mutex> lock(mtx_UD_NeckAngle);
    this->UD_NeckAngle_= UD_NeckAngle;
}
