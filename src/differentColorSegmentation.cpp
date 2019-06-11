#include "ros/ros.h"
#include <opencv2/opencv.hpp>

const std::string window_capture_name = "Original Video";
const std::string window_detection_name = "Object Detection";

std::string name_x, name_y, name_z;
int low_x, low_y, low_z;
int high_x, high_y, high_z;
int color_space, flag;
int max_x, max_yz;

void HSV() {
    flag = 1;  // Hue is a circular space
    color_space= cv::COLOR_BGR2HSV;
    name_x = "H";  name_y = "S";  name_z = "V";
    max_x = 180;  max_yz = 255;
}

void HLS() {
    flag = 1;  // Hue is a circular space
    color_space= cv::COLOR_BGR2HLS;
    name_x = "H";  name_y = "L";  name_z = "S";
    max_x = 180;  max_yz = 255;
}

void YCrCb() {
    flag = 0;  // No circular space
    color_space= cv::COLOR_BGR2YCrCb;
    name_x = "Y";  name_y = "Cr";  name_z = "Cb";
    max_x = 255;  max_yz = 255;
}

void Lab() {
    flag = 0;  // No circular space
    color_space= cv::COLOR_BGR2Lab;
    name_x = "L";  name_y = "a";  name_z = "b";
    max_x = 255;  max_yz = 255;
}

// CallBack functions definitions
static void on_low_x_thresh_trackbar(int, void *) {
    if (flag == 0) low_x = std::min(high_x-1, low_x);
    cv::setTrackbarPos("Low " + name_x, window_detection_name, low_x);
}
static void on_high_x_thresh_trackbar(int, void *) {
    if (flag == 0) high_x = std::max(high_x, low_x+1);
    cv::setTrackbarPos("High " + name_x, window_detection_name, high_x);
}
static void on_low_y_thresh_trackbar(int, void *) {
    low_y = std::min(high_y-1, low_y);
    cv::setTrackbarPos("Low " + name_y, window_detection_name, low_y);
}
static void on_high_y_thresh_trackbar(int, void *) {
    high_y = std::max(high_y, low_y+1);
    cv::setTrackbarPos("High " + name_y, window_detection_name, high_y);
}
static void on_low_z_thresh_trackbar(int, void *)
{
    low_z = std::min(high_z-1, low_z);
    cv::setTrackbarPos("Low " + name_z, window_detection_name, low_z);
}
static void on_high_z_thresh_trackbar(int, void *) {
    high_z = std::max(high_z, low_z+1);
    cv::setTrackbarPos("High " + name_z, window_detection_name, high_z);
}

int main(int argc, char **argv){
    int option;
    std::cout << "Choose option and hit ENTER:" << std::endl;
    std::cout << "1=HSV       2=HLS       3=Lab     4=YCrCb   : ";
    std::cin >> option;

    switch (option) {
        case 1:
            HSV();
            break;
        case 2:
            HLS();
            break;
        case 3:
            Lab();
            break;
        case 4:
            YCrCb();
            break;
    }
    low_x = 0;  low_y = 0;  low_z = 0;
    high_x = max_x;  high_y = max_yz; high_z = max_yz;
    
    ros::init(argc, argv, "spaceColors");
    ros::NodeHandle n;
    ros::Rate loop_rate(0.5);
    // -------------------------- INICIA CODIGO DE OPENCV --------------------------
    // Based on: https://docs.opencv.org/master/da/d97/tutorial_threshold_inRange.html
    // Based on: https://www.learnopencv.com/color-spaces-in-opencv-cpp-python/ 
    cv::VideoCapture cap(0);
    cv::namedWindow(window_capture_name);
    cv::moveWindow(window_capture_name, 100,100);
    cv::namedWindow(window_detection_name); 
    cv::moveWindow(window_detection_name, 800,100);
	 
    // Trackbars to set thresholds for xyz values
    cv::createTrackbar("Low " + name_x, window_detection_name, &low_x,  max_x,  on_low_x_thresh_trackbar);
    cv::createTrackbar("High "+ name_x, window_detection_name, &high_x, max_x,  on_high_x_thresh_trackbar);
    cv::createTrackbar("Low " + name_y, window_detection_name, &low_y,  max_yz, on_low_y_thresh_trackbar);
    cv::createTrackbar("High "+ name_y, window_detection_name, &high_y, max_yz, on_high_y_thresh_trackbar);
    cv::createTrackbar("Low " + name_z, window_detection_name, &low_z,  max_yz, on_low_z_thresh_trackbar);
    cv::createTrackbar("High "+ name_z, window_detection_name, &high_z, max_yz, on_high_z_thresh_trackbar);
    
    cv::Mat frame, frame_Out, frame_threshold, frame_threshold1, frame_threshold2;
    std::cout << "Hit ESC on cam-image to close node." << std::endl;

    while (true) {
        cap >> frame;
        if (frame.empty())  exit(1);
        // Convert from BGR to any colorspace  
        cv::cvtColor(frame, frame_Out, color_space);
        // Detect the object based on HSV Range Values
        if (flag == 1 && low_x > high_x){
            cv::inRange(frame_Out, cv::Scalar(low_x,low_y,low_z), cv::Scalar(max_x, high_y,high_z), frame_threshold1);
            cv::inRange(frame_Out, cv::Scalar(    0,low_y,low_z), cv::Scalar(high_x,high_y,high_z), frame_threshold2);
            cv::add(frame_threshold1, frame_threshold2, frame_threshold);
        }
        else {
            cv::inRange(frame_Out, cv::Scalar(low_x,low_y,low_z), cv::Scalar(high_x,high_y,high_z), frame_threshold);
        }
        // Show the frames
        cv::putText(frame, "BGR -> " + name_x + name_y + name_z, cv::Point(10,50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,255,0), 2, cv::LINE_8, false);
        cv::imshow(window_capture_name, frame);
        cv::imshow(window_detection_name, frame_threshold);
        if (cv::waitKey(30) == 27) break;
    }
    cv::destroyAllWindows();
    // -------------------------- TERMINA CODIGO DE OPENCV --------------------------
    ros::spinOnce();
    return 0;
}
