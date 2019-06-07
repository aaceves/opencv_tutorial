#include "ros/ros.h"
#include <opencv2/opencv.hpp>

const std::string window_capture_name = "Original Video";
const std::string window_detection_name = "Object Detection";

int low_H = 0, low_S = 0, low_V = 0;
int high_H = 180, high_S = 255, high_V = 255;

static void on_low_H_thresh_trackbar(int, void *)
{
    low_H = std::min(high_H-1, low_H);
    cv::setTrackbarPos("Low H", window_detection_name, low_H);
}
static void on_high_H_thresh_trackbar(int, void *)
{
    high_H = std::max(high_H, low_H+1);
    cv::setTrackbarPos("High H", window_detection_name, high_H);
}
static void on_low_S_thresh_trackbar(int, void *)
{
    low_S = std::min(high_S-1, low_S);
    cv::setTrackbarPos("Low S", window_detection_name, low_S);
}
static void on_high_S_thresh_trackbar(int, void *)
{
    high_S = std::max(high_S, low_S+1);
    cv::setTrackbarPos("High S", window_detection_name, high_S);
}
static void on_low_V_thresh_trackbar(int, void *)
{
    low_V = std::min(high_V-1, low_V);
    cv::setTrackbarPos("Low V", window_detection_name, low_V);
}
static void on_high_V_thresh_trackbar(int, void *)
{
    high_V = std::max(high_V, low_V+1);
    cv::setTrackbarPos("High V", window_detection_name, high_V);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "showimage1");
    ros::NodeHandle n;
    ros::Rate loop_rate(0.5);
    // -------------------------- INICIA CODIGO DE OPENCV --------------------------
    // Based on: https://docs.opencv.org/master/da/d97/tutorial_threshold_inRange.html
    cv::VideoCapture cap(0);
    cv::namedWindow(window_capture_name);
    cv::moveWindow(window_capture_name, 100,100);
    cv::namedWindow(window_detection_name); 
    cv::moveWindow(window_detection_name, 800,100);
 
    // Trackbars to set thresholds for HSV values
    cv::createTrackbar("Low H",  window_detection_name, &low_H,  high_H, on_low_H_thresh_trackbar);
    cv::createTrackbar("High H", window_detection_name, &high_H, high_H, on_high_H_thresh_trackbar);
    cv::createTrackbar("Low S",  window_detection_name, &low_S,  high_S, on_low_S_thresh_trackbar);
    cv::createTrackbar("High S", window_detection_name, &high_S, high_S, on_high_S_thresh_trackbar);
    cv::createTrackbar("Low V",  window_detection_name, &low_V,  high_V, on_low_V_thresh_trackbar);
    cv::createTrackbar("High V", window_detection_name, &high_V, high_V, on_high_V_thresh_trackbar);
    
    cv::Mat frame, frame_HSV, frame_threshold;

    while (true) {
        cap >> frame;
        if(frame.empty())  exit(1);
        // Convert from BGR to HSV colorspace
        cv::cvtColor(frame, frame_HSV, cv::COLOR_BGR2HSV);
        // Detect the object based on HSV Range Values
        cv::inRange(frame_HSV, cv::Scalar(low_H, low_S, low_V), cv::Scalar(high_H, high_S, high_V), frame_threshold);
        // Show the frames
        cv::imshow(window_capture_name, frame);
        cv::imshow(window_detection_name, frame_threshold);
        char key = (char) cv::waitKey(30);
        if (key == 27) exit(2);
    }    
    // -------------------------- TERMINA CODIGO DE OPENCV --------------------------
    ros::spinOnce();
    return 0;
}
