#include "ros/ros.h"
#include <opencv2/opencv.hpp>

int main(int argc, char **argv){
    ros::init(argc, argv, "showimage2");
    ros::NodeHandle n;
    ros::Rate loop_rate(0.5);
    // -------------------------- INICIA CODIGO DE OPENCV --------------------------
    // Based from: https://docs.opencv.org/3.0-beta/modules/videoio/doc/reading_and_writing_video.html
    cv::VideoCapture cap(0);
    if(!cap.isOpened())  {
        std::cout << "No se pudo abrir la cámara." << std::endl;
        return 1;
    }
    cv::Mat frame, edges;
    while (true) {
        cap >> frame;
        cv::cvtColor(frame, edges, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(edges, edges, cv::Size(7,7), 1.5, 1.5);
        cv::Canny(edges, edges, 0, 30, 3);
        cv::imshow("edges", edges);
        if (cv::waitKey(30) == 27) break;
    }    
    // -------------------------- TERMINA CODIGO DE OPENCV --------------------------
    ros::spinOnce();
    return 0;
}
