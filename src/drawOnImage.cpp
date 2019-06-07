#include "ros/ros.h"
#include <opencv2/opencv.hpp>

int main(int argc, char **argv){
    ros::init(argc, argv, "showimage3");
    ros::NodeHandle n;
    ros::Rate loop_rate(0.5);
    // -------------------------- INICIA CODIGO DE OPENCV --------------------------
    // Based on: https://docs.opencv.org/3.1.0/d3/d96/tutorial_basic_geometric_drawing.html

    cv::Mat image = cv::Mat::zeros(512, 512, CV_8UC3);
    
    cv::Scalar redColor     =  cv::Scalar(  0,   0, 255);
    cv::Scalar greenColor   =  cv::Scalar(  0, 255,   0);
    cv::Scalar blueColor    =  cv::Scalar(255,   0,   0);
    cv::Scalar magentaColor =  cv::Scalar(255,   0, 255);
    cv::Scalar whiteColor   =  cv::Scalar(255, 255, 255);

    cv::Point startingPoint, endingPoint, centerPoint;
    int thickness = 2, lineType = cv::LINE_8;


    startingPoint =  cv::Point(0,0); 
    endingPoint =  cv::Point(511,511);
    cv::line( image, startingPoint, endingPoint, redColor, thickness, lineType);


    startingPoint =  cv::Point(384,0);
    endingPoint =  cv::Point(510,128);
    cv::rectangle( image,  startingPoint,      endingPoint, greenColor, thickness, lineType);
    cv::rectangle( image, cv::Point(1,1), cv::Point(30,60), greenColor, -1); // Solid rectangle


    int radius = 63;
    centerPoint = cv::Point(447,63);
    cv::circle( image, centerPoint, radius, magentaColor, 1, lineType);


    centerPoint = cv::Point(256,256);
    cv::Size principalAxis = cv::Size(100,50);
    double rotationAngle = 45, startingAngle = 0, endingAngle = 360; // Angles in degress
    cv::ellipse( image, centerPoint, principalAxis, rotationAngle, startingAngle, endingAngle, blueColor, 8, lineType);
    cv::ellipse( image, centerPoint, principalAxis, 0, 0, 180, redColor, -1); // Filled sector


    double fontScale = 4;
    bool bottomLeftOrigin = false;
    int font = cv::FONT_HERSHEY_SIMPLEX;
    cv::putText(image,"OpenCV", cv::Point(20,500), font, fontScale, whiteColor, thickness, lineType, bottomLeftOrigin);


    cv::imshow("Image",image);
    cv::waitKey(0);
    cv::destroyAllWindows();
    // -------------------------- TERMINA CODIGO DE OPENCV --------------------------
    ros::spinOnce();
    return 0;
}
