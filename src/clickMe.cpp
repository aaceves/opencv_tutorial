#include "ros/ros.h"
#include <opencv2/opencv.hpp>

//Based on: https://stackoverflow.com/questions/33937800/how-to-make-a-simple-window-with-one-button-using-opencv-highgui-only

cv::Rect button;
cv::Mat3b canvas;
std::string winName =  "My cool GUI v0.1";

void callBackFunc(int event, int x, int y, int flags, void* userdata) {
    if ((event == cv::EVENT_LBUTTONDOWN) && (button.contains(cv::Point(x, y)))) {
        std::cout << "Clicked!" << std::endl;
        cv::rectangle(canvas, button, cv::Scalar(0,0,255), 4);
    }
    if (event == cv::EVENT_LBUTTONUP) {
        cv::rectangle(canvas, button, cv::Scalar(200, 200, 200), 4);
    }
    cv::imshow(winName, canvas);
    cv::waitKey(1);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "showDrawings");
    ros::NodeHandle n;
    ros::Rate loop_rate(0.5);
    // -------------------------- INICIA CODIGO DE OPENCV --------------------------
    cv::Mat3b img(300, 300, cv::Vec3b(0, 255, 0));  // A green image of 300x300 pixels
    button = cv::Rect(0,0,img.cols, 50);            // Your button defined as a rectangle
    canvas = cv::Mat3b(img.rows + button.height, img.cols, cv::Vec3b(0,0,0));// The canvas = Button + green image
    canvas(button) = cv::Vec3b(200,200,200);  // Draw the button on gray and a text on it
    cv::putText(canvas(button), "Click me!", cv::Point(button.width*0.35, 
                                             button.height*0.7), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,0,0));
    img.copyTo(canvas(cv::Rect(0, button.height, img.cols, img.rows)));  // Draw button on the green image
    
    cv::namedWindow(winName);  // Setup callback function
    cv::setMouseCallback(winName, callBackFunc);
    cv::imshow(winName, canvas);
    cv::waitKey(0);
    cv::destroyAllWindows();
    // -------------------------- TERMINA CODIGO DE OPENCV --------------------------
    ros::spinOnce();
    return 0;
}
