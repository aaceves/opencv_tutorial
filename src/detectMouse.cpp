#include "ros/ros.h"
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

void callBackFunc(int event, int x, int y, int flags, void* userdata) {
     if ( event == EVENT_MOUSEMOVE ) {
          cout << "Mouse move over the window - position (" << x << ", " << y << ")" << endl;
     }
     else if  ( event == EVENT_RBUTTONDOWN ) {
          cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
     }
     else if  ( event == EVENT_MBUTTONDOWN ) {
          cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
     }
     else if  ( event == EVENT_LBUTTONDOWN ) {
          cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
     } 
     else if  ( event == EVENT_RBUTTONUP ) {
          cout << "Right button of the mouse is released - position (" << x << ", " << y << ")" << endl;
     }
     else if  ( event == EVENT_MBUTTONUP ) {
          cout << "Middle button of the mouse is released - position (" << x << ", " << y << ")" << endl;
     }
     else if  ( event == EVENT_LBUTTONUP ) {
          cout << "Left button of the mouse is released - position (" << x << ", " << y << ")" << endl;
     } 
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "detectMouse");
    ros::NodeHandle n;
    ros::Rate loop_rate(0.5);
    // -------------------------- INICIA CODIGO DE OPENCV --------------------------
    // Based on: https://www.opencv-srf.com/2011/11/mouse-events.html
    
     Mat img = imread(argv[1]);

     //if fail to read the image
     if ( img.empty() ) { 
          cout << "Error loading the image" << endl;
          return -1; 
     }
    
    cv::namedWindow("Image");  // Setup callback function
    cv::setMouseCallback("Image", callBackFunc, NULL);
    cv::imshow("Image", img);
    
    while(true){
		if(cv::waitKey(10) == 27)  break;
	}
    cv::destroyAllWindows();
    // -------------------------- TERMINA CODIGO DE OPENCV --------------------------
    ros::spinOnce();
    return 0;
}
