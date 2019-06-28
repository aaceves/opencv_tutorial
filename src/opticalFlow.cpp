#include "ros/ros.h"
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "opticalFlow");
    ros::NodeHandle n;

    // ---------------------------------- INICIA CODIGO DE OPENCV ---------------------------------------------
    // Based on: https://funvision.blogspot.com/2016/02/opencv-31-tutorial-optical-flow.html

    VideoCapture cap;
    // Specify your file name. If nothing is specified, work with webcam
    if (argc == 1)   cap.open(0);
    else cap.open(argv[1]);
    
    if(!cap.isOpened())  {
        std::cout << "Can not open video-object." << std::endl;
        return 1;
    }

    Mat flow, color_frame, curr_grayframe, prev_grayframe;

    while (true) {
        cap >> color_frame; // Capture frame-by-frame
        cvtColor(color_frame, curr_grayframe, COLOR_BGR2GRAY);
        if (curr_grayframe.empty()) break; // Exit while at end of video
 
        if (!prev_grayframe.empty()) {
		    // Computes a dense optical flow using the Gunnar Farneback’s algorithm.

            double pyr_scale = 0.4;
            int levels = 1;
            int winsize = 12;
            int iterations = 2;
            int poly_n = 8;
            double poly_sigma = 1.2;
            int flags = OPTFLOW_USE_INITIAL_FLOW;
            calcOpticalFlowFarneback(prev_grayframe, curr_grayframe, flow, pyr_scale, levels, winsize, iterations, poly_n, poly_sigma, flags);
            
            int grid_size = 10;             // By y += grid_size, x += grid_size you can specify the grid 
            for (int y = 0; y < color_frame.rows; y += grid_size) {
                for (int x = 0; x < color_frame.cols; x += grid_size) {
                    // get the flow from y, x position * 10 for better visibility
                    const Point2f flowatxy = flow.at<Point2f>(y, x) * 2;
                    // draw line at flow direction
                    line(color_frame, Point(x, y), Point(cvRound(x + flowatxy.x), cvRound(y + flowatxy.y)), Scalar(255,0,0));
                    // draw initial point
                    //circle(color_frame, Point(x, y), 1, Scalar(0, 0, 0), -1);
                }
            }
        }

        curr_grayframe.copyTo(prev_grayframe);
        namedWindow("Optical Flow", WINDOW_AUTOSIZE);
        imshow("Optical Flow", color_frame);   // draw the results
        if(cv::waitKey(10)==27)  break; // Press ESC on keyboard to exit
    }
    cap.release();  // Release the video capture object
                               
    destroyAllWindows();
    // ---------------------------------- TERMINA CODIGO DE OPENCV ---------------------------------------------
    ros::spinOnce();
    return 0;
}
