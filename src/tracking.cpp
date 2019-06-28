#include "ros/ros.h"
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "tracking");
    ros::NodeHandle n;

    // ---------------------------------- INICIA CODIGO DE OPENCV ---------------------------------------------
    // Based on: https://docs.opencv.org/2.4/modules/video/doc/motion_analysis_and_object_tracking.html
    //  Look at: https://www.learnopencv.com/object-tracking-using-opencv-cpp-python/
    //  Look at: https://www.myzhar.com/blog/tutorials/tutorial-opencv-ball-tracker-using-kalman-filter/

    VideoCapture cap(0);
    if(!cap.isOpened())  {
        std::cout << "Can not open video-object." << std::endl;
        return 1;
    }

    Mat color_frame, curr_grayframe, prev_grayframe;
    std::vector<cv::Point2f> features_prev, features_curr;
    cout << "Put object to track in front of the webcam and hit ENTER." << endl;
    while (true) {
        cap >> color_frame; // Capture frame-by-frame
        cvtColor(color_frame, curr_grayframe, COLOR_BGR2GRAY);
        
        int max_count = 500;      // the maximum number of features 
        double qlevel = 0.02;    // quality level
        double minDist = 10;        // min distance between two features
        cv::goodFeaturesToTrack(curr_grayframe, features_curr, max_count, qlevel, minDist );
        
        for (int i = 0; i < features_curr.size(); i++) {
            circle( color_frame, features_curr[i], 3, Scalar(0,255,0), -1, 8);
        }
        namedWindow("To track", WINDOW_AUTOSIZE);
        imshow("To track", color_frame);   // draw the results
        if(cv::waitKey(10) == 13)  break; // Press ENTER on keyboard to exit
    }

    prev_grayframe = curr_grayframe.clone();  //curr_grayframe.copyTo(prev_grayframe);
    features_prev = features_curr;
        
    cout << "Now, tracking object. Hit ESC to finish." << endl;
    vector<uchar> status;
    vector<float> err;
    while (true) {
		
        //prev_grayframe = curr_grayframe.clone();  //curr_grayframe.copyTo(prev_grayframe);
        //feature_prev = features_next;
        
        cap >> color_frame; // Capture frame-by-frame
        cvtColor(color_frame, curr_grayframe, COLOR_BGR2GRAY);    

        // Find position of feature in new image
        cv::calcOpticalFlowPyrLK(prev_grayframe, curr_grayframe, features_prev, features_curr, status, err);
        
        for (int i = 0; i < features_curr.size(); i++) {
			if (err[i] < 10)
                circle( color_frame, features_curr[i], 3, Scalar(0,255,0), -1, 8);
        }
        
        namedWindow("Tracking", WINDOW_AUTOSIZE);
        imshow("Tracking", color_frame);   // draw the results
        if(cv::waitKey(10) == 27)  break; // Press ESC on keyboard to exit
    }
    
    cap.release();  // Release the video capture object
                               
    destroyAllWindows();
    // ---------------------------------- TERMINA CODIGO DE OPENCV ---------------------------------------------
    ros::spinOnce();
    return 0;
}
