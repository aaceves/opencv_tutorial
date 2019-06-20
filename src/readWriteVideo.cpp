#include "ros/ros.h"
#include <opencv2/opencv.hpp>

int main(int argc, char **argv){
    ros::init(argc, argv, "showVideo");
    ros::NodeHandle n;
    ros::Rate loop_rate(0.5);
    // -------------------------- INICIA CODIGO DE OPENCV --------------------------
    // Based on: https://www.learnopencv.com/read-write-and-display-a-video-using-opencv-cpp-python/

    std::string path_file = "/home/alex/catkin_ws/src/opencv_tutorial/img/red_panda_snow.mp4";
    cv::VideoCapture cap( path_file ); // Create a video object
    if(!cap.isOpened()) {
        std::cout << "Error opening video stream or file" << path_file << std::endl;
        return -1;
    }

    // Define the codec to 'outcpp.avi' file.
    int frame_rate = 10;
    int frame_width = cap.get(CV_CAP_PROP_FRAME_WIDTH); 
    int frame_height = cap.get(CV_CAP_PROP_FRAME_HEIGHT); 
    int fourcc1 = static_cast<int>(cap.get(CV_CAP_PROP_FOURCC));
    int fourcc2 = CV_FOURCC('M','J','P','G');
    int fourcc3 = cv::VideoWriter::fourcc('X','V','I','D');
    cv::VideoWriter video("outcpp.avi", fourcc2, frame_rate, cv::Size(frame_width,frame_height));
    std::cout << "Output video is in current directory" << std::endl;

    cv::Mat frame, fliped_frame;
    while(true) {    
        cap >> frame; // Capture frame-by-frame
        if (frame.empty()) break; // Exit while at end of video

        cv::flip(frame,fliped_frame,1);  // Flip image 0=horizontal, 1=vertical    
        video.write(fliped_frame); // Write the frame into the file 'outcpp.avi'
        cv::imshow( "Original Video", frame );
        if(cv::waitKey(10)==27)
            break; // Press ESC on keyboard to exit
    }
    cap.release();  // Release the video capture object
    video.release();

    cv::destroyAllWindows();
    // -------------------------- TERMINA CODIGO DE OPENCV --------------------------
    ros::spinOnce();
    return 0;
}


