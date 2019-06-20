#include "ros/ros.h"
#include <opencv2/opencv.hpp>


#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include <opencv2/nonfree/nonfree.hpp>
#include "opencv2/highgui/highgui.hpp"

#include <vector>

using namespace std;
using namespace cv;


int main(int argc, char **argv){
    ros::init(argc, argv, "showSiftFeat");
    ros::NodeHandle n;
    ros::Rate loop_rate(0.5);
    // -------------------------- INICIA CODIGO DE OPENCV --------------------------
    // Based on: http://gabrieleomodeo.it/blog/how-to-set-up-ros-with-opencv2-nonfree-package/
    //
    //  Si hay problemas para compilar significa que no se tienen instaladas las "nonfree" features.
    //  Se puede bajar de https://github.com/opencv/opencv_contrib/., pero hay que seguir las instrucciones.
    //  Una forma más sencilla de instalar estas funciones extras es ejecutando esto en una Terminal:
    //
    //      sudo add-apt-repository --yes ppa:xqms/opencv-nonfree
    //      sudo apt-get update
    //      sudo apt-get install libopencv-nonfree-dev
    //
    //    NO ME FUNCIONO. DICE QUE HAY DEPENDENCIAS QUE NO SE PUEDEN INSTALAR

    cv::Mat input = cv::imread("/home/kinetic/catkin_ws/src/opencv_tutorial/img/red_panda.jpg", 0); //Load as grayscale

    cv::SiftFeatureDetector detector;
    std::vector<cv::KeyPoint> keypoints;
    detector.detect(input, keypoints);

    // Add results to image and save.
    cv::Mat output;
    cv::drawKeypoints(input, keypoints, output);
    cv::imwrite("sift_result.jpg", output);

    cv::waitKey(0);

    // -------------------------- TERMINA CODIGO DE OPENCV --------------------------
    ros::spinOnce();
    return 0;
}
