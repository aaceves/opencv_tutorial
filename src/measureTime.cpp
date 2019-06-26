#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <time.h>

using namespace cv;
using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "showTime");
    ros::NodeHandle n;

    // ---------------------------------- INICIA CODIGO ---------------------------------------------
    // Based on: http://wiki.ros.org/roscpp/Overview/Time
    // and: http://opencvexamples.blogspot.com/2013/11/calculating-time-taken-by-code.html

    cout << "First way to measure time." << endl;
    long double tic = getTickCount();
    
    for (long int i=0; i < 100000000; i++) { 
        waitKey(1);
    }
    double toc = (getTickCount() - tic) / getTickFrequency();
    cout << "execTime = " << toc << " sec." << endl;



    cout << "Second way to measure time." << endl;
    ros::WallTime start1_ = ros::WallTime::now();
    for (long int i=0; i < 100000000; i++) { 
        waitKey(1);
    }
    ros::WallTime end1_ = ros::WallTime::now();
    double execution_time = (end1_ - start1_).toSec();
    cout << "execTime = " << execution_time << " sec." << endl;
    
    
    
    cout << "Third way to measure time." << endl;
    clock_t start2_ = clock();
    for (long int i=0; i < 100000000; i++) { 
        waitKey(1);
    }
    clock_t end2_ = clock();
    double elapsed_time = double(end2_ - start2_)/double(CLOCKS_PER_SEC);
    cout << "execTime = " << elapsed_time << " sec." << endl;
       

    // ---------------------------------- TERMINA CODIGO --------------------------------------------
    ros::spinOnce();
    return 0;
}
