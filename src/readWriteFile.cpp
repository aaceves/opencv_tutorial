#include "ros/ros.h"
#include <opencv2/opencv.hpp>

int main(int argc, char **argv){
    ros::init(argc, argv, "readWriteFile");
    ros::NodeHandle n;
    ros::Rate loop_rate(0.5);
    // -------------------------- INICIA CODIGO --------------------------
    // Based on: https://www.youtube.com/watch?v=lzxWNtjii8U


    // To write in a file
    std::ofstream ofile_handler;
    ofile_handler.open("colorSpace_thresholds.txt"); // Use: /home/kinetic/catkin_ws/src/opencv_tutorial/colorSpace_thresholds.txt
    if (ofile_handler.fail()) {
        std::cerr << "Error on creating file." << std::endl;
        exit(1);
    }
    ofile_handler << "low_H  " << 123 << std::endl; // This is the first line
    ofile_handler << "high_H " << 222 << std::endl; 
    ofile_handler << "low_S  " << 321 << std::endl; 
    ofile_handler << "high_S " << 404 << std::endl;
    ofile_handler << "low_I  " << 555 << std::endl;
    ofile_handler << "high_I " << 654 << std::endl; // This is the last line
    ofile_handler.close();
    std::cout << "File is saved in current directory." << std::endl;
    std::cout << "Hit ENTER to continue ... " << std::endl;
    std::cin.get();


    // To read from a file
    std::ifstream ifile_handler;
    std::string name;
    int value;
    ifile_handler.open("colorSpace_thresholds.txt");
    if (ifile_handler.is_open()) {
        while(ifile_handler >> name >> value) {
            std::cout << name << " " << value << std::endl;
        }
        ifile_handler.close();
    }
    else {
        std::cout << "File not found" << std::endl;
        exit(2);
    }
    
    std::cout << "\nHit ENTER to finish." << std::endl;
    std::cin.get();
    // -------------------------- TERMINA CODIGO --------------------------
    ros::spinOnce();
    return 0;
}
