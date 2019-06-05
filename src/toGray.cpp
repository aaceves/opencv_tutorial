#include "ros/ros.h"
#include <opencv2/opencv.hpp>

int main(int argc, char **argv){
    ros::init(argc, argv, "showimage");
    ros::NodeHandle n;
    ros::Rate loop_rate(0.5);
    // -------------------------- INICIA CODIGO DE OPENCV --------------------------
    cv::Mat image;
    cv::Mat gray_image;
    image = cv::imread("/home/kinetic/catkin_ws/src/opencv_tutorial/img/red_panda.jpg"); 
    if (!image.data) {
        std::cout << "Error al cargar imagen: red_panda.jpg"<< std::endl;
        exit(1);
    }
    cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);
    cv::imshow("Red panda", image);
    cv::imshow("Gray panda", gray_image);
    std::cout << "Hit ENTER to close window images." << std::endl;
    cv::waitKey(0);
    cv::destroyAllWindows();
    
    //std::cout << "Dimensión de la variable image: " << image.shape << std::endl;
    //std::cout << "Dimensión de la variable gray_image: " << gray_image.shape << std::endl;

    cv::imwrite("/home/kinetic/catkin_ws/src/opencv_tutorial/img/gray_panda2.jpg", gray_image);
    // -------------------------- TERMINA CODIGO DE OPENCV --------------------------
    ros::spinOnce();
    return 0;
}
