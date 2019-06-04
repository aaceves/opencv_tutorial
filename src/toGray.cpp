#include "ros/ros.h"
#include <opencv2/opencv.hpp>

int main(int argc, char **argv){
    ros::init(argc, argv, "showimage");
    ros::NodeHandle n;
    ros::Rate loop_rate(0.5);
    // -------------------------- INICIA CODIGO DE OPENCV --------------------------
    cv::Mat image;
    cv::Mat gray_image;
    image = cv::imread("red_panda.jpg"); 
    if (!image.data) {
        std::cout << "Error al cargar imagen: red_panda.jpg"<< std::endl;
        exit(1);
    }
    cv::imshow("Red panda", image);
    cv::imshow("Gray panda", gray_image)
    cv::waitKey(0);
    cv::destroyAllWindows();
    gray_image = cv::cvtColor(image, cv::COLOR_BGR2GRAY)
    std::cout << "Dimensión de la variable image: " << image.shape << std::endl;
    std::cout << "Dimensión de la variable gray_image: " << gray_image.shape << std::endl;
    cv::imwrite("gray_panda.jpg", gray_image);
    // -------------------------- TERMINA CODIGO DE OPENCV --------------------------
    ros::spinOnce();
    return 0;
}
