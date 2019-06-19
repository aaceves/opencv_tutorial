#include "ros/ros.h"
#include <opencv2/opencv.hpp>

int main(int argc, char **argv){
    ros::init(argc, argv, "showGray");
    ros::NodeHandle n;
    ros::Rate loop_rate(0.5);
    // -------------------------- INICIA CODIGO DE OPENCV --------------------------
    // Based on: https://docs.opencv.org/3.2.0/db/d64/tutorial_load_save_image.html
    cv::Mat image;
    cv::Mat gray_image;

    std::string path_image = "/home/alex/catkin_ws/src/opencv_tutorial/img/red_panda.jpg";
    image = cv::imread(path_image); 
    if (image.empty()) { // or (!image.data)
        std::cout << "Error al cargar imagen: "<< path_image << ". ¿La dirección es correcta?" << std::endl;
        exit(1);
    }
    std::cout << "Height : " << image.rows << std::endl;  // or image.size().height
    std::cout << "Width : " << image.cols << std::endl;  // or image.size().width
    std::cout << "Number of color channles : " << image.channels() << std::endl; 
    std::cout << "Data type : " << image.depth() << std::endl; 

    cv::namedWindow("Red panda");         // Crea una ventana con un nombre específico
    cv::moveWindow("Red panda", 100,100); // Mueve la ventana a la posición x,y
    cv::namedWindow("Gray panda");
    cv::moveWindow("Gray panda", 200,100);

    cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY); // Convierte de BGR a GRAY
    cv::imshow("Red panda", image);
    cv::imshow("Gray panda", gray_image);
    std::cout << "Hit ENTER to close window images." << std::endl;
    cv::waitKey(0);
    cv::destroyAllWindows();

    cv::imwrite("/home/kinetic/catkin_ws/src/opencv_tutorial/img/gray_panda.jpg", gray_image);
    // -------------------------- TERMINA CODIGO DE OPENCV --------------------------
    ros::spinOnce();
    return 0;
}
