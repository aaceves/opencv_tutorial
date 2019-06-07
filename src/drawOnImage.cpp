//https://docs.opencv.org/3.4/dc/da5/tutorial_py_drawing_functions.html
//https://pysource.com/2018/01/22/drawing-and-writing-on-images-opencv-3-4-with-python-3-tutorial-3/
//https://docs.opencv.org/3.1.0/d3/d96/tutorial_basic_geometric_drawing.html
//http://opencvexamples.blogspot.com/2013/10/basic-drawing-examples.html
//https://docs.opencv.org/3.4/d3/d96/tutorial_basic_geometric_drawing.html
//http://opencvexamples.blogspot.com/p/learning-opencv-functions-step-by-step.html


#include "ros/ros.h"
#include <opencv2/opencv.hpp>

int main(int argc, char **argv){
    ros::init(argc, argv, "showimage");
    ros::NodeHandle n;
    ros::Rate loop_rate(0.5);
    // -------------------------- INICIA CODIGO DE OPENCV --------------------------
    // Based from: https://docs.opencv.org/3.2.0/db/d64/tutorial_load_save_image.html
    cv::Mat image;
    cv::Mat gray_image;

    image = cv::imread("/home/kinetic/catkin_ws/src/opencv_tutorial/img/red_panda.jpg"); 
    if (image.empty()) { // or (!image.data)
        std::cout << "Error al cargar imagen."<< std::endl;
        exit(1);
    }
    std::cout << "Height : " << image.rows << std::endl;  // or image.size().height
    std::cout << "Width : " << image.cols << std::endl;  // or image.size().width

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
