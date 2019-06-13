#include "ros/ros.h"
#include <opencv2/opencv.hpp>

int main(int argc, char **argv){
    ros::init(argc, argv, "cropMergeWrite");
    ros::NodeHandle n;
    ros::Rate loop_rate(0.5);
    // -------------------------- INICIA CODIGO --------------------------
    // Based on: 

    cv::Mat image, nose, image_c, image_m;

    image = cv::imread("/home/kinetic/catkin_ws/src/opencv_tutorial/img/red_panda.jpg"); 
    if (image.empty()) { // or (!image.data)
        std::cout << "Error al cargar imagen."<< std::endl;
        exit(1);
    }

    cv::namedWindow("Red panda");         // Crea una ventana con un nombre específico
    cv::moveWindow("Red panda", 100,100); // Mueve la ventana a la posición x,y
    cv::namedWindow("Croped panda");
    cv::moveWindow("Croped panda", 200,100);
    cv::namedWindow("Merged panda");
    cv::moveWindow("Merdeg panda", 300,100);

    //image_c = image[80:260, 100:380];
    //nose = image[180:230,220:260];
    //image[100;150,100:150] = [255,255,255];

    cv::imshow("Red panda", image);
    cv::imshow("Croped panda", image);
    cv::imshow("Merged panda", image);
    std::cout << "Hit ENTER to close window images." << std::endl;
    cv::waitKey(0);
    cv::destroyAllWindows();

    // -------------------------- TERMINA CODIGO --------------------------
    ros::spinOnce();
    return 0;
}
