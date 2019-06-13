#include "ros/ros.h"
#include <opencv2/opencv.hpp>

int main(int argc, char **argv){
    ros::init(argc, argv, "cropMergeWrite");
    ros::NodeHandle n;
    ros::Rate loop_rate(0.5);
    // -------------------------- INICIA CODIGO --------------------------
    // Based on: https://answers.opencv.org/question/78994/how-to-crop-an-image-using-cc/
    // And: http://opencvexamples.blogspot.com/2013/10/split-and-merge-functions.html
    // And: http://acodigo.blogspot.com/2018/01/opencv-uso-de-las-funciones-split-y.html

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
    cv::moveWindow("Merged panda", 300,100);

    image_c = image(cv::Rect(100,80,280,180));  // roi=Rect(x,y,w,h)
    nose = image(cv::Rect(220,180,40,50)); // Extract some region of image
    image_m = image.clone();  // Copy image into image_m
    nose.copyTo(image_m(cv::Rect(0,0,40,50))); // Copy nose into image_m
    image_m(cv::Rect(100,100,50,50)) = cv::Scalar(255,255,255); // Write into image_m

    cv::imshow("Red panda", image);
    cv::imshow("Croped panda", image_c);
    cv::imshow("Merged panda", image_m);
    std::cout << "Hit ENTER to close window images." << std::endl;
    cv::waitKey(0);
    cv::destroyAllWindows();
    image.release();     nose.release();     image_c.release();     image_m.release(); 


    cv::Mat image1 = cv::imread("/home/kinetic/catkin_ws/src/opencv_tutorial/img/robot_teacher.jpg");
    cv::Mat image2 = cv::imread("/home/kinetic/catkin_ws/src/opencv_tutorial/img/OpenCV_logo.jpg");
    cv::Mat added, blend;
    cv::add(image1, image2, added);
    cv::addWeighted(image1, 0.7, image2, 0.3, 0, blend);
    cv::imshow("First image",image1);
    cv::imshow("Second image",image2);
    cv::imshow("Images added",added);
    cv::imshow("Images blended",blend);

    std::cout << "Hit ENTER to close window images." << std::endl;
    cv::waitKey(0);
    cv::destroyAllWindows();
    image1.release();     image2.release();     added.release();     blend.release(); 


    image = cv::imread("/home/kinetic/catkin_ws/src/opencv_tutorial/img/messi5.jpg");
    std::vector<cv::Mat> rgbChannels;
    cv::split(image, rgbChannels);

    // Show individual channels
    cv::Mat black_img, fin_img;
    black_img = cv::Mat::zeros(cv::Size(image.cols, image.rows), CV_8UC1);
     
    // Showing Red Channel
    // G and B channels are kept as zero matrix for visual perception
    {
    std::vector<cv::Mat> channels;
    channels.push_back(black_img);
    channels.push_back(black_img);
    channels.push_back(rgbChannels[2]);
    cv::merge(channels, fin_img);
    cv::namedWindow("R",1);
    cv::imshow("R", fin_img);
    }
 
    // Showing Green Channel
    {
    std::vector<cv::Mat> channels;
    channels.push_back(black_img);
    channels.push_back(rgbChannels[1]);
    channels.push_back(black_img);    
    cv::merge(channels, fin_img);
    cv::namedWindow("G",1);
    cv::imshow("G", fin_img);
    }
 
    // Showing Blue Channel
    {
    std::vector<cv::Mat> channels;
    channels.push_back(rgbChannels[0]);
    channels.push_back(black_img);
    channels.push_back(black_img);
    cv::merge(channels, fin_img);
    cv::namedWindow("B",1);
    cv::imshow("B", fin_img);
    }

    std::cout << "Hit ENTER to close window images." << std::endl;
    cv::waitKey(0);
    cv::destroyAllWindows();
    // -------------------------- TERMINA CODIGO --------------------------
    ros::spinOnce();
    return 0;
}
