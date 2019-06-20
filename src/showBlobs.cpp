#include "ros/ros.h"
#include <opencv2/opencv.hpp>

static void onMouseCallBack(int event, int x, int y, int flags, void* param) // Image is in param
{
    cv::Mat &xyz = *((cv::Mat*)param); // Cast and deref the param
    if (event == cv::EVENT_LBUTTONDOWN) {
        if (xyz.channels() == 1) {
            int val = xyz.at<uchar>(y,x); // OpenCV is row-major ! 
            std::cout << "pixel = [" << x << ", " << y << "]   value = "<< val << std::endl;
        }
        if (xyz.channels() == 3) {
            cv::Vec3b val = xyz.at<cv::Vec3b>(y,x); // OpenCV is row-major ! 
            std::cout << "pixel = [" << x << ", " << y << "]   value = "<< val << std::endl;
        }       
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "showBlobs");
    ros::NodeHandle n;
    ros::Rate loop_rate(0.5);
    // -------------------------- INICIA CODIGO DE OPENCV --------------------------
    // Based on: https://gist.github.com/JinpengLI/2cf814fe25222c645dd04e04be4de5a6
    // and: https://github.com/oreillymedia/Learning-OpenCV-3_examples/blob/master/example_14-03.cpp
    // and: https://answers.opencv.org/question/170527/how-to-get-the-pixel-value-by-clicking-on-image/

    // Read image
    std::string path_file = "/home/alex/catkin_ws/src/opencv_tutorial/img/OpenCV_logo.jpg";
    cv::Mat im = cv::imread( path_file );
    if (im.empty()) {
        std::cout << "Error abriendo archivo: " << path_file << std::endl;
        exit(1);
    }

    // Get black_white image
    cv::Mat im_g, im_bw;
    cv::cvtColor(im, im_g, cv::COLOR_BGR2GRAY);
    cv::threshold(im_g, im_bw, 100, 255, cv::THRESH_BINARY);
    cv::imshow("Original", im);
    cv::setMouseCallback("Original", onMouseCallBack, &im); // Activate mouse-clicking on image

    // Find connected components only
    int nccomps;
    cv::Mat labels, centroids, stats, normLabels;
    nccomps = cv::connectedComponents(im_bw, labels, 8);
    cv::normalize(labels, normLabels, 0, 255, cv::NORM_MINMAX, CV_8U);
    cv::imshow("Labels", normLabels);

    // Find connected componentes with bounding boxes, centroids and areas 
    nccomps = cv::connectedComponentsWithStats (im_bw, labels, stats, centroids);
    std::cout << "Total Connected Components Detected: " << nccomps << std::endl;
    std::cout << "Labels.size: " << labels.size() << std::endl;
    std::cout << "Stats.size(): " << stats.size() << std::endl;
    std::cout << "Centroids.size(): " << centroids.size() << std::endl << std::endl;
    int x, y, w, h, a;
    double cx, cy;
    for (int i=0; i < stats.rows; i++)
    {
        x = stats.at<int>(cv::Point(0, i));
        y = stats.at<int>(cv::Point(1, i));
        w = stats.at<int>(cv::Point(2, i));
        h = stats.at<int>(cv::Point(3, i));
        a = stats.at<int>(cv::Point(4, i));
        std::cout << "Bounding Box:  x=" << x << ", y=" << y << ", w=" << w << ", h=" << h << ", a=" << a << "    ";
        cv::Rect rect(x,y,w,h);
        cv::rectangle(im, rect, cv::Scalar(0,255,0));

        cx = centroids.at<double>(cv::Point(0, i));
        cy = centroids.at<double>(cv::Point(1, i));
        std::cout << "Centroid:  cx=" << cx << ", cy=" << cy << std::endl;
        cv::circle(im, cv::Point(cx,cy), 4, cv::Scalar(0,0,255), -1);
    }
    cv::imshow("Connected Components", im);

    // Select specifics connected componentes
    int desired_label = 3; // For example
    cv::Mat extracted_blob(im.size(), CV_8UC1, cv::Scalar(0));

    for (int r=0; r < im.rows; r++) {
        for (int c=0; c < im.cols; c++) {
            if (labels.at<int>(r,c) >= desired_label)
               extracted_blob.at<uchar>(r,c) = 255;
        }
    }
    cv::namedWindow("Extracted Component");
    cv::imshow("Extracted Component", extracted_blob);
    std::cout  << "\n Hit ENTER in any image to exit." << std::endl;
    cv::waitKey(0);
    cv::destroyAllWindows();
    // -------------------------- TERMINA CODIGO DE OPENCV --------------------------
    ros::spinOnce();
    return 0;
}
