#include "ros/ros.h"
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "findContours");
    ros::NodeHandle n;

    // ---------------------------------- INICIA CODIGO DE OPENCV ---------------------------------------------
    // Based on: http://opencvexamples.blogspot.com/2013/09/find-contour.html
    Mat image, gray;
    image = imread(argv[1]); 
    if( !image.data ) {
	    cout << "Archivo no encontrado o erroneamente especificado." << endl; 
	    exit(-1); 
    }
    
    // Display original image and gray image
    namedWindow( "Display window", CV_WINDOW_AUTOSIZE );  
    imshow( "Display window", image );
    cvtColor(image, gray, CV_BGR2GRAY);
    Canny(gray, gray, 100, 200, 3);
    imshow( "Gray image", gray );
    
    // Find contours   
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours( gray, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0,0) );
    cout << "I found " << contours.size() << " contours." << endl;
    
    // Draw contours
    RNG rng(12345); // Generate a random number
    Mat drawing = Mat::zeros( gray.size(), CV_8UC3 );
    int thickness=1, lineType=8;
    for( int i = 0; i< contours.size(); i++ ) { // i = contour to draw.
        Scalar random_color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        drawContours( drawing, contours, i, random_color, thickness, lineType, hierarchy, 0, Point() );
    }     
 
    // Find largest contour (this is a kind of tricky)
    Rect bounding_rect;
    int largest_area = 0;
    int largest_a_index = 0;
    vector<vector<Point> > hull( contours.size() );
    for( int index = 0; index < contours.size(); index++ ) {
		convexHull( contours[index], hull[index] );
        double area = contourArea( hull.at(index), false);   // contours[i] = contours.at(i)
        // It will most certainly give a wrong results for contours with self-intersections. Soo it is better to use convexhull.
        if(area > largest_area){
            largest_area = area;
            largest_a_index = index;               
            bounding_rect = boundingRect(contours[largest_a_index]);
        }
    }
    rectangle(drawing, bounding_rect,  Scalar(0,255,0),2, 8, 0);
 
    // Draw results
    namedWindow( "Result window", CV_WINDOW_AUTOSIZE );  
    imshow( "Result window", drawing );
    waitKey(0);                                         
    destroyAllWindows();
    // ---------------------------------- TERMINA CODIGO DE OPENCV ---------------------------------------------
    ros::spinOnce();
    return 0;
}
