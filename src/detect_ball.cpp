#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
using namespace cv;
using namespace std;

int main( int argc, const char** argv ){
	Mat frame, blur;
    Mat gray(frame.size(), CV_8UC1);
	Mat hsv, gray2;
    VideoCapture capture( 0 );
	int hMin = 100; int sMin = 0; int vMin = 0;
	int hMax = 130; int sMax = 100; int vMax = 100;
	int bounB = 40000; int minR = 27; int maxR = 32;
    namedWindow("Segmentacion");
	namedWindow("Bounding Box");
	createTrackbar("H (Min)", "Segmentacion", &hMin, 360, NULL);
	createTrackbar("S (Min)", "Segmentacion", &sMin, 100, NULL);
	createTrackbar("V (Min)", "Segmentacion", &vMin, 100, NULL);
	createTrackbar("H (Max)", "Segmentacion", &hMax, 360, NULL);
	createTrackbar("S (Max)", "Segmentacion", &sMax, 100, NULL);
	createTrackbar("V (Max)", "Segmentacion", &vMax, 100, NULL);
	createTrackbar("Boun Box", "Bounding Box", &bounB, 50000, NULL);
	createTrackbar("Min Radius", "Bounding Box", &minR, 100, NULL);
	createTrackbar("Max Radius", "Bounding Box", &maxR, 100, NULL);
	cout << "Press 'c' to exit." << endl;
    if(capture.isOpened()){
        while( true ){
			capture.read(frame);
			cvtColor(frame, hsv, CV_RGB2HSV);
			cvtColor(frame, gray2, CV_RGB2GRAY);
			medianBlur(gray2, blur, 5);

			int hMin = getTrackbarPos("H (Min)", "Segmentacion");
			int sMin = getTrackbarPos("S (Min)", "Segmentacion");
			int vMin = getTrackbarPos("V (Min)", "Segmentacion");
			int hMax = getTrackbarPos("H (Max)", "Segmentacion");
			int sMax = getTrackbarPos("S (Max)", "Segmentacion");
			int vMax = getTrackbarPos("V (Max)", "Segmentacion");
			int bBox = getTrackbarPos("Boun Box", "Bounding Box");

			hMin = (hMin * 180)/360;
			hMax = (hMax * 180)/360;

			sMin = (sMin * 255)/100;
			sMax = (sMax * 255)/100;

			vMin = (vMin * 255)/100;
			vMax = (vMax * 255)/100;


			Mat mask;
			inRange(hsv, Scalar(hMin, sMin, vMin), Scalar(hMax, sMax, vMax), mask);
			Mat o, thresh, mor2, edges;
			vector<Vec3f> circles;
			vector<vector<Point> > contours;
  			vector<Vec4i> hierarchy;

			bitwise_and(hsv, hsv, o, mask);
			adaptiveThreshold(mask, thresh, 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY_INV,11,2);

			findContours(thresh, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
			vector<vector<Point> > contours_rec(contours.size());
			vector<Rect> boundRect( contours.size() );
			Canny(blur, edges, 300, 350);
			for (size_t contour = 0; contour < contours.size(); contour++){
				double bound = contourArea(contours[contour]);
				if (bound > bBox){
					approxPolyDP( contours[contour], contours_rec[contour], 3, true );
					boundRect[contour] = boundingRect(contours_rec[contour]);
					//printf("%.2f", bound);
					rectangle(frame, boundRect[contour].tl(), boundRect[contour].br(), Scalar(0,0,255), 2 );
					int minR = getTrackbarPos("Min Radius", "Bounding Box");
					int maxR = getTrackbarPos("Max Radius", "Bounding Box");
					Canny(thresh, edges, 125, 255);
					HoughCircles(edges, circles, CV_HOUGH_GRADIENT, 5, 2000, 50, 30, minR, maxR);
					if (circles.size() > 0){
						for(int i = 0; i < circles.size(); i++){
							Point center(round(circles[i][0]), cvRound(circles[i][1]));
            				int radius = round(circles[i][2]);
							circle( frame, center, 3, Scalar(0,255,0), -1, 8, 0 );
            				circle( frame, center, radius, Scalar(0,0,255), 3, 8, 0 );
						}
					}
				}
			}
            imshow("Segmentacion",thresh);
			imshow("Bounding Box",frame);

			// Press 'c' to escape
			int c = waitKey(10);
			if( (char)c == 'c' ) { break; }
		}
	}
	return 0;
}
