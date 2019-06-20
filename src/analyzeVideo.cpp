#include <ros/ros.h>
#include <opencv2/opencv.hpp>
//#include <image_transport/image_transport.h>
//#include <cv_bridge/cv_bridge.h>
//#include <ackermann_msgs/AckermannDriveStamped.h>
//#include <stdio.h>
//#include <iostream>

using namespace cv;
using namespace std;

ros::WallTime tic_, toc_;
double execution_time;


int main(int argc, char** argv){
	ros::init(argc, argv, "analyzeVideo");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);


    Mat imagen;
    VideoCapture capture(argv[1]);
    int total = 0;
    while(true) {
       if (!capture.read(imagen))        
            break;
       total++;
    }
    cout << total << endl; // Cuenta correcta de Frames en el video
    capture.release();


	// READ VIDEO FILE
    VideoCapture cap(argv[1]);
    if (!cap.isOpened()){
        cout << "!!! Failed to open file: " << argv[1] << endl;
        return -1;
    }
    int currentframe = 1;
    int totalframes = cap.get(CV_CAP_PROP_FRAME_COUNT);
    cout << "Total frames to analize =" << totalframes << endl;
    waitKey(0);
    
    // ALLOCATE MEMORY FOR IMAGES
    int heigth = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
    int width  = cap.get(CV_CAP_PROP_FRAME_WIDTH);
    int cut = 180;
    cout << "(rows, cols) = " << heigth << " x "<< width << endl;
    
    Mat image_bgr = Mat(heigth, width, CV_8UC3, Scalar(0,0,0));
    Mat image_hsv = Mat(heigth, width, CV_8UC3, Scalar(0,0,0));
    Mat image_crop = Mat(heigth - cut, width, CV_8UC3, Scalar(0,0,0));
	Mat image_bw   = Mat(heigth - cut, width, CV_8UC1, Scalar(0));
	
	Mat mask_left_Initial  = Mat(heigth - cut, width, CV_8UC1, Scalar(0));
	mask_left_Initial( Range(heigth - cut - 80, heigth - cut), Range(0, 150)) = 255;
	Mat mask_right_Initial = Mat(heigth - cut, width, CV_8UC1, Scalar(0));
	mask_right_Initial(Range(heigth - cut - 80, heigth - cut), Range(width - 150, width)) = 255;
	
	Mat mask_left_Refind  = Mat(heigth - cut, width, CV_8UC1, Scalar(0));
	mask_left_Refind( Range(heigth - cut - 40, heigth - cut), Range(0, 80)) = 255;
	Mat mask_right_Refind = Mat(heigth - cut, width, CV_8UC1, Scalar(0));
	mask_right_Refind(Range(heigth - cut - 40, heigth - cut), Range(width - 80, width)) = 255;	
	
	Mat mask_left_Growing  = mask_left_Initial.clone();
	Mat mask_right_Growing = mask_right_Initial.clone();
	Mat common_mask = Mat(heigth - cut, width, CV_8UC1, Scalar(0));
	
	Mat image_lineLeft  = Mat(heigth - cut, width, CV_8UC1, Scalar(0));
	Mat image_lineRight = Mat(heigth - cut, width, CV_8UC1, Scalar(0));
	Mat image_label = Mat(heigth - cut, width, CV_8UC1, Scalar(0));
	
	Mat skel(heigth - cut, width, CV_8UC1, Scalar(0));

	Mat labels(heigth - cut, width, CV_8UC1, Scalar(0));
	
	Mat stats, centroids;
	int nLabels, the_max, index_max;
		
	double steering_angle=0.0, speed=0.0;
	int tracking_left = 2, tracking_right = 2;
	float PuntoI, PuntoD, AnguloI, AnguloD;

	int erosion_type = MORPH_ELLIPSE; //  Other options: MORPH_RECT; MORPH_CROSS;
	int erosion_size1 = 24;
	Mat element1 = getStructuringElement( erosion_type,
				   Size( 2*erosion_size1 + 1, 2*erosion_size1+1 ),
				   Point( erosion_size1, erosion_size1 ) );
	int erosion_size0 = 6;
	Mat element0 = getStructuringElement( erosion_type,
				   Size( 2*erosion_size0 + 1, 2*erosion_size0+1 ),
				   Point( erosion_size0, erosion_size0 ) );		   	

    while(n.ok()){
		
		tic_ = ros::WallTime::now();
		
        if (!cap.read(image_bgr))        
            break;

        //line(image_bgr, Point(0,cut), Point(width, cut), Scalar(255,0,0), 3, LINE_AA);
        //imshow("Original",image_bgr);
        
		// CUT THE IMAGE TO A REGION OF INTEREST
		Rect myROI(0, cut, width, heigth - cut); // myROI(x,y,w,h)
		image_crop = image_bgr(myROI);

        // CONVERT TO BLACK & WHITE
		cvtColor(image_crop, image_hsv, CV_BGR2HSV);
        inRange(image_crop, Scalar(0, 0, 0), Scalar(100, 100, 100), image_bw);
        //inRange(image_hsv , Scalar(0, 0, 0), Scalar(180, 255, 111), image_bw);
		
	    imshow("B&W", image_bw);	


		// APLICAMOS UNA MASCARA PARA CADA CARRIL
		if (tracking_left == 0){
			erode(mask_left_Growing, mask_left_Growing, element0);
			bitwise_or(mask_left_Growing, mask_left_Refind, mask_left_Growing);
		}
		if (tracking_right == 0){
			//mask_right_Growing = mask_right_Refind.clone();
			erode(mask_right_Growing, mask_right_Growing, element0);
			bitwise_or(mask_right_Growing, mask_right_Refind, mask_right_Growing);
		}
		
		bitwise_and(mask_left_Growing, mask_right_Growing, common_mask);
	
		subtract(mask_right_Growing, common_mask, mask_right_Growing);
		subtract(mask_left_Growing,  common_mask, mask_left_Growing );

		bitwise_and(image_bw, mask_left_Growing , image_lineLeft );
		bitwise_and(image_bw, mask_right_Growing, image_lineRight);
				
		imshow("mask_left_Growing", mask_left_Growing);
		imshow("mask_right_Growing", mask_right_Growing);


		// BUSCAMOS EL CARRIL IZQUIERDO
		nLabels = connectedComponentsWithStats(image_lineLeft, labels, stats, centroids, 8, CV_32S);
		//cout << endl << "Number of connected components = " << nLabels << endl << "All stats = " << stats << endl;
		the_max=0, index_max=-1;  // Find the biggest area
		if (nLabels > 1){
			for (int i=1; i<nLabels; i++){
				if (stats.at<int>(i,CC_STAT_AREA) > the_max){
					the_max = stats.at<int>(i,CC_STAT_AREA);
					index_max = i;
				}
			}
			//cout << " Bigest (area, index) = (" << the_max << ", " << index_max << ")" << endl;

			// Better way to get the skeleton of an binary image
			int inicio, num_pixels = 0;
			for (int row = 0; row < skel.rows; row++){
				bool flag = false;
				for (int col = 0; col < skel.cols; col++){
					skel.at<unsigned char>(row, col) = 0;
					if (flag == false && labels.at<unsigned char>(row,col,0) == index_max){
						inicio = col;
						flag = true;
					}
					if (flag == true && labels.at<unsigned char>(row,col,0) == 0){
						skel.at<unsigned char>(row, (inicio + col) / 2 ) = 255;
						flag = false;
						num_pixels++;
					}
				}
			}

			//imshow("Skeleton", skel);
			//waitKey(1);

			//cout << num_pixels << endl;
			if (num_pixels > 10){
				Mat FI(num_pixels, 2, CV_32F, Scalar(0));
				Mat  X(num_pixels, 1, CV_32F, Scalar(0));
				int index = 0;
				for(float y = 0; y < skel.rows; y++){
					for(float x = 0; x < skel.cols; x++){
						if (skel.at<unsigned char>(y,x) == 255){
							FI.at<float>(index,0) = y;
							FI.at<float>(index,1) = 1.0;
							X.at<float>(index,0) = x;
							index++;               
						}
					}
				}
				Mat FIT = FI.t();
				Mat Par = (FIT * FI).inv() * FIT *X;
				//cout << " Param = " << Par.at<float>(0,1) << endl;

				vector<Point2f> curvePoints;
				float x;
				Point2f new_point;
				for (float y = 0; y < heigth; y++){
					x = Par.at<float>(0,0) * y + Par.at<float>(0,1);
					new_point = Point2f(max(0.0, min(double(x), 800.0)), y);
					curvePoints.push_back(new_point); //add point to vector or list
				}
				for (int i = 0; i < curvePoints.size() - 1; i++){
					line(image_crop, curvePoints[i], curvePoints[i + 1], Scalar(0,255,0), 2, CV_AA);
				}

				//cout << "ENCONTRE linea IZQ" << endl;		
				PuntoI = Par.at<float>(0,0) * image_crop.rows + Par.at<float>(0,1);
				//cout << "PuntoI =" << PuntoI << endl;
				AnguloI = atan(-1.0/Par.at<float>(0,0))*180.0/CV_PI;
				//cout << "AnguloI = " << AnguloI << endl;
				tracking_left = 1;
				compare(labels, index_max, image_label, CMP_EQ);
				dilate(image_label, image_lineLeft, element1);
				erode(mask_left_Growing, mask_left_Growing, element0);
				bitwise_or(mask_left_Growing, image_lineLeft, mask_left_Growing);

			}
			else{
				//cout << "Perdí la linea IZQ" << endl;
				tracking_left = 0;
			}
		}
		else{
			//cout << "Perdí la linea IZQ" << endl;
			tracking_left = 0;
		}

		//imshow("Camera",image_crop);
		//waitKey(1);


		// Busquemos el carril DERECHO
		nLabels = connectedComponentsWithStats(image_lineRight, labels, stats, centroids, 8, CV_32S);
		//cout << endl << "Number of connected components = " << nLabels << endl << "All stats = " << stats << endl;
		the_max=0, index_max=-1;  // Find the biggest area
		if (nLabels > 1){
			for (int i=1; i<nLabels; i++){
				if (stats.at<int>(i,CC_STAT_AREA) > the_max){
					the_max = stats.at<int>(i,CC_STAT_AREA);
					index_max = i;
				}
			}
			//cout << " Bigest (area, index) = (" << the_max << ", " << index_max << ")" << endl;

			//compare(labels, index_max, image_label, CMP_EQ);
			// Better way to get the skeleton of an binary image
			//Mat skel(image_crop.rows, image_crop.cols, CV_8UC1, Scalar(0));
			int inicio, num_pixels = 0;
			for (int row = 0; row < skel.rows; row++){
			bool flag = false;
			for (int col = 0; col < skel.cols; col++){
				skel.at<unsigned char>(row, col) = 0;
				if (flag == false && labels.at<unsigned char>(row,col,0) == index_max){
					inicio = col;
					flag = true;
				}
				if (flag == true && labels.at<unsigned char>(row,col,0) == 0){
					skel.at<unsigned char>(row, (inicio + col) / 2 ) = 255;
					flag = false;
					num_pixels++;
				}
			}
			}
			//cout << num_pixels << endl;
			if (num_pixels > 10){
				Mat FI(num_pixels, 2, CV_32F, Scalar(0));
				Mat  X(num_pixels, 1, CV_32F, Scalar(0));
				int index = 0;
				for(float y = 0; y < skel.rows; y++){
					for(float x = 0; x < skel.cols; x++){
						if (skel.at<unsigned char>(y, x) == 255){
							FI.at<float>(index,0) = y;
							FI.at<float>(index,1) = 1.0;
							X.at<float>(index,0) = x;
							index++;               
						}
					}
				}
				Mat FIT = FI.t();
				Mat Par = (FIT * FI).inv() * FIT *X;
				//cout << " Param = " << Par.at<float>(0,1) << endl;

				vector<Point2f> curvePoints;
				float x;
				Point2f new_point;
				for (float y = 0; y < heigth; y++){
					x = Par.at<float>(0,0) * y + Par.at<float>(0,1);
					new_point = Point2f(max(0.0, min(double(x), 800.0)), y);
					curvePoints.push_back(new_point); //add point to vector or list
				}
				for (int i = 0; i < curvePoints.size() - 1; i++){
					line(image_crop, curvePoints[i], curvePoints[i + 1], Scalar(255,0,255), 2, CV_AA);
				}

				//cout << "ENCONTRE linea DER" << endl;
				PuntoD = Par.at<float>(0,0) * image_crop.rows + Par.at<float>(0,1);
				//cout << "PuntoD =" << PuntoD << endl;
				AnguloD = atan(-1.0/Par.at<float>(0,0))*180.0/CV_PI;
				//cout << "AnguloD = " << AnguloD << endl;
				tracking_right = 1;
				compare(labels, index_max, image_label, CMP_EQ);
				dilate(image_label, image_lineRight, element1);
				erode(mask_right_Growing, mask_right_Growing, element0);
				bitwise_or(mask_right_Growing, image_lineRight, mask_right_Growing);
			}
			else{
				//cout << "Perdí la linea DER" << endl;
				tracking_right = 0;
			}
		}
		else{
		 //cout << "Perdí la linea DER" << endl;
		 tracking_right = 0;
		}

		if (tracking_left == 1)
			putText(image_crop, "Found Left", cvPoint(30,30), 
				FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,255,0), 1, CV_AA);
		if (tracking_right == 1)
			putText(image_crop, "Found Right", cvPoint(width - 150,30), 
				FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(255,0,255), 1, CV_AA);
 


		imshow("Camera",image_crop);		
        int key = cvWaitKey(1);
        if (key == 27) // ESC
            break;
        if (key == 119){ // W = Wait 2sec
			cvWaitKey(2000);
            cout << "Waiting ..." << endl;
        }
        if (key == 114){ // R = Reset masks
			cvWaitKey(2000);
            cout << "Reseting masks." << endl;
			mask_right_Growing = mask_right_Initial.clone();
            mask_left_Growing  = mask_left_Initial.clone();            
        }
                ros::spinOnce();
                
                
	toc_ = ros::WallTime::now();
	execution_time = (toc_ - tic_).toNSec() * 1e-6;
	cout << "Frame " << currentframe++ << " of " <<  totalframes << "; Exectution time (ms): " << execution_time << endl;
    }
    return 0;
}
