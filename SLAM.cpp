#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <algorithm>
#include <vector>
#include <iterator>

/*
	depth/ir intrinsic parameters on the Kinect V2:
		fx = 363.58
		fy = 363.53
		cx = 250.32
		cy = 212.55
		r1 = 0.000
		r2 = 0.000
		r3 = 0.000
		t1 = 0.000
		t2 = 0.000
		offset = -18 mm
w*/

void error_message();

void filterDepthImage(cv::Mat &image, int maxDistance);

int curvature(cv::Mat roi);

std::vector<cv::Rect> calculateROIs(cv::Mat image, cv::Size2i roiSIZE, int numROIs, int margin);

int main( int argc, const char** argv )
{
	std::string path;
	bool paused = false;
	// TODO - Command prompt arguments

	if (argc > 1) {
		path.assign(argv[1]);
		std::cout << path << std::endl;
	}
	else {
		error_message();
		return 0;
	}

	if (argc > 2) {
			std::cout << argv[2] << std::endl;
		if (strcmp(argv[2], "-p") == 0) {
			// Begin paused
			paused = true;
		}
	}


	// Correct the path if it doesn't end in our format
	if (path.at(path.size() - 1) != '/') {
		path.append("/");
	}

	cv::namedWindow( "Original" , cv::WINDOW_AUTOSIZE ); // Create a window for display.
	cv::namedWindow( "Filtered" , cv::WINDOW_AUTOSIZE ); // Create a window for display.
	cv::moveWindow( "Filtered" , 500 , 0 );

	cv::namedWindow( "Sobel" , cv::WINDOW_AUTOSIZE ); // Create a window for display.
	cv::moveWindow( "Sobel" , 1000 , 0 );

	cv::namedWindow( "Color" , cv::WINDOW_AUTOSIZE ); // Create a window for display.
	cv::moveWindow( "Color" , 0 , 500 );

	cv::Mat image;

	// Camera Calibration Matrix
	float data[3][3] = {{363.58,0,250.32}, {0,363.53,212.55}, {0,0,1}}; 

	// Correct depth frame
	cv::Mat cameraMatrix(3, 3, CV_32FC1, &data);
	cv::Mat distortionMatrix;

	std::string line;
	std::string depth_list_file_name(path);

	depth_list_file_name.append("depth.txt");

	std::ifstream myfile (depth_list_file_name.c_str());

	if (myfile.is_open()) {
		while ( getline (myfile,line) ) {

			// Get Filename
			std::stringstream ss;
			ss.str(line);

			std::string depth_frame_file_name(path);
			std::string file;

			std::getline(ss, file, ' ');
			std::getline(ss, file, ' ');

			std::replace( file.begin(), file.end(), '\\', '/');

			// Remove whitespace
		    std::cout << file << std::endl;
		    for (int i = 0; i < file.size(); i++) {
		    	if (file[i] == ' ' || file[i] == '\n' || file[i] == '\r') {
		    		file.erase(i, 1);
		    	}
		    }

		    depth_frame_file_name.append(file.c_str());

		    std::cout << depth_frame_file_name << std::endl;

		    // Read next depth frame
		    image = cv::imread(depth_frame_file_name, CV_LOAD_IMAGE_COLOR);   // Read the file
		    // CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH
		    // image.convertTo(image, CV_16U);

		    if(! image.data ) // Check for invalid input
		    {
		        std::cout <<  "Could not open or find the image" << std::endl;
		        cv::waitKey(1);
		        continue;
		    }

		    double min;
			double max;

		    cv::Mat colorDepth;

		    cv::minMaxIdx(image, &min, &max);
			cv::Mat adjMap;

			// expand your range to 0..255. Similar to histEq();
			image.convertTo(adjMap,CV_8UC1, 255 / (max-min), -min); 

			applyColorMap(adjMap, colorDepth, cv::COLORMAP_JET);

		    cv::Mat undistortImage = image.clone();

		    cv::undistort(image, undistortImage, cameraMatrix, distortionMatrix);

		    cv::Mat filtered = image.clone();

		    filterDepthImage(filtered, 30);

		    cv::Mat sobelFilter;

		    cv::Sobel(filtered, sobelFilter, CV_8U, 1, 0, 3);

			std::vector<cv::Rect> rois = calculateROIs(sobelFilter, cv::Size2i(20, 20), 10, 40);

			// Draw rois
			for (int i = 0; i < rois.size(); i++) {
				cv::rectangle(colorDepth, rois[i], cv::Scalar(0,0,255), 3);
			}

		    cv::imshow( "Filtered", filtered );
		    cv::imshow( "Sobel", sobelFilter );
		    cv::imshow( "Original", image );
		    cv::imshow( "Color", colorDepth );

	    	if (paused) {
				cv::waitKey(0);
				paused = false;
			}
		    // cv::imshow( "Compare Images", 100 * (undistortImage - image) );

		    // TODO - The whole SLAM thing

		    cv::waitKey(33); // Wait for a keystroke in the window
		}

		myfile.close();
	}
	else {
		error_message();
	}

	return 0;
}

void error_message() {
	std::cout << "Usage: ./SLAM.exe <path to raw dataset>" << std::endl;
	std::cout << "Example: ./SLAM.exe /home/ben/Documents/D1_raw/" << std::endl;
	std::cout << "The datasets can be found at: corbs.dfki.uni-kl.de" << std::endl;
}

// Notes 
// Sensor accuracy increases quadratically with distance
// Sensor is noisy near depth discontinuities - Mask edges
void filterDepthImage(cv::Mat &image, int maxDistance) {

	cv::Size matSize = image.size();
	cv::Vec3b zero(0,0,0);

	// First erode the image to erode noisy edges
	cv::Mat erode_element = cv::getStructuringElement( cv::MORPH_RECT,
	                               cv::Size( 7, 7 ),
	                               cv::Point( 3, 3 ) );

	cv::Mat dilate_element = cv::getStructuringElement( cv::MORPH_RECT,
                               cv::Size( 7, 7 ),
                               cv::Point( 3, 3 ) );

	cv::erode( image, image, erode_element);
	cv::dilate( image, image, dilate_element);

	for ( int x = 0; x < matSize.width; x++) {
		for (int y = 0; y < matSize.height; y++) {
			// std:: cout << x << ", " << y << std::endl;
			if (image.at<cv::Vec3b>(y, x)[0] > maxDistance) {
				image.at<cv::Vec3b>(y, x) = zero;
			}
		}
	}

}

std::vector<cv::Rect> calculateROIs(cv::Mat image, cv::Size2i roiSIZE, int numROIs, int margin = 0) {

	std::vector<cv::Rect> roi_list;
	std::vector<int> curvature_list;

	cv::Size matSize = image.size();

	// Scan over the margin
	int x = margin;
	int y = margin;

	cv::Rect maxRect;

	curvature_list.clear();
	roi_list.clear();

	while (x < (matSize.width + 1 - roiSIZE.width - margin)) {

		y = 0;

		while (y < (matSize.height + 1 - roiSIZE.height - margin)) {
			cv::Rect roi(x, y, roiSIZE.width, roiSIZE.height);

			int c = curvature(image(roi));

			if (curvature_list.size() >= numROIs) {

				// If the smallest element is smaller than the new curvature, throw it out
				std::vector<int>::iterator min_element = std::min_element(curvature_list.begin(), curvature_list.end());

				if (*min_element < c) {
					int index = std::distance(curvature_list.begin(), min_element);
					roi_list.erase(roi_list.begin() + index);
					curvature_list.erase(curvature_list.begin() + index);

					curvature_list.push_back(c);
					roi_list.push_back(cv::Rect(x, y, roiSIZE.width, roiSIZE.height));
				}
			}
			else {
				curvature_list.push_back(c);
				roi_list.push_back(cv::Rect(x, y, roiSIZE.width, roiSIZE.height));
			}

			y += roiSIZE.height;
		}

		x += roiSIZE.width;
	}

	return roi_list;
}

// TODO - Actually calculate curvature
int curvature(cv::Mat roi) {

	// Step 1 - Calculate Surface Normals
	// f(x) = f(a) + f'(a)*(x - a) + Error which is a function of f''(a)
	// f' is the gradient depth function around f(x)


	// Step 2 - Estimate Curvature
	return cv::sum(roi)[0];


	/* int minDistance = roi.at<cv::Vec3b>(0, 0)[0];
	int maxDistance = roi.at<cv::Vec3b>(0, 0)[0];

	cv::Size matSize = roi.size();

	for ( int x = 0; x < matSize.width; x++) {
		for (int y = 0; y < matSize.height; y++) {
			// std:: cout << x << ", " << y << std::endl;
			// if (roi.at<cv::Vec3b>(y, x)[0] == 0) {
			//  	return -1;
			// } else
			if (roi.at<cv::Vec3b>(y, x)[0] > maxDistance) {
				maxDistance = roi.at<cv::Vec3b>(y, x)[0];
			} else if (roi.at<cv::Vec3b>(y, x)[0] < minDistance) {
				minDistance = roi.at<cv::Vec3b>(y, x)[0];
			}
		}
	}

	// std::cout << maxDistance - minDistance << std::endl;

	return maxDistance - minDistance; */
}