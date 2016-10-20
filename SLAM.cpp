#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <algorithm>

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

std::vector<cv::Rect> calculateROIs(cv::Mat image, cv::Size2i roiSIZE, int numROIs);

int main( int argc, const char** argv )
{
	std::string path;

	if (argc == 2) {
		path.assign(argv[1]);
		std::cout << path << std::endl;
	}
	else {
		error_message();
		return 0;
	}

	if (path.at(path.size() - 1) != '/') {
		path.append("/");
	}

	cv::namedWindow( "Depth Image" , cv::WINDOW_AUTOSIZE ); // Create a window for display.
	cv::namedWindow( "Undistorted Depth Image" , cv::WINDOW_AUTOSIZE ); // Create a window for display.
	cv::moveWindow( "Undistorted Depth Image" , 600 , 0 );

	cv::Mat image;

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

		    if(! image.data ) // Check for invalid input
		    {
		        std::cout <<  "Could not open or find the image" << std::endl;
		        cv::waitKey(1);
		        continue;
		    }

		    // cv::Mat undistortImage;

		    // cv::undistort(image, undistortImage, cameraMatrix, distortionMatrix);

			std::vector<cv::Rect> rois = calculateROIs(image, cv::Size2i(32, 32), 1);

			// Draw keypoints
			for (int i = 0; i < rois.size(); i++) {
				cv::rectangle(image, rois[i], cv::Scalar(255,255,0), 3);
			}

		    cv::imshow( "Depth Image", image );
		    // cv::imshow( "Undistorted Depth Image", undistortImage );
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
void filterDepthImage(cv::Mat image) {

}

std::vector<cv::Rect> calculateROIs(cv::Mat image, cv::Size2i roiSIZE, int numROIs) {

	std::vector<cv::Rect> roi_list;
	cv::Size matSize = image.size();

	int x = 0;
	int y = 0;

	cv::Scalar minScalar;
	cv::Rect maxRect;

	while (x < (matSize.width + 1 - roiSIZE.width)) {

		y = 0;

		while (y < (matSize.height + 1 - roiSIZE.height)) {
			cv::Rect roi(x, y, roiSIZE.width, roiSIZE.height);

			cv::Scalar s = cv::sum(image(roi));

			if (s[2] > minScalar[2]) {
				std::cout << s << std::endl;
				minScalar = s;
				maxRect = cv::Rect(x, y, roiSIZE.width, roiSIZE.height);
			}

			y += roiSIZE.height;
		}

		x += roiSIZE.width;
	}

	roi_list.push_back(maxRect);

	return roi_list;
}