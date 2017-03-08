#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/viz/vizcore.hpp"
#include "icp.hpp"

// #include "boost/program_options.hpp"

#include <iostream>
#include <string>
#include <fstream>
#include <stdio.h>
#include <algorithm>
#include <vector>
#include <iterator>
#include <boost/circular_buffer.hpp>

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

	Camera Trajectory is provided at 120 Hz
	Camera Depth Catpures are provided at 30 Hz
w*/

void errorMessage();

void showText(cv::viz::Viz3d& depthWindow, std::string text, cv::Point pos, std::string name);

cv::Mat makeRotationMatrix(float x, float y, float z);

void transformationMatToEulerianAngle(cv::Mat t, float& x, float&y, float& z);

void toEulerianAngle(float qw, float qx, float qy, float qz, float& x, float& y, float& z);

void filterDepthImage(cv::Mat &image, int maxDistance);

int curvature(cv::Mat roi);

std::vector<cv::Rect> calculateROIs(cv::Mat image, cv::Size2i roiSIZE, int numROIs, int margin);

cv::Mat getNextGroundTruth(double timestamp, std::ifstream& ground_truth_file);

float PI;

int main( int argc, const char** argv )
{
	std::string path;

	// Abbreviation for program options
	/* namespace po = boost::program_options;
	po::options_description desc("Options"); 
	desc.add_options()
    	("help", "produce help message")
    	("compression", po::value<int>(), "set compression level");
	*/

	bool paused = false;

	if (argc > 1) {
		path.assign(argv[1]);
	}
	else {
		errorMessage();
		return 0;
	}

	if (argc > 2) {
		if (strcmp(argv[2], "-p") == 0) {
			// Begin paused
			paused = true;
		}
	}


	// Correct the path if it doesn't end in our format
	if (path.at(path.size() - 1) != '/') {
		path.append("/");
	}

	/* 
	cv::namedWindow( "Original" , cv::WINDOW_AUTOSIZE ); // Create a window for display.

	cv::namedWindow( "Color" , cv::WINDOW_AUTOSIZE ); // Create a window for display.
	cv::moveWindow( "Color" , 0 , 500 );

	cv::namedWindow( "Translation" , cv::WINDOW_AUTOSIZE ); // Create a window for display.
	cv::moveWindow( "Translation" , 500 , 500 );

	cv::namedWindow( "Previous" , cv::WINDOW_AUTOSIZE ); // Create a window for display.
	cv::moveWindow( "Previous" , 1000 , 500 );

	cv::namedWindow( "Sobel" , cv::WINDOW_AUTOSIZE ); // Create a window for display.
	cv::moveWindow( "Sobel" , 500 , 700 );
	*/

	cv::namedWindow( "Filtered" , cv::WINDOW_AUTOSIZE ); // Create a window for display.
	cv::moveWindow( "Filtered" , 0 , 700 );

	cv::viz::Viz3d depthWindow("Depth Frame");

	cv::Mat image;
	cv::Mat filtered;
	// For sub sampling pixels
	cv::Mat image_sampled;
	cv::Mat colorDepth;
	cv::Mat colorFiltered;

	cv::Mat previous;
	cv::Mat previous_sampled;

	// Camera Calibration Matrix
	float data[3][3] = {{363.58,0,250.32}, {0,363.53,212.55}, {0,0,1}}; 

	// Correct depth frame
	cv::Mat cameraMatrix(3, 3, CV_32FC1, &data);
	cv::Mat distortionMatrix;
	cv::Mat translationPlot(500, 500, CV_8UC(3));
	cv::Mat initialPosition;
	boost::circular_buffer<cv::Mat> transformationBuffer{33};

	std::string line;
	std::string depth_list_file_name(path);
	std::string ground_truth_file_name(path);

	double timestamp;

	depth_list_file_name.append("depth.txt");
	ground_truth_file_name.append("groundtruth.txt");

	std::ifstream depth_list_file(depth_list_file_name.c_str());
	std::ifstream ground_truth_file(ground_truth_file_name.c_str());

	const int subsample_factor = 6;
	const int subsample_width = 512 / subsample_factor;
	const int subsample_height = 424 / subsample_factor;

	PI = (float) 2*std::acos(0.0);

	if (depth_list_file.is_open()) {
		if (ground_truth_file.is_open()) {
			while ( getline (depth_list_file, line) ) {

				// Skip comments
				while (line[0] == '#') {
					getline (depth_list_file, line);
				}

				// Get Filename
				std::stringstream ss;
				ss.str(line);

				std::string depth_frame_file_name(path);
				std::string file;
				std::string timestamp_string;

				// Parse next timestamp
				std::getline(ss, timestamp_string, ' ');
				timestamp = std::stod(timestamp_string);

				std::getline(ss, file, ' ');

				std::replace( file.begin(), file.end(), '\\', '/');

				// Remove whitespace
			    for (int i = 0; i < file.size(); i++) {
			    	if (file[i] == ' ' || file[i] == '\n' || file[i] == '\r') {
			    		file.erase(i, 1);
			    	}
			    }

			    depth_frame_file_name.append(file.c_str());

			    // Read next depth frame
			    image = cv::imread(depth_frame_file_name, CV_LOAD_IMAGE_ANYDEPTH);   // Read the fileW

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

			    cv::Mat undistortImage = image.clone();
			    cv::undistort(image, undistortImage, cameraMatrix, distortionMatrix);

			    filtered = image.clone();

			    // Filter all points > x * 5000 m away
			    filterDepthImage(filtered, 8500);

				// std::vector<cv::Rect> rois = calculateROIs(sobelFilter, cv::Size2i(20, 20), 10, 40);

				// Draw rois
				// for (int i = 0; i < rois.size(); i++) {
				// 	cv::rectangle(colorDepth, rois[i], cv::Scalar(0,0,255), 3);
				// }

				// Get the world origin and rotation so we can compute delta
				if (transformationBuffer.size() == 0) {
					cv::Mat groundTruth = getNextGroundTruth(timestamp, ground_truth_file);
					groundTruth.copyTo(initialPosition);
				}

				if (previous.size().area() > 0) {
					resize(filtered, image_sampled, cv::Size(subsample_width, subsample_height));
					resize(previous, previous_sampled, cv::Size(subsample_width, subsample_height));

				    // cv::Mat transformation = icp::getTransformation(image, image, 10, 10.0);
					cv::Mat transformation = icp::getTransformation(image_sampled, previous_sampled, 8, 0.0001, depthWindow);
					cv::Mat groundTruth = getNextGroundTruth(timestamp, ground_truth_file);

					
					float x, y, z;
					transformationMatToEulerianAngle(transformation, x, y, z);
					std::cout << "X: " << x << std::endl;
					std::cout << "Y: " << y << std::endl;
					std::cout << "Z: " << z << std::endl;

					if (transformation.at<float>(2,2) > 0) {
						previous = filtered.clone();
					}

					// cv::subtract(groundTruth, initialPosition, groundTruth);
					// std::cout << "Ground Truth:" << std::endl << groundTruth << std::endl;
					// std::cout << "Transformation:" << std::endl << transformation << std::endl;
					transformationBuffer.push_back(groundTruth);

					int i = 0;
					for (cv::Mat mat : transformationBuffer) {
						cv:circle(translationPlot, cv::Point(((int) (mat.at<float>(0,0) * 125) + 250), (int) (250 - mat.at<float>(0,2) * 125)), mat.at<float>(0,1) * 5 + 5, cv::Scalar(255, i * 5, i * 5), -1);
						i++;
					}
				}
				else {
					previous = filtered.clone();
				}

				// cv::bitwise_not(filtered, filtered);
				cv::minMaxIdx(filtered, &min, &max);
				cv::Mat adjMap;

				// expand your range to 0..255. Similar to histEq();
				filtered.convertTo(adjMap, CV_8UC1, 255 / (max-min), -min); 
				applyColorMap(adjMap, colorDepth, cv::COLORMAP_JET);

			    cv::imshow( "Filtered", colorDepth );
			    // cv::imshow( "Sobel", sobelFilter );
			    // cv::imshow( "Original", image );
			    // cv::imshow( "Color", colorDepth );
			    // cv::imshow( "Color Filtered", colorFiltered );
			    // cv::imshow( "Translation", translationPlot );


			    // depthWindow.spinOnce(33, true);

		    	if (paused) {
					cv::waitKey(0);
					paused = false;
				}
			    // cv::imshow( "Compare Images", 100 * (undistortImage - image) );

			    // TODO - The whole SLAM thing

			    cv::waitKey(33);
			}

			ground_truth_file.close();

		} else {
			errorMessage();
		}

		depth_list_file.close();

	} else {
		errorMessage();
	}

	return 0;
}

cv::Mat getNextGroundTruth(double timestamp, std::ifstream& ground_truth_file) {
	// Sum up the transformation and rotational components of ground truth until
	// Ground Truth timestamp would exceed depth frame timestamp

	std::string line;
	std::string temp;
	std::stringstream ss;

	double ground_truth_timestamp;

    // Get the first line
	getline (ground_truth_file, line);

	// Skip comments
	while (line[0] == '#') {
		getline (ground_truth_file, line);
	}

	ss.str(line);
	std::getline(ss, temp, ' ');
	ground_truth_timestamp = std::stod(temp);

    while (ground_truth_timestamp < timestamp) {
    	// Do something with the measurements

		// Get next measurement
		getline (ground_truth_file, line);
		ss.str(line);
		std::getline(ss, temp, ' ');
		ground_truth_timestamp = std::stod(temp);

		// std::cout << ground_truth_timestamp << ", " << timestamp << std::endl;
    }

    // Translation vectors
    float tx;
    float ty;
    float tz;

    std::getline(ss, temp, ' ');
    tx = std::stof(temp);
    // transformation.at<float>(0,3) = tx;
    std::getline(ss, temp, ' ');
    ty = std::stof(temp);
    // transformation.at<float>(1,3) = ty;
    std::getline(ss, temp, ' ');
    tz = std::stof(temp);
    // transformation.at<float>(2,3) = tz;

    float qx;
    float qy;
    float qz;
    float qw;

    std::getline(ss, temp, ' ');
    qx = std::stof(temp);
    std::getline(ss, temp, ' ');
    qy = std::stof(temp);
    std::getline(ss, temp, ' ');
    qz = std::stof(temp);
    std::getline(ss, temp, ' ');
    qw = std::stof(temp);

    float x, y, z;

    // Get X, Y, Z rotation
    toEulerianAngle(qw, qx, qy, qz, x, y, z);
	std::cout << "X: " << x << std::endl;
    std::cout << "Y: " << y << std::endl;
    std::cout << "Z: " << z << std::endl;

    float rot_data[3][3] = {{1 - 2*qy*qy - 2*qz*qz, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw},
    					   {2*qx*qy + 2*qz*qw, 1 - 2*qx*qx - 2*qz*qz, 2*qy*qz - 2*qx*qw},
    					   {2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx*qx - 2*qy*qy}};

	cv::Mat transformation(4, 4, CV_32FC1, &rot_data);
	transformation.at<float>(0,3) = tx;
	transformation.at<float>(1,3) = ty;
	transformation.at<float>(2,3) = tz;

    // TODO - Calculate delta instead of absolute

	return transformation;
}

cv::Mat makeRotationMatrix(float x, float y, float z) {
	double rotX = x * PI / 180;
	double rotY = y * PI / 180;
	double rotZ = z * PI / 180;

	float d[3][3] = {{1, 0, 0}, {0, (float) cos(rotX), (float) sin(rotX)}, {0, (float) -sin(rotX),(float) cos(rotX)}};
	float f[3][3] = {{(float) cos(rotY), 0, (float) -sin(rotY)}, {0, 1, 0}, {(float) sin(rotY), 0,(float) cos(rotY)}};
	float g[3][3] = {{(float) cos(rotZ), (float) sin(rotZ), 0}, {(float) -sin(rotZ), (float) cos(rotZ), 0}, {0, 0, 1}};
	cv::Mat a(3, 3, CV_32FC1, &d);
	cv::Mat b(3, 3, CV_32FC1, &f);
	cv::Mat c(3, 3, CV_32FC1, &g);

	return a * b * c;
}

void showText(cv::viz::Viz3d& depthWindow, std::string text, cv::Point pos, std::string name) {
	cv::viz::WText textWidget(text, pos);
	depthWindow.showWidget( name , textWidget);
}

void errorMessage() {
	std::cout << "Usage: ./SLAM.exe <path to raw dataset>" << std::endl;
	std::cout << "Example: ./SLAM.exe /home/ben/Documents/D1_raw/" << std::endl;
	std::cout << "The datasets can be found at: corbs.dfki.uni-kl.de" << std::endl;
	std::cout << "Make sure the file groundtruth.txt is located in the root of the provided directory." << std::endl;
}

// Notes 
// Sensor accuracy increases quadratically with distance
// Sensor is noisy near depth discontinuities - Mask edges
void filterDepthImage(cv::Mat &image, int maxDistance) {
	cv::MatIterator_<uint16_t> it, end;
	it = image.begin<uint16_t>();
	end = image.end<uint16_t>();

	while (it != end) {
		if ((*it) > maxDistance) {
			(*it) = 0;
		}
		it++;
	}

  	// Using Canny's output as a mask, we display our result
	cv::Mat element = cv::getStructuringElement( cv::MORPH_RECT,
	                               cv::Size( 7, 7 ),
	                               cv::Point( 4, 4 ) );
	cv::Mat image8u;
	cv::Mat detected_edges;
	cv::Mat maskedImage;

	cv::Mat detected_edges_16u;
	image.convertTo(image8u, CV_8U);

	// Canny detector for edges
  	cv::Canny( image8u, detected_edges, 1.0, 5.0, 3 );

  	// Subtract edges  from the image
	cv::dilate( detected_edges, detected_edges, element);
	detected_edges.convertTo(detected_edges_16u, CV_16U);
	cv::bitwise_not(detected_edges, detected_edges);
 	image.copyTo(maskedImage, detected_edges);
 	image = maskedImage;

 	// cv::dilate( image, image, element);
 	// cv::dilate( image, image, element);
}

void toEulerianAngle(float qw, float qx, float qy, float qz, float& x, float& y, float& z)
{
	float ysqr = qy * qy;

	// roll (x-axis rotation)
	float t0 = 2.0 * (qw * qx + qy * qz);
	float t1 = 1.0 - 2.0 * (qx * qx + ysqr);
	x = (float) std::atan2(t0, t1);

	// pitch (y-axis rotation)
	float t2 = +2.0 * (qw * qy - qz * qx);
	t2 = t2 > 1.0 ? 1.0 : t2;
	t2 = t2 < -1.0 ? -1.0 : t2;
	y = (float) std::asin(t2);

	// yaw (z-axis rotation)
	float t3 = +2.0 * (qw * qz + qx * qy);
	float t4 = +1.0 - 2.0 * (ysqr + qz * qz);  
	z = (float) std::atan2(t3, t4);

	x = x * 180 / PI;
	y = y * 180 / PI;
	z = z * 180 / PI;
}

void transformationMatToEulerianAngle(cv::Mat t, float& x, float&y, float& z) {
    float c2 = sqrt(pow(t.at<float>(0,0), 2) + pow(t.at<float>(0,1), 2));

	x = std::atan2(t.at<float>(1,2), t.at<float>(2,2));
    y = std::atan2(-t.at<float>(0,2), c2);
    z = std::atan2(t.at<float>(0,1),t.at<float>(0,0));

    x = x * 180 / PI;
	y = y * 180 / PI;
	z = z * 180 / PI;
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