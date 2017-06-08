#include "SLAM.hpp"

#include <iostream>
#include <iomanip>
#include <string>
#include <fstream>
#include <stdio.h>
#include <algorithm>
#include <vector>
#include <iterator>
#include <ctime>
#include <ratio>
#include <chrono>

#include "icp.hpp"
#include "quaternion.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/viz/vizcore.hpp"

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
	color:
		fx = 1054.35
		fy = 1054.51
		cx = 956.12
		cy = 548.99
		r1 = 0.050
		r2 = -0.062
		r3 = -0.002
		t1 = -0.002
		t2 = 0.000

	Camera Trajectory is provided at 120 Hz
	Camera Depth Catpures are provided at 30 Hz
w*/


std::chrono::high_resolution_clock::time_point start, t2;
Quaternion initialRotation;
cv::Point3f initialPosition;
logEntry myLog[10];

int main(int argc, const char** argv)
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

	cv::namedWindow("Filtered", cv::WINDOW_AUTOSIZE); // Create a window for display.
	cv::moveWindow("Filtered", 0, 700);
	// cv::namedWindow( "Normals" , cv::WINDOW_AUTOSIZE ); // Create a window for display.
	// cv::moveWindow( "Normals" , 550 , 700 );
	// cv::namedWindow( "RGB" , cv::WINDOW_AUTOSIZE ); // Create a window for display.
	// cv::moveWindow( "RGB" , 1100 , 700 );
	// cv::namedWindow("Features", cv::WINDOW_AUTOSIZE); // Create a window for display.
	// cv::moveWindow("Features", 900, 0);


	cv::viz::Viz3d depthWindow("Depth Frame");

	cv::Mat image;
	cv::Mat rgbImage;
	cv::Mat keypointsImage;
	cv::Mat filtered;
	cv::Mat colorDepth;
	cv::Mat colorFiltered;
	cv::Mat previous;
	cv::Mat normals;
	cv::Mat adjMap;

	// Camera Calibration Matrix
	float data[3][3] = { {363.58f,0.0f,250.32f}, {0.0f,363.53f,212.55f}, {0.0f,0.0f,1.0f} };

	// Correct depth frame
	cv::Mat cameraMatrix(3, 3, CV_32FC1, &data);
	cv::Mat distortionMatrix;
	cv::Mat translationPlot(500, 500, CV_8UC(3));
	cv::Mat rotation = icp::makeRotationMatrix(0, 0, 0);

	std::string line;
	std::string depth_list_file_name(path);
	std::string rgb_list_file_name(path);
	std::string ground_truth_file_name(path);

	double timestamp;

	depth_list_file_name.append("depth.txt");
	ground_truth_file_name.append("groundtruth.txt");
	rgb_list_file_name.append("rgb.txt");

	std::ifstream rgb_list_file(rgb_list_file_name.c_str());
	std::ifstream depth_list_file(depth_list_file_name.c_str());
	std::ifstream ground_truth_file(ground_truth_file_name.c_str());
	std::ofstream logfile("log.txt");

	// Euler rotation and transformation variables for groundtruth and latest scan
	Quaternion previousRotation;
	Quaternion currentRotation;
	Quaternion deltaRotation;

	Quaternion rotationQ;

	cv::Vec3f groundTruthPosition;
	cv::Vec3f previousPosition;
	cv::Vec3f currentPosition;
	cv::Vec3f deltaPosition;

	cv::Affine3f initialGroundTruthAffine;
	cv::Affine3f currentGroundTruthAffine;
	cv::Affine3f deltaGroundTruthAffine;

	// Init profiling names
	myLog[LOG_NEAREST_NEIGHBOR].name = "Nearest Neighbor";
	myLog[LOG_UI].name = "UI";
	myLog[LOG_LOAD_IMAGE].name = "Load Image";
	myLog[LOG_FILTER_IMAGE].name = "Filter Image";
	myLog[LOG_GEN_POINT_CLOUD].name = "Gen Point Cloud";
	myLog[LOG_RECONSTRUCT_POINT_CLOUDS].name = "Reconstruct Point Clouds";
	myLog[LOG_SVD].name = "SVD";
	myLog[LOG_ROTATE].name = "Rotate";
	myLog[LOG_RETRIEVE_TRANSFORM].name = "Retrieve Transform";
	myLog[LOG_MSE].name = "MSE";

	start = std::chrono::high_resolution_clock::now();
	t2 = std::chrono::high_resolution_clock::now();

	int frame_counter = 0;

	if (depth_list_file.is_open() && rgb_list_file.is_open() && logfile.is_open()) {
		if (ground_truth_file.is_open()) {
			// While their is still something to parse
			while (!depth_list_file.eof() && !rgb_list_file.eof()) {

				std::string depth_frame_file_name = getNextImageFileName(depth_list_file, path, timestamp);
				std::string rgb_frame_file_name = getNextImageFileName(rgb_list_file, path, timestamp);

				// Read next depth frame
				image = cv::imread(depth_frame_file_name, CV_LOAD_IMAGE_ANYDEPTH);   // Read the file

				// Read the next color frame
				// rgbImage = cv::imread(rgb_frame_file_name, CV_LOAD_IMAGE_GRAYSCALE);
				// cv::resize(rgbImage, rgbImage, cv::Size(960,540));

				// std::vector<cv::KeyPoint> keypoints;
				// cv::FAST(rgbImage, keypoints, 40, true, cv::FastFeatureDetector::TYPE_9_16);
				// cv::drawKeypoints(rgbImage, keypoints, keypointsImage, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

				// CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH
				// image.convertTo(image, CV_16U);

				if (!image.data) // Check for invalid input
				{
					std::cout << "Could not open or find the image" << std::endl;
					cv::waitKey(1);
					continue;
				}

				frame_counter++;

				std::cout << frame_counter << ",";

				logDeltaTime(LOG_LOAD_IMAGE);

				double min;
				double max;

				cv::Mat undistortImage = image.clone();
				cv::undistort(image, undistortImage, cameraMatrix, distortionMatrix);
				image = undistortImage;

				filtered = image.clone();

				// Filter all points > x * 5000 m away - 25000
				filterDepthImage(filtered, MAX_FILTER_DISTANCE);

				logDeltaTime(LOG_FILTER_IMAGE);

				if (previous.size().area() > 0) {
					cv::Mat transformation = icp::getTransformation(filtered, previous, rotation, NUM_ICP_ITERATIONS, 0.0001f, depthWindow);
					cv::Mat icpRotation = transformation(cv::Rect(0, 0, 3, 3));
					currentPosition = getNextGroundTruth(timestamp, ground_truth_file, currentRotation);
					currentGroundTruthAffine = cv::Affine3f(currentRotation.toRotationMatrix(), currentPosition);
					deltaGroundTruthAffine = initialGroundTruthAffine.inv() * currentGroundTruthAffine;

					// icpRotation = icpRotation * icp::makeRotationMatrix(0, 0, 180);
					deltaRotation = (initialRotation.inverse() * currentRotation).inverse();
					// rotation = rotation * icpRotation;

					float rx, ry, rz;
					// rotationQ = Quaternion(icpRotation);
					/* toEulerianAngle(icpRotation, rx, ry, rz);
					std::cout << rx << "," << ry << "," << rz;
					logfile << rx << "," << ry << "," << rz;
					toEulerianAngle(deltaRotation, rx, ry, rz);
					std::cout << "," << rx << "," << ry << "," << rz << std::endl;
					logfile << "," << rx << "," << ry << "," << rz << std::endl;
					*/

					showText(depthWindow, "Ground Truth", cv::Point(10, 30), "Ground Truth Text");
					showTransfom(depthWindow, deltaRotation, cv::Point(10, 10), "Ground Truth", logfile);
					showText(depthWindow, "ICP", cv::Point(10, 80), "ICP Text");
					showTransfom(depthWindow, icpRotation, cv::Point(10, 60), "ICP", logfile);
					logfile << std::endl;
					std::cout << std::endl;

					// TODO - FIX THIS
					
					// Show camera position
					cv::viz::WCone gtPosition(0.1f, cv::Point3f(0,0,0), cv::Point3f(0, 0, 0.2), 16, cv::viz::Color::red());
					gtPosition.applyTransform(deltaGroundTruthAffine); // , cv::Point3f(currentPosition)
					depthWindow.showWidget("Ground Truth Position", gtPosition);

					/*
					cv::viz::WCone zeroPosition(0.1, cv::Point3f(currentPosition), cv::Point3f(currentPosition) + cv::Point3f(0, 0, 0.2), 16, cv::viz::Color::green());
					gtPosition.updatePose(cv::Affine3f(deltaRotation.toRotationMatrix())); // , cv::Point3f(currentPosition)
					depthWindow.showWidget("Zero Position", zeroPosition);
					*/

					depthWindow.spinOnce(1, true);

					logDeltaTime(LOG_UI);

					previous = filtered.clone();
					previousRotation = currentRotation;

				}
				// First Frame
				else {
					previous = filtered.clone();

					// Get the initial ground truth
					initialPosition = getNextGroundTruth(timestamp, ground_truth_file, initialRotation);

					currentRotation = initialRotation;
					rotation = icp::makeRotationMatrix(0, 0, 0);
					initialGroundTruthAffine = cv::Affine3f(initialRotation.toRotationMatrix(), initialPosition);
					// rotation = icp::makeRotationMatrix(0, 0, 0);

					// Print out column names
					// std::cout << "MSE,ICP rX,ICP rY,ICP rZ,GT rX,GT rY,GT rZ" << std::endl;
					//  logfile << "MSE,ICP rX,ICP rY,ICP rZ,GT rX,GT rY,GT rZ" << std::endl;
					std::cout << "ICP rX,ICP rY,ICP rZ,GT rX,GT rY,GT rZ" << std::endl;
					logfile << "ICP rX,ICP rY,ICP rZ,GT rX,GT rY,GT rZ" << std::endl;
				}

				// cv::bitwise_not(filtered, filtered);
				cv::minMaxIdx(filtered, &min, &max);


				// expand your range to 0..255. Similar to histEq();
				filtered.convertTo(adjMap, CV_8UC1, 255 / (max - min), -min);
				cv::applyColorMap(adjMap, colorDepth, cv::COLORMAP_JET);

				cv::imshow( "Filtered", colorDepth );
				// cv::imshow("Normals", normals);
				// cv::imshow("RGB", rgbImage);
				// cv::imshow("Features", keypointsImage);
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

				// cv::waitKey(1);

				logDeltaTime(LOG_UI);
			}

			ground_truth_file.close();

		}
		else {
			errorMessage();
		}

		depth_list_file.close();
		rgb_list_file.close();

	}
	else {
		errorMessage();
	}

	return 0;
}

std::string getNextImageFileName(std::ifstream& list_file, std::string path, double& timestamp) {

	std::string line;
	getline(list_file, line);

	if (line.size() == 0) {
		std::cout << "End of file" << std::endl;
		std::cout << "Nearest Neighbor " << myLog[LOG_NEAREST_NEIGHBOR].time << " " << myLog[LOG_NEAREST_NEIGHBOR].count << " " << myLog[LOG_NEAREST_NEIGHBOR].quantity << std::endl;
		std::cout << "UI " << myLog[LOG_UI].time << " " << myLog[LOG_UI].count << " " << myLog[LOG_UI].quantity << std::endl;
		std::cout << "Load Image " << myLog[LOG_LOAD_IMAGE].time << " " << myLog[LOG_LOAD_IMAGE].count << " " << myLog[LOG_LOAD_IMAGE].quantity << std::endl;
		std::cout << "Filter Image " << myLog[LOG_FILTER_IMAGE].time << " " << myLog[LOG_FILTER_IMAGE].count << " " << myLog[LOG_FILTER_IMAGE].quantity << std::endl;
		std::cout << "Generate Point Cloud " << myLog[LOG_GEN_POINT_CLOUD].time << " " << myLog[LOG_GEN_POINT_CLOUD].count << " " << myLog[LOG_GEN_POINT_CLOUD].quantity << std::endl;
		std::cout << "Reconstruct Point Clouds " << myLog[LOG_RECONSTRUCT_POINT_CLOUDS].time << " " << myLog[LOG_RECONSTRUCT_POINT_CLOUDS].count << " " << myLog[LOG_RECONSTRUCT_POINT_CLOUDS].quantity << std::endl;
		std::cout << "SVD " << myLog[LOG_SVD].time << " " << myLog[LOG_SVD].count << " " << myLog[LOG_SVD].quantity << std::endl;
		std::cout << "Rotate " << myLog[LOG_ROTATE].time << " " << myLog[LOG_ROTATE].count << " " << myLog[LOG_ROTATE].quantity << std::endl;
		std::cout << "Retrieve Transform " << myLog[LOG_RETRIEVE_TRANSFORM].time << " " << myLog[LOG_RETRIEVE_TRANSFORM].count << " " << myLog[LOG_RETRIEVE_TRANSFORM].quantity << std::endl;
		std::cout << "MSE " << myLog[LOG_MSE].time << " " << myLog[LOG_MSE].count << " " << myLog[LOG_MSE].quantity << std::endl;
		cv::waitKey(0);
		exit(0);
	}

	// Skip comments
	while (line[0] == '#') {
		getline(list_file, line);
	}

	// Get Filename
	std::stringstream ss;
	ss.str(line);

	std::string frame_file_name(path);

	std::string file;
	std::string timestamp_string;

	// Parse next timestamp
	std::getline(ss, timestamp_string, ' ');

	timestamp = std::stod(timestamp_string);

	std::getline(ss, file, ' ');
	std::replace(file.begin(), file.end(), '\\', '/');

	// Remove whitespace
	for (int i = 0; i < file.size(); i++) {
		if (file[i] == ' ' || file[i] == '\n' || file[i] == '\r') {
			file.erase(i, 1);
		}
	}

	frame_file_name.append(file.c_str());
	return frame_file_name;
}

cv::Point3f getInitialPose(Quaternion & rotation)
{
	rotation = initialRotation;
	return initialPosition;
}

void getNormalMap(cv::Mat& image, cv::Mat& normals) {
	cv::Mat image32FC1;
	image.convertTo(image32FC1, CV_32FC1);

	for (int x = 1; x < image32FC1.rows; x++)
	{
		for (int y = 1; y < image32FC1.cols; y++)
		{

			float dzdx = (image32FC1.at<float>(x + 1, y) - image32FC1.at<float>(x - 1, y)) / 2.0f;
			float dzdy = (image32FC1.at<float>(x, y + 1) - image32FC1.at<float>(x, y - 1)) / 2.0f;

			cv::Vec3f d(-dzdx, -dzdy, 1.0f);
			cv::Vec3f n = cv::normalize(d);

			normals.at<cv::Vec3f>(x, y) = n;
		}
	}
}

cv::Point3f getNextGroundTruth(double timestamp, std::ifstream& ground_truth_file, Quaternion& rotation) {
	// Sum up the transformation and rotational components of ground truth until
	// Ground Truth timestamp would exceed depth frame timestamp

	std::string line;
	std::string temp;
	std::stringstream ss;

	double ground_truth_timestamp;

	// Get the first line
	getline(ground_truth_file, line);

	// Skip comments
	while (line[0] == '#') {
		getline(ground_truth_file, line);
	}

	ss.str(line);
	std::getline(ss, temp, ' ');
	ground_truth_timestamp = std::stod(temp);

	while (ground_truth_timestamp < timestamp) {
		// Do something with the measurements

		// Get next measurement
		getline(ground_truth_file, line);
		ss.str(line);
		std::getline(ss, temp, ' ');
		ground_truth_timestamp = std::stod(temp);
	}

	// Get translation vectors
	std::getline(ss, temp, ' ');
	float tx = std::stof(temp);
	std::getline(ss, temp, ' ');
	float ty = std::stof(temp);
	std::getline(ss, temp, ' ');
	float tz = std::stof(temp);

	float qx;
	float qy;
	float qz;
	float qw;

	// Get quaternions
	std::getline(ss, temp, ' ');
	qx = std::stof(temp);
	std::getline(ss, temp, ' ');
	qy = std::stof(temp);
	std::getline(ss, temp, ' ');
	qz = std::stof(temp);
	std::getline(ss, temp, ' ');
	qw = std::stof(temp);

	rotation = Quaternion(qw, qx, qy, qz);
	cv::Point3f position(tx, ty, tz);
	return position;
}


void logDeltaTime(int logKey, int quantity) {
	std::chrono::high_resolution_clock::time_point t = std::chrono::high_resolution_clock::now();
	auto int_us = std::chrono::duration_cast<std::chrono::microseconds>(t - t2);

	t2 = t;
	myLog[logKey].time += int_us.count();
	myLog[logKey].count++;
	myLog[logKey].quantity = quantity;

	// std::cout << myLog[logKey].name << " " << int_us.count() << std::endl;


	/*
	if (verbose) {
		std::cout << log[logKey].name << " " << int_us.count() << std::endl;
	}
	*/
}

void showText(cv::viz::Viz3d& depthWindow, std::string text, cv::Point pos, std::string name) {
	cv::viz::WText textWidget(text, pos);
	depthWindow.showWidget(name, textWidget);
}

void showTransfom(cv::viz::Viz3d& depthWindow, cv::Mat t, cv::Point pos, std::string name, std::ofstream &logfile) {
	float rx, ry, rz;
	transformationMatToEulerianAngle(t, rx, ry, rz);
	std::ostringstream ss;
	ss << rx;
	ss << " ";
	ss << ry;
	ss << " ";
	ss << rz;

	std::cout << rx << "," << ry << "," << rz << ",";
	logfile << rx << "," << ry << "," << rz << ",";

	std::string s(ss.str());
	showText(depthWindow, s, pos, name);
}

void showTransfom(cv::viz::Viz3d& depthWindow, Quaternion q, cv::Point pos, std::string name, std::ofstream &logfile) {
	float rx, ry, rz;
	toEulerianAngle(q, rx, ry, rz);
	std::ostringstream ss;
	ss << rx;
	ss << " ";
	ss << ry;
	ss << " ";
	ss << rz;
	std::string s(ss.str());

	std::cout << rx << "," << ry << "," << rz << ",";
	logfile << rx << "," << ry << "," << rz << ",";

	showText(depthWindow, s, pos, name);
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
	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,
		cv::Size(3, 3),
		cv::Point(2, 2));

	cv::erode(image, image, element);
	// cv::dilate(image, image, element);

	/*
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
	cv::Canny( image8u, detected_edges, 1.0, 5.0, 5 );

	// Subtract edges  from the image
	cv::dilate( detected_edges, detected_edges, element);
	detected_edges.convertTo(detected_edges_16u, CV_16U);
	cv::bitwise_not(detected_edges, detected_edges);
	image.copyTo(maskedImage, detected_edges);
	image = maskedImage;

	cv::dilate( image, image, element);
	cv::erode( image, image, element);
	*/

	/*
	it = image.begin<uint16_t>();
	end = image.end<uint16_t>();

	it++;

	while (it != end) {
		if ((*it) - (*(it - 1)) > 5000) {
			(*it) = 0;
		}
		it++;
	}

	/*
		it++;
	while (it != end) {
		if ((*it) - (*it2) > 100) {
			(*it2) = 0;
		}
		it++;
		it2++;
	}
	*/
	// cv::dilate( image, image, element);
}

void toEulerianAngle(Quaternion q, float& x, float& y, float& z)
{
	float ysqr = q.y * q.y;

	// roll (x-axis rotation)
	float t0 = 2.0f * (q.w * q.x + q.y * q.z);
	float t1 = 1.0f - 2.0f * (q.x * q.x + ysqr);
	x = (float)std::atan2(t0, t1);

	// pitch (y-axis rotation)
	float t2 = +2.0f * (q.w * q.y - q.z * q.x);
	t2 = t2 > 1.0f ? 1.0f : t2;
	t2 = t2 < -1.0f ? -1.0f : t2;
	y = (float)std::asin(t2);

	// yaw (z-axis rotation)
	float t3 = +2.0f * (q.w * q.z + q.x * q.y);
	float t4 = +1.0f - 2.0f * (ysqr + q.z * q.z);
	z = (float)std::atan2(t3, t4);

	x = x * 180.0f / PI;
	y = y * 180.0f / PI;
	z = z * 180.0f / PI;
}

void transformationMatToEulerianAngle(cv::Mat t, float& x, float& y, float& z) {
	float c2 = sqrt(pow(t.at<float>(0, 0), 2) + pow(t.at<float>(0, 1), 2));

	x = std::atan2(t.at<float>(1, 2), t.at<float>(2, 2));
	y = std::atan2(-t.at<float>(0, 2), c2);
	z = std::atan2(t.at<float>(0, 1), t.at<float>(0, 0));

	x = x * 180.0f / PI;
	y = y * 180.0f / PI;
	z = z * 180.0f / PI;
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
					int index = (int)std::distance(curvature_list.begin(), min_element);
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
	return (int)cv::sum(roi)[0];
}