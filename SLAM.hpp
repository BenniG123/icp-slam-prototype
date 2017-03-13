#ifndef SLAM_HPP
#define SLAM_HPP

#include "quaternion.hpp"
#include "opencv2/viz/vizcore.hpp"

void errorMessage();

void logDeltaTime();

void showText(cv::viz::Viz3d& depthWindow, std::string text, cv::Point pos, std::string name);

void transformationMatToEulerianAngle(cv::Mat t, float& x, float&y, float& z);

void toEulerianAngle(Quaternion q, float& x, float& y, float& z);

void filterDepthImage(cv::Mat &image, int maxDistance);

int curvature(cv::Mat roi);

std::vector<cv::Rect> calculateROIs(cv::Mat image, cv::Size2i roiSIZE, int numROIs, int margin);

cv::Vec3f getNextGroundTruth(double timestamp, std::ifstream& ground_truth_file, Quaternion& rotation);

#endif