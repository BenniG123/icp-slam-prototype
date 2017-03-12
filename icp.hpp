#ifndef ICP_HPP
#define ICP_HPP

#define SUBSAMPLE_FACTOR 12

#include "pointcloud.hpp"

namespace icp {
	cv::Mat getTransformation(cv::Mat& data, cv::Mat& previous, int maxIterations, float threshold, cv::viz::Viz3d& depthWindow);
	float getNearestPoint(cv::Point3f point, cv::Point3f& nearest, PointCloud& cloud);
	float meanSquareError(std::vector<float> errors);
	float distance(cv::Point3f a, cv::Point3f b);
	void showPointCloud(PointCloud p, cv::viz::Viz3d& depthWindow, cv::viz::Color color, std::string name);
	void findNearestNeighborAssociations(PointCloud& data, PointCloud& previous, std::vector<float>& errors, std::vector<std::pair<cv::Point3f, cv::Point3f>>& associations);
}

#endif