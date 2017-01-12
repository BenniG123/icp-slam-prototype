#ifndef ICP_HPP
#define ICP_HPP

#include "pointcloud.hpp"

namespace icp {
	cv::Mat getTransformation(cv::Mat& data, cv::Mat& previous, int maxIterations, float threshold);
	float getNearestPoint(cv::Point3i point, cv::Point3i& nearest, PointCloud cloud);
	float meanSquareError(std::vector<float> errors);
	float distance(cv::Point3i a, cv::Point3i b);
	void findNearestNeighborAssociations(PointCloud data, PointCloud previous, std::vector<float>& errors);
}

#endif