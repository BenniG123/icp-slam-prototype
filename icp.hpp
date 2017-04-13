#ifndef ICP_HPP
#define ICP_HPP

#define SUBSAMPLE_FACTOR 12
#define PI 3.14159265358979
#define MAX_NN_POINT_DISTANCE 1.0f
#define MAX_NN_COLOR_DISTANCE 2.0f

#include "pointcloud.hpp"

typedef std::vector<std::pair<color_point_t, color_point_t>> associations_t;

namespace icp {
	cv::Mat makeRotationMatrix(float x, float y, float z);
	void processVoxel(color_point_t point, color_point_t &nearest, float& shortestDistance, int x, int y, int z);
	cv::Mat getTransformation(cv::Mat& data, cv::Mat& previous, cv::Mat color, cv::Mat& rotation, int maxIterations, float threshold, cv::viz::Viz3d& depthWindow);
	float getNearestPoint(color_point_t point, color_point_t nearest, PointCloud& cloud);
	float meanSquareError(std::vector<float> errors);
	cv::Point3f calculateOffset(associations_t associations);
	float distance(cv::Point3f a, cv::Point3f b);
	float distance(color_point_t a, color_point_t b);
	void findNearestNeighborAssociations(PointCloud& data, PointCloud& previous, std::vector<float>& errors, associations_t associations);
	float getNearestMappedPoint(color_point_t point, color_point_t &nearest);
	void findMappedNearestNeighborAssociations(PointCloud& data, std::vector<float>& errors, associations_t& associations);
}

#endif