#ifndef ICP_HPP
#define ICP_HPP

#define SUBSAMPLE_FACTOR 12
#define PI 3.14159265358979f
#define MAX_NN_POINT_DISTANCE 1.5f
#define COLOR_WEIGHT 0.0f
#define DISTANCE_WEIGHT 1.0f - COLOR_WEIGHT
#define MAX_NN_COLOR_DISTANCE 0.75f
#define MIN_NN_COLOR_DISTANCE 0.2f
#define MIN_ASSOCIATION_DRAW_DISTANCE 0.3f
// The longest point association distance that can be factored into translation
#define MAX_TRANSLATION_NN_DISTANCE 0.3f
#define MAX_TRANSLATION_DISTANCE 3.5f

#include "pointcloud.hpp"

typedef std::vector<std::pair<color_point_t, color_point_t>> associations_t;

namespace icp {
	cv::Mat makeRotationMatrix(float x, float y, float z);
	void processVoxel(color_point_t point, color_point_t &nearest, float& shortestDistance, int x, int y, int z);
	cv::Mat getTransformation(cv::Mat& data, cv::Mat& previous, cv::Mat color, cv::Mat& rotation, int maxIterations, float threshold, cv::viz::Viz3d& depthWindow);
	float getNearestPoint(color_point_t point, color_point_t& nearest, PointCloud& cloud);
	float meanSquareError(std::vector<float> errors);
	void showAssocations(associations_t associations, std::vector<float> errors, cv::viz::Viz3d& depthWindow);
	cv::Point3f calculateOffset(associations_t associations);
	float distance(cv::Point3f a, cv::Point3f b);
	float distance(color_point_t a, color_point_t b);
	void findNearestNeighborAssociations(PointCloud& data, PointCloud& previous, std::vector<float>& errors, associations_t& associations);
	float getNearestMappedPoint(color_point_t point, color_point_t &nearest);
	void findMappedNearestNeighborAssociations(PointCloud& data, std::vector<float>& errors, associations_t& associations);
}

#endif