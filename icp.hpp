#ifndef ICP_HPP
#define ICP_HPP

#define PI 3.14159265358979f
#define MAX_NN_POINT_DISTANCE 1.5f
#define COLOR_WEIGHT 0.0f
#define DISTANCE_WEIGHT 1.0f - COLOR_WEIGHT
#define MAX_NN_COLOR_DISTANCE 0.75f

#define MAX_NN_KEYPOINT_DISTANCE 0.1f
#define MIN_NN_COLOR_DISTANCE 0.2f

#define MIN_ASSOCIATION_DRAW_DISTANCE 0.05f
// The longest point association distance that can be factored into translation
#define MAX_TRANSLATION_NN_DISTANCE 0.3f
#define MAX_TRANSLATION_DISTANCE 3.5f

#include "pointcloud.hpp"

typedef std::vector<std::pair<color_point_t, color_point_t>> associations_t;

namespace icp {
	cv::Mat getTransformation(cv::Mat& data, cv::Mat& previous, cv::Mat color, std::vector<cv::KeyPoint> keypoints, cv::Mat& rotation, int maxIterations, float threshold, cv::viz::Viz3d& depthWindow);

	cv::Mat makeRotationMatrix(float x, float y, float z);

	float meanSquareError(std::vector<float> errors);

	void showAssocations(associations_t associations, std::vector<float> errors, cv::viz::Viz3d& depthWindow);

	cv::Point3f calculateOffset(associations_t associations);

	float distance(cv::Point3f a, cv::Point3f b);
	float distance(color_point_t a, color_point_t b);

	void findGlobalNearestNeighborAssociations(PointCloud& data, PointCloud& previous, std::vector<float>& errors, associations_t& associations);
	void findGlobalKeyPointAssociations(PointCloud& data, std::vector<float>& errors, associations_t& associations, point_list_t& nonAssociations);
	void findMappedNearestNeighborAssociations(PointCloud& data, std::vector<float>& errors, associations_t& associations);

	void processVoxel(color_point_t point, color_point_t &nearest, float& shortestDistance, int x, int y, int z);
	float getNearestMappedPoint(color_point_t point, color_point_t &nearest);
	float getNearestPoint(color_point_t point, color_point_t& nearest, PointCloud& cloud);
	float getNearestKeyPoint(color_point_t point, color_point_t& nearest, PointCloud& cloud);

}

#endif