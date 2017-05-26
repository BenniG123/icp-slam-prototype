#ifndef ICP_HPP
#define ICP_HPP

#define SUBSAMPLE_FACTOR 12
#define PI 3.14159265358979f

#include "pointcloud.hpp"
// #include "opencv2/viz/vizcore.hpp"

namespace icp {
	cv::Mat makeRotationMatrix(float x, float y, float z);
	cv::Mat getTransformation(cv::Mat& data, cv::Mat& previous, cv::Mat& rotation, int maxIterations, float threshold); // cv::viz::Viz3d& depthWindow
	float getNearestPoint(cv::Point3f point, cv::Point3f& nearest, PointCloud& cloud);
	float meanSquareError(std::vector<float> errors);
	cv::Point3f calculateOffset(std::vector< std::pair <cv::Point3f, cv::Point3f> > associations);
	float distance(cv::Point3f a, cv::Point3f b);
	void findNearestNeighborAssociations(PointCloud& data, PointCloud& previous, std::vector<float>& errors, std::vector<std::pair<cv::Point3f, cv::Point3f> >& associations);
}

#endif