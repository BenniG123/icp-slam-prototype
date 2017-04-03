#ifndef MAP_HPP
#define MAP_HPP

#include "opencv2/imgproc/imgproc.hpp"
#include "pointcloud.hpp"

#define MAP_HEIGHT 300
#define PHYSICAL_HEIGHT 6.0f
#define DELTA_CONFIDENCE 15
#define MIN_CONFIDENCE 100
#define MAX_CONFIDENCE 180
#define CELL_PHYSICAL_HEIGHT PHYSICAL_HEIGHT / MAP_HEIGHT

namespace map {
	void init();
	void updateMap(icp::PointCloud data, cv::Point3f position, cv::Mat rotation);
	void rayTrace(cv::Point3f point, cv::Point3f position);
}

#endif
