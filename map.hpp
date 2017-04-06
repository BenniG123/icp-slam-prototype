#ifndef MAP_HPP
#define MAP_HPP

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/viz/vizcore.hpp"
#include "pointcloud.hpp"

#define MAP_HEIGHT 50
#define PHYSICAL_HEIGHT 10.0
#define DELTA_CONFIDENCE 15
#define MIN_CONFIDENCE 100
#define MAX_CONFIDENCE 180
#define CELL_PHYSICAL_HEIGHT PHYSICAL_HEIGHT / ((float) MAP_HEIGHT)

namespace map {

	static const cv::Point3f empty = cv::Point3f(-100, -100, -100);

	// The point cloud
	static icp::PointCloud mapCloud;

	// The point cloud lookup table
	static cv::Point3f pointLookupTable[MAP_HEIGHT][MAP_HEIGHT][MAP_HEIGHT];

	// The certainty grid
	static unsigned char world[MAP_HEIGHT][MAP_HEIGHT][MAP_HEIGHT];

	void init();
	void updateMap(icp::PointCloud data);
	void rayTrace(cv::Point3f point, cv::Point3f position);
	void drawCertaintyMap(cv::viz::Viz3d& depthWindow);
	cv::Point3i getVoxelCoordinates(cv::Point3f);
}

#endif
