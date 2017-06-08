#ifndef MAP_HPP
#define MAP_HPP

#include "opencv2/imgproc/imgproc.hpp"
// #include "opencv2/viz/vizcore.hpp"
#include "pointcloud.hpp"
#include "icp.hpp"

#define MAP_HEIGHT 300
#define PHYSICAL_HEIGHT 10.0f
#define DELTA_CONFIDENCE 25
#define MIN_CONFIDENCE 50
#define MAX_CONFIDENCE 180
#define MAX_POINT_DISTANCE 0.15f
#define MAX_KEYPOINT_ADD_DISTANCE 0.1f

#define CELL_PHYSICAL_HEIGHT PHYSICAL_HEIGHT / ((float) MAP_HEIGHT)

namespace map {
	class Map {
	public:
		cv::Point3f empty;
		icp::PointCloud mapCloud;
		cv::Point3f pointLookupTable[MAP_HEIGHT][MAP_HEIGHT][MAP_HEIGHT];
		unsigned char world[MAP_HEIGHT][MAP_HEIGHT][MAP_HEIGHT];

		Map();
		void update(std::vector< std::pair<cv::Point3f, cv::Point3f> > associations, int delta_confidencec);
		// void rayTrace(cv::Point3i point, cv::Point3i origin, cv::viz::Viz3d& depthWindow);
		void drawCertaintyMap(cv::viz::Viz3d& depthWindow);
		cv::Point3i getVoxelCoordinates(cv::Point3f);
		void update(std::vector<std::pair<cv::Point3f, cv::Point3f>> associations, std::vector<float> errors, int delta_confidence);
		void update(icp::PointCloud newCloud, int delta_confidence);
		bool isOccupied(cv::Point3f);
	};
}

#endif