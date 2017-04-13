#ifndef MAP_HPP
#define MAP_HPP

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/viz/vizcore.hpp"
#include "pointcloud.hpp"
#include "icp.hpp"

#define MAP_HEIGHT 75
#define PHYSICAL_HEIGHT 10.0f
#define DELTA_CONFIDENCE 90
#define MIN_CONFIDENCE 100
#define MAX_CONFIDENCE 180
#define CELL_PHYSICAL_HEIGHT PHYSICAL_HEIGHT / ((float) MAP_HEIGHT)

namespace map {
	class Map {
		public:
			color_point_t empty;
			icp::PointCloud mapCloud;
			color_point_t pointLookupTable[MAP_HEIGHT][MAP_HEIGHT][MAP_HEIGHT];
			unsigned char world[MAP_HEIGHT][MAP_HEIGHT][MAP_HEIGHT];

			Map();
			void update(icp::PointCloud data, int delta_confidencec);
			void update(associations_t associations, int delta_confidencec);
			void rayTrace(cv::Point3f point, cv::Point3f position);
			void drawCertaintyMap(cv::viz::Viz3d& depthWindow);
			cv::Point3i getVoxelCoordinates(cv::Point3f);
	};
}

#endif
