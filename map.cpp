#include <iostream>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/viz/vizcore.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "map.hpp"
#include "icp.hpp"
#include "pointcloud.hpp"

namespace map {
	// 6x6x6 meter space with 2 cm block
	unsigned char world[MAP_HEIGHT][MAP_HEIGHT][MAP_HEIGHT]; // = {{{100}}};
	icp::PointCloud mapCloud;
	
	void init() {
		for (int i = 0; i < MAP_HEIGHT; i++) {
			for (int j = 0; j < MAP_HEIGHT; j++) {
				for (int k = 0; k < MAP_HEIGHT; k++) {
					world[i][j][k] = 0;
				}
			}
		}
	}

	void updateMap(icp::PointCloud data, cv::Point3f position, cv::Mat rotation) {
		data.rotate(rotation);
		data.translate(position);

		std::vector<cv::Point3f>::iterator it, end;
		it = data.points.begin();
		end = data.points.end();

		while (it != end) {
			rayTrace(*it, position);
			it++;
		}
	}

	// Algorithm from "A Fast Voxel Traversal Algorithm for Ray Tracing"
	void rayTrace(cv::Point3f point, cv::Point3f origin) {
		// Point Voxel Coordinates
		int x, y, z;

		x = point.x / CELL_PHYSICAL_HEIGHT;
		y = point.y / CELL_PHYSICAL_HEIGHT;
		z = point.z / CELL_PHYSICAL_HEIGHT;

		world[x][y][z] += DELTA_CONFIDENCE;
		if (world[x][y][z] > 255) {
			world[x][y][z] = 255;
		}

		// Origin Voxel Coordinates
		int ox, oy, oz;

		ox = origin.x / CELL_PHYSICAL_HEIGHT;
		oy = origin.y / CELL_PHYSICAL_HEIGHT;
		oz = origin.z / CELL_PHYSICAL_HEIGHT;

		// Slope of voxel travel
		float mx, my, mz;

		mx = origin.x - point.x;
		my = origin.y - point.y;
		mz = origin.z - point.z;

		// Normalize slope
		float magnitude = sqrt(pow(mx, 2) + pow(my, 2) + pow(mz, 2));
		mx /= magnitude;
		my /= magnitude;
		mz /= magnitude;

		// Direction of voxel stepping
		int stepX, stepY, stepZ;

		if (x < ox) {
			stepX = 1;
		} else if (x == ox) {
			stepX = 0;
		} else {
			stepX = -1;
		}

		if (y < oy) {
			stepY = 1;
		} else if (y == oy) {
			stepY = 0;
		} else {
			stepY = -1;
		}

		if (z < oz) {
			stepZ = 1;
		} else if (z == oz) {
			stepZ = 0;
		} else {
			stepZ = -1;
		}

		// How far along the ray must we move for the horizontal component
		// of such a movement to equal the width of a voxel
		float tDeltaX, tDeltaY, tDeltaZ;

		if (mx != 0)
			tDeltaX = 1 / mx;

		if (my != 0)
			tDeltaY = 1 / my;

		if (mz != 0)
			tDeltaZ = 1 / mz;

		// How far can we travel along the ray and stay in the first voxel row
		float tMaxX, tMaxY, tMaxZ;

		// TODO - This is wrong
		// The inter voxel distance between point and voxel edge
		float interVoxelX = point.x / CELL_PHYSICAL_HEIGHT - ((float) x);
		float interVoxelT = (1 - interVoxelX) / mx;

		tMaxX = tDeltaX - point.x / CELL_PHYSICAL_HEIGHT - ((float) x);
		tMaxY = tDeltaY - point.y / CELL_PHYSICAL_HEIGHT - ((float) y);
		tMaxZ = tDeltaZ - point.z / CELL_PHYSICAL_HEIGHT - ((float) z);

		do {
			if (tMaxX < tMaxY) {
				if (tMaxX < tMaxZ) {
					x += stepX;
					if (x < 0 || x > MAP_HEIGHT) {
						std::cout << "Point out of bounds: " << cv::Point3i(x,y,z) << std::endl;
						break;
					}
					tMaxX += tDeltaX;
				}
				else {
					z += stepZ;
					if (z < 0 || z > MAP_HEIGHT) {
						std::cout << "Point out of bounds: " << cv::Point3i(x,y,z) << std::endl;
						break;
					}
					tMaxZ += tDeltaZ;
				}
			}
			else {
				if (tMaxY < tMaxZ) {
					y += stepY;
					if (y < 0 || y > MAP_HEIGHT) {
						std::cout << "Point out of bounds: " << cv::Point3i(x,y,z) << std::endl;
						break;
					}
					tMaxY += tDeltaY;
				}
				else {
					z += stepZ;
					if (z < 0 || z > MAP_HEIGHT) {
						std::cout << "Point out of bounds: " << cv::Point3i(x,y,z) << std::endl;
						break;
					}
					tMaxZ += tDeltaZ;
				}
			}

			// Process Voxel Here - Reduce confidence in empty cells
			if (world[x][y][z] > 0) {
				world[x][y][z] -= DELTA_CONFIDENCE;
				if (world[x][y][z] < 0) {
					world[x][y][z] = 0;
				}
			}

		} while (x != ox && y != oy && z != oz);

	}

	void getPoints(cv::Mat& depthMap, cv::Point3i& position, cv::Mat& rotation) {
		// Foreach space between camera and raycast, increase value.

		// Foreach space behind raycast, decrease value.
	}
}