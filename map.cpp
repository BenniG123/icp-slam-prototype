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
			// rayTrace(*it, position);
			it++;
		}
	}

	// Algorithm from "A Fast Voxel Traversal Algorithm for Ray Tracing"
	void rayTrace(cv::Point3i point, cv::Point3i origin, cv::viz::Viz3d& depthWindow) {
		// Point Voxel Coordinates
		int x, y, z;

		x = point.x;
		y = point.y;
		z = point.z;

		// Origin Voxel Coordinates
		int ox, oy, oz;

		ox = origin.x;
		oy = origin.y;
		oz = origin.z;

		cv::viz::WSphere originWidget(cv::Vec3d(ox, oy, oz), 0.1, 10, cv::viz::Color::yellow());
		depthWindow.showWidget("origin", originWidget);

		// Slope of voxel travel
		float mx, my, mz;

		mx = (float)x - ox;
		my = (float)y - oy;
		mz = (float)z - oz;

		// Normalize slope
		float magnitude = sqrt(pow(mx, 2) + pow(my, 2) + pow(mz, 2));
		mx /= magnitude;
		my /= magnitude;
		mz /= magnitude;

		// Direction of voxel stepping
		int stepX, stepY, stepZ;

		if (ox < x) {
			stepX = 1;
		}
		else if (x == ox) {
			stepX = 0;
		}
		else {
			stepX = -1;
		}

		if (oy < y) {
			stepY = 1;
		}
		else if (y == oy) {
			stepY = 0;
		}
		else {
			stepY = -1;
		}

		if (oz < z) {
			stepZ = 1;
		}
		else if (z == oz) {
			stepZ = 0;
		}
		else {
			stepZ = -1;
		}

		// How far along the ray must we move for the horizontal component
		// of such a movement to equal the width of a voxel
		float tDeltaX, tDeltaY, tDeltaZ = 0;

		if (mx != 0)
			tDeltaX = stepX / mx;

		if (my != 0)
			tDeltaY = stepY / my;

		if (mz != 0)
			tDeltaZ = stepZ / mz;

		// How far can we travel along the ray and stay in the first voxel row
		float tMaxX, tMaxY, tMaxZ;
		tMaxX = 1 / mx;
		tMaxY = 1 / my;
		tMaxZ = 1 / mz;

		// std::cout << "tMaxX: " << tMaxX << std::endl;
		// std::cout << "tMaxY: " << tMaxY << std::endl;
		// std::cout << "tMaxZ: " << tMaxZ << std::endl;

		// tMaxX = bound(ox, mx);
		// tMaxY = bound(oy, my);
		// tMaxZ = bound(oz, mz);

		// TODO - This is wrong
		// The inter voxel distance between point and voxel edge
		/* float interVoxelX = point.x / CELL_PHYSICAL_HEIGHT - ((float) x);
		float interVoxelY = point.y / CELL_PHYSICAL_HEIGHT - ((float) y);
		float interVoxelZ = point.z / CELL_PHYSICAL_HEIGHT - ((float) z);

		float interVoxelT = (1 - interVoxelX) / mx;

		tMaxX = ( tDeltaX - point.x ) / CELL_PHYSICAL_HEIGHT - ((float) x);
		tMaxY = ( tDeltaY - point.y ) / CELL_PHYSICAL_HEIGHT - ((float) y);
		tMaxZ = ( tDeltaZ - point.z ) / CELL_PHYSICAL_HEIGHT - ((float) z);
		*/

		int dx = x;
		int dy = y;
		int dz = z;

		x = ox;
		y = oy;
		z = oz;

		do {
			if (tMaxX < tMaxY) {
				if (tMaxX < tMaxZ) {
					x += stepX;
					if (x < 0 || x > MAP_HEIGHT) {
						std::cout << "X: Point out of bounds: " << cv::Point3i(x, y, z) << std::endl;
						break;
					}
					tMaxX += tDeltaX;
				}
				else {
					z += stepZ;
					if (z < 0 || z > MAP_HEIGHT) {
						std::cout << "Z1: Point out of bounds: " << cv::Point3i(x, y, z) << std::endl;
						break;
					}
					tMaxZ += tDeltaZ;
				}
			}
			else {
				if (tMaxY < tMaxZ) {
					y += stepY;
					if (y < 0 || y > MAP_HEIGHT) {
						std::cout << "Y: Point out of bounds: " << cv::Point3i(x, y, z) << std::endl;
						break;
					}
					tMaxY += tDeltaY;
				}
				else {
					z += stepZ;
					if (z < 0 || z > MAP_HEIGHT) {
						std::cout << "Z2: Point out of bounds: " << cv::Point3i(x, y, z) << std::endl;
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

			// cv::viz::WCube cubeWidget(cv::Vec3d(x, y, z), cv::Vec3d(x+1, y+1, z+1), 
			//	true, cv::viz::Color(cv::Scalar(255, 127, world[x][y][z])));

			// depthWindow.showWidget( "rayTrace"  + std::to_string(x + y * MAP_HEIGHT + z * MAP_HEIGHT * MAP_HEIGHT), cubeWidget);

		} while (x != dx || y != dy || z != dz);

		// depthWindow.spinOnce(1);
		// depthWindow.removeAllWidgets();
	}


	void getPoints(cv::Mat& depthMap, cv::Point3i& position, cv::Mat& rotation) {
		// Foreach space between camera and raycast, increase value.

		// Foreach space behind raycast, decrease value.
	}
}