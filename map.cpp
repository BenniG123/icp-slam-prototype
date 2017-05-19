#include <iostream>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/viz/vizcore.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "map.hpp"
#include "icp.hpp"
#include "pointcloud.hpp"

namespace map {
	// 6x6x6 meter space with 2 cm block

	// We can have up to 2 points stored in any given cell
	// Lookup real point locations by cell
	
	Map::Map() {
		mapCloud = icp::PointCloud();
		
		// empty.point = cv::Point3f(-100,-100,-100);
		// empty.color = cv::Vec3b();

		for (int i = 0; i < MAP_HEIGHT; i++) {
			for (int j = 0; j < MAP_HEIGHT; j++) {
				for (int k = 0; k < MAP_HEIGHT; k++) {
					world[i][j][k] = 0; //MAX_CONFIDENCE - DELTA_CONFIDENCE;
					pointLookupTable[i][j][k] = empty;
				}
			}
		}
	}

	void Map::drawCertaintyMap(cv::viz::Viz3d& depthWindow) {
		for (int i = 0; i < MAP_HEIGHT; i++) {
			for (int j = 0; j < MAP_HEIGHT; j++) {
				for (int k = 0; k < MAP_HEIGHT; k++) {
					if (world[i][j][k] > 0) {
						cv::viz::WCube cubeWidget(cv::Vec3d(i * CELL_PHYSICAL_HEIGHT, 
							j * CELL_PHYSICAL_HEIGHT, k * CELL_PHYSICAL_HEIGHT),
							cv::Vec3d( (i+1) * CELL_PHYSICAL_HEIGHT, (j+1) * CELL_PHYSICAL_HEIGHT,
							 (k+1) * CELL_PHYSICAL_HEIGHT),
						 true, cv::viz::Color(cv::Scalar(255, 127, world[i][j][k])));

						depthWindow.showWidget( "Certainty" + std::to_string(i + j * MAP_HEIGHT + k * MAP_HEIGHT * MAP_HEIGHT)
							, cubeWidget);
					}
				}
			}
		}

		// const Point3d& min_point, const Point3d& max_point 
	}

	// Get Voxel Coordinates from a worldspace point
	cv::Point3i Map::getVoxelCoordinates(cv::Point3f point) {
		cv::Point3i p;

		float c = float(CELL_PHYSICAL_HEIGHT);

		p.x = int(point.x / c);
		p.y = int(point.y / c);
		p.z = int(point.z / c);

		// Check for out of bounds
		if (p.x < 0) {
			p.x = 0;
		}
		if (p.x >= MAP_HEIGHT) {
			p.x = MAP_HEIGHT - 1;
		}
		if (p.y < 0) {
			p.y = 0;
		}
		if (p.y >= MAP_HEIGHT) {
			p.y = MAP_HEIGHT - 1;
		}
		if (p.z < 0) {
			p.z = 0;
		}
		if (p.z >= MAP_HEIGHT) {
			p.z = MAP_HEIGHT - 1;
		}

		return p;
	}

	// Update Map with new scan
	void Map::update(associations_t associations, int delta_confidence) {
		associations_t::iterator it, begin, end;
		begin, it = associations.begin();
		end = associations.end();

		float c = float(CELL_PHYSICAL_HEIGHT);
		// cv::Point3i cameraVoxelPoint = getVoxelCoordinates(cv::Point3f(3,3,3));

		while (it != end) {
			color_point_t c_point = (*it).first;
			cv::Point3i voxelPoint = getVoxelCoordinates(c_point.point);
			// rayTrace(voxelPoint, cameraVoxelPoint);

			unsigned char* certainty = &world[voxelPoint.x][voxelPoint.y][voxelPoint.z];

			// Update certainty
			if (*certainty > (255 - delta_confidence)) {
				*certainty = 255;
				// Populate lookup table if we are certain about this point
				if (pointLookupTable[voxelPoint.x][voxelPoint.y][voxelPoint.z] == empty) { // && icp::distance((*it).first.point, (*it).second.point) > MAX_POINT_DISTANCE) {
					pointLookupTable[voxelPoint.x][voxelPoint.y][voxelPoint.z] = (*it).first;
					mapCloud.points.push_back((*it).first);
				}
			} else {
				*certainty += delta_confidence;
			}

			// std::cout << (*it).first << std::endl;

			it++;
		}
	}

	// Update Map with KeyPoints and Cloud data
	void Map::update(associations_t keyPointAssociations, std::vector<float> errors, point_list_t nonAssociations, int delta_confidence) {

		if (keyPointAssociations.size() == 0) {
			return;
		}

		float c = float(CELL_PHYSICAL_HEIGHT);

		for (int i = 0; i < nonAssociations.size(); i++) {

			color_point_t c_point = nonAssociations[i];

			cv::Point3i voxelPoint = getVoxelCoordinates(c_point.point);

			unsigned char* certainty = &world[voxelPoint.x][voxelPoint.y][voxelPoint.z];

			// Update certainty
			if (*certainty >= (MAX_CONFIDENCE - delta_confidence)) {
				*certainty = 255;
				// Populate lookup table if we are certain about this point
				if (pointLookupTable[voxelPoint.x][voxelPoint.y][voxelPoint.z] == empty) { // && icp::distance((*it).first.point, (*it).second.point) > MAX_POINT_DISTANCE) {
					pointLookupTable[voxelPoint.x][voxelPoint.y][voxelPoint.z] = c_point;
					mapCloud.keypoints.push_back(c_point);
				}
			}
			else {
				*certainty += delta_confidence;
			}

		}

		// cv::Point3i cameraVoxelPoint = getVoxelCoordinates(cv::Point3f(3,3,3));

		/* for (int i = 0; i < keyPointAssociations.size(); i++) {

		color_point_t c_point = keyPointAssociations[i].first;

		cv::Point3i voxelPoint = getVoxelCoordinates(c_point.point);

		unsigned char* certainty = &world[voxelPoint.x][voxelPoint.y][voxelPoint.z];

		// Update certainty
		if (*certainty >= (MAX_CONFIDENCE - delta_confidence)) {
		*certainty = 255;
		// Populate lookup table if we are certain about this point
		if (pointLookupTable[voxelPoint.x][voxelPoint.y][voxelPoint.z] == empty) { // && icp::distance((*it).first.point, (*it).second.point) > MAX_POINT_DISTANCE) {
		pointLookupTable[voxelPoint.x][voxelPoint.y][voxelPoint.z] = c_point;
		mapCloud.keypoints.push_back(c_point);
		}
		}
		else {
		*certainty += delta_confidence;
		}

		}
		*/

		// TODO - Update Cloud Points
	}

	// Find the smallest positive t such that s + t * ds is an integer
	int Map::bound(int t, int ds) {
		if (ds < 0) {
			return bound(-t, -ds);
		}
		else {
			t = ((t % 1) + 1) % 1;
			return (1 - t)/ds;
		}
	}

	// Update Map with new scan
	void Map::update(icp::PointCloud data, int delta_confidence, cv::viz::Viz3d& depthWindow) {
		point_list_t::iterator it, end;
		it = data.points.begin();
		end = data.points.end();

		float c = float(CELL_PHYSICAL_HEIGHT);
		cv::Point3i cameraVoxelPoint = getVoxelCoordinates(cv::Point3f( 5 * CELL_PHYSICAL_HEIGHT, 5 * CELL_PHYSICAL_HEIGHT, 5 * CELL_PHYSICAL_HEIGHT));

		while (it != end) {
			color_point_t c_point = *it;
			cv::Point3i voxelPoint = getVoxelCoordinates(c_point.point);
			// rayTrace(voxelPoint, cameraVoxelPoint, depthWindow);

			// Check for out of bounds
			/* if (x < 0 || x >= MAP_HEIGHT) {
				it++;
				continue;
			} else if (y < 0 || y >= MAP_HEIGHT) {
				it++;
				continue;
			} else if (y < 0 || y >= MAP_HEIGHT) {
				it++;
				continue;
			}
			*/

			unsigned char* certainty = &world[voxelPoint.x][voxelPoint.y][voxelPoint.z];

			// Update certainty
			if (*certainty > (255 - delta_confidence)) {
				*certainty = 255;
			} else {
				*certainty += delta_confidence;
			}

			// Populate lookup table if we are certain about this point
			if (pointLookupTable[voxelPoint.x][voxelPoint.y][voxelPoint.z] == empty && *certainty >= MAX_CONFIDENCE) { // pointLookupTable[voxelPoint.x][voxelPoint.y][voxelPoint.z] == empty &&
				pointLookupTable[voxelPoint.x][voxelPoint.y][voxelPoint.z] = *it;
				mapCloud.points.push_back(*it);
			}
			/* else if (pointLookupTable[voxelPoint.x][voxelPoint.y][voxelPoint.z] != empty) {
				// mapCloud.points.erase();
				pointLookupTable[voxelPoint.x][voxelPoint.y][voxelPoint.z] = *it;
				// mapCloud.points.push_back(*it);
			}
			*/

			it++;
		}
	}

	// Algorithm from "A Fast Voxel Traversal Algorithm for Ray Tracing"
	void Map::rayTrace(cv::Point3i point, cv::Point3i origin, cv::viz::Viz3d& depthWindow) {
		// Point Voxel Coordinates
		int x, y, z;

		x = point.x;
		y = point.y;
		z = point.z;

		// depthWindow.removeWidget("destination");
		// depthWindow.removeWidget("origin");

		// cv::viz::WSphere destionationWidget(cv::Vec3d(x * CELL_PHYSICAL_HEIGHT, y * CELL_PHYSICAL_HEIGHT, z * CELL_PHYSICAL_HEIGHT), 0.5, 10, cv::viz::Color::green());
		// depthWindow.showWidget( "destination", destionationWidget);

		// Origin Voxel Coordinates
		int ox, oy, oz;

		ox = origin.x;
		oy = origin.y;
		oz = origin.z;

		cv::viz::WSphere originWidget(cv::Vec3d(ox, oy, oz), 0.1, 10, cv::viz::Color::yellow());
		depthWindow.showWidget( "origin", originWidget);

		// Slope of voxel travel
		float mx, my, mz;

		mx = (float) x - ox;
		my = (float) y - oy;
		mz = (float) z - oz;

		// Normalize slope
		float magnitude = sqrt(pow(mx, 2) + pow(my, 2) + pow(mz, 2));
		mx /= magnitude;
		my /= magnitude;
		mz /= magnitude;

		// Direction of voxel stepping
		int stepX, stepY, stepZ;

		if (ox < x) {
			stepX = 1;
		} else if (x == ox) {
			stepX = 0;
		} else {
			stepX = -1;
		}

		if (oy < y) {
			stepY = 1;
		} else if (y == oy) {
			stepY = 0;
		} else {
			stepY = -1;
		}

		if (oz < z) {
			stepZ = 1;
		} else if (z == oz) {
			stepZ = 0;
		} else {
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
						std::cout << "X: Point out of bounds: " << cv::Point3i(x,y,z) << std::endl;
						break;
					}
					tMaxX += tDeltaX;
				}
				else {
					z += stepZ;
					if (z < 0 || z > MAP_HEIGHT) {
						std::cout << "Z1: Point out of bounds: " << cv::Point3i(x,y,z) << std::endl;
						break;
					}
					tMaxZ += tDeltaZ;
				}
			}
			else {
				if (tMaxY < tMaxZ) {
					y += stepY;
					if (y < 0 || y > MAP_HEIGHT) {
						std::cout << "Y: Point out of bounds: " << cv::Point3i(x,y,z) << std::endl;
						break;
					}
					tMaxY += tDeltaY;
				}
				else {
					z += stepZ;
					if (z < 0 || z > MAP_HEIGHT) {
						std::cout << "Z2: Point out of bounds: " << cv::Point3i(x,y,z) << std::endl;
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

	bool Map::isOccupied(cv::Point3f p) {
		cv::Point3i v = getVoxelCoordinates(p);
		return world[v.x][v.y][v.z] >= MAX_CONFIDENCE;
	}

}