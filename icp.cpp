#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/viz/vizcore.hpp"
#include "map.hpp"
#include "SLAM.hpp"
#include "icp.hpp"
#include <iostream>
#include <stdio.h>

namespace icp {

	/*
		Start from initial guess
		while not within threshold:
			For each point on M, find closest point on P
			Find best transform for this
			correspondance
			Transform M
	*/

	cv::Mat cameraRotation(3, 3, CV_32FC1);
	cv::Mat lastRotation(3, 3, CV_32FC1);
	cv::Point3f cameraPosition;
	cv::Point3f lastTranslation;
	map::Map map;

	cv::Mat getTransformation(cv::Mat& data, cv::Mat& previous, cv::Mat color, cv::Mat& rotation, int maxIterations, float threshold, cv::viz::Viz3d& depthWindow) {
		cv::Mat rigidTransformation(4, 4, CV_32FC1);
		associations_t associations;
		associations_t::iterator it1, end1;

		std::vector<float> errors;
		cv::Mat R;

		cv::Point3f offset;

		PointCloud dataCloud(data, color);
		PointCloud previousCloud(previous, color);

		// cv::Mat m = makeRotationMatrix(20, 0, 0);
		// dataCloud.rotate(m);

		// Initialize map and variables
		if (map.mapCloud.points.size() == 0) {
			// Starting Rotation
			cameraRotation = makeRotationMatrix(0, 0, 0);
			lastRotation = makeRotationMatrix(0, 0, 0);

			// Starting Position
			cameraPosition = cv::Point3f(5, 5, 5);
			lastTranslation = cv::Point3f(0, 0, 0);

			// Init certainty grid
			// map = map::Map();
			previousCloud.rotate(cameraRotation);
			previousCloud.translate(cameraPosition);

			// map.mapCloud.translate(cameraPosition);

			map.update(previousCloud, MAX_CONFIDENCE, depthWindow);
		}
		// else {
		dataCloud.rotate(cameraRotation);
		dataCloud.translate(cameraPosition);
		// }


		logDeltaTime(LOG_GEN_POINT_CLOUD);

		PointCloud tempDataCloud = PointCloud();
		PointCloud tempMapCloud = PointCloud();

		tempDataCloud.center = dataCloud.center;
		tempMapCloud.center = previousCloud.center;

		// cv::Mat a = makeRotationMatrix(rand() % 5, rand() % 5, rand() % 5);

		// Optimization - let's assume the camera will generally preserve its momementum between
		// frames
		// dataCloud.rotate(lastRotation);
		// dataCloud.translate(lastTranslation);

		// lastRotation = makeRotationMatrix(0,0,0);
		// lastTranslation = cv::Point3f(0,0,0);
		// previousCloud.rotate(rotation);

		// findNearestNeighborAssociations(dataCloud, map.mapCloud, errors, associations);
		findMappedNearestNeighborAssociations(dataCloud, errors, associations);

		int i = 0;

		// While we haven't gotten close enough yet and we haven't iterated too much
		while (meanSquareError(errors) > threshold && i < maxIterations) {
			// std::cout << i << std::endl;

			// dataCloud.display(depthWindow, "Data", 3);
			// map.mapCloud.display(depthWindow, "Previous", 3);
			// showPointCloud(zeroCloud, depthWindow, cv::viz::Color().red(), "Zero", 6);
			// showPointCloud(mZeroCloud, depthWindow, cv::viz::Color().white(), "MZero", 6);

			// depthWindow.spinOnce(0, true);

			logDeltaTime(LOG_UI);

			tempDataCloud.points.clear();
			tempMapCloud.points.clear();

			// Reassociate the points inside the pointclouds
			it1 = associations.begin();
			end1 = associations.end();

			while (it1 != end1) {
				tempDataCloud.points.push_back((*it1).first);
				tempMapCloud.points.push_back((*it1).second);
				it1++;
			}

			cv::Mat dataMat = tempDataCloud.centered_matrix();
			cv::Mat previousMat = tempMapCloud.centered_matrix();

			// Make sure the data is the same size - proper alignment
			if (dataMat.size().area() > previousMat.size().area()) {
				dataMat = dataMat(cv::Rect(0, 0, previousMat.cols, previousMat.rows));
			}
			else if (previousMat.size().area() > dataMat.size().area()) {
				previousMat = previousMat(cv::Rect(0, 0, dataMat.cols, dataMat.rows));
			}

			logDeltaTime(LOG_RECONSTRUCT_POINT_CLOUDS);

			cv::Mat M = previousMat.t() * dataMat;

			// Perform SVD
			cv::SVD svd(M);

			// Rotational Matrix
			cv::Mat R = svd.vt.t() * svd.u.t();

			if (cv::determinant(R) < 0) {
				// std::cout << "Reflection Detected" << std::endl;
				R.col(2) *= -1;
			}

			logDeltaTime(LOG_SVD);

			if (i == 0) {
				R.copyTo(rigidTransformation(cv::Rect(0, 0, 3, 3)));
			}
			else {
				cv::Mat Rp = R * rigidTransformation(cv::Rect(0, 0, 3, 3));
				Rp.copyTo(rigidTransformation(cv::Rect(0, 0, 3, 3)));
			}

			R = R.inv();
			dataCloud.rotate(R);
			cameraRotation *= R;

			offset = calculateOffset(associations);

			// Update translation
			// map.translate(offset);
			// std::cout << offset << std::endl;
			dataCloud.translate(-offset);
			cameraPosition -= offset;
			// lastTranslation = -offset;
			// lastRotation = -offset;

			logDeltaTime(LOG_ROTATE);

			// Find nearest neighber associations
			// findNearestNeighborAssociations(dataCloud, map.mapCloud, errors, associations);
			findMappedNearestNeighborAssociations(dataCloud, errors, associations);

			i++;
		}

		// lastTranslation = -offset;
		// lastRotation = R;

		// Log MSE
		std::cout << meanSquareError(errors);

		rigidTransformation.at<float>(0, 3) = offset.x;
		rigidTransformation.at<float>(1, 3) = offset.y;
		rigidTransformation.at<float>(2, 3) = offset.z;

		// map.update(dataCloud, DELTA_CONFIDENCE, depthWindow);
		map.update(associations, DELTA_CONFIDENCE);

		// showAssocations(associations, errors, depthWindow);
		dataCloud.display(depthWindow, "Data", 3);
		map.mapCloud.display(depthWindow, "Map", 3);
		// map.drawCertaintyMap(depthWindow);

		std::cout << std::endl << map.mapCloud.points.size() << std::endl;

		// Update Certainties and display
		depthWindow.spinOnce(1, true);

		return rigidTransformation;
	}

	// Only add points if they are greater than 1.5f away from each other
	void showAssocations(associations_t associations, std::vector<float> errors, cv::viz::Viz3d& depthWindow) {
		associations_t::iterator it1, end1;
		it1 = associations.begin();
		end1 = associations.end();

		int i = 0;

		while (it1 != end1) {

			if (errors[i] > MIN_ASSOCIATION_DRAW_DISTANCE) {
				cv::viz::WLine line((*it1).first.point, (*it1).second.point, cv::viz::Color().red());
				depthWindow.showWidget("Association" + std::to_string(it1 - end1), line);
			}

			i++;
			it1++;
		}
	}


	// Don't factor in associations which are actually new points
	cv::Point3f calculateOffset(associations_t associations) {
		associations_t::iterator it1, end1;
		cv::Point3f offset(0, 0, 0);

		it1 = associations.begin();
		end1 = associations.end();

		int offset_count = 0;

		while (it1 != end1) {
			// Lol at this syntax
			cv::Point3f a = (*it1).first.point;
			cv::Point3f b = (*it1).second.point;

			// Only factor in translation to points that are close enough
			// distance(a, cameraPosition) < MAX_TRANSLATION_DISTANCE 
			if (map.isOccupied(a) && distance(a, b) < MAX_TRANSLATION_NN_DISTANCE) { // } && a.z < 3) {
				offset += a - b;
				offset_count++;
			}
			it1++;
		}

		offset.x /= offset_count;
		offset.y /= offset_count;
		offset.z /= offset_count;

		return offset;
	}

	// Use the certainty map lookup to find cached nearest neighbors quickly
	void findMappedNearestNeighborAssociations(PointCloud& data, std::vector<float>& errors, associations_t& associations) {
		point_list_t::iterator it, end;
		it = data.points.begin();
		end = data.points.end();

		errors.clear();
		associations.clear();

		while (it != end) {
			color_point_t nearestNeighbor;
			// std::cout << *it << std::endl;
			float distance = getNearestMappedPoint(*it, nearestNeighbor);
			if (distance < MAX_NN_COLOR_DISTANCE) {
				associations.push_back(std::make_pair(*it, nearestNeighbor));
				errors.push_back(distance);
			}

			it++;
		}

		logDeltaTime(LOG_NEAREST_NEIGHBOR, (int)associations.size());

	}

	float getNearestMappedPoint(color_point_t c_point, color_point_t &c_nearest) {
		// Iterate through image
		int x, y, z;
		cv::Point3i voxelPoint = map.getVoxelCoordinates(c_point.point);
		int radius = 1;

		// Arbitrarily Large Number
		float shortestDistance = MAX_NN_COLOR_DISTANCE;

		// TODO - Change this metric for color-weighted distance
		int maxRadius = int(float(MAX_NN_POINT_DISTANCE) / float(CELL_PHYSICAL_HEIGHT));

		// Edge case - center voxel is already occupied
		processVoxel(c_point, c_nearest, shortestDistance, voxelPoint.x, voxelPoint.y, voxelPoint.z);

		if (shortestDistance < MAX_NN_COLOR_DISTANCE) {
			return shortestDistance;
		}

		// Expanding Voxel Cube Search
		// While we don't have a matching point which satisifies the distance
		// criteria, search all voxels exactly N blocks away from the center
		while (shortestDistance >= MIN_NN_COLOR_DISTANCE && radius < maxRadius) {

			// Check leftmost and rightmost planes
			for (y = voxelPoint.y - radius; y < voxelPoint.y + radius; y++) {
				for (z = voxelPoint.z - radius; z < voxelPoint.z + radius; z++) {
					// Check for out of bounds
					if (voxelPoint.x - radius < 0 || voxelPoint.x - radius >= MAP_HEIGHT) {
						continue;
					}
					else if (voxelPoint.x + radius < 0 || voxelPoint.x + radius >= MAP_HEIGHT) {
						continue;
					}
					else if (y < 0 || y >= MAP_HEIGHT) {
						continue;
					}
					else if (y < 0 || y >= MAP_HEIGHT) {
						continue;
					}

					// Check left plane
					processVoxel(c_point, c_nearest, shortestDistance, voxelPoint.x - radius, y, z);

					// Check right plane
					processVoxel(c_point, c_nearest, shortestDistance, voxelPoint.x + radius, y, z);
				}
			}

			// Check inner lines
			for (x = voxelPoint.x - radius + 1; x < voxelPoint.x + radius - 1; x++) {
				for (z = voxelPoint.z - radius; z < voxelPoint.z + radius; z++) {
					if (x < 0 || x >= MAP_HEIGHT) {
						continue;
					}
					else if (z < 0 || z >= MAP_HEIGHT) {
						continue;
					}
					else if (voxelPoint.y - radius < 0 || voxelPoint.y - radius >= MAP_HEIGHT) {
						continue;
					}
					else if (voxelPoint.y + radius < 0 || voxelPoint.y + radius >= MAP_HEIGHT) {
						continue;
					}
					// Check top line
					processVoxel(c_point, c_nearest, shortestDistance, x, voxelPoint.y - radius, z);

					// Check bottom line
					processVoxel(c_point, c_nearest, shortestDistance, x, voxelPoint.y + radius, z);
				}
			}

			// Check inner front and back plane
			for (x = voxelPoint.x - radius + 1; x < voxelPoint.x + radius - 1; x++) {
				for (y = voxelPoint.y - radius + 1; y < voxelPoint.y + radius - 1; y++) {
					if (x < 0 || x >= MAP_HEIGHT) {
						continue;
					}
					else if (y < 0 || y >= MAP_HEIGHT) {
						continue;
					}
					else if (voxelPoint.z - radius < 0 || voxelPoint.z - radius >= MAP_HEIGHT) {
						continue;
					}
					else if (voxelPoint.z + radius < 0 || voxelPoint.z + radius >= MAP_HEIGHT) {
						continue;
					}

					// Check front plane
					processVoxel(c_point, c_nearest, shortestDistance, x, y, voxelPoint.z - radius);

					// Check back plane
					processVoxel(c_point, c_nearest, shortestDistance, x, y, voxelPoint.z + radius);
				}
			}

			radius++;
		}

		// std::cout << "Shortest Distance: " << shortestDistance << std::endl;
		// std::cout << point << " -> " << nearest << std::endl;
		return shortestDistance;
	}

	// Helper function for findNearestMappedPoint
	void processVoxel(color_point_t c_point, color_point_t& c_nearest, float& shortestDistance, int x, int y, int z) {
		if (map.pointLookupTable[x][y][z].color[0] != -1) {
			color_point_t p = map.pointLookupTable[x][y][z];
			float d = distance(c_point, p);
			// std::cout << d << std::endl;
			if (d < shortestDistance) {
				shortestDistance = d;
				c_nearest = p;
			}
		}
	}

	void findNearestNeighborAssociations(PointCloud& data, PointCloud& previous, std::vector<float>& errors, associations_t& associations) {
		// Iterate through pointcloud
		point_list_t::iterator it, end;
		it = data.points.begin();
		end = data.points.end();

		errors.clear();
		associations.clear();

		while (it != end) {
			color_point_t nearestNeighbor;
			float distance = getNearestPoint(*it, nearestNeighbor, previous);
			if (distance < MAX_NN_COLOR_DISTANCE) {
				associations.push_back(std::make_pair(*it, nearestNeighbor));
				errors.push_back(distance);
			}

			it++;
		}

		logDeltaTime(LOG_NEAREST_NEIGHBOR, associations.size());

	}

	// Bottlenecking function - Hardware acceleration candidate
	float getNearestPoint(color_point_t point, color_point_t& nearest, PointCloud& cloud) {
		// Iterate through image
		point_list_t::iterator it, end; //, near;
		it = cloud.points.begin();
		end = cloud.points.end();

		nearest = *it;
		float shortestDistance = distance(point, nearest);
		it++;

		while (it != end) {
			float d = distance(point, *it);
			if (d < shortestDistance) {
				shortestDistance = d;
				nearest = *it;
			}

			it++;
		}

		// Deep copy of point
		// nearest = cv::Point3f(nearest.x, nearest.y, nearest.z);
		// cloud.points.erase(near);
		// std::cout << cloud.points.size() << std::endl;
		// nearest = deepCopy;

		return shortestDistance;
	}

	float distance(cv::Point3f a, cv::Point3f b) {
		float x = a.x - b.x;
		float y = a.y - b.y;
		float z = a.z - b.z;
		// std::cout << a << ", " << b << std::endl;
		// cv::waitKey(0);
		return sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
	}

	// Factor color into distance calc.  Every SQRT Op is a huge hit in performance
	// TODO - Weight color and distance
	float distance(color_point_t a, color_point_t b) {
		float x = a.point.x - b.point.x;
		float y = a.point.y - b.point.y;
		float z = a.point.z - b.point.z;

		float xyz = pow(x, 2) + pow(y, 2) + pow(z, 2);

		float red = a.color[0] - b.color[0];
		float green = a.color[1] - b.color[1];
		float blue = a.color[2] - b.color[2];

		float rgb = pow(red, 2) + pow(blue, 2) + pow(green, 2);

		return sqrt(xyz * DISTANCE_WEIGHT + rgb  * COLOR_WEIGHT);
	}

	float meanSquareError(std::vector<float> errors) {
		// Return the MSE between source and data
		float error_sum = 0;
		for (int i = 0; i < errors.size(); i++) {
			error_sum += errors[i];
		}

		error_sum /= errors.size();
		error_sum = pow(error_sum, 2);

		// std::cout << "MSE: " << error_sum << std::endl;

		logDeltaTime(LOG_MSE, errors.size());

		return error_sum;
	}

	cv::Mat makeRotationMatrix(float x, float y, float z) {
		double rotX = x * PI / 180;
		double rotY = y * PI / 180;
		double rotZ = z * PI / 180;

		float d[3][3] = { {1, 0, 0}, {0, (float)cos(rotX), (float)sin(rotX)}, {0, (float)-sin(rotX),(float)cos(rotX)} };
		float f[3][3] = { {(float)cos(rotY), 0, (float)-sin(rotY)}, {0, 1, 0}, {(float)sin(rotY), 0,(float)cos(rotY)} };
		float g[3][3] = { {(float)cos(rotZ), (float)sin(rotZ), 0}, {(float)-sin(rotZ), (float)cos(rotZ), 0}, {0, 0, 1} };
		cv::Mat a(3, 3, CV_32FC1, &d);
		cv::Mat b(3, 3, CV_32FC1, &f);
		cv::Mat c(3, 3, CV_32FC1, &g);

		return a * b * c;
	}


}