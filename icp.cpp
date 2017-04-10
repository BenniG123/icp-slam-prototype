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
		std::vector<std::pair<cv::Point3f, cv::Point3f>> associations;
		std::vector<std::pair<cv::Point3f, cv::Point3f>>::iterator it1, end1;

		std::vector<float> errors;
		cv::Mat R;

		cv::Point3f offset;

		PointCloud dataCloud(data, color);
		PointCloud previousCloud(previous, color);

		dataCloud.rotate(cameraRotation);
		dataCloud.translate(cameraPosition);

		// Initialize map and variables
		if (map.mapCloud.points.size() == 0) {
			// Starting Rotation
			cameraRotation = makeRotationMatrix(0,0,0);
			lastRotation = makeRotationMatrix(0,0,0);

			// Starting Position
			cameraPosition = cv::Point3f(3,3,3);
			lastTranslation = cv::Point3f(0,0,0);

			// Init certainty grid
			// map = map::Map();
			previousCloud.rotate(cameraRotation);
			previousCloud.translate(cameraPosition);

			// map.mapCloud.translate(cameraPosition);

			map.update(previousCloud, MAX_CONFIDENCE);
		}

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

			showPointCloud(dataCloud, depthWindow, cv::viz::Color().green(), "Data", 3);
			// showPointCloud(map.mapCloud, depthWindow, cv::viz::Color().yellow(), "Previous", 3);
			// showPointCloud(zeroCloud, depthWindow, cv::viz::Color().red(), "Zero", 6);
			// showPointCloud(mZeroCloud, depthWindow, cv::viz::Color().white(), "MZero", 6);

			depthWindow.spinOnce(1, true);

			logDeltaTime( LOG_UI );

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

			logDeltaTime( LOG_RECONSTRUCT_POINT_CLOUDS );

			cv::Mat M = previousMat.t() * dataMat;

			// Perform SVD
			cv::SVD svd(M);

			// Rotational Matrix
			cv::Mat R =  svd.vt.t() * svd.u.t();

			if (cv::determinant(R) < 0) {
				// std::cout << "Reflection Detected" << std::endl;
				R.col(2) *= -1;
			}

			logDeltaTime( LOG_SVD );
			
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

			logDeltaTime( LOG_ROTATE );

			// Find nearest neighber associations
			// findNearestNeighborAssociations(dataCloud, map.mapCloud, errors, associations);
			findMappedNearestNeighborAssociations(dataCloud, errors, associations);

			i++;
		}

		// lastTranslation = -offset;
		// lastRotation = R;

		// Log MSE
		std::cout << meanSquareError(errors);

		rigidTransformation.at<float>(0,3) = offset.x;
		rigidTransformation.at<float>(1,3) = offset.y;
		rigidTransformation.at<float>(2,3) = offset.z;

		map.update(dataCloud, DELTA_CONFIDENCE);

		showPointCloud(dataCloud, depthWindow, cv::viz::Color().green(), "Data", 3);
		showPointCloud(map.mapCloud, depthWindow, cv::viz::Color().yellow(), "Previous", 3);
		// map.drawCertaintyMap(depthWindow);

		std::cout << std::endl << map.mapCloud.points.size() << std::endl;

		// Update Certainties and display
		depthWindow.spinOnce(1, true);
		
		return rigidTransformation;
	}

	cv::Point3f calculateOffset(std::vector<std::pair<cv::Point3f, cv::Point3f>> associations) {
		std::vector<std::pair<cv::Point3f, cv::Point3f>>::iterator it1, end1;
		cv::Point3f offset(0,0,0);

		it1 = associations.begin();
		end1 = associations.end();

		int offset_count = 0;

		while (it1 != end1) {
			cv::Point3f a = (*it1).first;
			cv::Point3f b = (*it1).second;

			// Only factor in translation to points that are close enough
			if (distance(a, b) < .3) { // } && a.z < 3) {
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

	void showPointCloud(PointCloud p, cv::viz::Viz3d& depthWindow, cv::viz::Color color, std::string name, int size) {
	    cv::Mat pointCloudMat(p.points.size(), 1, CV_32FC3);
	    cv::Mat colorMap(p.points.size(), 1, CV_8UC3);

	    for (int i = 0; i < p.points.size(); i++) {
	    	pointCloudMat.at<cv::Vec3f>(i,0) = p.points[i];
	    	colorMap.at<cv::Vec3b>(i,0) = p.colors[i];
	    }

		cv::viz::WCloud cloudWidget(pointCloudMat, colorMap);
		cloudWidget.setRenderingProperty( cv::viz::POINT_SIZE, size);
		depthWindow.showWidget( name , cloudWidget);
	}

	// Use the certainty map lookup to find cached nearest neighbors quickly
	void findMappedNearestNeighborAssociations(PointCloud& data, std::vector<float>& errors, std::vector<std::pair<cv::Point3f, cv::Point3f>>& associations) {
		std::vector<cv::Point3f>::iterator it, end;
		it = data.points.begin();
		end = data.points.end();
		errors.clear();
		associations.clear();

		while (it != end) {
			cv::Point3f nearestNeighbor;
			float distance = getNearestMappedPoint(*it, nearestNeighbor);
			if (distance < MAX_NN_DISTANCE) {
				associations.push_back(std::make_pair(*it, nearestNeighbor));
				errors.push_back(distance);
			}

			it++;
		}

		logDeltaTime(LOG_NEAREST_NEIGHBOR, associations.size());

	}

	float getNearestMappedPoint(cv::Point3f point, cv::Point3f& nearest) {
		// Iterate through image
		int x, y, z;
		cv::Point3i voxelPoint = map.getVoxelCoordinates(point);
		int radius = 1;

		// Arbitrarily Large Number
		float shortestDistance = MAX_NN_DISTANCE;
		int maxRadius = int(float(MAX_NN_DISTANCE) / float(CELL_PHYSICAL_HEIGHT));

		// Edge case - center voxel is already occupied
		processVoxel(point, nearest, shortestDistance, voxelPoint.x, voxelPoint.y, voxelPoint.z);
		if (shortestDistance < MAX_NN_DISTANCE) {
			return shortestDistance;
		}

		// Expanding Voxel Cube Search
		// While we don't have a matching point which satisifies the distance
		// criteria, search all voxels exactly N blocks away from the center
		while(shortestDistance >= MAX_NN_DISTANCE && radius < maxRadius) {
			
			// Check leftmost and rightmost planes
			for (y = voxelPoint.y - radius; y < voxelPoint.y + radius; y++) {
				for (z = voxelPoint.z - radius; z < voxelPoint.z + radius; z++) {
					// Check for out of bounds
					if (voxelPoint.x - radius < 0 || voxelPoint.x - radius >= MAP_HEIGHT) {
						continue;
					} else if (voxelPoint.x + radius < 0 || voxelPoint.x + radius >= MAP_HEIGHT) {
						continue;
					} else if (y < 0 || y >= MAP_HEIGHT) {
						continue;
					} else if (y < 0 || y >= MAP_HEIGHT) {
						continue;
					}

					// Check left plane
					processVoxel(point, nearest, shortestDistance, voxelPoint.x - radius, y, z);

					// Check right plane
					processVoxel(point, nearest, shortestDistance, voxelPoint.x + radius, y, z);
				}
			}

			// Check inner lines
			for (x = voxelPoint.x - radius + 1; x < voxelPoint.x + radius - 1; x++) {
				for (z = voxelPoint.z - radius; z < voxelPoint.z + radius; z++) {
					if (x < 0 || x >= MAP_HEIGHT) {
						continue;
					} else if (z < 0 || z >= MAP_HEIGHT) {
						continue;
					} else if (voxelPoint.y - radius < 0 || voxelPoint.y - radius >= MAP_HEIGHT) {
						continue;
					} else if (voxelPoint.y + radius < 0 || voxelPoint.y + radius >= MAP_HEIGHT) {
						continue;
					}
					// Check top line
					processVoxel(point, nearest, shortestDistance, x, voxelPoint.y - radius, z);

					// Check bottom line
					processVoxel(point, nearest, shortestDistance, x, voxelPoint.y + radius, z);
				}
			}

			// Check inner front and back plane
			for (x = voxelPoint.x - radius + 1; x < voxelPoint.x + radius - 1; x++) {
				for (y = voxelPoint.y - radius + 1; y < voxelPoint.y + radius - 1; y++) {
					if (x < 0 || x >= MAP_HEIGHT) {
						continue;
					} else if (y < 0 || y >= MAP_HEIGHT) {
						continue;
					} else if (voxelPoint.z - radius < 0 || voxelPoint.z - radius >= MAP_HEIGHT) {
						continue;
					} else if (voxelPoint.z + radius < 0 || voxelPoint.z + radius >= MAP_HEIGHT) {
						continue;
					}

					// Check front plane
					processVoxel(point, nearest, shortestDistance, x, y, voxelPoint.z - radius);

					// Check back plane
					processVoxel(point, nearest, shortestDistance, x, y, voxelPoint.z + radius);
				}
			}

			radius++;
		}

		// std::cout << "Shortest Distance: " << shortestDistance << std::endl;
		// std::cout << point << " -> " << nearest << std::endl;
		return shortestDistance;
	}

	void processVoxel(cv::Point3f point, cv::Point3f& nearest, float& shortestDistance, int x, int y, int z) {
		if (map.pointLookupTable[x][y][z] != map.empty) {
			cv::Point3f p = map.pointLookupTable[x][y][z];
			float d = distance(point, p);
			if (d < shortestDistance) {
				shortestDistance = d;
				nearest = p;
			}
		}
	}

	void findNearestNeighborAssociations(PointCloud& data, PointCloud& previous, std::vector<float>& errors, std::vector<std::pair<cv::Point3f, cv::Point3f>>& associations) {
		// Iterate through pointcloud
		std::vector<cv::Point3f>::iterator it, end;
		it = data.points.begin();
		end = data.points.end();
		errors.clear();
		associations.clear();

		while (it != end) {
			cv::Point3f nearestNeighbor;
			float distance = getNearestPoint(*it, nearestNeighbor, previous);
			if (distance < MAX_NN_DISTANCE) {
				associations.push_back(std::make_pair(*it, nearestNeighbor));
				errors.push_back(distance);
			}

			it++;
		}

		logDeltaTime( LOG_NEAREST_NEIGHBOR, associations.size());

	}

	// Bottlenecking function - Hardware acceleration candidate
	float getNearestPoint(cv::Point3f point, cv::Point3f& nearest, PointCloud& cloud) {
		// Iterate through image
		std::vector<cv::Point3f>::iterator it, end; //, near;
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

	float meanSquareError(std::vector<float> errors) {
		// Return the MSE between source and data
		float error_sum = 0;
		for(int i = 0; i < errors.size(); i++){
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

		float d[3][3] = {{1, 0, 0}, {0, (float) cos(rotX), (float) sin(rotX)}, {0, (float) -sin(rotX),(float) cos(rotX)}};
		float f[3][3] = {{(float) cos(rotY), 0, (float) -sin(rotY)}, {0, 1, 0}, {(float) sin(rotY), 0,(float) cos(rotY)}};
		float g[3][3] = {{(float) cos(rotZ), (float) sin(rotZ), 0}, {(float) -sin(rotZ), (float) cos(rotZ), 0}, {0, 0, 1}};
		cv::Mat a(3, 3, CV_32FC1, &d);
		cv::Mat b(3, 3, CV_32FC1, &f);
		cv::Mat c(3, 3, CV_32FC1, &g);

		return a * b * c;
	}


}