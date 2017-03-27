#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/viz/vizcore.hpp"
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

	PointCloud map;

	cv::Mat getTransformation(cv::Mat& data, cv::Mat& previous, cv::Mat& rotation, int maxIterations, float threshold, cv::viz::Viz3d& depthWindow) {
		cv::Mat rigidTransformation(4, 4, CV_32FC1);
		std::vector<std::pair<cv::Point3f, cv::Point3f>> associations;
		std::vector<float> errors;
		cv::Mat R;

		PointCloud dataCloud(data);

		if (map.points.size() == 0) {
			std::cout << "Map initialized" << std::endl;
			map = PointCloud(previous);
		}

		PointCloud previousCloud(previous);

		logDeltaTime(LOG_GEN_POINT_CLOUD);

		std::vector<cv::Point3f> zeroList;
		zeroList.push_back(dataCloud.center);
		PointCloud zeroCloud = PointCloud(zeroList);

		PointCloud tempDataCloud = PointCloud();
		PointCloud tempPreviousCloud = PointCloud();

		tempDataCloud.center = dataCloud.center;
		tempPreviousCloud.center = previousCloud.center;

		cv::Mat a = makeRotationMatrix(1, 0, 0);
		dataCloud.rotate(a);
		// previousCloud.rotate(rotation);

		findNearestNeighborAssociations(dataCloud, map, errors, associations);
		
		int i = 0;

		// While we haven't gotten close enough yet and we haven't iterated too much
		while (meanSquareError(errors) > threshold && i < maxIterations) {

			showPointCloud(dataCloud, depthWindow, cv::viz::Color().green(), "Data", 3);
			showPointCloud(map, depthWindow, cv::viz::Color().yellow(), "Previous", 3);
			showPointCloud(zeroCloud, depthWindow, cv::viz::Color().red(), "Zero", 6);

			depthWindow.spinOnce(1, true);

			logDeltaTime( LOG_UI );

			tempDataCloud.points.clear();
			tempPreviousCloud.points.clear();

			// Reassociate the points inside the pointclouds
			std::vector<std::pair<cv::Point3f, cv::Point3f>>::iterator it1, end1;
			it1 = associations.begin();
			end1 = associations.end();

			while (it1 != end1) {
				tempDataCloud.points.push_back((*it1).first);
				tempPreviousCloud.points.push_back((*it1).second);
				it1++;
			}

			cv::Mat dataMat = tempDataCloud.centered_matrix();
			cv::Mat previousMat = tempPreviousCloud.centered_matrix();

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

			map.rotate(R);

			logDeltaTime( LOG_ROTATE );

			// Find nearest neighber associations
			findNearestNeighborAssociations(dataCloud, map, errors, associations);

			i++;
		}

		// Log MSE
		std::cout << meanSquareError(errors);

		showPointCloud(dataCloud, depthWindow, cv::viz::Color().green(), "Data", 3);
		showPointCloud(map, depthWindow, cv::viz::Color().yellow(), "Previous", 3);

		cv::Point3f translation(0,0,0);
		translation = dataCloud.center - map.center;

		rigidTransformation.at<float>(0,3) = translation.x;
		rigidTransformation.at<float>(1,3) = translation.y;
		// TODO - zScale this to m
		rigidTransformation.at<float>(2,3) = translation.z; // / 5;

		std::vector<cv::Point3f>::iterator it, end;


		// Update translation
		/* it = map.points.begin();
		end = map.points.end();

		if (distance(translation, cv::Point3f(0,0,0)) > .04) {
			while (it < end) {
				*it += translation;
				it++;
			}

			map.center += translation;
		}

		std::cout << translation << std::endl;

		it = dataCloud.points.begin();
		end = dataCloud.points.end();

		while (it < end) {
			if (rand() % 10 == 0) {
				map.points.push_back(*it);
			}
			it++;
		}

		std::cout << std::endl << map.points.size() << std::endl;

		if (map.points.size() > 4000) {
			it = map.points.begin();
			end = map.points.end();
			while (it < end) {
				if (rand() % 10 == 0) {
					map.points.erase(it);
				}
				it++;
			}
		}

		*/

		// if (distance(translation, cv::Point3f(0,0,0)) > .04) {

		// }

		// map.points.insert(map.points.end(), dataCloud.points.begin(), dataCloud.points.end());

		// map.center += cv::Point3f(5,5,5);
		// std::cout << map.center << std::endl;
		// map.center_points();
		
		return rigidTransformation;
	}

	void showPointCloud(PointCloud p, cv::viz::Viz3d& depthWindow, cv::viz::Color color, std::string name, int size) {
		double min;
		double max;

		cv::Mat adjMap;
		cv::Mat colorMap;

	    cv::Mat pointCloudMat(p.points.size(), 1, CV_32FC3);

	    for (int i = 0; i < p.points.size(); i++) {
	    	pointCloudMat.at<cv::Vec3f>(i,0) = p.points[i];
	    }

		cv::viz::WCloud cloudWidget(pointCloudMat, color);
		cloudWidget.setRenderingProperty( cv::viz::POINT_SIZE, size);
		depthWindow.showWidget( name , cloudWidget);
	}

	void findNearestNeighborAssociations(PointCloud& data, PointCloud& previous, std::vector<float>& errors, std::vector<std::pair<cv::Point3f, cv::Point3f>>& associations) {
		// Iterate through image
		std::vector<cv::Point3f>::iterator it, end;
		it = data.points.begin();
		end = data.points.end();
		errors.clear();
		associations.clear();

		// std::cout << previous.points.size() << std::endl;

		while (it != end) {
			cv::Point3f nearestNeighbor;
			float distance = getNearestPoint(*it, nearestNeighbor, previous);
			// std::cout << end - it << std::endl;
			// std::cout << nearestNeighbor << std::endl;
			if (distance < .1) {
				// std::cout << "Pushing back association" << std::endl;

				associations.push_back(std::make_pair(*it, nearestNeighbor));
				errors.push_back(distance);

				// std::cout << "Pushed back association" << std::endl;
			}

			// else {
			// 	data.points.erase(it);
			// }
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
				// near = it;
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