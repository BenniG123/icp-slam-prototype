#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/viz/vizcore.hpp"
#include "icp.hpp"
#include <iostream>
#include <stdio.h>
#include <Eigen/Dense>

namespace icp {

	/*
		Start from initial guess
		while not within threshold:
			For each point on M, find closest point on P
			Find best transform for this 
			correspondance
			Transform M
	*/
	cv::Mat getTransformation(cv::Mat& data, cv::Mat& previous, int maxIterations, float threshold, cv::viz::Viz3d& depthWindow) {
		cv::Mat rigidTransformation(4, 4, CV_32FC1);
		std::vector<std::pair<cv::Point3f, cv::Point3f>> associations;
		std::vector<float> errors;
		cv::Mat R;

		PointCloud dataCloud(data);
		PointCloud previousCloud(previous);

		PointCloud tempDataCloud = PointCloud();
		PointCloud tempPreviousCloud = PointCloud();

		tempDataCloud.center = dataCloud.center;
		tempPreviousCloud.center = previousCloud.center;

		/*
		double x = -5 * 3.14 / 180;
		float d[3][3] = {{1, 0, 0}, {0, (float) cos(x), (float) -sin(x)}, {0, (float) sin(x),(float) cos(x)}};
		cv::Mat a(3, 3, CV_32FC1, &d);
		
		dataCloud.rotate(a);
		*/

		findNearestNeighborAssociations(dataCloud, previousCloud, errors, associations);
		
		int i = 0;

		// While we haven't gotten close enough yet and we haven't iterated too much
		while (meanSquareError(errors) > threshold && i < maxIterations) {

			showPointCloud(dataCloud, depthWindow, cv::viz::Color().green(), "Data");
			showPointCloud(previousCloud, depthWindow, cv::viz::Color().yellow(), "Previous");

			depthWindow.spinOnce(33, true);

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

			cv::Mat M = previousMat.t() * dataMat;

			// Perform SVD
			cv::SVD svd(M);

			// Rotational Matrix
			cv::Mat R =  svd.vt.t() * svd.u.t();

			if (cv::determinant(R) < 0) {
				std::cout << "Reflection Detected" << std::endl;
				R.col(2) *= -1;
			}
			
			if (i == 0) {
				R.copyTo(rigidTransformation(cv::Rect(0, 0, 3, 3)));
			}
			else {
				cv::Mat Rp = R * rigidTransformation(cv::Rect(0, 0, 3, 3));
				Rp.copyTo(rigidTransformation(cv::Rect(0, 0, 3, 3)));
			}

			previousCloud.rotate(R);

			// Find nearest neighber associations
			findNearestNeighborAssociations(dataCloud, previousCloud, errors, associations);

			i++;
		}

		showPointCloud(dataCloud, depthWindow, cv::viz::Color().green(), "Data");
		showPointCloud(previous, depthWindow, cv::viz::Color().yellow(), "Previous");
		depthWindow.spinOnce(0, true);

		cv::Point3f translation(0,0,0);
		translation = dataCloud.center - previousCloud.center;

		rigidTransformation.at<float>(0,3) = translation.x;
		rigidTransformation.at<float>(1,3) = translation.y;
		// TODO - zScale this to m
		rigidTransformation.at<float>(2,3) = translation.z / 5;
		
		return rigidTransformation;
	}

	void showPointCloud(PointCloud p, cv::viz::Viz3d& depthWindow, cv::viz::Color color, std::string name) {
		double min;
		double max;

		cv::Mat adjMap;
		cv::Mat colorMap;

	    cv::Mat pointCloudMat(p.points.size(), 1, CV_32FC3);

	    for (int i = 0; i < p.points.size(); i++) {
	    	pointCloudMat.at<cv::Vec3f>(i,0) = p.points[i];
	    }

	   	/*
	   	cv::minMaxIdx(pointCloudMat, &min, &max);
		pointCloudMat.convertTo(adjMap,CV_8UC1, 255 / (max-min), -min);
		applyColorMap(adjMap, colorMap, cv::COLORMAP_JET);
		*/

		cv::viz::WCloud cloudWidget(pointCloudMat, color);
		cloudWidget.setRenderingProperty( cv::viz::POINT_SIZE, 3);
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
			// if (distance < 2) {
			// std::cout << "Pushing back association" << std::endl;

			associations.push_back(std::make_pair(*it, nearestNeighbor));
			errors.push_back(distance);

			// std::cout << "Pushed back association" << std::endl;
			// }

			// else {
			// 	data.points.erase(it);
			// }
			it++;
		}
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

   		std::cout << "MSE: " << error_sum << std::endl;

		return error_sum;
	}
}