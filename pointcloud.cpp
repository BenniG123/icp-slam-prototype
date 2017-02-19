#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "pointcloud.hpp"
#include <iostream>

namespace icp {
	// Build 3D point cloud from depth image
	PointCloud::PointCloud(cv::Mat& data) {
		center = cv::Point3f(0,0,0);
		points = std::vector<cv::Point3f>();

		cv::MatIterator_<cv::Vec3b> it, end;
		it = data.begin<cv::Vec3b>();
		end = data.end<cv::Vec3b>();

		int index = 0;
		it++;

		while ( it != end) {
			int z = (*it)[0];

			// Blank cells aren't relevant
			if (z == 0) 
			{
				it++;
				continue;
			}

			// TODO - Worldspace from cameraspace
			int x = index % data.size().width;
			int y = index / data.size().width;

			// Update Center
			center.x += x;
			center.y += y;
			center.z += z;

			// Add point to point cloud
			points.push_back(cv::Point3f(x,y,z));

			index++;
			it++;
		}

		// Average Center
		center.x /= index;
		center.y /= index;
		center.z /= index;

		center_points();
	}

	// Build a point cloud from a vector
	PointCloud::PointCloud(std::vector<cv::Point3f> data) {
		center = cv::Point3f(0,0,0);
		points = std::vector<cv::Point3f>();

		std::vector<cv::Point3f>::iterator it, end;
		it = data.begin();
		end = data.end();

		int index = 0;

		while ( it != end) {
			points.push_back(*it);

			// Update Center
			center.x += (*it).x;
			center.y += (*it).y;
			center.z += (*it).z;

			index++;
			it++;
		}

		// Average Center
		center.x /= index;
		center.y /= index;
		center.z /= index;

		center_points();
	}

	void PointCloud::center_points() {
		std::vector<cv::Point3f>::iterator it, end;
		it = points.begin();
		end = points.end();

		while (it != end) {
			cv::Point3f p = *it;
			p.x -= center.x;
			p.y -= center.y;
			p.z -= center.z;
			it++;
		}
	}

	void PointCloud::rotate(cv::Mat& rotationMatrix) {
		cv::Mat M = matrix();

		for (int i = 0; i < points.size(); i++) {
			cv::Mat point(1, 3, CV_32FC1);

			point.at<float>(0,0) = M.col(i).at<float>(0,0);
			point.at<float>(0,1) = M.col(i).at<float>(1,0);
			point.at<float>(0,2) = M.col(i).at<float>(2,0);

			cv::Mat m = point * rotationMatrix;
			std::cout << "Rotate: " << point << ", " << m << std::endl;

			points[i].x = m.at<float>(0, 0);
			points[i].y = m.at<float>(0, 1);
			points[i].z = m.at<float>(0, 2);
		}
	}


	void PointCloud::translate(cv::Mat& transformationMatrix) {
		cv::Mat M = matrix();

		for (int i = 0; i < points.size(); i++) {
			cv::Mat m = M.col(i) * transformationMatrix;

			points[i].x = m.at<float>(i, 0);
			points[i].y = m.at<float>(i, 1);
			points[i].z = m.at<float>(i, 2);
		}
	}

	cv::Mat PointCloud::centered_matrix() {
		cv::Mat M(3, points.size(), CV_32FC1);

		for (int i = 0; i < points.size(); i++) {
			M.at<float>(0, i) = points[i].x;
			M.at<float>(1, i) = points[i].y;
			M.at<float>(2, i) = points[i].z;
		}

		return M;
	}

	cv::Mat PointCloud::matrix() {
		cv::Mat M(3, points.size(), CV_32FC1);

		for (int i = 0; i < points.size(); i++) {
			M.at<float>(0, i) = points[i].x + center.x;
			M.at<float>(1, i) = points[i].y + center.y;
			M.at<float>(2, i) = points[i].z + center.z;
		}

		return M;
	}
}