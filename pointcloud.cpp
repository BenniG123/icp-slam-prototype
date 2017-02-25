#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "pointcloud.hpp"
#include <iostream>

namespace icp {
	// Build 3D point cloud from depth image
	PointCloud::PointCloud(cv::Mat& data) {
		center = cv::Point3f(0,0,0);
		points = std::vector<cv::Point3f>();

		cv::MatIterator_<uint16_t> it, end;
		it = data.begin<uint16_t>();
		end = data.end<uint16_t>();

		int index = 0;
		it++;

		while ( it != end) {
			// Blank cells aren't relevant
			if ((*it) == 0) 
			{
				index++;
				it++;
				continue;
			}

			// TODO - Worldspace from cameraspace
			int x = index % data.size().width;
			int y = index / data.size().width;
			float z = ((float) (*it))/ 5000.0;

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
		cv::Mat M = centered_matrix().t();
		std::cout << rotationMatrix.size() << M.size() << std::endl;
		cv::Mat RM = rotationMatrix * M;
		cv::Mat RMT = RM.t();
		
		for (int i = 0; i < points.size(); i++) {
			points[i].x = RMT.at<float>(i, 0);
			points[i].y = RMT.at<float>(i, 1);
			points[i].z = RMT.at<float>(i, 2);
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
		cv::Mat M(points.size(), 3, CV_32FC1);

		for (int i = 0; i < points.size(); i++) {
			M.at<float>(i, 0) = points[i].x;
			M.at<float>(i, 1) = points[i].y;
			M.at<float>(i, 2) = points[i].z;
		}

		return M;
	}

	cv::Mat PointCloud::matrix() {
		cv::Mat M(points.size(), 3, CV_32FC1);

		for (int i = 0; i < points.size(); i++) {
			M.at<float>(i, 0) = points[i].x + center.x;
			M.at<float>(i, 1) = points[i].y + center.y;
			M.at<float>(i, 2) = points[i].z + center.z;
		}

		return M;
	}
}