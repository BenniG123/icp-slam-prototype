#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/viz/vizcore.hpp"
#include "icp.hpp"
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

		int p_index = 0;
		int index = 0;

		int width = data.size().width;

		float y_prev = 0;
		
		while ( it != end) {
			// Blank cells aren't relevant
			// < 4500
			if ((*it) == 0) 
			{
				p_index++;
				it++;
				continue;
			}

			// Subsample
			if (rand() % 80) {
				p_index++;
				it++;
				continue;
			}

			// TODO - Worldspace from cameraspace
			// http://nicolas.burrus.name/index.php/Research/KinectCalibration#tocLink7
			// P3D.x = (x_d - cx_d (250.32)) * depth(x_d,y_d) / fx_d (363.58)
			// P3D.y = (y_d - cy_d (212.55)) * depth(x_d,y_d) / fy_d (363.53)
			// P3D.z = depth(x_d,y_d)
			float x = (float) (p_index % width);
			float y = (float) (p_index / width);
			float p_z = ((float) (*it)) / 5000;
			float p_x = (x - 250.32) * p_z / 363.58;
			float p_y = (y - 212.55) * p_z / 363.53;

			// std::cout << p_x << " " << p_y << " " << p_z << std::endl;

			// Update Center
			center.x += p_x;
			center.y += p_y;
			center.z += p_z;

			// Add point to point cloud
			points.push_back(cv::Point3f(p_x,p_y,p_z));

				p_index++; // += SUBSAMPLE_FACTOR;
				index++;
				it++; // += SUBSAMPLE_FACTOR;
		}

		/*
		std::vector<cv::Point3f>::iterator itp, endp;
		itp = points.begin();
		endp = points.end();

		while ( itp != endp ) {
			if (rand() % 8) {
				points.erase(itp);
			}
			itp++;
		}
		*/
		

		// Average Center
		center.x /= index;
		center.y /= index;
		center.z /= index;

		// std_dev_filter_points();
		center_points();
	}

	void PointCloud::std_dev_filter_points() {
		std::vector<cv::Point3f>::iterator it, end;
		it = points.begin();
		end = points.end();

		float stddev = 0;
		float sum = 0;

		while ( it != end) {
			sum += pow(center.x - (*it).x, 2);
			sum += pow(center.y - (*it).y, 2);
			sum += pow(center.z - (*it).z, 2);
			it++;
		}

		sum /= points.size();

		stddev = sqrt(sum);

		it = points.begin();
		while ( it != end) {
			if (icp::distance(center, *it) > stddev) {
				// std::cout << (*it).z << std::endl;
				points.erase(it);
			}

			it++;
		}
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

	// Initialize an empty point cloud
	PointCloud::PointCloud() {
		center = cv::Point3f(0,0,0);
		points = std::vector<cv::Point3f>();
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
		// std::cout << rotationMatrix.size() << M.size() << std::endl;
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