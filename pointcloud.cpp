#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/viz/vizcore.hpp"
#include "icp.hpp"
#include "pointcloud.hpp"
#include <iostream>

namespace icp {

	// Build 3D point cloud from depth image
	PointCloud::PointCloud(cv::Mat& data, cv::Mat colorMat) {
		center = cv::Point3f(0,0,0);
		points = point_list_t();

		int index = 0;
		
		for (int y = 0; y < data.rows; y++) {
			for (int x = 0; x < data.cols; x++) {

				if (data.at<uint16_t>(y,x) == 0) 
				{
					continue;
				}

				// Random Subsample

				if (rand() % SUBSAMPLE_FACTOR) {
				 	continue;
				}

				// TODO - Worldspace from cameraspace
				// http://nicolas.burrus.name/index.php/Research/KinectCalibration#tocLink7
				// P3D.x = (x_d - cx_d (250.32)) * depth(x_d,y_d) / fx_d (363.58)
				// P3D.y = (y_d - cy_d (212.55)) * depth(x_d,y_d) / fy_d (363.53)
				// P3D.z = depth(x_d,y_d)
				float p_z = ((float) data.at<uint16_t>(y,x)) / 5000.0f;
				float p_x = (x - CX) * p_z / FX;
				float p_y = (y - CX) * p_z / FX;
		      	cv::Point3f p(p_x, p_y, p_z);

		      	// Update Center
				center.x += p_x;
				center.y += p_y;
				center.z += p_z;

				cv::Vec3b c = colorMat.at<cv::Vec3b>(y,x);

				color_point_t clr_pt;
				clr_pt.point = p;
				clr_pt.color = c;

				// Add point to point cloud
				points.push_back(clr_pt);

				index++;
			}
		}

		// Average Center
		center.x /= index;
		center.y /= index;
		center.z /= index;

		center_points();

		// std_dev_filter_points();
	}

	// Display PointCloud with colorMap in Viz Window
	void PointCloud::display(cv::viz::Viz3d& depthWindow, std::string name, int size) {
	    cv::Mat pointCloudMat(points.size(), 1, CV_32FC3);
	    cv::Mat colorMap(points.size(), 1, CV_8UC3);

	    for (int i = 0; i < points.size(); i++) {
	    	pointCloudMat.at<cv::Vec3f>(i,0) = points[i].point;
	    	colorMap.at<cv::Vec3b>(i,0) = points[i].color;
	    }

		cv::viz::WCloud cloudWidget(pointCloudMat, colorMap);
		cloudWidget.setRenderingProperty( cv::viz::POINT_SIZE, size);
		depthWindow.showWidget( name , cloudWidget);
	}

	// Display without colorMap
	void PointCloud::display(cv::viz::Viz3d& depthWindow, std::string name, int size, cv::viz::Color color) {
		cv::Mat pointCloudMat(points.size(), 1, CV_32FC3);

		for (int i = 0; i < points.size(); i++) {
	    	pointCloudMat.at<cv::Vec3f>(i,0) = points[i].point;
	    }

		cv::viz::WCloud cloudWidget(pointCloudMat, color);
		cloudWidget.setRenderingProperty( cv::viz::POINT_SIZE, size);
		depthWindow.showWidget( name , cloudWidget);
	}

	cv::Vec2i depthToRGB(cv::Point3f point) {
		cv::Mat p3d(point);
		// p3d = extRotation * p3d + extTranslation;

		float x = p3d.at<float>(0,0);
		float y = p3d.at<float>(1,0);
		float z = p3d.at<float>(2,0);

		cv::Vec2i result;
		// result[0] = (int) std::round( (x * rgbFX / z) + rgbCX);
		result[0] = (int) std::round( (x * 1054.35 / z) + 956.12);
		// result[1] = (int) std::round( (y * rgbFY / z) + rgbCY);
		result[1] = (int) std::round( (y * 1054.51 / z) + 548.99);
		return result;
	}

	void PointCloud::std_dev_filter_points() {
		point_list_t::iterator it, end;
		it = points.begin();
		end = points.end();

		float stddev = 0;
		float sum = 0;

		while ( it != end) {
			sum += pow(center.x - (*it).point.x, 2);
			sum += pow(center.y - (*it).point.y, 2);
			sum += pow(center.z - (*it).point.z, 2);
			it++;
		}

		sum /= points.size();

		stddev = sqrt(sum);

		it = points.begin();
		while ( it != end) {
			if (icp::distance(center, (*it).point) > stddev * 1.5) {
				// std::cout << (*it).z << std::endl;
				points.erase(it);
			}

			it++;
		}
	}

	// Build a point cloud from a vector
	PointCloud::PointCloud(std::vector<cv::Point3f> data) {
		center = cv::Point3f(0,0,0);
		points = point_list_t();

		std::vector<cv::Point3f>::iterator it, end;
		it = data.begin();
		end = data.end();

		int index = 0;

		while ( it != end) {
			color_point_t clr_pt;
			clr_pt.point = *it;

			points.push_back(clr_pt);

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
		points = point_list_t();
	}

	void PointCloud::center_points() {
		point_list_t::iterator it, end;
		it = points.begin();
		end = points.end();

		while (it != end) {
			cv::Point3f p = (*it).point;
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
			points[i].point.x = RMT.at<float>(i, 0);
			points[i].point.y = RMT.at<float>(i, 1);
			points[i].point.z = RMT.at<float>(i, 2);
		}
	}


	void PointCloud::translate(cv::Point3f offset) {
		for (int i = 0; i < points.size(); i++) {
			points[i].point += offset;
		}

		center += offset;
	}

	cv::Mat PointCloud::centered_matrix() {
		cv::Mat M(points.size(), 3, CV_32FC1);

		for (int i = 0; i < points.size(); i++) {
			M.at<float>(i, 0) = points[i].point.x;
			M.at<float>(i, 1) = points[i].point.y;
			M.at<float>(i, 2) = points[i].point.z;
		}

		return M;
	}

	cv::Mat PointCloud::matrix() {
		cv::Mat M(points.size(), 3, CV_32FC1);

		for (int i = 0; i < points.size(); i++) {
			M.at<float>(i, 0) = points[i].point.x + center.x;
			M.at<float>(i, 1) = points[i].point.y + center.y;
			M.at<float>(i, 2) = points[i].point.z + center.z;
		}

		return M;
	}
}