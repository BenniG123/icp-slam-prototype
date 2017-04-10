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
		points = std::vector<cv::Point3f>();
		colors = std::vector<cv::Vec3b>();

		cv::MatIterator_<uint16_t> it, end;
		it = data.begin<uint16_t>();
		end = data.end<uint16_t>();

		cv::MatIterator_<cv::Vec3b> color_it, color_end;
		color_it = colorMat.begin<cv::Vec3b>();
		color_end = colorMat.end<cv::Vec3b>();

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
				color_it++;
				it++;
				continue;
			}

			// Subsample
			if (rand() % 10) {
				p_index++;
				color_it++;
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
			float p_x = (x - CX) * p_z / FX;
			float p_y = (y - CX) * p_z / FX;
	      	cv::Point3f p(p_x, p_y, p_z);

	      	/*
	      	cv::Mat colorDepthRotation(3,3,CV_32FC1);
	      	colorDepthRotation = makeRotationMatrix(0.050 * 180.0/PI, -0.062 * 180.0/PI, -0.002 * 180/PI);

			cv::Mat p3d(p);
	      	p3d = colorDepthRotation * p3d;
			p3d.at<float>(0,0) -= 0.02;

	      	cv::Point3f p_c(p3d.at<float>(0,0), p3d.at<float>(1,0), p3d.at<float>(2,0));
			*/

	      	/* 
	      	int colorX = (int) std::round((p.x * FX / p.z) + CX);
	      	int colorY = (int) std::round((p.y * FY / p.z) + CY);

	      	// std::cout << colorX << " " << colorY << std::endl;
	      	if (colorX > 640 || colorX < 0 || colorY > 480 || colorY < 0) {
				p_index++;
				it++;
	      		continue;
	      	}
	      	*/

	      	// cv::Vec3b c = *color_it;
	      	// std::cout << x << " " << y << std::endl;
	      	cv::Vec3b c = *color_it; // colorMat.at<cv::Vec3b>((int(y)), int(x));
	      	/* if (int(x) % 2) {
	      		c = cv::Vec3b(255,75,75);
	      	}
	      	else {
	      		c = cv::Vec3b(100,255,75);
	      	}*/

			// P3D' = R.P3D + T
			// P2D_rgb.x = (P3D'.x * fx_rgb / P3D'.z) + cx_rgb
			// P2D_rgb.y = (P3D'.y * fy_rgb / P3D'.z) + cy_rgb

			// Update Center
			center.x += p_x;
			center.y += p_y;
			center.z += p_z;

			// Add point to point cloud
			points.push_back(p);
			colors.push_back(c);

			p_index++; // += SUBSAMPLE_FACTOR;
			index++;
			color_it++;
			it++; // += SUBSAMPLE_FACTOR;
		}

		// std::cout << points.size() << std::endl;
		// std::cout << colors.size() << std::endl;

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

		center_points();
		std_dev_filter_points();
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
			if (icp::distance(center, *it) > stddev * 1.5) {
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

		// std::cout << center << std::endl;

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


	void PointCloud::translate(cv::Point3f offset) {
		for (int i = 0; i < points.size(); i++) {
			points[i] += offset;
		}

		center += offset;
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