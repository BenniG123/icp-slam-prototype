#ifndef POINTCLOUD_HPP
#define POINTCLOUD_HPP

#include "opencv2/viz/vizcore.hpp"
#define SUBSAMPLE_FACTOR 80

namespace icp {
	class PointCloud {
		public:
	      cv::Point3f center;
	      std::vector<cv::Point3f> points;
	      PointCloud(cv::Mat& data);
	      PointCloud(std::vector<cv::Point3f> data);
		  void display(cv::viz::Viz3d & depthWindow, std::string name, int size, cv::viz::Color color);
	      PointCloud();
	      void rotate(cv::Mat& transformationMatrix);
	      void translate(cv::Point3f offset);
	      cv::Mat matrix();
	      cv::Mat centered_matrix();
	      void center_points();
	    private:
		  void std_dev_filter_points();
	};
}

#endif