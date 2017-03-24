#ifndef POINTCLOUD_HPP
#define POINTCLOUD_HPP

namespace icp {
	class PointCloud {
		public:
	      cv::Point3f center;
	      std::vector<cv::Point3f> points;
	      PointCloud(cv::Mat& data);
	      PointCloud(std::vector<cv::Point3f> data);
	      PointCloud();
	      void rotate(cv::Mat& transformationMatrix);
	      void translate(cv::Mat& transformationMatrix);
	      cv::Mat matrix();
	      cv::Mat centered_matrix();
	      void center_points();
	    private:
		  void std_dev_filter_points();
	};
}

#endif