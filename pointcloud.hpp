#ifndef POINTCLOUD_HPP
#define POINTCLOUD_HPP

namespace icp {
	class PointCloud {
		public:
	      cv::Point3f center;
	      std::vector<cv::Point3f> points;
	      PointCloud(cv::Mat& data);
	      void rotate(cv::Mat& transformationMatrix);
	      void translate(cv::Mat& transformationMatrix);
	      cv::Mat matrix();
	      cv::Mat centered_matrix();
	};
}

#endif