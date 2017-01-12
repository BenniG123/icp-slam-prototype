#ifndef POINTCLOUD_HPP
#define POINTCLOUD_HPP

namespace icp {
	class PointCloud {
		public:
	      cv::Point3i center;
	      std::vector<cv::Point3i> points;
	      PointCloud(cv::Mat& data);
	      void transform(cv::Mat& transformationMatrix);
	};
}

#endif