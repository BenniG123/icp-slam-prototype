#include "opencv2/imgproc/imgproc.hpp"
#include "pointcloud.hpp"

namespace icp {
	// Build 3D point cloud from depth image
	PointCloud::PointCloud(cv::Mat& data) {
		center = cv::Point3i(0,0,0);
		points = std::vector<cv::Point3i>();

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
			points.push_back(cv::Point3i(x,y,z));

			index++;
			it++;
		}

		// Average Center
		center.x /= index;
		center.y /= index;
		center.z /= index;
	}

	void PointCloud::rotate(cv::Mat& transformationMatrix) {
	}

	void PointCloud::translate(cv::Mat& transformationMatrix) {
	}

	cv::Mat PointCloud::matrix() {
		cv::Mat M(points.size(),3, CV_32FC1);

		for (int i = 0; i < points.size(); i++) {
			M.at<float>(i, 0) = points[i].x - center.x;
			M.at<float>(i, 1) = points[i].y - center.y;
			M.at<float>(i, 2) = points[i].z - center.z;
		}

		return M;
	}
}