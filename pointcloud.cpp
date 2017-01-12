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

	void PointCloud::transform(cv::Mat& transformationMatrix) {

	}
}