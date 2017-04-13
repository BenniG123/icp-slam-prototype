#ifndef POINTCLOUD_HPP
#define POINTCLOUD_HPP

// Preregistered data values
#define FX 468.60f
#define FY 468.61f
#define CX 318.27f
#define CY 243.99f

struct color_point_t {
	cv::Point3f point;
	cv::Vec3b color;

	inline bool operator==(const color_point_t  &c) const {return point == c.point && color == c.color;}
	inline bool operator!=(const color_point_t  &c) const {return point != c.point || color != c.color;}

	// bool operator==(const color_point_t &) const;
	// bool color_point_t::operator== (const color_point_t  &c) const {return point == c.point && color == c.color;}
	// bool color_point_t::operator!=(const color_point_t  &c) const {return point != c.point || color != c.color;}
	// bool operator!=(const color_point_t &) const;
};

// inline bool color_point_t::operator==(const color_point_t  &c) const {return point == c.point && color == c.color;}
// inline bool color_point_t::operator!=(const color_point_t  &c) const {return point != c.point || color != c.color;}

typedef std::vector<color_point_t> point_list_t;

namespace icp {

	class PointCloud {
		public:
	      cv::Point3f center;
	      point_list_t points;
	      PointCloud(cv::Mat& data, cv::Mat color);
	      PointCloud(std::vector<cv::Point3f> data);
	      PointCloud();
	      void rotate(cv::Mat& transformationMatrix);
	      void translate(cv::Point3f offset);
	      cv::Mat matrix();
	      cv::Mat centered_matrix();
	      void center_points();
	      void display(cv::viz::Viz3d& depthWindow, std::string name, int size);
	      void display(cv::viz::Viz3d& depthWindow, std::string name, int size, cv::viz::Color color);
	    private:
		  void std_dev_filter_points();
		  cv::Vec2i depthToRGB(cv::Point3f point);
	};
}

#endif