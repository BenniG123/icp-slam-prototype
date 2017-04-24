#ifndef SLAM_HPP
#define SLAM_HPP

#define LOG_NEAREST_NEIGHBOR 0
#define LOG_UI 1
#define LOG_LOAD_IMAGE 2
#define LOG_FILTER_IMAGE 3
#define LOG_GEN_POINT_CLOUD 4
#define LOG_RECONSTRUCT_POINT_CLOUDS 5
#define LOG_SVD 6
#define LOG_ROTATE 7
#define LOG_RETRIEVE_TRANSFORM 7
#define LOG_MSE 8

#define MAX_16_CHANNEL_DISTANCE 25000

#include "quaternion.hpp"
#include "opencv2/viz/vizcore.hpp"

struct logEntry {
  long long time;
  long count;
  std::string name;
  int quantity;
};

void errorMessage();

void initLog();

void logDeltaTime(int logKey, int quantity = 0);

std::string getNextImageFileName(std::ifstream& list_file, std::string path, double& timestamp);

void showText(cv::viz::Viz3d& depthWindow, std::string text, cv::Point pos, std::string name);

void showTransfom(cv::viz::Viz3d& depthWindow, cv::Mat t, cv::Point pos, std::string name);

void showTransfom(cv::viz::Viz3d& depthWindow, Quaternion q, cv::Point pos, std::string name);

void transformationMatToEulerianAngle(cv::Mat t, float& x, float&y, float& z);

void toEulerianAngle(Quaternion q, float& x, float& y, float& z);

void filterDepthImage(cv::Mat &image, cv::Mat &rgbImage, int maxDistance);

int curvature(cv::Mat roi);

std::vector<cv::Rect> calculateROIs(cv::Mat image, cv::Size2i roiSIZE, int numROIs, int margin);

cv::Vec3f getNextGroundTruth(double timestamp, std::ifstream& ground_truth_file, Quaternion& rotation);

void getNormalMap(cv::Mat& image, cv::Mat& normals);

#endif