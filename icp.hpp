namespace icp {
	cv::Mat getTransformation(cv::Mat& data, cv::Mat& previous, int maxIterations, float threshold);
	cv::Point3i getNearestPoint(cv::Point3i point, cv::Mat& data);
	cv::Mat applyTransformation(cv::Mat& data, cv::Mat& tranformation);
	float meanSquareError(std::vector<std::pair<cv::Point3i, cv::Point3i>> associations);
}