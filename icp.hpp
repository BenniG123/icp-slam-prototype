namespace icp {
	cv::Mat getTransformation(cv::Mat& data, cv::Mat& previous, int maxIterations, float threshold);
	float getNearestPoint(cv::Point3i point, cv::Point3i& nearest, cv::Mat& data);
	cv::Mat applyTransformation(cv::Mat& data, cv::Mat& tranformation);
	float meanSquareError(std::vector<float> errors);
	float initializePointCloud(cv::Mat& data, std::vector<cv::Point3i>& cloud_vector);
	float distance(cv::Point3i a, cv::Point3i b);
	void findNearestNeighborAssociations(std::vector<cv::Point3i> data, std::vector<cv::Point3i> previous, std::vector<float>& errors);
}