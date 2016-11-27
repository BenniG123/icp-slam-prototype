#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>

namespace icp {

	/*
		Start from initial guess
		while not within threshold:
			For each point on M, find closest point on P
			Find best transform for this 
			correspondance
			Transform M
	*/
	cv::Mat getTransformation(cv::Mat& source, cv::Mat& data, int maxIterations, float threshDistance) {
		cv::Mat rigidTransformation(4, 4, CV_32F);
		// Guess 0
		// cv::Mat correspondances = getCorrespondences();
		
		return rigidTransformation;
	}
}