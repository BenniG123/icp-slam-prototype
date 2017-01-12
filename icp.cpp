#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "icp.hpp"
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
		OpenCV FLANN tutorial here - http://www.morethantechnical.com/2010/06/06/iterative-closest-point-icp-with-opencv-w-code/
	*/
	cv::Mat getTransformation(cv::Mat& data, cv::Mat& previous, int maxIterations, float threshold) {
		cv::Mat rigidTransformation(4, 4, CV_32F);
		// std::vector<std::pair<cv::Point3i, cv::Point3i>> associations;
		std::vector<float> errors;

		PointCloud dataCloud(data);
		PointCloud previousCloud(previous);

		findNearestNeighborAssociations(dataCloud, previousCloud, errors);
		int i = 0;

		// While we haven't gotten close enough yet and we haven't iterated too much
		while (meanSquareError(errors) > threshold && i++ < maxIterations) {
			// Guess new transform

			// Transform the new data
			dataCloud.transform(rigidTransformation);

			// Find nearest neighber associations
			findNearestNeighborAssociations(dataCloud, previousCloud, errors);
		}
		// imshow("Temp", previous);
		// cv::waitKey(0);
		
		return rigidTransformation;
	}

	void findNearestNeighborAssociations(PointCloud data, PointCloud previous, std::vector<float>& errors) {
		// Iterate through image
		std::vector<cv::Point3i>::iterator it, end;
		it = data.points.begin();
		end = data.points.end();

		while (it != end) {
			cv::Point3i nearestNeighbor;
			float distance = getNearestPoint(*it, nearestNeighbor, previous);
			// associations.push_back(std::make_pair(sourcePoint, nearestNeighbor));
			errors.push_back(distance);

			it++;
		}
	}

	// Bottlenecking function - Hardware acceleration candidate
	float getNearestPoint(cv::Point3i point, cv::Point3i& nearest, PointCloud cloud) {
		// Iterate through image
		std::vector<cv::Point3i>::iterator it, end;
		it = cloud.points.begin();
		end = cloud.points.end();

		nearest = *it;
		float shortestDistance = distance(point, nearest);
		it++;

		while ( it != end) {
			float d = distance(point, *it);
			if (d < shortestDistance) {
				shortestDistance = d;
				nearest = *it;
			}

			it++;
		}

		return shortestDistance;
	}

	float distance(cv::Point3i a, cv::Point3i b) {
		int x = a.x - b.x;
		int y = a.y - b.y;
		int z = a.z - b.z;
		// std::cout << a << ", " << b << std::endl;
		// cv::waitKey(0);
		return sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
	}

	float meanSquareError(std::vector<float> errors) {
		// Return the MSE between source and data
		float error_sum = 0;
		for(int i = 0; i < errors.size(); i++){
			error_sum += errors[i];
   		}

   		error_sum /= errors.size();
   		error_sum = pow(error_sum, 2);

   		std::cout << "MSE: " << error_sum << std::endl;

		return error_sum;
	}

	void initializePointCloud(cv::Mat& data, PointCloud* cloud) {

	}

	/* High level ICP Code
	pair.clear(); dists.clear();
		    double dist = flann_knn(destination, X, pair, dists);
		 
		    if(lastDist <= dist) {
		        X = lastGood;
		        break;  //converged?
		    }
		    
		    lastDist = dist;
		    X.copyTo(lastGood);
		 
		    std::cout << "distance: " << std::dist << std::endl;
		 
		    cv::Mat X_bar(X.size(),X.type());
		    for(int i=0;i<X.rows;i++) {
		        cv::Point p = destination.at<cv::Point>(pair[i],0);
		        X_bar.at<cv::Point>(i,0) = p;
		    }
		 
		    ShowQuery(destination,X,X_bar);
		 
		    X = X.reshape(2);
		    X_bar = X_bar.reshape(2);
		    findBestReansformSVD(X,X_bar);
		    X = X.reshape(1); // back to 1-channel
		}
		*/

	/* float flann_knn(Mat& m_destinations, Mat& m_object, vector<int>& ptpairs, vector<float>& dists = vector<float>()) {
	    // find nearest neighbors using FLANN
	    cv::Mat m_indices(m_object.rows, 1, CV_32S);
	    cv::Mat m_dists(m_object.rows, 1, CV_32F);
	 
	    Mat dest_32f; m_destinations.convertTo(dest_32f,CV_32FC2);
	    Mat obj_32f; m_object.convertTo(obj_32f,CV_32FC2);
	 
	    assert(dest_32f.type() == CV_32F);
	 
	    cv::flann::Index flann_index(dest_32f, cv::flann::KDTreeIndexParams(2));  // using 2 randomized kdtrees
	    flann_index.knnSearch(obj_32f, m_indices, m_dists, 1, cv::flann::SearchParams(64) ); 
	 
	    int* indices_ptr = m_indices.ptr<int>(0);
	    //float* dists_ptr = m_dists.ptr<float>(0);
	    for (int i=0;i<m_indices.rows;++i) {
	        ptpairs.push_back(indices_ptr[i]);
	    }
	 
	    dists.resize(m_dists.rows);
	    m_dists.copyTo(Mat(dists));
	 
	    return cv::sum(m_dists)[0];
	}

	void findBestReansformSVD(Mat& _m, Mat& _d) {
	    Mat m; _m.convertTo(m,CV_32F);
	    Mat d; _d.convertTo(d,CV_32F);
	 
	    Scalar d_bar = mean(d);
	    Scalar m_bar = mean(m);
	    Mat mc = m - m_bar;
	    Mat dc = d - d_bar;
	 
	    mc = mc.reshape(1); dc = dc.reshape(1);
	     
	    Mat H(2,2,CV_32FC1);
	    for(int i=0;i<mc.rows;i++) {
	        Mat mci = mc(Range(i,i+1),Range(0,2));
	        Mat dci = dc(Range(i,i+1),Range(0,2));
	        H = H + mci.t() * dci;
	    }
	 
	    cv::SVD svd(H);
	 
	    Mat R = svd.vt.t() * svd.u.t();
	    double det_R = cv::determinant(R);
	    if(abs(det_R + 1.0) < 0.0001) {
	        float _tmp[4] = {1,0,0,cv::determinant(svd.vt*svd.u)};
	        R = svd.u * Mat(2,2,CV_32FC1,_tmp) * svd.vt;
	    }

	    float* _R = R.ptr<float>(0);
	    Scalar T(d_bar[0] - (m_bar[0]*_R[0] + m_bar[1]*_R[1]),d_bar[1] - (m_bar[0]*_R[2] + m_bar[1]*_R[3]));
	 
	    m = m.reshape(1);
	    m = m * R;
	    m = m.reshape(2);
	    m = m + T;// + m_bar;
	    m.convertTo(_m,CV_32S);
	} */
}