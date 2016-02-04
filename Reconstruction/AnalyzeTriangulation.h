#pragma once
#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\opencv.hpp>
#include <opencv2\calib3d\calib3d.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\objdetect\objdetect.hpp>
#include <iostream>
#include <sstream>
#include <string>
#include <fstream>
#include <vector>
#include <iomanip>
#include <numeric>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "StereoCamera.h"

//#define SAVE_3D

class AnalyzeTriangulation
{
public:

	std::string inPath;
	StereoCamera *sCam;
	cv::Mat c1_projMat;
	cv::Mat c2_projMat;
	std::string algorithm;

	cv::Mat pointCloud;
	cv::Mat essentialMat;
	cv::Mat fundamentalMat;

	cv::Mat disp_gt;
	cv::Mat X_gt;
	cv::Mat Z_gt;
	cv::Mat Y_gt;
	cv::Mat disp_mask_gt;

	float min;
	float max;
	float sum;
	float mean;
	float deviation;
	float median;
	float RMSE;

	AnalyzeTriangulation(std::string inPath);
	~AnalyzeTriangulation(void);

	void triangulate(std::string algo);
	void initialize(int imgNumber, std::string inPathStr);
	void runTestOpenCV(std::string inPathStr, std::string outPathStr, int imgNumber);

	double standard_deviation(std::vector<float> v, float avg)
	{
		   double E=0;
		   for(int i=0;i<v.size();i++)
				   E+=(v[i] - avg)*(v[i] - avg);
		   return sqrt((1.0/(v.size()-1))*E);
	}

	std::string AnalyzeTriangulation::matType2Str(int type);
	float AnalyzeTriangulation::calculateRMSE(std::vector<float> distances);
};

