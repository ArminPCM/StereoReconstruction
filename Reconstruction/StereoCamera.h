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
#include <boost/thread/thread.hpp>

//#define PLOT_IMAGES
#define SAVE_DISPARITY
#define SAVE_3D
#define SAVE_IMAGES

#define MIN(a,b) ((a) < (b) ? (a) : (b))

class StereoCamera
{
public:
	// Path
	std::string path;
	// Algorithm Type
	std::string algorithmType;
	// Intrinsic and extrinsic camera parameters
	// Left Camera is the Origin
	cv::Mat c1_calib;		// Camera calibration for the first camera
	cv::Mat c1_undistortion;// Undistortion Parameters for Camera 1
	cv::Mat c2_undistortion;// Undistortion Parameters for Camera 2
	cv::Mat c2_calib;		// Camera calibration for the second camera
	cv::Mat c1_R_c2;		// Rotation from the first camera to the second camera
	cv::Mat c1_T_c2;		// Translation from the first camera to the second camera
	cv::StereoBM sbm;		// Stereo BM object
	cv::StereoSGBM sgbm;	// Stereo SGBM object
	cv::StereoVar svar;		// Stereo Var object

	// Cmaera Matrices after Rectification
	cv::Mat c1_RectTransform; // Rectification Transform for Cam1
	cv::Mat c2_RectTransform; // Rectification Transform for Cam2
	cv::Mat c1_ProjRect;	  // projection matrix in the new (rectified) coordinate systems for cam1
	cv::Mat c2_ProjRect;	  // projection matrix in the new (rectified) coordinate systems for cam2
	cv::Mat disparityToDepth; // Output 4*4 disparity-to-depth mapping matrix
	cv::Rect c1_RectROI;	  // valid ROI in Cam1 after Rectification
	cv::Rect c2_RectROI;	  // valid ROI in Cam2 after Rectification

	cv::Mat c1_finalCamMat;   // Final Calib Mat for cam1 after Undistortion and Rectification
	cv::Mat c1_map1;		  // map1 for cam1 
	cv::Mat c1_map2;		  // map2 for cam1 
	cv::Mat c2_finalCamMat;   // Final Calib Mat for cam2 after Undistortion and Rectification
	cv::Mat c2_map1;		  // map1 for cam2 
	cv::Mat c2_map2;		  // map2 for cam2

	// Images
	cv::Mat leftImage;
	cv::Mat rightImage;
	cv::Mat leftImage_gray;
	cv::Mat rightImage_gray;
	cv::Mat leftImageRect;
	cv::Mat rightImageRect;
	cv::Mat leftImageRect_gray;
	cv::Mat rightImageRect_gray;
	cv::Mat maskImage;
	cv::Mat maskImageRect;

	// Disparity Map
	cv::Mat disparityMap;
	int dispFileNum;
	// 3D Point Cloud
	cv::Mat pointCloud;
	pcl::PointCloud<pcl::PointXYZRGB> PCL_pointCloud;


	// Resizing option:
	bool boolResize;
	float resizeFactor;
public:
	StereoCamera(std::string path);
	~StereoCamera(void);

	// Load camera calibration parameters
	void loadCameraCalibParams(std::string fileAddr);
	// Load SBM Params
	void load_SBM_Params(std::string fileAddr);
	// Load SGBM Params
	void load_SGBM_Params(std::string fileAddr);
	// Load Svar Params
	void load_SVAR_Params(std::string fileAddr);

	// Initialize parameters
	void initStereoParams(std::string camFileAddr, std::string SBM_FileAddr, std::string SGBM_FileAddr, std::string SVAR_FileAddr);

	// Load Images
	void loadImages(std::string leftImageAddr, std::string rightImageAddr, std::string maskImageAddr);

	// Rectify Images
	void rectifyImages();

	// Run Stereo Matching selectedAlgorithm
	// selectedAlgorithm = SBM or SGBM or SVAR
	void runDisparity(bool boolShowPointCloud);

	// Draw point cloud using PCL
	void drawPointCloud3D();

	// Transform rotation and translation from cam2->cam1 to cam1->cam2
	void transformOrigin(cv::Mat c2_R_c1, cv::Mat c2_T_c1);

	// Create GUI
	void createGUI(std::string sAlgo);
};

