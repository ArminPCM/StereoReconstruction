#include "StereoCamera.h"

// Which disparity selectedAlgorithm?
std::string selectedAlgorithm = "SBM"; //SBM or SGBM or SVAR
int algo = 2; // 1=SBM, 2=SGBM, 3=SVAR

/// Convert Matrix Type to String\\\

std::string type2str(int type) 
{
	std::string r;

	uchar depth = type & CV_MAT_DEPTH_MASK;
	uchar chans = 1 + (type >> CV_CN_SHIFT);

	switch ( depth ) {
	case CV_8U:  r = "8U"; break;
	case CV_8S:  r = "8S"; break;
	case CV_16U: r = "16U"; break;
	case CV_16S: r = "16S"; break;
	case CV_32S: r = "32S"; break;
	case CV_32F: r = "32F"; break;
	case CV_64F: r = "64F"; break;
	default:     r = "User"; break;
	}

	r += "C";
	r += (chans+'0');

	return r;
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> createVisualizer (pcl::PointCloud<pcl::PointXYZRGB> cloud)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud.makeShared());
	viewer->addPointCloud<pcl::PointXYZRGB> (cloud.makeShared(), rgb, "reconstruction");
	//viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "reconstruction");
	viewer->addCoordinateSystem ( 0.01 );
	viewer->initCameraParameters ();
	return (viewer);
}

/// Loads Paramteres for cameras and disparity methods + images from files\\\
// Constructor
StereoCamera::StereoCamera(std::string path)
{
	dispFileNum = 0;
	// Load Parameters
	std::cout << "Loading Parameters from Files...\n";
	std::string camFileAddr = path + "\\camParams.txt";
	std::string SBM_FileAddr = path + "\\sbmParams.txt";
	std::string SGBM_FileAddr = path + "\\sgbmParams.txt";
	std::string SVAR_FileAddr = path + "\\svarParams.txt";

	initStereoParams(camFileAddr, SBM_FileAddr, SGBM_FileAddr, SVAR_FileAddr);

	std::cout << "Parameters Loaded:\n";
	std::cout << "\n\nCalibration Matrix Cam1:\n" << c1_calib;
	std::cout << "\n\nUndistortion Paramters Cam1:\n" << c1_undistortion;
	std::cout << "\n\nCalibration Matrix Cam2:\n" << c2_calib;
	std::cout << "\n\nUndistortion Paramters Cam2:\n" << c2_undistortion;
	std::cout << "\n\nRotation from Cam1 to Cam2:\n" << c1_R_c2;
	std::cout << "\n\nTranslation from Cam1 to Cam2:\n" << c1_T_c2;
	std::cout << "\n\n";

	// Load Images
	std::string leftImageAddr = path + "\\images\\Left_img5.png";
	std::string rightImageAddr = path + "\\images\\Right_img5.png";
	std::string maskImageAddr = path + "\\images\\MaskImageRect.jpg";
	/*loadImages(leftImageAddr, rightImageAddr, maskImageAddr);

	 Rectify & Undistort Images
	leftImageRect_gray = leftImage_gray;
	rightImageRect_gray = rightImage_gray;
	leftImageRect = leftImage;
	rightImageRect = rightImage;*/
	//rectifyImages();
}


StereoCamera::~StereoCamera(void)
{
}

///Inilialize Stereo Parameters\\\

void StereoCamera::initStereoParams(std::string camFileAddr, std::string SBM_FileAddr, std::string SGBM_FileAddr, std::string SVAR_FileAddr)
{
	selectedAlgorithm = "SGBM";
	loadCameraCalibParams(camFileAddr);
	load_SBM_Params(SBM_FileAddr);
	load_SGBM_Params(SGBM_FileAddr);
	load_SVAR_Params(SVAR_FileAddr);
	boolResize = false;
	resizeFactor = 1.0;
}

///Loading Intrinsic & Extrinsic Parameters\\\
// Loads parameters from a file with the following format:
//		1. 3*3 calibration matrix for cam1
//		2. 3*3 calibration matrix for cam2
//		3. scalar: number of undistortion parameters
//		4. n*1 Undistortion Params for cam1
//		5. n*1 Undistortion Params for cam2
//		6. 3*3 rotation matrix from cam1 to cam2
//		7. 3*1 translation matrix from cam1 to cam2
void StereoCamera::loadCameraCalibParams(std::string fileAddr)
{
	std::ifstream ccFile;
	ccFile.open(fileAddr, std::ios::in);
	// Read Camera Calibration Matrix for Cam1
	c1_calib.create(3, 3, CV_64FC1);
	for(int i=0; i<3; i++)
	{
		for(int j=0; j<3; j++)
		{
			if(!(ccFile >> c1_calib.at<double>(i, j)))
			{
				throw std::runtime_error("Failed to read camera 1 calibration matrix...");
			}
		}
	}

	// Read Camera Calibration Matrix for Cam2
	c2_calib.create(3, 3, CV_64FC1);
	for(int i=0; i<3; i++)
	{
		for(int j=0; j<3; j++)
		{
			if(!(ccFile >> c2_calib.at<double>(i, j)))
			{
				throw std::runtime_error("Failed to read camera 2 calibration matrix...");
			}
		}
	}

	// Load Undistortion Parameters for Cam1
	// How many Undistortion Parameters?
	int numParamsUndistort;
	ccFile >> numParamsUndistort;
	c1_undistortion.create(1, numParamsUndistort, CV_64FC1);
	for(int i=0; i<numParamsUndistort; i++)
	{
		int j = 0;
		if(!(ccFile >> c1_undistortion.at<double>(j, i)))
		{
			throw std::runtime_error("Failed to read camera 1 undistortion matrix...");
		}
	}

	// Load Undistortion Parameters for Cam2
	c2_undistortion.create(1, numParamsUndistort, CV_64FC1);
	for(int i=0; i<numParamsUndistort; i++)
	{
		int j = 0;
		if(!(ccFile >> c2_undistortion.at<double>(j, i)))
		{
			throw std::runtime_error("Failed to read camera 2 undistortion matrix...");
		}
	}

	// Read Rotation Matrix from cam1 to cam2
	c1_R_c2.create(3, 3, CV_64FC1);
	for(int i=0; i<3; i++)
	{
		for(int j=0; j<3; j++)
		{
			if(!(ccFile >> c1_R_c2.at<double>(i, j)))
			{
				throw std::runtime_error("Failed to read rotation matrix...");
			}
		}
	}

	// Read Translation Matrix from cam1 to cam2 (3*1 matrix)
	c1_T_c2.create(3, 1, CV_64FC1);
	for(int i=0; i<3; i++)
	{
		int j=0;
		if(!(ccFile >> c1_T_c2.at<double>(i, j)))
		{
			throw std::runtime_error("Failed to read translation matrix...");
		}
	}
}

///Load Parameters for StereoBM disparity method\\\

void StereoCamera::load_SBM_Params(std::string fileAddr)
{
	std::ifstream sbmFile;
	sbmFile.open(fileAddr, std::ios::in);
	// Parameters should be in the following order:
	sbmFile >> sbm.state->disp12MaxDiff;
	sbmFile >> sbm.state->minDisparity;
	sbmFile >> sbm.state->numberOfDisparities;
	sbmFile >> sbm.state->preFilterCap;
	sbmFile >> sbm.state->preFilterSize;
	// CV_STEREO_BM_NORMALIZED_RESPONSE = 0, CV_STEREO_BM_BASIC = 0, CV_STEREO_BM_FISH_EYE = 1, CV_STEREO_BM_NARROW = 2
	sbmFile >> sbm.state->preFilterType;
	sbmFile >> sbm.state->SADWindowSize;
	sbmFile >> sbm.state->speckleRange;
	sbmFile >> sbm.state->speckleWindowSize;
	sbmFile >> sbm.state->textureThreshold;
	// 1 may result in better accuracy
	sbmFile >> sbm.state->trySmallerWindows;
	sbmFile >> sbm.state->uniquenessRatio;
}

///Load Parameters for StereoSGBM disparity method\\\

void StereoCamera::load_SGBM_Params(std::string fileAddr)
{
	std::ifstream sgbmFile;
	sgbmFile.open(fileAddr, std::ios::in);
	// Parameters should be in the following order:
	sgbmFile >> sgbm.disp12MaxDiff;
	// 0:False, 1:True
	sgbmFile >> sgbm.fullDP;
	sgbmFile >> sgbm.minDisparity;
	sgbmFile >> sgbm.numberOfDisparities;
	sgbmFile >> sgbm.P1;
	sgbmFile >> sgbm.P2;
	sgbmFile >> sgbm.preFilterCap;
	sgbmFile >> sgbm.SADWindowSize;
	sgbmFile >> sgbm.speckleRange;
	sgbmFile >> sgbm.speckleWindowSize;
	sgbmFile >> sgbm.uniquenessRatio;
}

///Load Parameters for SVar disparity method\\\

void StereoCamera::load_SVAR_Params(std::string fileAddr)
{
	std::ifstream svarFile;
	svarFile.open(fileAddr, std::ios::in);
	// Parameters should be in the following order:
	// CYCLE_O = 0; CYCLE_V = 1
	svarFile >> svar.cycle;
	svarFile >> svar.fi;
	//USE_AUTO_PARAMS = 8; USE_EQUALIZE_HIST = 2; USE_INITIAL_DISPARITY = 1; USE_MEDIAN_FILTERING = 16; USE_SMART_ID = 4
	svarFile >> svar.flags;
	svarFile >> svar.lambda;
	svarFile >> svar.levels;
	svarFile >> svar.maxDisp;
	svarFile >> svar.minDisp;
	svarFile >> svar.nIt;
	// PENALIZATION_CHARBONNIER = 1; PENALIZATION_PERONA_MALIK = 2; PENALIZATION_TICHONOV = 0
	svarFile >> svar.penalization;
	svarFile >> svar.poly_n;
	svarFile >> svar.poly_sigma;
	svarFile >> svar.pyrScale;
}


/// Loading Images and Resizing them \\\

void StereoCamera::loadImages(std::string leftImageAddr, std::string rightImageAddr, std::string maskImageAddr)
{
	// Loading color images
	leftImage = cv::imread(leftImageAddr);
	rightImage = cv::imread(rightImageAddr);
	//maskImage = cv::imread(maskImageAddr, CV_LOAD_IMAGE_GRAYSCALE);

	// Resize Images
	if(boolResize)
	{
		cv::resize(leftImage, leftImage, cv::Size(resizeFactor, resizeFactor));
		cv::resize(rightImage, rightImage, cv::Size(resizeFactor, resizeFactor));
		//cv::resize(maskImage, maskImage, cv::Size(resizeFactor, resizeFactor));
	}
	if(leftImage.type() != CV_8UC3 || rightImage.type() != CV_8UC3 /*|| maskImage.type() != CV_8UC1*/)
	{
		throw std::runtime_error("Input Images are not grayscale...");
	}

	// Convert Images to Grayscale
	cv::cvtColor(leftImage, leftImage_gray, CV_RGB2GRAY);
	cv::cvtColor(rightImage, rightImage_gray, CV_RGB2GRAY);

	// Fill rectified images
	leftImageRect_gray = leftImage_gray;
	rightImageRect_gray = rightImage_gray;

#ifdef PLOT_IMAGES
	cv::imshow("Left", leftImage);
	cv::imshow("Right", rightImage);
	//cv::imshow("Mask", maskImage);

	cv::imshow("LeftGray", leftImage_gray);
	cv::imshow("RightGray", rightImage_gray);
	cv::waitKey(0);
#endif
}

/// Rectify Images Using Camera Calibration Matrices \\\

void StereoCamera::rectifyImages()
{
	// Stereo Rectify Images
	cv::stereoRectify(c1_calib, c1_undistortion, c2_calib, c2_undistortion, rightImage.size(), 
		c1_R_c2, c1_T_c2, c1_RectTransform, c2_RectTransform, c1_ProjRect, c2_ProjRect, disparityToDepth, 0, 0.7, rightImage.size(), &c1_RectROI, &c2_RectROI);

	std::cout << "Disparity to depth matrix:\n" << disparityToDepth << "\n\n";
	std::cout << "c1_RectTransform:\n" << c1_RectTransform << "\n\n";
	std::cout << "c2_RectTransform:\n" << c2_RectTransform << "\n\n";
	std::cout << "c1_ProjRect:\n" << c1_ProjRect << "\n\n";
	std::cout << "c2_ProjRect:\n" << c2_ProjRect << "\n\n";
	// Initialize Undistortion and Rectification Maps
	cv::initUndistortRectifyMap(c1_calib, c1_undistortion, c1_RectTransform, c1_ProjRect, rightImage.size(), CV_16SC2, c1_map1, c1_map2);
	cv::initUndistortRectifyMap(c2_calib, c2_undistortion, c2_RectTransform, c2_ProjRect, leftImage.size(), CV_16SC2, c2_map1, c2_map2);
	std::cout << "c1_ProjRect:\n" << c1_ProjRect << "\n\n";
	std::cout << "c2_ProjRect:\n" << c2_ProjRect << "\n\n";
	// Undistort and rectify images
	cv::remap(leftImage, leftImageRect, c2_map1, c2_map2, CV_INTER_LINEAR);
	cv::remap(leftImage_gray, leftImageRect_gray, c2_map1, c2_map2, CV_INTER_LINEAR);
	cv::remap(rightImage, rightImageRect, c1_map1, c1_map2, CV_INTER_LINEAR);
	cv::remap(rightImage_gray, rightImageRect_gray, c1_map1, c1_map2, CV_INTER_LINEAR);
	maskImageRect = maskImage;
	/*// Remap mask image assuming it is in left camera
	cv::remap(maskImage, maskImageRect, c1_map1, c1_map2, CV_INTER_LINEAR);
	// Convert the mask to black and white
	cv::threshold(maskImageRect, maskImageRect, 127, 255, CV_8UC1);*/

#ifdef PLOT_IMAGES
	cv::imshow("leftImage_gray", leftImageRect_gray);
	cv::imshow("rightImage_gray", rightImageRect_gray);
	cv::imshow("LeftRect", leftImageRect);
	cv::imshow("RightRect", rightImageRect);
	cv::imshow("MaskRect", maskImageRect);
	cv::waitKey(0);
#endif

#ifdef SAVE_IMAGES
	cv::imwrite("..\\leftImageRect.jpg", leftImageRect);
	cv::imwrite("..\\rightImageRect.jpg", rightImageRect);
#endif
}

/// Rund Disparity Calculation selectedAlgorithm \\\

void StereoCamera::runDisparity(bool boolShowPointCloud)
{
	if(algorithmType == "SBM")
	{
		sbm.state->numberOfDisparities = sbm.state->numberOfDisparities*16;
		sbm(leftImageRect_gray, rightImageRect_gray, disparityMap, CV_32F);
		sbm.state->numberOfDisparities = sbm.state->numberOfDisparities/16;
	}
	else if(algorithmType == "SGBM")
	{
		sgbm.numberOfDisparities = sgbm.numberOfDisparities*16;
		sgbm(leftImageRect_gray, rightImageRect_gray, disparityMap);
		sgbm.numberOfDisparities = sgbm.numberOfDisparities/16;
	}
	else if(algorithmType == "SVAR")
	{
		//svar.penalization = svar.PENALIZATION_TICHONOV;   // ignored with USE_AUTO_PARAMS
		//svar.cycle = svar.CYCLE_V;                        // ignored with USE_AUTO_PARAMS
		//svar.flags = svar.USE_AUTO_PARAMS;
		disparityMap = cv::Mat::zeros(leftImageRect_gray.rows, leftImageRect_gray.cols, CV_8UC1);
		svar(leftImageRect_gray, rightImageRect_gray, disparityMap);
	}

#ifdef PLOT_IMAGES
	double min, max;
	//disparityMap = cv::abs(disparityMap);
	minMaxLoc(disparityMap, &min, &max);
	cv::Mat dispMatToShow;
	if(algorithmType != "SVAR")
		disparityMap.convertTo(dispMatToShow, CV_8U, -255.0/max, 255);
	else
	{
		disparityMap.convertTo(dispMatToShow, CV_8U);
		disparityMap.convertTo(disparityMap, CV_8U);
		//disparityMap.convertTo(dispMatToShow, CV_8U, 255.0/max, 0);
	}
	cv::imshow("Disparity", disparityMap);
#endif
#ifdef SAVE_DISPARITY
	dispFileNum++;
	std::string fileNumberString;
	std::ostringstream convert;
	convert << dispFileNum;
	fileNumberString = convert.str();
	std::ofstream dispFile;
	std::string dispFileName = "..\\dispSVAR_img4_" + fileNumberString + ".txt";
	dispFile.open(dispFileName, std::ios::out);
	dispFile << disparityMap;
#endif
}

/// Trinagulating and Drawing 3D Point Cloud using DisparityMap \\\

void StereoCamera::drawPointCloud3D()
{
	// Triangulate and reproject images to 3D
#ifdef SAVE_3D
	std::ofstream recFile;
	recFile.open("..\\recontruction.txt", std::ios::out);
#endif SAVE_3D
	cv::reprojectImageTo3D(disparityMap, pointCloud, disparityToDepth, true);
	PCL_pointCloud.clear(); 
	for(int i=0; i<disparityMap.rows; i++)
	{
		for(int j=0; j<disparityMap.cols; j++)
		{
			if(maskImageRect.at<uchar>(i, j) == 0 && pointCloud.at<cv::Vec3f>(i, j)[2] < 10000)
			{
				pcl::PointXYZRGB point3D;
				point3D.x = pointCloud.at<cv::Vec3f>(i, j)[0];
				point3D.y = pointCloud.at<cv::Vec3f>(i, j)[1];
				point3D.z = pointCloud.at<cv::Vec3f>(i, j)[2];
				#ifdef SAVE_3D
					recFile << point3D.x << ", " << point3D.y << ", " << point3D.z << "\n";
				#endif SAVE_3D
				point3D.r = rightImageRect.at<cv::Vec3b>(i, j)[0];
				point3D.g = rightImageRect.at<cv::Vec3b>(i, j)[1];
				point3D.b = rightImageRect.at<cv::Vec3b>(i, j)[2];
				PCL_pointCloud.push_back(point3D);
			}
		}
	}
	PCL_pointCloud.width = (int) PCL_pointCloud.points.size();
	PCL_pointCloud.height = 1;
	//Create visualizer
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	viewer = createVisualizer( PCL_pointCloud );
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "reconstruction");
	viewer->setBackgroundColor(255, 255, 255,0);
	//Main loop
	while ( !viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
}

/// Transforms rotation and translation from cam2->cam1 to cam1->cam2 and saves them in attributes of class: c1_R_c2, c1_T_c2
// finish this later
void StereoCamera::transformOrigin(cv::Mat c2_R_c1, cv::Mat c2_T_c1)
{
	cv::transpose(c2_R_c1, c1_R_c2);

}

/// GUI Callback function\\\

void updateParameters(int, void*)
{
	switch(algo)
	{
	case 1: selectedAlgorithm = "SBM";
		break;
	case 2: selectedAlgorithm = "SGBM";
		break;
	case 3: selectedAlgorithm = "SVAR";
	}
}

void StereoCamera::createGUI(std::string sAlgo)
{
	selectedAlgorithm = sAlgo;
	cv::namedWindow("Controls", CV_GUI_NORMAL);
	cv::createTrackbar("selectedAlgorithm", "Controls", &algo, 200, updateParameters);
	if(selectedAlgorithm == "SBM")
	{
		cv::createTrackbar("disp12MaxDiff", "Controls", &sbm.state->disp12MaxDiff, 200, updateParameters);
		cv::createTrackbar("minDisp", "Controls", &sbm.state->minDisparity, 50, updateParameters);
		cv::createTrackbar("numDisps", "Controls", &sbm.state->numberOfDisparities, 20, updateParameters);
		cv::createTrackbar("preFiltCap", "Controls", &sbm.state->preFilterCap, 63, updateParameters);
		cv::createTrackbar("preFiltSize", "Controls", &sbm.state->preFilterSize, 128, updateParameters);
		cv::createTrackbar("SADWinSize", "Controls", &sbm.state->SADWindowSize, 128, updateParameters);
		cv::createTrackbar("speckleRange", "Controls", &sbm.state->speckleRange, 50, updateParameters);
		cv::createTrackbar("speckleWinSize", "Controls", &sbm.state->speckleWindowSize, 255, updateParameters);
		cv::createTrackbar("texThresh", "Controls", &sbm.state->textureThreshold, 100, updateParameters);
		cv::createTrackbar("uniqRatio", "Controls", &sbm.state->uniquenessRatio, 200, updateParameters);
	}
	else if(selectedAlgorithm == "SGBM")
	{
		cv::createTrackbar("disp12MaxDiff", "Controls", &sgbm.disp12MaxDiff, 200, updateParameters);
		cv::createTrackbar("minDisp", "Controls", &sgbm.minDisparity, 50, updateParameters);
		cv::createTrackbar("numDisps", "Controls", &sgbm.numberOfDisparities, 20, updateParameters);
		cv::createTrackbar("P1", "Controls", &sgbm.P1, 1000, updateParameters);
		cv::createTrackbar("P2", "Controls", &sgbm.P2, 10000, updateParameters);
		cv::createTrackbar("preFiltCap", "Controls", &sgbm.preFilterCap, 63, updateParameters);
		cv::createTrackbar("SADWinSize", "Controls", &sgbm.SADWindowSize, 30, updateParameters);
		cv::createTrackbar("speckleRange", "Controls", &sgbm.speckleRange, 50, updateParameters);
		cv::createTrackbar("speckleWinSize", "Controls", &sgbm.speckleWindowSize, 255, updateParameters);
		cv::createTrackbar("uniqRatio", "Controls", &sbm.state->uniquenessRatio, 200, updateParameters);
	}
	else if(selectedAlgorithm == "SVAR")
	{
		cv::createTrackbar("maxDisp", "Controls", &svar.maxDisp, 150, updateParameters);
		cv::createTrackbar("levels", "Controls", &svar.levels, 10, updateParameters);
		cv::createTrackbar("nIt", "Controls", &svar.nIt, 100, updateParameters);
		cv::createTrackbar("poly_n", "Controls", &svar.poly_n, 255, updateParameters);
	}
	cv::waitKey(0);
}
