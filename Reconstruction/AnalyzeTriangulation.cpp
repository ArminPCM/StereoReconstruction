#include "AnalyzeTriangulation.h"


AnalyzeTriangulation::AnalyzeTriangulation(std::string inPath)
{
	sCam = new StereoCamera(inPath);
}


AnalyzeTriangulation::~AnalyzeTriangulation(void)
{
}

void AnalyzeTriangulation::initialize(int imgNumber, std::string inPathStr)
{
	c1_projMat = cv::Mat::zeros(3, 4, CV_32FC1);
	c2_projMat = cv::Mat::zeros(3, 4, CV_32FC1);
	for (int row=0; row<3; row++)
	{
		for(int col=0; col<3; col++)
		{
			if(row==col)
			{
				c1_projMat.at<float>(row, col) = 1;
			}
			else 
			{
				c1_projMat.at<float>(row, col) = 0;
			}

			c2_projMat.at<float>(row, col) = (*sCam).c1_R_c2.at<double>(row, col);
		}
	}

	int col = 3;
	for (int row=0; row<3; row++)
	{
		c1_projMat.at<float>(row, col) = 0;
		c2_projMat.at<float>(row, col) = (*sCam).c1_T_c2.at<double>(row, 0);
	}
	cv::Mat c1_calib_32F;
	sCam->c1_calib.convertTo(c1_calib_32F, CV_32FC1);
	c1_projMat = c1_calib_32F*c1_projMat;
	c2_projMat = c1_calib_32F*c2_projMat;

	std::cout << "c1_projMat Matrix: " <<std::endl<< c1_projMat <<std::endl<<std::endl;
	std::cout << "c2_projMat Matrix: " <<std::endl<< c2_projMat <<std::endl<<std::endl;

	/*cv::stereoRectify(sCam->c1_calib, sCam->c1_undistortion, sCam->c2_calib, sCam->c2_undistortion, sCam->rightImage.size(), 
		sCam->c1_R_c2, sCam->c1_T_c2, sCam->c1_RectTransform, sCam->c2_RectTransform, sCam->c1_ProjRect, sCam->c2_ProjRect, sCam->disparityToDepth, 0, 0.7, sCam->rightImage.size(), &sCam->c1_RectROI, &sCam->c2_RectROI);*/
	// Create Q matrix for OpenCV triangulation
	sCam->disparityToDepth = cv::Mat::zeros(4, 4, CV_64FC1);
	sCam->disparityToDepth.at<double>(0, 0) = 1;
	sCam->disparityToDepth.at<double>(1, 1) = 1;
	sCam->disparityToDepth.at<double>(0, 3) = -sCam->c1_calib.at<double>(0, 2);
	sCam->disparityToDepth.at<double>(1, 3) = -sCam->c1_calib.at<double>(1, 2);
	sCam->disparityToDepth.at<double>(2, 3) = sCam->c1_calib.at<double>(0, 0);
	sCam->disparityToDepth.at<double>(3, 2) = -1/sCam->c1_T_c2.at<double>(0, 0);
	sCam->disparityToDepth.at<double>(3, 3) = 0;


	std::cout << "Disparity to depth matrix:\n" <<std::endl<< sCam->disparityToDepth << "\n\n";

	std::cout << "c1_T_c2 Matrix: " <<std::endl<< sCam->c1_T_c2 << std::endl << std::endl;
	
	// Create Skew Symmetric of Translation
	cv::Mat c1_T_c2_Skew(3, 3, CV_64FC1);
	c1_T_c2_Skew.at<double>(0, 0) = 0;
	c1_T_c2_Skew.at<double>(0, 1) = -sCam->c1_T_c2.at<double>(2, 0);
	c1_T_c2_Skew.at<double>(0, 2) = sCam->c1_T_c2.at<double>(1, 0);
	c1_T_c2_Skew.at<double>(1, 0) = sCam->c1_T_c2.at<double>(2, 0);
	c1_T_c2_Skew.at<double>(1, 1) = 0;
	c1_T_c2_Skew.at<double>(1, 2) = -sCam->c1_T_c2.at<double>(0, 0);
	c1_T_c2_Skew.at<double>(2, 0) = -sCam->c1_T_c2.at<double>(1, 0);
	c1_T_c2_Skew.at<double>(2, 1) = sCam->c1_T_c2.at<double>(0, 0);
	c1_T_c2_Skew.at<double>(2, 2) = 0;

	std::cout << "c1_T_c2_Skew Matrix: " <<std::endl<< c1_T_c2_Skew << std::endl << std::endl;
	// Create Essential Matrix
	essentialMat = sCam->c1_R_c2*c1_T_c2_Skew;

	std::cout << "essentialMat Matrix: " <<std::endl<< essentialMat << std::endl << std::endl;

	cv::Mat cam1Mat_inverse, cam2Mat_transpose, cam2Mat__transpose_inverse;
	cv::transpose(sCam->c2_calib, cam2Mat_transpose);
	cv::invert(sCam->c1_calib, cam1Mat_inverse);
	cv::invert(cam2Mat_transpose, cam2Mat__transpose_inverse);

	std::cout << "cam1Mat_inverse Matrix: " <<std::endl<< cam1Mat_inverse <<std::endl;
	std::cout << "cam2Mat__transpose_inverse Matrix: " <<std::endl<< cam2Mat__transpose_inverse <<std::endl;
	
	// Calculate Fundamental Matrix
	fundamentalMat = cam2Mat__transpose_inverse*essentialMat*cam1Mat_inverse;
	std::cout << "Fundamental Matrix: " <<std::endl<< fundamentalMat <<std::endl;

	// Load Images
	std::string fileNumberString;
	std::ostringstream convert;
	convert << imgNumber;
	fileNumberString = convert.str();

	std::string leftImageAddr = inPathStr + "\\images\\Left_img" + fileNumberString + ".png";
	std::string rightImageAddr = inPathStr + "\\images\\Right_img" + fileNumberString + ".png";
	std::string maskImageAddr = inPathStr + "\\images\\MaskImageRect.jpg";
	sCam->loadImages(leftImageAddr, rightImageAddr, maskImageAddr);

	cv::Mat origLeft = sCam->leftImageRect_gray;
	cv::Mat origRight = sCam->rightImageRect_gray;
	origLeft.convertTo(origLeft, CV_32FC1);
	origRight.convertTo(origRight, CV_32FC1);

	// Load GroundTruth Disparity
	std::string dispFileName = inPathStr + "\\GroundTruthDisparities\\Disparity_GT" + fileNumberString + ".csv";
		
	CvMLData mlData;
	mlData.read_csv(dispFileName.c_str());
	const CvMat* tmp = mlData.get_values();
	cv::Mat Disparity_gt(tmp, true);
	tmp->CvMat::~CvMat();
	disp_gt = Disparity_gt.clone();


	// Load GroundTruth 3D
	std::string xFileName = inPathStr + "\\3D_GT\\3D_GT" + fileNumberString + "_X.csv";

	mlData.read_csv(xFileName.c_str());
	const CvMat* tmpX = mlData.get_values();
	cv::Mat X(tmpX, true);
	tmpX->CvMat::~CvMat();
	X_gt = X.clone();

	std::string yFileName = inPathStr + "\\3D_GT\\3D_GT" + fileNumberString + "_Y.csv";

	mlData.read_csv(yFileName.c_str());
	const CvMat* tmpY = mlData.get_values();
	cv::Mat Y(tmpY, true);
	tmpY->CvMat::~CvMat();
	Y_gt = Y.clone();

	std::string zFileName = inPathStr + "\\3D_GT\\3D_GT" + fileNumberString + "_Z.csv";

	mlData.read_csv(zFileName.c_str());
	const CvMat* tmpZ = mlData.get_values();
	cv::Mat Z(tmpZ, true);
	tmpZ->CvMat::~CvMat();
	Z_gt = Z.clone();

	// Load Mask
	std::string maskFileName = inPathStr + "\\GroundTruthDisparities\\Mask_Disparity_GT" + fileNumberString + ".csv";

	mlData.read_csv(maskFileName.c_str());
	const CvMat* tmpMask = mlData.get_values();
	cv::Mat mask_disparity(tmpMask, true);
	tmpMask->CvMat::~CvMat();
	disp_mask_gt = mask_disparity.clone();
}

float AnalyzeTriangulation::calculateRMSE(std::vector<float> distances)
{
	float rmse = 0;
	for (int i=0; i<distances.size(); i++)
	{
		rmse += distances[i]*distances[i];
	}
	rmse = rmse / distances.size();
	rmse = sqrt(rmse);
	return rmse;
}

void AnalyzeTriangulation::triangulate(std::string algo)
{
	std::vector<cv::Point3f> points_3D;
	if(algo == "OpenCV")
	{
		// reprojectImageTo3D doesn't work properly
		//cv::reprojectImageTo3D(disp_gt, pointCloud, sCam->disparityToDepth, true, CV_32F);
		// Comparison
		std::vector<float> depth_values;
		depth_values.clear();
		for(int row=0; row<X_gt.rows; row++)
		{
			for(int col=0; col<X_gt.cols; col++)
			{
				if(int(disp_mask_gt.at<float>(row, col)) != 0)
				{
					if(col + disp_gt.at<float>(row, col) < disp_gt.cols && col + disp_gt.at<float>(row, col) >= 0)
					{
						// Rotated by rotox(-pi/2)
						// std::cout << matType2Str(X_gt.type()) << std::endl;
						float W = sCam->disparityToDepth.at<double>(3, 2) * disp_gt.at<float>(row, col) + sCam->disparityToDepth.at<double>(3, 3);
						float X_3d = (sCam->disparityToDepth.at<double>(0, 0) * row + sCam->disparityToDepth.at<double>(0, 3)) / W;
						float Y_3d = (sCam->disparityToDepth.at<double>(1, 1) * col + sCam->disparityToDepth.at<double>(1, 3)) / W;
						float Z_3d = sCam->disparityToDepth.at<double>(2, 3) / W;
						float difX = (X_gt.at<float>(row, col) - X_3d*1000)/1000;//pointCloud.at<cv::Vec3f>(row, col)[0]*1000;
						float difY = (Y_gt.at<float>(row, col) - Y_3d*1000)/1000;//(-pointCloud.at<cv::Vec3f>(row, col)[2]*1000);
						float difZ = (Z_gt.at<float>(row, col) - Z_3d*1000)/1000;//pointCloud.at<cv::Vec3f>(row, col)[1]*1000;

						float dist = sqrt(difX*difX + difY*difY + difZ*difZ);

						if(row == 346 && col == 1469)
						{
							std::cout << "Dist is: " << dist << " =  " << difX*difX << " + " << difY*difY << " + " << difZ*difZ <<
								", Disp:" << disp_gt.at<float>(row, col) << "\n";
							std::cout << "X_gt: " << X_gt.at<float>(row, col) << ", " << X_3d*1000;
							std::cout << "\nY_gt: " << Y_gt.at<float>(row, col) << ", " << Y_3d*1000;
							std::cout << "\nZ_gt: " << Z_gt.at<float>(row, col) << ", " << Z_3d*1000;
							std::cout << "\nDisparity: " << disp_gt.at<float>(row, col) << std::endl;
						}

						depth_values.push_back(dist);

						// Insert the point into the vector to visualize
						cv::Point3f tmpPoint;
						tmpPoint.x = X_3d;
						tmpPoint.y = Y_3d;
						tmpPoint.z = Z_3d;
						points_3D.push_back(tmpPoint);
					}
				}
			}
		}
		// Calculate statistics
		auto result = std::minmax_element (depth_values.begin(),depth_values.end());
		min = *result.first;
		max = *result.second;
		sum = std::accumulate(depth_values.begin(),depth_values.end(), 0.0);
		mean = sum/depth_values.size();
		deviation = standard_deviation(depth_values, mean);
		std::nth_element(depth_values.begin(), depth_values.begin()+depth_values.size()/2, depth_values.end());
		median = depth_values[depth_values.size()/2];
		RMSE = calculateRMSE(depth_values);
	}
	else if(algo == "Optimal")
	{

		// for each point that we have a correspondance!
		std::vector<float> depth_values;
		for(int row=0; row<disp_gt.rows; row++)
		{
			for(int col=0; col<disp_gt.cols; col++)
			{
				if(int(disp_mask_gt.at<float>(row, col)) != 0)
				{
					// Create points matrix
					cv::Mat c1_point(1, 1, CV_32FC2);
					cv::Mat c2_point(1, 1, CV_32FC2);
					cv::Mat points4D(4, 1, CV_32FC1);

					cv::Mat c1_newPoint(1, 2, CV_32FC2);
					cv::Mat c2_newPoint(1, 2, CV_32FC2);

					cv::Mat c1_triPoint(2, 1, CV_32FC1);
					cv::Mat c2_triPoint(2, 1, CV_32FC1);

					c1_point.at<cv::Vec2f>(0, 0)[0] = col;
					c1_point.at<cv::Vec2f>(0, 0)[1] = row;

					c2_point.at<cv::Vec2f>(0, 0)[0] = col - disp_gt.at<float>(row, col);
					c2_point.at<cv::Vec2f>(0, 0)[1] = row;

					// Correcting Matches
					cv::correctMatches(fundamentalMat, c1_point, c1_point, c2_newPoint, c2_newPoint);
					c1_triPoint.at<float>(0, 0) = c1_newPoint.at<cv::Vec2f>(0, 0)[0];
					c1_triPoint.at<float>(1, 0) = c1_newPoint.at<cv::Vec2f>(0, 0)[1];
					c2_triPoint.at<float>(0, 0) = c2_newPoint.at<cv::Vec2f>(0, 0)[0];
					c2_triPoint.at<float>(1, 0) = c2_newPoint.at<cv::Vec2f>(0, 0)[1];

					// Triangulate Points
					cv::triangulatePoints(c1_projMat, c2_projMat, c1_triPoint, c2_triPoint, points4D);

					cv::Mat pt_3d; cv::convertPointsFromHomogeneous(cv::Mat(points4D.t()).reshape(4, 1),pt_3d);
					std::cout << "points4D:\n" << points4D << "\n\n";
					std::cout << "c1_point:\n" << c1_point << "\n\n";
					std::cout << "c2_point:\n" << c2_point << "\n\n";
					std::cout << "c1_triPoint:\n" << c1_triPoint << "\n\n";
					std::cout << "c2_triPoint:\n" << c2_triPoint << "\n\n";

					float X_hat = pt_3d.at<cv::Point3f>(0).x;
					float Y_hat = pt_3d.at<cv::Point3f>(0).y;
					float Z_hat = pt_3d.at<cv::Point3f>(0).z;

					if(c2_point.at<float>(1, 0) < disp_gt.cols && c2_point.at<float>(1, 0) >= 0)
					{

						float difX = (X_gt.at<float>(row, col) - X_hat*1000)/1000;
						float difY = (Y_gt.at<float>(row, col) - Y_hat*1000)/1000;
						float difZ = (Z_gt.at<float>(row, col) - Z_hat*1000)/1000;
						float dist = sqrt(difX*difX + difY*difY + difZ*difZ);

						if(row == 346 && col == 1469)
						{
							std::cout << "points4D: \n" << points4D << "\n\n";
							std::cout << "Dist is: " << dist << " =  " << difX*difX << " + " << difY*difY << " + " << difZ*difZ <<
								", Disp:" << disp_gt.at<float>(row, col) << "\n";
							std::cout << "X_gt: " << X_gt.at<float>(row, col) << ", " << X_hat*1000;
							std::cout << "\nY_gt: " << Y_gt.at<float>(row, col) << ", " << Y_hat*1000;
							std::cout << "\nZ_gt: " << Z_gt.at<float>(row, col) << ", " << Z_hat*1000;
							std::cout << "\nDisparity: " << disp_gt.at<float>(row, col) << std::endl;

							std::cout << "c1_point: \n" << c1_point << "\n\n";
							std::cout << "c2_point: \n" << c2_point << "\n\n";
						}

						depth_values.push_back(dist);
						
						// Insert the point into the vector to visualize
						cv::Point3f tmpPoint;
						tmpPoint.x = X_hat;
						tmpPoint.y = Y_hat;
						tmpPoint.z = Z_hat;
						points_3D.push_back(tmpPoint);
					}
				}
			}
		}
		// Calculate statistics
		auto result = std::minmax_element (depth_values.begin(),depth_values.end());
		min = *result.first;
		max = *result.second;
		sum = std::accumulate(depth_values.begin(),depth_values.end(), 0.0);
		mean = sum/depth_values.size();
		deviation = standard_deviation(depth_values, mean);
		std::nth_element(depth_values.begin(), depth_values.begin()+depth_values.size()/2, depth_values.end());
		median = depth_values[depth_values.size()/2];
		RMSE = calculateRMSE(depth_values);
#if defined SAVE_3D
	ofstream out3D;
	out3D.open("..\\3D.txt", std::ios::out);
	for(int point=0; point<points_3D.size()-1; point++)
	{
		out3D << points_3D[point].x << ", ";
	}
	out3D << points_3D[points_3D.size()-1].x << "\n";
	for(int point=0; point<points_3D.size()-1; point++)
	{
		out3D << points_3D[point].y << ", ";
	}
	out3D << points_3D[points_3D.size()-1].y << "\n";
	for(int point=0; point<points_3D.size()-1; point++)
	{
		out3D << points_3D[point].z << ", ";
	}
	out3D << points_3D[points_3D.size()-1].z << "\n";
#endif
	}
	else if(algo == "Linear")
	{
		// for each point that we have a correspondance!
		std::vector<float> depth_values;
		for(int row=0; row<disp_gt.rows; row++)
		{
			for(int col=0; col<disp_gt.cols; col++)
			{
				if(int(disp_mask_gt.at<float>(row, col)) != 0)
				{
					// Create points matrix
					cv::Mat c1_point(2, 1, CV_32FC1);
					cv::Mat c2_point(2, 1, CV_32FC1);
					cv::Mat points4D(4, 1, CV_32FC1);

					c1_point.at<float>(0, 0) = row;
					c1_point.at<float>(1, 0) = col;

					c2_point.at<float>(0, 0) = row;
					c2_point.at<float>(1, 0) = col - disp_gt.at<float>(row, col);

					// Triangulate Points
					cv::triangulatePoints(c1_projMat, c2_projMat, c1_point, c2_point, points4D);

					cv::Mat pt_3d; cv::convertPointsFromHomogeneous(cv::Mat(points4D.t()).reshape(4, 1),pt_3d);

					float X_hat = pt_3d.at<cv::Point3f>(0).x;
					float Y_hat = pt_3d.at<cv::Point3f>(0).y;
					float Z_hat = pt_3d.at<cv::Point3f>(0).z;

					if(c2_point.at<float>(1, 0) < disp_gt.cols && c2_point.at<float>(1, 0) >= 0)
					{

						float difX = (X_gt.at<float>(row, col) - X_hat*1000)/1000;
						float difY = (Y_gt.at<float>(row, col) - Y_hat*1000)/1000;
						float difZ = (Z_gt.at<float>(row, col) - Z_hat*1000)/1000;
						float dist = sqrt(difX*difX + difY*difY + difZ*difZ);

						if(row == 346 && col == 1469)
						{
							std::cout << "points4D: \n" << points4D << "\n\n";
							std::cout << "Dist is: " << dist << " =  " << difX*difX << " + " << difY*difY << " + " << difZ*difZ <<
								", Disp:" << disp_gt.at<float>(row, col) << "\n";
							std::cout << "X_gt: " << X_gt.at<float>(row, col) << ", " << X_hat*1000;
							std::cout << "\nY_gt: " << Y_gt.at<float>(row, col) << ", " << Y_hat*1000;
							std::cout << "\nZ_gt: " << Z_gt.at<float>(row, col) << ", " << Z_hat*1000;
							std::cout << "\nDisparity: " << disp_gt.at<float>(row, col) << std::endl;

							std::cout << "c1_point: \n" << c1_point << "\n\n";
							std::cout << "c2_point: \n" << c2_point << "\n\n";
						}

						depth_values.push_back(dist);
						
						// Insert the point into the vector to visualize
						cv::Point3f tmpPoint;
						tmpPoint.x = X_hat;
						tmpPoint.y = Y_hat;
						tmpPoint.z = Z_hat;
						points_3D.push_back(tmpPoint);
					}
				}
			}
		}

		// Calculate statistics
		auto result = std::minmax_element (depth_values.begin(),depth_values.end());
		min = *result.first;
		max = *result.second;
		sum = std::accumulate(depth_values.begin(),depth_values.end(), 0.0);
		mean = sum/depth_values.size();
		deviation = standard_deviation(depth_values, mean);
		std::nth_element(depth_values.begin(), depth_values.begin()+depth_values.size()/2, depth_values.end());
		median = depth_values[depth_values.size()/2];
		RMSE = calculateRMSE(depth_values);
	}
#if defined SAVE_3D
	ofstream out3D;
	out3D.open("..\\3D.txt", std::ios::out);
	for(int point=0; point<points_3D.size()-1; point++)
	{
		out3D << points_3D[point].x << ", ";
	}
	out3D << points_3D[points_3D.size()-1].x << "\n";
	for(int point=0; point<points_3D.size()-1; point++)
	{
		out3D << points_3D[point].y << ", ";
	}
	out3D << points_3D[points_3D.size()-1].y << "\n";
	for(int point=0; point<points_3D.size()-1; point++)
	{
		out3D << points_3D[point].z << ", ";
	}
	out3D << points_3D[points_3D.size()-1].z << "\n";
#endif
}

void AnalyzeTriangulation::runTestOpenCV(std::string inPathStr, std::string outPathStr, int imgNumber)
{
	std::string algo = "Optimal";
	std::string outFilePath;
	std::ofstream err_stats;
	if(algo == "Linear")
	{
		//sCam->c1_T_c2.at<double>(0, 0) = sCam->c1_T_c2.at<double>(1, 0);
		//sCam->c1_T_c2.at<double>(1, 0) = 0;
		outFilePath = outPathStr + "\\err_Reconstruction_Linear" + static_cast<std::ostringstream*>( &(std::ostringstream() << imgNumber) )->str() + ".txt";
	}
	if(algo == "OpenCV")
	{
		sCam->c1_T_c2.at<double>(0, 0) = sCam->c1_T_c2.at<double>(1, 0);
		sCam->c1_T_c2.at<double>(1, 0) = 0;
		outFilePath = outPathStr + "\\err_Reconstruction_OpenCV" + static_cast<std::ostringstream*>( &(std::ostringstream() << imgNumber) )->str() + ".txt";
	}
	if(algo == "Optimal") 
	{
		sCam->c1_T_c2.at<double>(0, 0) = sCam->c1_T_c2.at<double>(1, 0);
		sCam->c1_T_c2.at<double>(1, 0) = 0;
		outFilePath = outPathStr + "\\err_Reconstruction_Optimal" + static_cast<std::ostringstream*>( &(std::ostringstream() << imgNumber) )->str() + ".txt";
	}
	int numberOfImages = 5;
	int numberOfIterations = 30;
	float noiseLevels[5] = {0.00, 0.01, 0.05, 0.10, 0.15};
	err_stats.open(outFilePath, std::ios::out);
	time_t t = time(0);   // get time now
	struct tm * now = localtime( & t );
    err_stats << "Date: " << (now->tm_year + 1900) << '-' 
         << (now->tm_mon + 1) << '-'
         <<  now->tm_mday
         << endl;
	err_stats << "Reconstruction Analyzer\n";
	err_stats << "Algorithm: OpenCV\n";
	err_stats << "Trial_# - (Min, Max, Mean, STD, Median, RMSE)\n";
	err_stats << "================================================================================\n";
	err_stats << "================================================================================\n";

	initialize(imgNumber, inPathStr);
	cv::Mat OrigDisp = disp_gt.clone();
	for(int noise=0; noise<=4; noise++)
	{
		err_stats << "================================================================================\n";
			err_stats << "Noise Level: " << noiseLevels[noise]*100 << "%\n";
			err_stats << "================================================================================\n";
		for(int trial=29; trial<30; trial++)
		{
			if(noise > 0)
			{
				// Add noise to the disparity
				cv::Mat gaussian_noise_disp = OrigDisp.clone();
				cv::randn(gaussian_noise_disp,0, noiseLevels[noise]*90);

				disp_gt = OrigDisp + gaussian_noise_disp;
			}
			else
				disp_gt = OrigDisp;
			//if(noise==3 && trial==16)
			triangulate(algo);
			err_stats << "Trial_" << noise << "_" << trial << " - (" << min << ", " << max << ", " << mean << ", " <<  deviation << ", " << median << ", " << RMSE << ")\n";
			err_stats.flush();
		}
	}
}


std::string AnalyzeTriangulation::matType2Str(int type) 
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