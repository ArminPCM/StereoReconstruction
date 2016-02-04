#include "StereoCamera.h"
#include "MiscFunctions.h"
#include <sstream>
#include <string>
#include <ctime>
#include <windows.h>
#include "AnalyzeTriangulation.h"

/** Use to init the clock */
#define TIMER_INIT \
	LARGE_INTEGER frequency; \
	LARGE_INTEGER t1,t2; \
	double elapsedTime; \
	QueryPerformanceFrequency(&frequency);


/** Use to start the performance timer */
#define TIMER_START QueryPerformanceCounter(&t1);

/** Use to stop the performance timer and output the result to the standard stream. Less verbose than \c TIMER_STOP_VERBOSE */
#define TIMER_STOP \
	QueryPerformanceCounter(&t2); \
	elapsedTime=(float)(t2.QuadPart-t1.QuadPart)/frequency.QuadPart; \
	std::wcout<<elapsedTime<<endl;//L" sec"<<endl;

void findParam_SGBM(StereoCamera sCam, std::string inPathStr, std::string outPathStr, int imgNumber)
{
	sCam.algorithmType = "SGBM";
	int numberOfImages = 1;
	int numberOfIterations = 30;
	for(imgNumber=1; imgNumber<=numberOfImages; imgNumber++)
	{
		// Load Images
		std::string fileNumberString;
		std::ostringstream convert;
		convert << imgNumber;
		fileNumberString = convert.str();

		std::string leftImageAddr = inPathStr + "\\images\\Left_img" + fileNumberString + ".png";
		std::string rightImageAddr = inPathStr + "\\images\\Right_img" + fileNumberString + ".png";
		std::string maskImageAddr = inPathStr + "\\images\\MaskImageRect.jpg";
		sCam.loadImages(leftImageAddr, rightImageAddr, maskImageAddr);

		cv::Mat origLeft = sCam.leftImageRect_gray;
		cv::Mat origRight = sCam.rightImageRect_gray;
		origLeft.convertTo(origLeft, CV_32FC1);
		origRight.convertTo(origRight, CV_32FC1);
		int paramNumber = 1;

		for(int numDisps=1; numDisps<=4; numDisps++)
		{
			for(int P1=300; P1<=500; P1+=100)
			{
				for(int WinSize=3; WinSize<=15; WinSize+=2)
				{
					for(int preFilterCap=3; preFilterCap<=7; preFilterCap+=2)
					{
						int speckleRange=10;
						//for(int speckleRange=10; speckleRange<=255; speckleRange+=50)
						{
							int speckleWindowSize=10;
							//for(int speckleWindowSize=10; speckleWindowSize<=255; speckleWindowSize+=50)
							{
								origLeft.convertTo(sCam.leftImageRect_gray, CV_8UC1);
								origRight.convertTo(sCam.rightImageRect_gray, CV_8UC1);
								// Set Images & Parameters:
								sCam.sgbm.numberOfDisparities = numDisps;
								sCam.sgbm.P1 = P1;
								sCam.sgbm.P2 = 10000;
								sCam.sgbm.preFilterCap = preFilterCap;
								sCam.sgbm.SADWindowSize = WinSize;
								sCam.sgbm.speckleRange = speckleRange;
								sCam.sgbm.speckleWindowSize = speckleWindowSize;

								TIMER_INIT
									TIMER_START
									sCam.runDisparity(false);
								TIMER_STOP

									//save disparity image
									std::string paramNumberString;
								std::ostringstream convertParamNumber;
								convertParamNumber << paramNumber;
								paramNumberString = convertParamNumber.str();

								std::string dispFileAddr = outPathStr + "\\Disps\\SGBM\\disp_img" + paramNumberString + ".txt";
								ofstream dispFile;
								dispFile.open(dispFileAddr, std::ios::out);
								dispFile << sCam.disparityMap;
								paramNumber++;
							}
						}
					}
				}
			}
		}
	}
}

void findParam_SVAR(StereoCamera sCam, std::string inPathStr, std::string outPathStr, int imgNumber)
{
	sCam.algorithmType = "SVAR";
	int numberOfImages = 1;
	for(imgNumber=1; imgNumber<=numberOfImages; imgNumber++)
	{
		// Load Images
		std::string fileNumberString;
		std::ostringstream convert;
		convert << imgNumber;
		fileNumberString = convert.str();

		std::string leftImageAddr = inPathStr + "\\images\\Left_img" + fileNumberString + ".png";
		std::string rightImageAddr = inPathStr + "\\images\\Right_img" + fileNumberString + ".png";
		std::string maskImageAddr = inPathStr + "\\images\\MaskImageRect.jpg";
		sCam.loadImages(leftImageAddr, rightImageAddr, maskImageAddr);

		cv::Mat origLeft = sCam.leftImageRect_gray;
		cv::Mat origRight = sCam.rightImageRect_gray;
		origLeft.convertTo(origLeft, CV_32FC1);
		origRight.convertTo(origRight, CV_32FC1);
		int paramNumber = 1;
		for(int minDisp=40; minDisp<=50; minDisp+=5)
		{
			for(int levels=10; levels<=30; levels+=5)
			{
				for(int nIt=5; nIt<=15; nIt+=5)
				{
					//for(int poly_n=3; poly_n<=15; poly_n+=2)
					int poly_n = 3;
					{
						for(int poly_sigma=85; poly_sigma<=95; poly_sigma+=5)
						{
							for(int fi=2000; fi<=2500; fi+=500)
							{
								//for(int lambda=3; lambda<=7; lambda+=2)
								int lambda = 5;
								{
									//for(int penalization=0; penalization<=2; penalization++)
									int penalization=0;
									{
										//for(int cycle=0; cycle<=1; cycle++)
										int cycle=1;
										{
											origLeft.convertTo(sCam.leftImageRect_gray, CV_8UC1);
											origRight.convertTo(sCam.rightImageRect_gray, CV_8UC1);
											// Set Images & Parameters:
											sCam.svar.cycle = cycle;
											sCam.svar.fi = fi;
											sCam.svar.lambda = lambda/10.0;
											sCam.svar.levels = levels;
											sCam.svar.maxDisp = 0;
											sCam.svar.minDisp = - minDisp;
											sCam.svar.nIt = nIt;
											sCam.svar.penalization = penalization;
											sCam.svar.poly_n = poly_n;
											sCam.svar.poly_sigma = poly_sigma/100.0;

											TIMER_INIT
												TIMER_START
												sCam.runDisparity(false);
											TIMER_STOP
												//save disparity image
												std::string paramNumberString;
											std::ostringstream convertParamNumber;
											convertParamNumber << paramNumber;
											paramNumberString = convertParamNumber.str();

											std::string dispFileAddr = outPathStr + "\\Disps\\SVAR\\disp_img" + paramNumberString + ".txt";
											ofstream dispFile;
											dispFile.open(dispFileAddr, std::ios::out);
											dispFile << sCam.disparityMap;
											paramNumber++;
										}
									}
								}
							}
						}
					}
				}
			}
		}
	}
}

void findParam_SBM(StereoCamera sCam, std::string inPathStr, std::string outPathStr, int imgNumber)
{
	sCam.algorithmType = "SBM";
	for(imgNumber=1; imgNumber<2; imgNumber++)
	{
		// Load Images
		std::string fileNumberString;
		std::ostringstream convert;
		convert << imgNumber;
		fileNumberString = convert.str();

		std::string leftImageAddr = inPathStr + "\\images\\Left_img" + fileNumberString + ".png";
		std::string rightImageAddr = inPathStr + "\\images\\Right_img" + fileNumberString + ".png";
		std::string maskImageAddr = inPathStr + "\\images\\MaskImageRect.jpg";
		sCam.loadImages(leftImageAddr, rightImageAddr, maskImageAddr);

		cv::Mat origLeft = sCam.leftImageRect_gray;
		cv::Mat origRight = sCam.rightImageRect_gray;
		origLeft.convertTo(origLeft, CV_32FC1);
		origRight.convertTo(origRight, CV_32FC1);
		int paramNumber = 1;
		for(int numDisps=1; numDisps<=3; numDisps++)
		{
			//int preFilterSize=57;
			for(int preFilterSize=45; preFilterSize<=105; preFilterSize+=40)
			{
				for(int preFilterCap=11; preFilterCap<=63; preFilterCap+=20)
					//int preFilterCap=27;
				{

					for(int WinSize=5; WinSize<=91; WinSize+=10)
					{
						int speckleRange=27;
						//for(int speckleRange=10; speckleRange<=255; speckleRange+=50)
						{
							int speckleWindowSize=217;
							for(int speckleWindowSize=10; speckleWindowSize<=255; speckleWindowSize+=50)
							{
								for(int trial=1; trial<=1; trial++)
								{
									origLeft.convertTo(sCam.leftImageRect_gray, CV_8UC1);
									origRight.convertTo(sCam.rightImageRect_gray, CV_8UC1);
									// Set Images & Parameters:
									sCam.sbm.state->numberOfDisparities = numDisps;
									sCam.sbm.state->preFilterCap = preFilterCap;
									sCam.sbm.state->preFilterSize = preFilterSize;
									sCam.sbm.state->SADWindowSize = WinSize;
									sCam.sbm.state->speckleRange = speckleRange;
									sCam.sbm.state->speckleWindowSize = speckleWindowSize;

									TIMER_INIT
										TIMER_START
										sCam.runDisparity(false);
									TIMER_STOP

										//save disparity image
										std::string paramNumberString;
									std::ostringstream convertParamNumber;
									convertParamNumber << paramNumber;
									paramNumberString = convertParamNumber.str();

									std::string dispFileAddr = outPathStr + "\\Disps\\disp_img" + paramNumberString + ".txt";
									ofstream dispFile;
									dispFile.open(dispFileAddr, std::ios::out);
									dispFile << sCam.disparityMap;
									paramNumber++;
								}
							}
						}
					}
				}
			}
		}
	}
}

void main(int argc, char* argv[])
{
	/*std::string inPathStr = "C:\\Users\\MOSTAFAP\\Desktop\\TestKhaliliReconstruction";
	StereoCamera sCam(inPathStr);
	std::string leftImageAddr = inPathStr + "\\images\\Left_img4.png";
	std::string rightImageAddr = inPathStr + "\\images\\Right_img4.png";
	std::string maskImageAddr = inPathStr + "\\images\\MaskImageRect.jpg";

	sCam.loadImages(leftImageAddr, rightImageAddr, maskImageAddr);

	// Add noise
	cv::Mat gaussian_noise_left = sCam.leftImageRect_gray.clone();
	cv::randn(gaussian_noise_left,0, 0.5*255);
	cv::Mat gaussian_noise_right = sCam.rightImageRect_gray.clone();
	cv::randn(gaussian_noise_right,0,0.5*255);

	cv::Mat noisyLeft, noisyRight;
	noisyLeft = sCam.leftImageRect_gray + gaussian_noise_left;
	noisyRight = sCam.rightImageRect_gray + gaussian_noise_right;
	// Upload Images
	noisyLeft.convertTo(noisyLeft, CV_8UC1);
	noisyRight.convertTo(noisyRight, CV_8UC1);
	sCam.leftImageRect_gray = noisyLeft;
	sCam.rightImageRect_gray = noisyRight;

	sCam.algorithmType = "SBM";
	sCam.createGUI("SBM");
	while(true)
	{
	sCam.runDisparity(true);
	std::cout << "numDisps is: " << sCam.sgbm.numberOfDisparities << "\n";
	switch(cv::waitKey(0))
	{
	case ' ':
	//sCam.drawPointCloud3D();
	break;
	case 'x':
	cv::destroyAllWindows();
	sCam.createGUI("SBM");
	break;
	default:
	continue;
	}
	sCam.loadImages(leftImageAddr, rightImageAddr, maskImageAddr);
	}
	*/

	// Remember argv[0] is the path to the program, we want from argv[1] onwards
	std::string inPathStr = "C:\\Users\\MOSTAFAP\\Documents\\Hamlyn database\\Reconstruction\\";
	std::string outPathStr = "C:\\Users\\MOSTAFAP\\Documents\\Hamlyn database\\Reconstruction\\";
	std::string imageNumberStr = "1";
	int imageNumber;
	std::istringstream(imageNumberStr) >> imageNumber;

	// Print command prompt parameters
	std::cout << "Input Path is: " << inPathStr << "\n";
	std::cout << "Output Path is: " << outPathStr << "\n";
	std::cout << "Image Number is: " << imageNumber << "\n";

	std::cout.flush();


	StereoCamera sCam(inPathStr);

	//findParam_SGBM(sCam, inPathStr, outPathStr, imageNumber);
	findParam_SBM(sCam, inPathStr, outPathStr, imageNumber);
	//findParam_SVAR(sCam, inPathStr, outPathStr, imageNumber);

}