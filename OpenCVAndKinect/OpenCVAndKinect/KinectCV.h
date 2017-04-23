/**
* @mainpage KinectCV 0.1\n
* @brief Wrapping Kinect SDK library for OpenCV\n
* @details This is a library for OpenCV to control kinect V2.\n
Made by D.H. Hwang.\n
*/

#pragma once
#include <Kinect.h>
#include <opencv2\opencv.hpp>

using namespace cv;

#define LIB_VER 0.1f

#define COLOR_HEIGHT 1080
#define COLOR_WIDTH 1920
#define DEPTH_HEIGHT 424
#define DEPTH_WIDTH 512


class KinectCV
{
private:
	bool bKinectOpen;
	//Kinect Device
	HRESULT hResult;
	IKinectSensor* pSensor;

	//Depth
	IDepthFrameSource* pDepthSource;
	IDepthFrameReader* pDepthReader;
	IDepthFrame* pDepthFrame;

	//Color
	IColorFrameSource* pColorSource;  
	IColorFrameReader* pColorReader;  
	IColorFrame* pColorFrame;

	//OpenCV Datatype
	Mat colorBufferMat; 
	Mat colorMat;
	Mat depthBufferMat;
	Mat depthMat;

public:
	KinectCV();
	bool OpenKinectDevice();
	void CheckDeviceOpen();
	Mat GetColorImage();
	Mat GetDepthImage();

};