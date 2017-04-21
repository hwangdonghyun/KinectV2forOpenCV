#pragma once
#include <Kinect.h>
#include <opencv2\opencv.hpp>

using namespace cv;
class KinectCV
{
public:
	bool bKinectOpen;
	//Kinect Library
	HRESULT hResult;
	IKinectSensor* pSensor;
	
	IDepthFrameSource* pDepthSource;
	
	IDepthFrame* pDepthFrame;

	//OpenCV Library
	Mat colorBufferMat; 
	Mat colorMat;
	Mat depthBufferMat;
	Mat depthMat;


	KinectCV();
	bool OpenKinectDevice();
	Mat GetColorImage();
	Mat GetDepthImage();

};