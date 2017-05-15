/**
* @mainpage KinectCV 0.1\n
* @brief Wrapping Kinect SDK library for OpenCV\n
* @details This is a library for OpenCV to control kinect V2.\n
Made by D.H. Hwang.\n
*/

#pragma once
#include <Kinect.h>
#include <opencv2\opencv.hpp>
#include <vector>
using namespace cv;

#define LIB_VER 0.1f

#define COLOR_HEIGHT 1080
#define COLOR_WIDTH 1920
#define DEPTH_HEIGHT 424
#define DEPTH_WIDTH 512
#define OPEN_COLOR 1
#define OPEN_DEPTH 2
#define OPEN_BODYINDEX 4
#define OPEN_BODYPARTS 8
#define MAX_BODY_COUNT 6

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

	//Body
	IBodyIndexFrameSource* pBodyIndexSource;
	IBodyIndexFrameReader* pBodyIndexReader;
	IBodyIndexFrame* pBodyIndexFrame;

	//BodyParts
	IBodyFrameSource* pBodySource;
	IBodyFrameReader* pBodyReader;
	IBodyFrame* pBodyFrame;
	ICoordinateMapper* pCoordinateMapper;

	//OpenCV Datatype
	Mat colorBufferMat; 
	Mat colorMat;
	Mat depthBufferMat;
	Mat depthMat;
	Mat bodyIndexMat;

	Mat bpBufferMat;
	Mat bpMat;
	Vec3b color[MAX_BODY_COUNT];

public:
	
	KinectCV(uchar functionSet);
	bool OpenKinectDevice();
	void CheckDeviceOpen();
	Mat GetColorImage();
	Mat GetDepthImage();
	Mat GetBodyIndex();
	Mat GetBodyParts();
	
};