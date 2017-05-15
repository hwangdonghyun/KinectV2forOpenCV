#include "opencv2\opencv.hpp"
#include <stdio.h>
#include <Kinect.h>
#include "KinectCV.h"

using namespace cv;


template <class T> 
void SafeRelease(T **ppT)
{
	if (*ppT)
	{
		(*ppT)->Release();
		*ppT = NULL;
	}
}

int main(void)
{
	KinectCV kcv(OPEN_COLOR | OPEN_DEPTH | OPEN_BODYINDEX | OPEN_BODYPARTS);

	while (1)
	{
		//Mat indexImage = kcv.GetBodyIndex();
		Mat colorImage = kcv.GetColorImage();
		//Mat depthImage = kcv.GetDepthImage();
		Mat partsImage = kcv.GetBodyParts2();

	//	imshow("color", colorImage);
	//imshow("depth", depthImage);
	//	imshow("body", indexImage);
		imshow("index", partsImage);
		if (cvWaitKey(1) > 0)
			break;
	}
}
