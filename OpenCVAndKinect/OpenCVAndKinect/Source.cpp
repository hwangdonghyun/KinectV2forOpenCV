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
	KinectCV kcv;
	kcv.OpenKinectDevice();
	while (1)
	{
		Mat colorImage = kcv.GetColorImage();
		//Mat depthImage = kcv.GetDepthImage();
		
		imshow("color", colorImage);
		//imshow("depth", depthImage);
		if (cvWaitKey(30) > 0)
			break;
	}
}
