#include "opencv2\opencv.hpp"
#include <stdio.h>
#include <Kinect.h>


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
	IKinectSensor* pSensor;
	HRESULT hResult = S_OK;
	hResult = GetDefaultKinectSensor(&pSensor);
	if (FAILED(hResult))
	{
		std::cerr << "Error : GetDefaultKinectSensor" << std::endl;
		return -1;
	}

	hResult = pSensor->Open(); 
	if (FAILED(hResult)) {
		std::cerr << "Error : IKinectSensor::Open()" << std::endl;
		return -1;
	}

	//Source
	IColorFrameSource* pColorSource;  //・・1
	hResult = pSensor->get_ColorFrameSource(&pColorSource);  //・・2
	if (FAILED(hResult)) {
		std::cerr << "Error : IKinectSensor::get_ColorFrameSource()" << std::endl;
		return -1;
	}

	IDepthFrameSource* pDepthSource;  //・・1
	hResult = pSensor->get_DepthFrameSource(&pDepthSource);  //・・2
	if (FAILED(hResult)) {
		std::cerr << "Error : IKinectSensor::get_DepthFrameSource()" << std::endl;
		return -1;
	}

	// Reader
	IColorFrameReader* pColorReader;  //・・1
	hResult = pColorSource->OpenReader(&pColorReader);  //・・2
	if (FAILED(hResult)) {
		std::cerr << "Error : IColorFrameSource::OpenReader()" << std::endl;
		return -1;
	}

	IDepthFrameReader* pDepthReader;  //・・1
	hResult = pDepthSource->OpenReader(&pDepthReader);  //・・2
	if (FAILED(hResult)) {
		std::cerr << "Error : IDepthFrameSource::OpenReader()" << std::endl;
		return -1;
	}

	int width = 1920;   //・・1
	int height = 1080;  //・・1

	int dWidth = 512;
	int dHeight = 424;

	
	unsigned int bufferSize = width * height * 4 * sizeof(unsigned char);  //・・2
	unsigned int dBufferSize = dWidth * dHeight * sizeof(unsigned short);  //・・2
	cv::Mat bufferMat(height, width, CV_8UC4);  //・・3
	cv::Mat colorMat(height / 2, width / 2, CV_8UC4);  //・・3

	cv::Mat dBufferMat(dHeight, dWidth, CV_16UC1);  //・・3
	cv::Mat depthMat(dHeight, dWidth, CV_8UC1);  //・・3
	cv::namedWindow("Depth");
	cv::namedWindow("Color");

	while (1) {
		// Frame
		IColorFrame* pColorFrame = nullptr;  //・・4
		hResult = pColorReader->AcquireLatestFrame(&pColorFrame);  //・・5
		if (SUCCEEDED(hResult)) {
			hResult = pColorFrame->CopyConvertedFrameDataToArray(bufferSize, reinterpret_cast<BYTE*>(bufferMat.data), ColorImageFormat_Bgra);  //・・6
			if (SUCCEEDED(hResult)) {
				cv::resize(bufferMat, colorMat, cv::Size(), 0.5, 0.5);   //・・7
			}
		}
		
		SafeRelease(&pColorFrame);   //・・8

		IDepthFrame* pDepthFrame = nullptr;  //・・4
		hResult = pDepthReader->AcquireLatestFrame(&pDepthFrame);  //・・5
		if (SUCCEEDED(hResult)) {
			hResult = pDepthFrame->AccessUnderlyingBuffer(&dBufferSize, reinterpret_cast<UINT16**>(&dBufferMat.data));  //・・6
			if (SUCCEEDED(hResult)) {
				dBufferMat.convertTo(depthMat, CV_16U, -16000.f / 8000.0f, 16000);  //・・7
				//depthMat = dBufferMat.clone();
			}
			
		}
		SafeRelease(&pDepthFrame);  //・・8

		cv::imshow("Color", colorMat);
		cv::imshow("Depth", depthMat);
		if (cv::waitKey(30) == VK_ESCAPE) {
			break;
		}
	}
}
