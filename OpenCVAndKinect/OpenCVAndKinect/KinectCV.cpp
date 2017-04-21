#include "KinectCV.h"

template <class T>
void SafeRelease(T **ppT)
{
	if (*ppT)
	{
		(*ppT)->Release();
		*ppT = NULL;
	}
}

KinectCV::KinectCV()
{
	bKinectOpen = false;
}

bool KinectCV::OpenKinectDevice()
{
	hResult = S_OK;
	hResult = GetDefaultKinectSensor(&pSensor);
	if (FAILED(hResult))
	{
		std::cerr << "Error : GetDefaultKinectSensor" << std::endl;
		return false;
	}
	
	hResult = pSensor->Open();
	if (FAILED(hResult)) 
	{
		std::cerr << "Error : IKinectSensor::Open()" << std::endl;
		return false;
	}
	
	bKinectOpen = true;

	return true;
}

Mat KinectCV::GetColorImage()
{
	if (!bKinectOpen)
	{
		printf("kinect is not opened\n");
		exit(-1);
	}
	unsigned int bufferSize = 1920 * 1080 * 4 * sizeof(unsigned char);

	IColorFrameSource* pColorSource;  //・・1
	hResult = pSensor->get_ColorFrameSource(&pColorSource);  //・・2
	if (FAILED(hResult)) 
	{
		std::cerr << "Error : IKinectSensor::get_ColorFrameSource()" << std::endl;
		//return -1;
	}

	IColorFrameReader* pColorReader;  //・・1
	hResult = pColorSource->OpenReader(&pColorReader);  //・・2
	if (FAILED(hResult)) 
	{
		std::cerr << "Error : IColorFrameSource::OpenReader()" << std::endl;
		//return -1;
	}

	IColorFrame* pColorFrame = nullptr;  //・・4
	hResult = pColorReader->AcquireLatestFrame(&pColorFrame);  //・・5
	if (SUCCEEDED(hResult)) {
		hResult = pColorFrame->CopyConvertedFrameDataToArray(bufferSize, reinterpret_cast<BYTE*>(colorBufferMat.data), ColorImageFormat_Bgra);  //・・6
		if (SUCCEEDED(hResult)) {
			cv::resize(colorBufferMat, colorMat, cv::Size(), 0.5, 0.5);   //・・7
		}
	}
	SafeRelease(&pColorFrame);   //・・8

}

Mat KinectCV::GetDepthImage()
{
	if (!bKinectOpen)
	{
		printf("kinect is not opened\n");
		exit(-1);
	}

	int dWidth = 512;
	int dHeight = 424;
	unsigned int BufferSize = dWidth * dHeight * sizeof(unsigned short);
	IDepthFrameSource* pDepthSource = nullptr;  
	IDepthFrameReader* pDepthReader = nullptr;
	hResult = pSensor->get_DepthFrameSource(&pDepthSource);  
	
	if (FAILED(hResult)) 
	{
		std::cerr << "Error : IKinectSensor::get_DepthFrameSource()" << std::endl;
		return depthMat;
	}
	
	hResult = pDepthSource->OpenReader(&pDepthReader);  
	if (FAILED(hResult)) {
		std::cerr << "Error : IDepthFrameSource::OpenReader()" << std::endl;
		return depthMat;
	}

	IDepthFrame* pDepthFrame = nullptr;
	if (SUCCEEDED(hResult)) 
	{
		hResult = pDepthFrame->AccessUnderlyingBuffer(&BufferSize, reinterpret_cast<UINT16**>(&depthBufferMat.data));  //・・6
		if (SUCCEEDED(hResult)) 
		{
			depthBufferMat.convertTo(depthMat, CV_16U, -16000.f / 8000.0f, 16000);  
		}
	}
	SafeRelease(&pDepthFrame);  //・・8

	return depthMat;
}
