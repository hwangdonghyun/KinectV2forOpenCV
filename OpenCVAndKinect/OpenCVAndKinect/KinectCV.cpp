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
	hResult = false;
	pSensor = NULL;
	pColorSource = NULL;
	pColorReader = NULL;
	pColorFrame = NULL;

	pDepthReader = NULL;
	pDepthSource = NULL;
	pDepthFrame = NULL;

	colorBufferMat = Mat(COLOR_HEIGHT, COLOR_WIDTH, CV_8UC4);
	colorMat = Mat(COLOR_HEIGHT / 2, COLOR_WIDTH / 2, CV_8UC4);
	depthBufferMat = Mat(DEPTH_HEIGHT, DEPTH_WIDTH, CV_16UC1);
	depthMat = Mat(DEPTH_HEIGHT, DEPTH_WIDTH, CV_16UC1);

	printf("KinectCV Library %.1f\nMade by D.H. Hwang\n",LIB_VER);
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

void KinectCV::CheckDeviceOpen()
{
	if (!bKinectOpen)
	{
		printf("kinect device is not opened\nOpen device first\n");
		exit(-1);
	}
}

Mat KinectCV::GetColorImage()
{
	CheckDeviceOpen();
	
	//Source
	hResult = pSensor->get_ColorFrameSource(&pColorSource);  
	if (FAILED(hResult)) 
	{
		std::cerr << "Error : IKinectSensor::get_ColorFrameSource()" << std::endl;
		return colorMat;
	}

	//Reader
	hResult = pColorSource->OpenReader(&pColorReader); 
	if (FAILED(hResult)) 
	{
		std::cerr << "Error : IColorFrameSource::OpenReader()" << std::endl;
		return colorMat;
	}

	unsigned int bufferSize = COLOR_WIDTH * COLOR_HEIGHT * 4 * sizeof(unsigned char);  

	//Frame
	hResult = pColorReader->AcquireLatestFrame(&pColorFrame);  
	if (SUCCEEDED(hResult)) {
		hResult = pColorFrame->CopyConvertedFrameDataToArray(bufferSize, reinterpret_cast<BYTE*>(colorBufferMat.data), ColorImageFormat_Bgra);  //・・6
		if (SUCCEEDED(hResult)) {
			cv::resize(colorBufferMat, colorMat, cv::Size(), 0.5, 0.5);  
		}
	}
	SafeRelease(&pColorFrame);  

	return colorMat;

}

Mat KinectCV::GetDepthImage()
{
	CheckDeviceOpen();

	unsigned int BufferSize = DEPTH_WIDTH * DEPTH_HEIGHT * sizeof(unsigned short);
	
	//Source
	hResult = pSensor->get_DepthFrameSource(&pDepthSource);  
	if (FAILED(hResult)) 
	{
		std::cerr << "Error : IKinectSensor::get_DepthFrameSource()" << std::endl;
		return depthMat;
	}

	// Reader
	hResult = pDepthSource->OpenReader(&pDepthReader);  
	if (FAILED(hResult)) 
	{
		std::cerr << "Error : IDepthFrameSource::OpenReader()" << std::endl;
		return depthMat;
	}

	// Frame
	hResult = pDepthReader->AcquireLatestFrame(&pDepthFrame);
	if (SUCCEEDED(hResult)) 
	{
		hResult = pDepthFrame->AccessUnderlyingBuffer(&BufferSize, reinterpret_cast<UINT16**>(&depthBufferMat.data));  //・・6
		if (SUCCEEDED(hResult)) 
		{
			depthBufferMat.convertTo(depthMat, CV_8U, -255.0f / 8000.0f, 255.0f);
		}
	}
	SafeRelease(&pDepthFrame); 

	return depthMat;
}
