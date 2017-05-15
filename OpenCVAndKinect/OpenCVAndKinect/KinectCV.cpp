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

KinectCV::KinectCV(uchar functionSet)
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

	pBodyIndexReader = NULL;
	pBodyIndexSource = NULL;
	pBodyIndexFrame = NULL;

	pBodyReader = NULL;
	pBodySource = NULL;
	pBodyFrame = NULL;

	colorBufferMat = Mat(COLOR_HEIGHT, COLOR_WIDTH, CV_8UC4);
	colorMat = Mat(COLOR_HEIGHT / 2, COLOR_WIDTH / 2, CV_8UC4);
	depthBufferMat = Mat(DEPTH_HEIGHT, DEPTH_WIDTH, CV_16UC1);
	depthMat = Mat(DEPTH_HEIGHT, DEPTH_WIDTH, CV_16UC1);
	bodyIndexMat = Mat(DEPTH_HEIGHT, DEPTH_WIDTH, CV_8UC3);

	bpBufferMat = Mat(COLOR_HEIGHT, COLOR_WIDTH, CV_8UC4);
	bpMat = Mat(COLOR_HEIGHT / 2, COLOR_WIDTH / 2, CV_8UC4);

	color[0] = cv::Vec3b(255, 0, 0);
	color[1] = cv::Vec3b(0, 255, 0);
	color[2] = cv::Vec3b(0, 0, 255);
	color[3] = cv::Vec3b(255, 255, 0);
	color[4] = cv::Vec3b(255, 0, 255);
	color[5] = cv::Vec3b(0, 255, 255);

	printf("KinectCV Library %.1f\nMade by D.H. Hwang\n", LIB_VER);

	OpenKinectDevice();

	if (functionSet & OPEN_COLOR)
	{
		//Source
		hResult = pSensor->get_ColorFrameSource(&pColorSource);
		if (FAILED(hResult))
		{
			std::cerr << "Error : IKinectSensor::get_ColorFrameSource()" << std::endl;
			exit(-1);
		}

		//Reader
		hResult = pColorSource->OpenReader(&pColorReader);
		if (FAILED(hResult))
		{
			std::cerr << "Error : IColorFrameSource::OpenReader()" << std::endl;
			exit(-1);
		}

	}

	if (functionSet & OPEN_DEPTH)
	{
		//Source
		hResult = pSensor->get_DepthFrameSource(&pDepthSource);
		if (FAILED(hResult))
		{
			std::cerr << "Error : IKinectSensor::get_DepthFrameSource()" << std::endl;
			exit(-1);
		}

		// Reader
		hResult = pDepthSource->OpenReader(&pDepthReader);
		if (FAILED(hResult))
		{
			std::cerr << "Error : IDepthFrameSource::OpenReader()" << std::endl;
			exit(-1);
		}
	}

	if (functionSet & OPEN_BODYINDEX)
	{
		hResult = pSensor->get_BodyIndexFrameSource(&pBodyIndexSource);  //……2
		if (FAILED(hResult)) {
			std::cerr << "Error : IKinectSensor::get_BodyIndexFrameSource()" << std::endl;
			exit(-1);
		}
		
		hResult = pBodyIndexSource->OpenReader(&pBodyIndexReader);  //……2
		if (FAILED(hResult)) {
			std::cerr << "Error : IBodyIndexFrameSource::OpenReader()" << std::endl;
			exit(-1);
		}
	}

	if (functionSet & OPEN_BODYPARTS)
	{
		hResult = pSensor->get_BodyFrameSource(&pBodySource);  //……2
		if (FAILED(hResult)) {
			std::cerr << "Error : IKinectSensor::get_BodyFrameSource()" << std::endl;
			exit(-1);
		}

		hResult = pBodySource->OpenReader(&pBodyReader);  //……2
		if (FAILED(hResult)) 
		{
			std::cerr << "Error : IBodyFrameSource::OpenReader()" << std::endl;
			exit(-1);
		}

		hResult = pSensor->get_CoordinateMapper(&pCoordinateMapper);  //……2
		if (FAILED(hResult)) 
		{
			std::cerr << "Error : IKinectSensor::get_CoordinateMapper()" << std::endl;
			exit(-1);
		}

	}
}

bool KinectCV::OpenKinectDevice()
{
	hResult = S_OK;
	hResult = GetDefaultKinectSensor(&pSensor);
	if (FAILED(hResult))
	{
		std::cerr << "Error : GetDefaultKinectSensor" << std::endl;
		exit(-1);
	}
	
	hResult = pSensor->Open();
	if (FAILED(hResult)) 
	{
		std::cerr << "Error : IKinectSensor::Open()" << std::endl;
		exit(-1);
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
	
	
	unsigned int bufferSize = COLOR_WIDTH * COLOR_HEIGHT * 4 * sizeof(unsigned char);  

	//Frame
	hResult = pColorReader->AcquireLatestFrame(&pColorFrame);  
	if (SUCCEEDED(hResult)) {
		hResult = pColorFrame->CopyConvertedFrameDataToArray(bufferSize, reinterpret_cast<BYTE*>(colorBufferMat.data), ColorImageFormat_Bgra);  //……6
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
	
	

	// Frame
	hResult = pDepthReader->AcquireLatestFrame(&pDepthFrame);
	if (SUCCEEDED(hResult)) 
	{
		hResult = pDepthFrame->AccessUnderlyingBuffer(&BufferSize, reinterpret_cast<UINT16**>(&depthBufferMat.data));  //……6
		if (SUCCEEDED(hResult)) 
		{
			depthBufferMat.convertTo(depthMat, CV_8U, -255.0f / 8000.0f, 255.0f);
		}
	}
	SafeRelease(&pDepthFrame); 

	return depthMat;
}

Mat KinectCV::GetBodyIndex()
{
	hResult = pBodyIndexReader->AcquireLatestFrame(&pBodyIndexFrame); 
	if (SUCCEEDED(hResult)) 
	{
		unsigned int bufferSize = 0;
		unsigned char* buffer = nullptr;
		hResult = pBodyIndexFrame->AccessUnderlyingBuffer(&bufferSize, &buffer);

		if (SUCCEEDED(hResult)) 
		{
			for (int y = 0; y < DEPTH_HEIGHT; y++) 
			{
				for (int x = 0; x < DEPTH_WIDTH; x++) 
				{
					unsigned int index = y * DEPTH_WIDTH + x;
					
					if (buffer[index] != 0xff) 
						bodyIndexMat.at<cv::Vec3b>(y, x) = color[buffer[index]];  
					else 
						bodyIndexMat.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0);  	
				}
			}
		}
	}

	SafeRelease(&pBodyIndexFrame);

	return bodyIndexMat;
}

Mat KinectCV::GetBodyParts()
{
	hResult = pBodyReader->AcquireLatestFrame(&pBodyFrame);  //……2
	if (SUCCEEDED(hResult)) {
		IBody* pBody[BODY_COUNT] = { 0 };  //……3
		hResult = pBodyFrame->GetAndRefreshBodyData(BODY_COUNT, pBody);  //……3
		if (SUCCEEDED(hResult)) {
			for (int count = 0; count < BODY_COUNT; count++) {
				BOOLEAN bTracked = false;  //……4
				hResult = pBody[count]->get_IsTracked(&bTracked);  //……4
				if (SUCCEEDED(hResult) && bTracked) {
					Joint joint[JointType::JointType_Count];  //……5
					hResult = pBody[count]->GetJoints(JointType::JointType_Count, joint);  //……5
					if (SUCCEEDED(hResult)) {
						// Left Hand State
						HandState leftHandState = HandState::HandState_Unknown;  //……6
						hResult = pBody[count]->get_HandLeftState(&leftHandState);  //……6
						if (SUCCEEDED(hResult)) {
							ColorSpacePoint colorSpacePoint = { 0 };  //……7
							hResult = pCoordinateMapper->MapCameraPointToColorSpace(joint[JointType::JointType_HandLeft].Position, &colorSpacePoint);  //……7
							if (SUCCEEDED(hResult)) {
								int x = static_cast<int>(colorSpacePoint.X);
								int y = static_cast<int>(colorSpacePoint.Y);
								if ((x >= 0) && (x < COLOR_WIDTH) && (y >= 0) && (y < COLOR_HEIGHT)) {
									if (leftHandState == HandState::HandState_Open) {  //……8
										cv::circle(colorBufferMat, cv::Point(x, y), 75, cv::Scalar(0, 128, 0), 5, CV_AA);
									}
									else if (leftHandState == HandState::HandState_Closed) {  //……8
										cv::circle(colorBufferMat, cv::Point(x, y), 75, cv::Scalar(0, 0, 128), 5, CV_AA);
									}
									else if (leftHandState == HandState::HandState_Lasso) {  //……8
										cv::circle(colorBufferMat, cv::Point(x, y), 75, cv::Scalar(128, 128, 0), 5, CV_AA);
									}
								}
							}
						}

						HandState rightHandState = HandState::HandState_Unknown;  //……6
						hResult = pBody[count]->get_HandRightState(&rightHandState);  //……6
						if (SUCCEEDED(hResult)) {
							ColorSpacePoint colorSpacePoint = { 0 };  //……7
							hResult = pCoordinateMapper->MapCameraPointToColorSpace(joint[JointType::JointType_HandRight].Position, &colorSpacePoint);  //……7
							if (SUCCEEDED(hResult)) {
								int x = static_cast<int>(colorSpacePoint.X);
								int y = static_cast<int>(colorSpacePoint.Y);
								if ((x >= 0) && (x < COLOR_WIDTH) && (y >= 0) && (y < COLOR_HEIGHT)) {
									if (rightHandState == HandState::HandState_Open) {  //……8
										cv::circle(colorBufferMat, cv::Point(x, y), 75, cv::Scalar(0, 128, 0), 5, CV_AA);
									}
									else if (rightHandState == HandState::HandState_Closed) {  //……8
										cv::circle(colorBufferMat, cv::Point(x, y), 75, cv::Scalar(0, 0, 128), 5, CV_AA);
									}
									else if (rightHandState == HandState::HandState_Lasso) {  //……8
										cv::circle(colorBufferMat, cv::Point(x, y), 75, cv::Scalar(128, 128, 0), 5, CV_AA);
									}
								}
							}
						}

						// Right Hand State
						/* 左手と同様に右手のHand Stateを取得、状態を描画する。 */

						// Joint  //……9
						for (int type = 0; type < JointType::JointType_Count; type++) {
							ColorSpacePoint colorSpacePoint = { 0 };
							pCoordinateMapper->MapCameraPointToColorSpace(joint[type].Position, &colorSpacePoint);
							int x = static_cast<int>(colorSpacePoint.X);
							int y = static_cast<int>(colorSpacePoint.Y);
							if ((x >= 0) && (x < COLOR_WIDTH) && (y >= 0) && (y < COLOR_HEIGHT)) 
								cv::circle(colorBufferMat, cv::Point(x, y), 5, static_cast<cv::Scalar>(color[3]), -1, CV_AA);
						}
					}
				}
			}
			cv::resize(colorBufferMat, bpMat, cv::Size(), 0.5, 0.5);
		}
		for (int count = 0; count < BODY_COUNT; count++) {  //……10
			SafeRelease(&pBody[count]);
		}
	}
	SafeRelease(&pColorFrame);  //……10
	SafeRelease(&pBodyFrame);  //……10

	return bpMat;
}

