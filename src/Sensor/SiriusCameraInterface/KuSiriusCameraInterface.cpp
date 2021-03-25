#include "stdafx.h"
#include "KuSiriusCameraInterface.h"


KuSiriusCameraInterface::KuSiriusCameraInterface()
{
	m_hWndDisp = 0;

	bSiriusCameraflag=false;
	g_bCallbackEnd = true;

#ifdef CAM_USE_OPENCV
	capture=NULL;
#endif

	init();
	cout<<"[KuSiriusCameraInterface]: Singletone type instance is created!!!"<<endl;
}
KuSiriusCameraInterface::~KuSiriusCameraInterface()
{
#ifdef CAM_USE_FLYCAP

	using namespace FlyCapture2;

	Error error;

	if(m_flycap.IsConnected())
	{
		// Stop capturing images
		error = m_flycap.StopCapture();

		if(error != ErrorType::PGRERROR_OK)
		{
			ANS_LOG_ERROR(error.GetDescription(), false);
		}

		// Disconnect the camera
		error = m_flycap.Disconnect();

		if(error != ErrorType::PGRERROR_OK)
		{
			ANS_LOG_ERROR(error.GetDescription(), false);
		}
	}

#endif

	cvReleaseImage(&m_mapx);
	cvReleaseImage(&m_mapy);
	cvReleaseMat(&m_mapxy);
	cvReleaseMat(&m_mapa);
	
	cout<<"[KuSiriusCameraInterface]: Singletone type instance is destroyed!!!"<<endl;
}
void KuSiriusCameraInterface::disconnection()
{
	while(!g_bCallbackEnd) { Sleep(10); };

#ifdef CAM_USE_OPENCV

	if(capture!=NULL)	cvReleaseCapture(&capture);

#endif
// 	if(m_camcontext.cam_mode!=0)
// 	{
// 		m_pDev=NULL;
// 		m_pDataBf = new char[m_nmvImpactSize];
// 	}

	if(m_hWndDisp)
	{
		delete m_hWndDisp;
	}
	if(m_pBmInfo){
		free(m_pBmInfo) ;
		m_pBmInfo=NULL;
	}
}

CamContext* KuSiriusCameraInterface::getCamContext(void)
{
	return &m_camcontext;
}
void KuSiriusCameraInterface::init()
{
	
	//camera coefficient
	m_dCam_fx = KuRobotParameter::getInstance()->getCeilingCameraPrameterFx();
	m_dCam_fy = KuRobotParameter::getInstance()->getCeilingCameraPrameterFy();
	m_dCam_cx = KuRobotParameter::getInstance()->getCeilingCameraPrameterCx();
	m_dCam_cy = KuRobotParameter::getInstance()->getCeilingCameraPrameterCy();

	//camera undistortion parameter
	m_dCam_d1 = KuRobotParameter::getInstance()->getCeilingCameraPrameterD1();
	m_dCam_d2 = KuRobotParameter::getInstance()->getCeilingCameraPrameterD2();
	m_dCam_d3 = KuRobotParameter::getInstance()->getCeilingCameraPrameterD3();
	m_dCam_d4 = KuRobotParameter::getInstance()->getCeilingCameraPrameterD4();
	
	int nImg_width = Sensor::CEILING_IMAGE_WIDTH;
	int nImg_height = Sensor::CEILING_IMAGE_HEIGHT;
	
	m_camcontext.img_width=nImg_width;
	m_camcontext.img_height=nImg_height;

	m_camcontext.cam_fx=m_dCam_fx;
	m_camcontext.cam_fy=m_dCam_fy;
	m_camcontext.cam_cx=m_dCam_cx;
	m_camcontext.cam_cy=m_dCam_cy;
	m_camcontext.cam_offset_x = KuRobotParameter::getInstance()->getCeilingCameraPrameterOffsetX();
	m_camcontext.cam_px2m= 5.0400e-006;

	m_camcontext.cam_f=(double)((m_dCam_fx + m_dCam_fy) / 2. * m_camcontext.cam_px2m);
//	m_camcontext.img_center_u = (int)((m_dCam_cy - nImg_height / 2) + nImg_width / 2);
//	m_camcontext.img_center_v = (int)((m_dCam_cx - nImg_width / 2) + nImg_height / 2);
	m_camcontext.img_center_u = m_dCam_cx;
	m_camcontext.img_center_v = nImg_height - m_dCam_cy;

	m_camcontext.cam_d1=m_dCam_d1;
	m_camcontext.cam_d2=m_dCam_d2;
	m_camcontext.cam_d3=m_dCam_d3;
	m_camcontext.cam_d4=m_dCam_d4;
	m_camcontext.cam_mode = KuRobotParameter::getInstance()->getCeilingCameraPrameterMode();

	m_camcontext.cam_auto_exposure = KuRobotParameter::getInstance()->getCeilingCameraAutoExposure();
	m_camcontext.cam_exposure = KuRobotParameter::getInstance()->getCeilingCameraExposure();
	
	m_hWndDisp = new HWND;
	m_pBmInfo = (BITMAPINFO*)malloc(sizeof(BITMAPINFO) + 256 * sizeof(RGBQUAD));
	
	m_bInitialized= false;
	
	m_img_original=cvCreateImage(cvSize(m_camcontext.img_width,m_camcontext.img_height),8,1);
	m_img_undistorted=cvCreateImage(cvSize(m_camcontext.img_width,m_camcontext.img_height),8,1);

	initBitmapInfo(m_camcontext.img_width, m_camcontext.img_height);

	// Undistortion parameters
	m_mapx = cvCreateImage(cvSize (m_camcontext.img_width, m_camcontext.img_height), IPL_DEPTH_32F, 1);
	m_mapy = cvCreateImage(cvSize (m_camcontext.img_width, m_camcontext.img_height), IPL_DEPTH_32F, 1);

	CvMat* m_matCameraMatrix = cvCreateMat (3, 3, CV_32FC1);
	CvMat* m_matDistortionMatrix = cvCreateMat (1, 4, CV_32FC1);

	cvSetReal2D(m_matCameraMatrix , 0, 0, m_camcontext.cam_fx);
	cvSetReal2D(m_matCameraMatrix , 0, 1, 0);
	cvSetReal2D(m_matCameraMatrix , 0, 2, m_camcontext.cam_cx);

	cvSetReal2D(m_matCameraMatrix , 1, 0, 0);
	cvSetReal2D(m_matCameraMatrix , 1, 1, m_camcontext.cam_fy);
	cvSetReal2D(m_matCameraMatrix , 1, 2, m_camcontext.cam_cy);

	cvSetReal2D(m_matCameraMatrix , 2, 0, 0);
	cvSetReal2D(m_matCameraMatrix , 2, 1, 0);
	cvSetReal2D(m_matCameraMatrix , 2, 2, 1);

	cvSetReal1D(m_matDistortionMatrix , 0, m_camcontext.cam_d1);
	cvSetReal1D(m_matDistortionMatrix , 1, m_camcontext.cam_d2);
	cvSetReal1D(m_matDistortionMatrix , 2, m_camcontext.cam_d3);
	cvSetReal1D(m_matDistortionMatrix , 3, m_camcontext.cam_d4);

	cvInitUndistortMap(m_matCameraMatrix, m_matDistortionMatrix, m_mapx, m_mapy);

	// for computation speed
	m_mapxy = cvCreateMat(m_camcontext.img_height, m_camcontext.img_width, CV_16SC2);
	m_mapa = cvCreateMat(m_camcontext.img_height, m_camcontext.img_width, CV_16UC1);

	convertMaps(cv::Mat(m_mapx), cv::Mat(m_mapy), cv::Mat(m_mapxy), cv::Mat(m_mapa), CV_16SC2);

// 	if(m_camcontext.cam_mode!=0)
// 	{
// 		m_pDev=NULL;
// 		m_nmvIMPACTWidth=1280;
// 		m_nmvIMPACTHeight=960;
// 		m_nmvImpactSize=m_nmvIMPACTWidth*m_nmvIMPACTHeight*4;//camera input size
// 		m_pDataBf = new char[m_nmvImpactSize];
// 	}	
}


void KuSiriusCameraInterface::initBitmapInfo(int nImgWidth, int nImgHeight)
{
	int i;
	int nWidth = nImgWidth;
	int nHeight = nImgHeight;
	int rwsize = (((nWidth) + 31) / 32 * 4);

	m_pBmInfo->bmiHeader.biBitCount=8;
	m_pBmInfo->bmiHeader.biClrImportant=256;
	m_pBmInfo->bmiHeader.biClrUsed=256;
	m_pBmInfo->bmiHeader.biCompression=0;
	m_pBmInfo->bmiHeader.biHeight = nHeight;
	m_pBmInfo->bmiHeader.biWidth = nWidth;
	m_pBmInfo->bmiHeader.biPlanes=1;
	m_pBmInfo->bmiHeader.biSize=40;
	m_pBmInfo->bmiHeader.biSizeImage = rwsize * nHeight;
	m_pBmInfo->bmiHeader.biXPelsPerMeter=0;
	m_pBmInfo->bmiHeader.biYPelsPerMeter=0;

	for(i=0; i<256; i++) // Palette number is 256
	{
		m_pBmInfo->bmiColors[i].rgbRed= m_pBmInfo->bmiColors[i].rgbGreen = m_pBmInfo->bmiColors[i].rgbBlue = (BYTE)i;
		m_pBmInfo->bmiColors[i].rgbReserved = 0;
	}
}


bool KuSiriusCameraInterface::connect()
{
	if(m_camcontext.cam_mode==0)
	{
#ifdef CAM_USE_OPENCV

		capture = cvCaptureFromCAM(0); //현재 인식된 웹캠을 찾고,

#endif
#ifdef CAM_USE_FLYCAP

		FlyCapture2::Error error;
		FlyCapture2::BusManager busMgr;
		unsigned int nNumCameras;
		error = busMgr.GetNumOfCameras(&nNumCameras);

		stringstream ss;
		ss << "[Cam] Number of Pointgrey cameras detected: " << nNumCameras;
		ANS_LOG_WRITE(ss.str());

		FlyCapture2::PGRGuid guid;
		error = busMgr.GetCameraFromIndex(0, &guid);

		if(error != FlyCapture2::ErrorType::PGRERROR_OK)
		{
			ANS_LOG_ERROR(error.GetDescription(), false);
			return false;
		}

		// Connect
		error = m_flycap.Connect(&guid);

		if(error != FlyCapture2::ErrorType::PGRERROR_OK)
		{
			ANS_LOG_ERROR(error.GetDescription(), false);
			return false;
		}

		// Get the camera information
		FlyCapture2::CameraInfo camInfo;
		error = m_flycap.GetCameraInfo(&camInfo);

		if(error != FlyCapture2::ErrorType::PGRERROR_OK)
		{
			ANS_LOG_ERROR(error.GetDescription(), false);
			return false;
		}

		// Set property
		FlyCapture2::Property camProp;

		memset (&camProp, 0, sizeof(camProp));
		camProp.type = FlyCapture2::AUTO_EXPOSURE;
		camProp.onOff = m_camcontext.cam_auto_exposure;
		error = m_flycap.SetProperty(&camProp);
		if (error != FlyCapture2::PGRERROR_OK)
		{
			ANS_LOG_ERROR(error.GetDescription(), false);
			return false;
		}
	/*
		memset (&camProp, 0, sizeof(camProp));
		camProp.type = FlyCapture2::BRIGHTNESS;
		camProp.absControl = false;
		camProp.absValue = 200; // 1 ~ 255
		error = m_flycap.SetProperty(&camProp);
		if (error != FlyCapture2::PGRERROR_OK)
		{
			ANS_LOG_ERROR(error.GetDescription(), false);
			return false;
		}
	*/

		memset (&camProp, 0, sizeof(camProp));
		camProp.type = FlyCapture2::SHUTTER;
		camProp.absControl = true;
		camProp.absValue = m_camcontext.cam_exposure; // 0 ~ 30 ms
		camProp.onOff = false;
		error = m_flycap.SetProperty(&camProp);
		if (error != FlyCapture2::PGRERROR_OK)
		{
			ANS_LOG_ERROR(error.GetDescription(), false);
			return false;
		}

		// Image format
		error = m_flycap.SetVideoModeAndFrameRate(FlyCapture2::VIDEOMODE_640x480Y8, FlyCapture2::FRAMERATE_30);

		if (error != FlyCapture2::PGRERROR_OK)
		{
			ANS_LOG_ERROR(error.GetDescription(), false);
			return false;
		}

		// Start capturing images
		error = m_flycap.StartCapture();

		if(error != FlyCapture2::ErrorType::PGRERROR_OK)
		{
			ANS_LOG_ERROR(error.GetDescription(), false);
			return false;
		}

#endif

		m_bInitialized = true;

		if(m_camcontext.cam_auto_exposure)
		{
#ifdef CAM_USE_OPENCV

			cvSetCaptureProperty(capture, CV_CAP_PROP_AUTO_EXPOSURE, m_camcontext.cam_exposure); // -10 ~ 1

#endif
		}

	}
// 	else
// 	{
// 		m_pDev = m_devMgr[0];
// 		m_bInitialized = true;
// 		if( !m_pDev )
// 		{
// 			cout << "No mvIMPACT device! Enter if you want to finish" << endl;
// 			m_bInitialized = false;
// 		}
// 
// 		try
// 		{
// 			m_pDev->open();
// 		}
// 		catch( const ImpactAcquireException& e )
// 		{
// 			cout << "Same device is already opened! Enter if you want to finish" << endl;
// 			m_bInitialized = false;
// 		}
// 	}

	

	return true;
}

IplImage* KuSiriusCameraInterface::calcUndistortedImage(void)
{
	g_bCallbackEnd=false;

	int i, j;
	IplImage* originalImage;
	IplImage* undistortedImage;
	int nWidth = m_camcontext.img_width;
	int nHeight = m_camcontext.img_height;

	originalImage = cvCreateImage(cvSize(nWidth, nHeight), IPL_DEPTH_8U, 1);
	undistortedImage = cvCreateImage(cvSize(nWidth, nHeight), IPL_DEPTH_8U, 1);

	if(m_bInitialized)
	{
		if(m_camcontext.cam_mode==0)
		{
#ifdef CAM_USE_OPENCV

			if(!m_camcontext.cam_auto_exposure)
			{
				cvSetCaptureProperty(capture, CV_CAP_PROP_EXPOSURE, m_camcontext.cam_exposure); // -10 ~ 1
			}

			cvGrabFrame( capture ); 

			IplImage *iplcolorImage = cvRetrieveFrame( capture ); // 현재 인식된 장면을 받아오고image에 넣는다.
			IplImage * iplgrayImage= cvCreateImage(cvSize(iplcolorImage->width,iplcolorImage->height), IPL_DEPTH_8U, 1);
			cvCvtColor(iplcolorImage,iplgrayImage,CV_RGB2GRAY);
			cvResize(iplgrayImage,m_img_original);
			cvReleaseImage(&iplgrayImage);

#endif
#ifdef CAM_USE_FLYCAP

			using namespace FlyCapture2;

			Error error;
			Image img_raw;
			Image img_converted;

			// Retrieve an image
			error = m_flycap.RetrieveBuffer(&img_raw);

			if(error != ErrorType::PGRERROR_OK)
			{
				ANS_LOG_ERROR(error.GetDescription(), false);

				// Black image
				for(i = 0; i < nWidth * nHeight; i++)
				{
					m_img_original->imageData[i] = 0;
				}
			}
			else
			{
				// Convert the raw image
				error = img_raw.Convert(PIXEL_FORMAT_MONO8, &img_converted);

				if(error != ErrorType::PGRERROR_OK)
				{
					ANS_LOG_ERROR(error.GetDescription(), false);
				}
				else
				{
					// Convert to CANSImage
					for(i = 0; i < nWidth; i++)
					{
						for(j = 0; j < nHeight; j++)
						{
							int nLoc(j * nWidth + i);
// 							int nLocFlip((nHeight - j - 1) * nWidth + i);

							m_img_original->imageData[nLoc] = img_converted.GetData()[nLoc];
						}
					}
				}
			}

#endif
		}
// 		else
// 		{
// 			FunctionInterface fi(m_pDev);
// 
// 			// send a request to the default request queue of the device and wait for the result.
// 			fi.imageRequestSingle();
// 
// 			// Define the Image Result Timeout (The maximum time allowed for the Application
// 			// to wait for a Result). Infinity value:-1
// 			const int iMaxWaitTime_ms = 500;   // USB 1.1 on an embedded system needs a large timeout for the first image.
// 			// wait for results from the default capture queue.
// 			int requestNr = fi.imageRequestWaitFor( iMaxWaitTime_ms );
// 
// 
// 			const Request* pRequest = fi.getRequest( requestNr );
// 			int nSize = pRequest->imageSize.read();
// 			memcpy( m_pDataBf, pRequest->imageData.read(), pRequest->imageSize.read() );
// 
// 
// 			//---------------------------------mvIMPACT --> Mat---------------------------------------------------
// 			Mat cvMatInput3CImage(m_nmvIMPACTHeight,m_nmvIMPACTWidth,CV_8UC3,Scalar(0,0,0));
// 			Mat cvMatInput1CImage(m_nmvIMPACTHeight,m_nmvIMPACTWidth,CV_8UC1,Scalar(0));
// 			Mat cvMatOriginImg;
// 			for(int i=0; i<m_nmvIMPACTHeight; i++)
// 			{
// 				for(int j=0; j<m_nmvIMPACTWidth; j++)
// 				{
// 					cvMatInput3CImage.at<Vec3b>(i,j)[0]=m_pDataBf[i*m_nmvIMPACTWidth*4+j*4+2];
// 					cvMatInput3CImage.at<Vec3b>(i,j)[1]=m_pDataBf[i*m_nmvIMPACTWidth*4+j*4+1];
// 					cvMatInput3CImage.at<Vec3b>(i,j)[2]=m_pDataBf[i*m_nmvIMPACTWidth*4+j*4+0];
// 				}
// 			}
// 			cvtColor(cvMatInput3CImage,cvMatInput1CImage,CV_BGR2GRAY);
// 			resize(cvMatInput1CImage,cvMatOriginImg,Size(m_camcontext.img_width,m_camcontext.img_height),1,1,CV_INTER_LINEAR);
// 			for(int i=0; i<m_camcontext.img_width*m_camcontext.img_height; i++)
// 			{
// 				m_img_original->imageData[i] = cvMatOriginImg.data[i];
// 			}
// 			fi.imageRequestUnlock( requestNr );
// 		}

	}
	
	// convert char img into IplImage
	for(i = 0; i < nWidth * nHeight; i++)
	{
		originalImage->imageData[i] = m_img_original->imageData[i];
	}

	cvRemap(originalImage, undistortedImage, m_mapxy, m_mapa); // fast undistortion

	// convert IplImage into char img
	for(i = 0; i < nWidth * nHeight; i++)
	{
		m_img_undistorted->imageData[i]= undistortedImage->imageData[i];
	}

	cvReleaseImage(&originalImage);
	cvReleaseImage(&undistortedImage);

	g_bCallbackEnd=true;

	return m_img_undistorted;
}

IplImage* KuSiriusCameraInterface::getUndistortedImage(void)
{
	return m_img_undistorted;
}
