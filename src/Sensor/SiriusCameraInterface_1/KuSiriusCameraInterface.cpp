#include "stdafx.h"
#include "KuSiriusCameraInterface.h"

ImageFormat g_imgFormat;
IplImage* g_pImg;
bool g_bCallbackEnd = true;

KuSiriusCameraInterface::KuSiriusCameraInterface()
{
	bSiriusCameraflag=false;
	g_bCallbackEnd = true;
	init();
	cout<<"[KuSiriusCameraInterface]: Singletone type instance is created!!!"<<endl;
}
KuSiriusCameraInterface::~KuSiriusCameraInterface()
{
	cvReleaseImage(&m_mapx);
	cvReleaseImage(&m_mapy);
	cvReleaseMat(&m_mapxy);
	cvReleaseMat(&m_mapa);

	cout<<"[KuSiriusCameraInterface]: Singletone type instance is destroyed!!!"<<endl;
}
void KuSiriusCameraInterface::disconnection()
{
	while(!g_bCallbackEnd) { Sleep(10); };


	capDriverDisconnect(*m_hWndDisp);

	if(m_hWndDisp) delete m_hWndDisp;
	if(m_pBmInfo) delete m_pBmInfo;
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
	m_camcontext.cam_offset_x = 0.05;
	m_camcontext.cam_px2m= 5.0400e-006;

	m_camcontext.cam_f=(double)((m_dCam_fx + m_dCam_fy) / 2. * m_camcontext.cam_px2m);
	m_camcontext.img_center_u = (int)((m_dCam_cy - nImg_height / 2) + nImg_width / 2);
	m_camcontext.img_center_v = (int)((m_dCam_cx - nImg_width / 2) + nImg_height / 2);

	m_camcontext.cam_d1=m_dCam_d1;
	m_camcontext.cam_d2=m_dCam_d2;
	m_camcontext.cam_d3=m_dCam_d3;
	m_camcontext.cam_d4=m_dCam_d4;
	
	m_hWndDisp = new HWND;
	m_pBmInfo = (BITMAPINFO*)malloc(sizeof(BITMAPINFO) + 256 * sizeof(RGBQUAD));
	
	m_bInitialized= false;
	
	m_img_original=cvCreateImage(cvSize(m_camcontext.img_width,m_camcontext.img_height),8,1);
	m_img_undistorted=cvCreateImage(cvSize(m_camcontext.img_width,m_camcontext.img_height),8,1);

	g_pImg = m_img_original;


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


LRESULT CALLBACK capCallbackOnFrame(HWND hWnd, LPVIDEOHDR lpVHdr)
{
	g_bCallbackEnd = false;

	if(g_pImg)
	{
		if(g_imgFormat == IMAGE_FORMAT_YUV || g_imgFormat == IMAGE_FORMAT_YUY2)
		{
			int nWidth = g_pImg->width;
			int nHeight = g_pImg->height;

			// for YUY2 image format
			int i(0), j(0), k(0);
			int rows(0), cols(0);
			float R, G, B;
			float Y0, U, Y1, V;
			unsigned char* pData = (unsigned char*)lpVHdr->lpData;

			for(i = 0; i < lpVHdr->dwBufferLength; i = i + 4)
			{
				Y0 = *pData++;
				U =  *pData++;
				Y1 =  *pData++;
				V =  *pData++;

				R=(float)(Y0 + (1.370705 * (V-128)));
				G=(float)(Y0 - (0.698001 * (V-128)) - (0.337633 * (U-128)));
				B=(float)(Y0 + (1.732446 * (U-128)));

				if(R<0) R =0;
				if(R>255) R=255;
				if(G<0) G =0;
				if(G>255) G=255;
				if(B<0) B =0;
				if(B>255) B=255;

				//	g_pImg->imageData[nHeight - j - 1][k] = (int)((B + G + R) / 3.);
				g_pImg->imageData[j * nWidth + (nWidth - k - 1)] = (unsigned char)((B + G + R) / 3.);

				k++;

				R=(float)(Y1 + (1.370705 * (V-128)));
				G=(float)(Y1 - (0.698001 * (V-128)) - (0.337633 * (U-128)));
				B=(float)(Y1 + (1.732446 * (U-128)));

				if(R<0) R =0;
				if(R>255) R=255;
				if(G<0) G =0;
				if(G>255) G=255;
				if(B<0) B =0;
				if(B>255) B=255;

				//	g_pImg->imageData[nHeight - j - 1][k] = (int)((B + G + R) / 3.);
				g_pImg->imageData[j * nWidth + (nWidth - k - 1)] = (unsigned char)((B + G + R) / 3.);

				k++;

				if(k >= nWidth)
				{
					k = 0;
					j++;
				}
			}
		}
		else if(g_imgFormat == IMAGE_FORMAT_RGB)
		{
			// for RGB image format
			int i;
			int nWidth = g_pImg->width;
			int nHeight = g_pImg->height;


			//흑백으로 변환하여 img에 저장
			for(i = 0; i < nHeight * nWidth; i++)
			{
				g_pImg->imageData[i] = (unsigned char)((*(lpVHdr->lpData + i * 3) + *(lpVHdr->lpData + i * 3 + 1) + *(lpVHdr->lpData + i * 3 + 2)) / 3);
			}
		}
	}

	g_bCallbackEnd = true;

	return (LRESULT) TRUE;
}

bool KuSiriusCameraInterface::initCam(CWnd* pWnd, ImageFormat format, bool bViewSettingsWindow /*= false*/)
{
	g_imgFormat = format;

	*m_hWndDisp = capCreateCaptureWindow(_T("Camera"),
		WS_CHILD,
		0,
		0,
		m_camcontext.img_width,
		m_camcontext.img_height,
		pWnd->m_hWnd,
		NULL);

	capSetCallbackOnVideoStream(*m_hWndDisp, capCallbackOnFrame);

	if(!capDriverConnect(*m_hWndDisp, 0))
	{
		return false;
	}

	capGetVideoFormat(*m_hWndDisp, m_pBmInfo,  sizeof(BITMAPINFO));
	m_pBmInfo->bmiHeader.biWidth = 320;
	m_pBmInfo->bmiHeader.biHeight = 240;
	capSetVideoFormat(*m_hWndDisp, m_pBmInfo, sizeof(BITMAPINFO));

	if(bViewSettingsWindow)
	{
		if(!capDlgVideoFormat(*m_hWndDisp)) // initialize video settings
		{
			return false;
		}
	}

	CAPTUREPARMS captureParms;
	captureParms.fCaptureAudio = false;
	captureParms.fMCIControl = false;
	captureParms.fLimitEnabled = false;
	captureParms.dwRequestMicroSecPerFrame = 66667;
	captureParms.wTimeLimit = 20000;
	captureParms.fUsingDOSMemory = false;
	captureParms.fMakeUserHitOKToCapture = false;
	captureParms.wPercentDropForError = 10;
	captureParms.wNumVideoRequested = 32;
	captureParms.fAbortLeftMouse = false;
	captureParms.fAbortRightMouse = false;
	captureParms.wChunkGranularity = 0;
	captureParms.fYield = true;
	captureParms.dwIndexSize = 27000;
	captureParms.wNumAudioRequested = 4;
	captureParms.vKeyAbort = VK_ESCAPE;
	captureParms.fStepMCIDevice = 0;
	captureParms.dwMCIStartTime = 10000;
	captureParms.dwMCIStopTime = 20000;
	captureParms.fStepCaptureAt2x = 0;
	captureParms.wStepCaptureAverageFrames = 5;
	captureParms.dwAudioBufferSize = 0;
	captureParms.fDisableWriteCache = 0;
	captureParms.AVStreamMaster = 0;

	capCaptureSetSetup(*m_hWndDisp, &captureParms, sizeof(CAPTUREPARMS));
	capCaptureSequenceNoFile(*m_hWndDisp);

	m_bInitialized = true;

	return true;
}

IplImage* KuSiriusCameraInterface::calcUndistortedImage(void)
{
	int i;
	IplImage* originalImage;
	IplImage* undistortedImage;
	int nWidth = m_camcontext.img_width;
	int nHeight = m_camcontext.img_height;

	originalImage = cvCreateImage(cvSize(nWidth, nHeight), IPL_DEPTH_8U, 1);
	undistortedImage = cvCreateImage(cvSize(nWidth, nHeight), IPL_DEPTH_8U, 1);

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


	return m_img_undistorted;
}

IplImage* KuSiriusCameraInterface::getUndistortedImage(void)
{
	return m_img_undistorted;
}
