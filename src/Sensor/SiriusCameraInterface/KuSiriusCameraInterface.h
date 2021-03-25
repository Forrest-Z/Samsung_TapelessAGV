/*______________________________________________________________________________________________
PROJECT : Intelligent Robotics Lab. Department of Mechanical Engineering Korea University                                    
(c) Copyright 2012 KUME Intelligent Robotics Lab.                                             
All rights reserved                                                                         
$Directed by Jae-Bok Song                                                                     
$Designed by Joong-Tae Park                                                                  
$Description :Sirius에 사용되는 카메라 인터페이스
$Created on: 2012. 6. 9.                                                                        
$Author: Joong-Tae Park      
$e-mail: jtpark1114@gmail.com
______________________________________________________________________________________________*/

#ifndef KUNS_SIRIUS_CAMERA_INTERFACE_H
#define KUNS_SIRIUS_CAMERA_INTERFACE_H

//#define CAM_USE_OPENCV // OpenCV (for Venus cameras)
#define CAM_USE_FLYCAP // FlyCapture (for FireFly cameras)

#include <vfw.h>
#include <iostream>
#include <ctype.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include "../../src/KUNSUtil/KUNSSingletone/KuSingletone.h"
#include "../../MobileSupervisor/KuRobotParameter.h"
#include "../Sensor.h"
#include "FlyCapture2.h" // FlyCapture
#include "../../ANSCommon/ANSCommon.h"
/*#include "mvIMPACT_CPP/mvIMPACT_acquire.h"*/
using namespace cv;
using namespace std;
/*using namespace mvIMPACT::acquire;*/

typedef struct _tagCamContext
{
	int img_width;
	int img_height;
	double cam_px2m;
	float cam_fx;
	float cam_fy;
	float cam_f;
	float cam_cx;
	float cam_cy;
	float cam_d1;
	float cam_d2;
	float cam_d3;
	float cam_d4;
	float cam_offset_x;
	int cam_mode;
	int img_center_u;
	int img_center_v;
	int cam_exposure;
	bool cam_auto_exposure;
} CamContext;



class KuSiriusCameraInterface : public KuSingletone <KuSiriusCameraInterface>
{
private:
	//camera coefficient
	double m_dCam_fx;
	double m_dCam_fy;
	double m_dCam_cx;
	double m_dCam_cy;

	//camera undistortion parameter
	double m_dCam_d1;
	double m_dCam_d2;
	double m_dCam_d3;
	double m_dCam_d4;

	CamContext m_camcontext;

private:
// 	//mvIMPACT
// 	DeviceManager m_devMgr;
// 	mvIMPACT::acquire::Device* m_pDev;
// 	int m_nmvIMPACTWidth;
// 	int m_nmvIMPACTHeight;
// 	int m_nmvImpactSize;
// 	char* m_pDataBf;

private:

	HWND* m_hWndDisp;
	BITMAPINFO* m_pBmInfo;
	bool m_bInitialized;
	IplImage* m_mapx;
	IplImage* m_mapy;
	CvMat* m_mapxy;
	CvMat* m_mapa;
	IplImage* m_img_original;
	IplImage* m_img_undistorted;
#ifdef CAM_USE_OPENCV
	CvCapture* capture;
#endif
#ifdef CAM_USE_FLYCAP
	FlyCapture2::Camera m_flycap; // FlyCapture
#endif
	bool g_bCallbackEnd ;

private:
	void init();
	bool bSiriusCameraflag;

public:
	CamContext* getCamContext(void);
	void disconnection();
	void initBitmapInfo(int nImgWidth, int nImgHeight);
	bool connect();
	IplImage* calcUndistortedImage(void);
	IplImage* getUndistortedImage(void);


public:
	KuSiriusCameraInterface();
	virtual ~KuSiriusCameraInterface();

};

#endif /*KUNS_SIRIUS_CAMERA_INTERFACE_H*/
