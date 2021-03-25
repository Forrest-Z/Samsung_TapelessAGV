#pragma once

#include <iostream>
#include <list>

#include <math.h>
#include <list>

//OpenCV 헤더
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>

//ALRecognizer 헤더
#include "./AL_Recognizer/FrameThresholder.h"
#include "./AL_Recognizer/FidtrackFinder.h"
//
#include "../../KUNSPose/KuPose.h"
#include "../../MobileSupervisor/KuRobotParameter.h"
#include "../../sensor/Sensor.h"
#include "../../Sensor/SiriusCameraInterface/KuSiriusCameraInterface.h"

//
#include"newmat.h"

using namespace std;

class CALRecognizer
{
	static const int IMG_WIDTH = 640;//320; // 영상 가로 크기
	static const int IMG_HEIGHT = 480;//240; // 영상 세로 크기

	FrameThresholder* m_pFrameThresholder;
	FidtrackFinder* m_pFidtrackFinder;
	unsigned char m_srcImage[IMG_WIDTH*IMG_HEIGHT];
	unsigned char m_buffImage[IMG_WIDTH*IMG_HEIGHT];

	double m_dHeight_Camera2Mark;
	int m_nHeight_Camera2Mark;

	list<KuPose> m_RecogFiducialLandMarkList;
	KuPose m_FiducialLandMarkGolbalPos;

	int m_nLandMarkNum;//랜드마크 의 갯수 3개이면 0,1,2 번순으로 사용한다.

	
	KuPose calcFiducialLandMarkPosimg2global(list <KuPose> lFiducialLandMarkPos, KuPose RobotPos);
	void trans_global2img(const float& fRx, const float& fRy, const float& fRth, const float& fFx, const float& fFy, const float& fFz, int& nu, int& nv, const CamContext* pContext);
	void trans_img2global(const float& fRx, const float& fRy, const float& fRth, const int& nu, const int& nv, float& fFx, float& fFy, float& fFz, const CamContext* pContext, double dHeight);

	//특정 id의 마크의 camera2heigh를 바꿔주는 부분140309(허환)
	vector<int> m_vecnMarkIdx2changeHeigh;
	vector<double> m_vecdCeilingHeigh2change;//m단위
	void loadCeilingInfo2changeHeigh();


public:
	void setRecogFiducialLandMarkList(list<KuPose> RecogFiducialLandMarkList);
	list<KuPose> getRecogFiducialLandMarkList();

	KuPose start(IplImage* inimg, KuPose RobotPos);
	CALRecognizer(void);
	~CALRecognizer(void);
};
