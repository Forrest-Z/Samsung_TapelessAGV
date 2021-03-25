#pragma once

#include <iostream>
#include <list>

#include <math.h>
#include <list>

//OpenCV ���
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>

//ALRecognizer ���
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
	static const int IMG_WIDTH = 640;//320; // ���� ���� ũ��
	static const int IMG_HEIGHT = 480;//240; // ���� ���� ũ��

	FrameThresholder* m_pFrameThresholder;
	FidtrackFinder* m_pFidtrackFinder;
	unsigned char m_srcImage[IMG_WIDTH*IMG_HEIGHT];
	unsigned char m_buffImage[IMG_WIDTH*IMG_HEIGHT];

	double m_dHeight_Camera2Mark;
	int m_nHeight_Camera2Mark;

	list<KuPose> m_RecogFiducialLandMarkList;
	KuPose m_FiducialLandMarkGolbalPos;

	int m_nLandMarkNum;//���帶ũ �� ���� 3���̸� 0,1,2 �������� ����Ѵ�.

	
	KuPose calcFiducialLandMarkPosimg2global(list <KuPose> lFiducialLandMarkPos, KuPose RobotPos);
	void trans_global2img(const float& fRx, const float& fRy, const float& fRth, const float& fFx, const float& fFy, const float& fFz, int& nu, int& nv, const CamContext* pContext);
	void trans_img2global(const float& fRx, const float& fRy, const float& fRth, const int& nu, const int& nv, float& fFx, float& fFy, float& fFz, const CamContext* pContext, double dHeight);

	//Ư�� id�� ��ũ�� camera2heigh�� �ٲ��ִ� �κ�140309(��ȯ)
	vector<int> m_vecnMarkIdx2changeHeigh;
	vector<double> m_vecdCeilingHeigh2change;//m����
	void loadCeilingInfo2changeHeigh();


public:
	void setRecogFiducialLandMarkList(list<KuPose> RecogFiducialLandMarkList);
	list<KuPose> getRecogFiducialLandMarkList();

	KuPose start(IplImage* inimg, KuPose RobotPos);
	CALRecognizer(void);
	~CALRecognizer(void);
};
