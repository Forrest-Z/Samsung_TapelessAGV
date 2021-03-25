/*
 * KUNSThreadTestTutorial.h
 *
 * Created on: Apr 26, 2012
 * Author: Joong-Tae Park
 */

#ifndef KINECTINTERFACE_H
#define KINECTINTERFACE_H

#include <iostream>
#include <XnOS.h>
#include <XnCppWrapper.h>
#include "../Sensor.h"
#include "../../KUNSUtil/KUNSThread/KuThread.h"
#include "../../KUNSUtil/KUNSSingletone/KuSingletone.h"
#include "../../KUNSUtil/KUNSTimer/KuTimer.h"
#include "../../KUNSMath/KuMath.h"
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include "../../KUNSPose/KuPose.h"
#include "../../KUNSUtil/KuUtil.h"
#include "../../KUNSUtil/KUNSCriticalSection/KuCriticalSection.h"
#include "../../MobileSupervisor/KuRobotParameter.h"

#define SAMPLE_XML_PATH  "SamplesConfig.xml"
#define GL_WIN_SIZE_X 1280
#define GL_WIN_SIZE_Y 1024

#define DISPLAY_MODE_OVERLAY	1
#define DISPLAY_MODE_DEPTH		2
#define DISPLAY_MODE_IMAGE		3
#define DEFAULT_DISPLAY_MODE	DISPLAY_MODE_DEPTH

#define MAX_DEPTH 10000


using namespace std;
using namespace xn;

class KinectSensorInterface : public KuSingletone <KinectSensorInterface>
{
private:
	CCriticalSection m_CriticalSection;
private:
	KuThread m_Thread;
	KuUtil m_KuUtil;

public:
	static const int KINECT_TRASH_VALUE = -999999;


private:
	static const int MAX_VAL = 1000000;

	IplImage* m_IplColorImg;
	IplImage* m_Ipl320ColorImg;
	IplImage* m_IplDepthColorImg;
	IplImage* m_Ipl320DepthColorImg;

	KuPose m_TmpGlobal3DPose[320*240];
	KuPose m_Global3DPose[320*240];
	unsigned char DepthColor[640*480*3];
	float* m_fDistance;

	int_1DArray m_nKinectRangeData;
	int_1DArray m_nTmpKinectRangeData;

	int_1DArray m_nHeightDataOfKinectRangeData;
	int_1DArray m_nTmpHeightDataOfKinectRangeData;
	
	bool g_bCallbackEnd;

private:
	void translateKinectDataToRangeData(KuPose* p3DDataPos);
	void translateKinectDepthToWorld();
	void KinectRGBData();
	void Kinect3DData();

public:

	int_1DArray getRangeData();
	int_1DArray getHeightDataOfRangeData();
	void execute();

	bool connect();
	IplImage* getColorImage();
	IplImage* get320ColorImage();
	IplImage* getDepthImage();
	IplImage* get320DepthImage();
	float* getDepthBuffer();
	KuPose* getGlobal3DPose();
	void motorHoming();
	float* getDistanceImage();

public:
	static void doThread();
	void start();
	void terminate();
	void suspend();
	void resume();



	KinectSensorInterface();
	virtual ~KinectSensorInterface();

};

#endif 
