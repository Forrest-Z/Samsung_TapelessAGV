#ifndef KUNS_GLOBAL_MAP_BUILDING_BEHAVIOR_H
#define KUNS_GLOBAL_MAP_BUILDING_BEHAVIOR_H

#include <iostream>
#include <conio.h>
#include "../../KUNSUtil/KUNSThread/KuThread.h"
#include "../../KUNSGUI/KuDrawingInfo.h"
#include "../../KUNSPose/KuPose.h"
#include "../../KUNSMath/KuMath.h"
#include "../../KUNSUtil/KUNSCriticalSection/KuCriticalSection.h"
#include "../../KUNSMap/KuMapRepository.h"
#include "../../Sensor/WheelActuatorInterface/SSAGVWheelActuatorInterface.h"
#include "../../KUNSUtil/KUNSINIReadWriter/KuINIReadWriter.h"
#include "../../Sensor/SensorSupervisor.h"
#include "../../KUNSUtil/KuUtil.h"
#include "../../KUNSProcess/KUNSSURFbasedGlobalLocalizerPr/KuSURFbasedGlobalLocalizerPr.h"
#include "../../Localizer/GlobalLocalizerSupervisor.h"
#include "../../KUNSProcess/KUNSGlobalMapBuildingPr/KuGlobalMapBuildingPr.h"
#include "../../KUNSProcess/KUNSRetinexPr/KuRetinexPr.h"
#include <opencv2/stitching/stitcher.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

using namespace std;
using namespace cv;

class GlobalMapBuilding
{
private:
	KuThread m_GCMBuildingThread;//Global Ceiling Map Building Thread
	KuThread m_FDBBuildingThread;//Feature DataBase Building Thread
	KuUtil m_KuUtil;

private:	
	KuPose m_RobotPos;
	KuPose m_DelEncoderData;
	bool m_bThreadFlag;
	bool m_bMapBuilding;
	bool m_bDBBuilding;
	
private:
	vector<KuPose> m_vecImagePath;
	vector<Mat> m_vecmatCeilingImages;
	vector<Mat> m_vecmatCeilingAffineImages;
	vector<Mat> m_vecmatRetinexImages;
	vector<Mat> m_vecmatRetinexAffineImages;
private:
	IplImage* m_CeilingCamera;
	int_1DArray m_nLaserData181;
	int_1DArray m_nKinectLaserData;

private:
	static void doGCMBuildingThread(void* arg);
	static void doFDBBuildingThread(void* arg);
	void startLocalizer(KuPose RobotPos);
	bool initialize();
	bool initialize(KuCommandMessage CMessage);
	void drawNaviData();
	
public:
	bool execute(KuCommandMessage CMessage);
	void terminate();
	bool getBehaviorStates();
	Localizer* getLocalizer();

	GlobalMapBuilding();
	~GlobalMapBuilding();

};

#endif 