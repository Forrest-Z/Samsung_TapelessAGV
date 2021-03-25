#ifndef C_PATH_TEACHING_BEHAVIOR_H
#define C_PATH_TEACHING_BEHAVIOR_H

#include <iostream>
#include "../../KUNSUtil/KUNSThread/KuThread.h"
#include "../../KUNSGUI/KuDrawingInfo.h"
#include "../../KUNSPose/KuPose.h"
#include "../../KUNSMath/KuMath.h"
#include "../../KUNSProcess/KUNSLaserBasedParticleFilterLocalizerPr/KuLBPFLocalizerPr.h"
#include "../../Sensor/SensorSupervisor.h"
#include "../../KUNSUtil/KuUtil.h"
#include "../../KUNSProcess/KUNSTeachingPathPlannerPr/KuTeachingPathPlannerPr.h"
#include "../../KUNSMap/KuMapRepository.h"
#include "../../KUNSProcess/KUNSICPLocalizerPr/KuICPLocalizerPr.h"
#include "../../KUNSProcess/KUNSScanMatchingLocalizerPr/KuScanMatchingLocalizerPr.h"
#include "../../KUNSProcess/KUNSZoneControlPr/KuZoneControlPr.h"
#include "../../KUNSProcess/KuFiducialbasedLocalizerPr/KuFiducialbasedLocalizerPr.h"

using namespace std;
class PathTeachingBehavior :public KuThread
{
private:
	Localizer* m_pLocalizer; 
	KuTeachingPathPlannerPr m_TPPlanner;
	list<KuPose> m_PathList;

	KuPose m_RobotPos;
	KuPose m_saveRobotPos;

	//스레드 관련 변수----------------------
	bool m_bThreadFlag;
	//--------------------------------------

	KuMap* m_pMap;
	int m_nMapSizeX;
	int m_nMapSizeY;

	int m_nIFDX;
	int m_nFCount;
	double m_dRecognizingMarkDistTh;
	double m_dDistMarkfromRobot;
private:
	KuICPLocalizerPr m_ICPLocalizer;
	KuScanMatchingLocalizerPr m_ScanMatchingLocalizer;

private:
	KuPose m_DelEncoderData;
	int_1DArray m_nLaserData181;
	int_1DArray m_nKinectLaserData;
	IplImage* m_IplCeilingCamera;

private:
	bool initialize(KuCommandMessage CMessage);
	static void doThread(void* arg);
	void startLocalizer(KuPose RobotPos,bool blocalizer);
	void saveOutlineData(int_1DArray nData, KuPose RobotPos);

private:
	void initICPProcess();
	void initTeachingPathProcess();

public:	
	bool execute(KuCommandMessage CMessage);
	void terminate();
	bool getBehaviorStates();
	Localizer* getLocalizer();
	void  savePath();
	void drawNaviData();

public:	
	PathTeachingBehavior();
	~PathTeachingBehavior();
};

#endif