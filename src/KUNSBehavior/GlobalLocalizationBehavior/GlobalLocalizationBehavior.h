#ifndef KUNS_GLOBAL_LOCALIZATION_BEHAVIOR_H
#define KUNS_GLOBAL_LOCALIZATION_BEHAVIOR_H

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
#include "../../KUNSProcess/KUNSRetinexPr/KuRetinexPr.h"

using namespace std;

class GlobalLocalizationBehavior
{
private:
	KuThread m_doThread;
	KuThread m_SURFThread;

	KuUtil m_KuUtil;

private:	
	KuPose m_RobotPos;
	KuPose m_DelEncoderData;
	KuPose m_saveRobotPos;
	bool m_bThreadFlag;
	bool m_bStartSURFThread;
	bool m_bGlobalLocalizationFlag;
	KuRetinexPr RetinexPr;
private:
	IplImage* m_CeilingCamera;
	int_1DArray m_nLaserData181;
	int_1DArray m_nKinectLaserData;
	Mat m_cvCeilingImage;

private:
	bool initialize();
	static void doThread(void* arg);
	static void doSURFThread(void* arg);
	void startLocalizer(KuPose RobotPos);
	bool initialize(KuCommandMessage CMessage);
	void drawNaviData();
	void findBestAngle(IplImage* CeilingCamera, int_1DArray nLaserData181,double dGyroData );
	bool TranslationMotion(IplImage* CeilingCamera, int_1DArray nLaserData181,KuPose DelEncoderData );
	KuPose rotateBestAngle(double dAngle);
	void rotate(double dAngle);



public:	
	bool execute(KuCommandMessage CMessage);
	void terminate();
	bool getBehaviorStates();
	Localizer* getLocalizer();
	bool doGlobalLocaliztion();
	void terminateSavingPathThread();
	void startSavingPathThrerad();

public:
	GlobalLocalizationBehavior();
	~GlobalLocalizationBehavior();

};

#endif 