#ifndef MOBILE_SUPERVISOR_H
#define MOBILE_SUPERVISOR_H

// #define _CRTDBG_MAP_ALLOC
// #include <stdlib.h>
// #include <crtdbg.h>

#include <Windows.h>
#include <MMSystem.h>
#include <conio.h>
#include<iostream>
#include<fstream>
#include "../KUNSUtil/KUNSINIReadWriter/KuINIReadWriter.h"
#include "../KUNSUtil/KuUtil.h"
#include "../KUNSMap/KuMapRepository.h"
#include "../KUNSUtil/KUNSSingletone/KuSingletone.h"
#include "../KUNSBehavior/GotoGoalBehavior/GotoGoalBehavior.h"
#include "../KUNSBehavior/MapBuildingBehavior/MapBuildingBehavior.h"
#include "KuCommandMessage.h"
#include "KuRobotParameter.h"
#include "../KUNSBehavior/PathTeachingBehavior/PathTeachingBehavior.h"
#include "../KUNSProcess/KUNSTeachingPathPlannerPr/KuTeachingPathPlannerPr.h"
#include "../KUNSBehavior/GlobalLocalizationBehavior/GlobalLocalizationBehavior.h"
#include "../KUNSBehavior/GlobalMapBuildingBehavior/GlobalMapBuilding.h"
#include "../MultiRobotSupervisor/MultiRobotSupervisor.h"
#include "../Localizer/GlobalLocalizerSupervisor.h"
#include "../sensor/SensorSupervisor.h"
#include "../KUNSProcess/KuFiducialbasedLocalizerPr/KuFiducialbasedLocalizerPr.h"
#include "../sensor/SwitchInterface/SwitchInterface.h"
#include "../MultiRobotSupervisor/XmldataSetting.h"

using namespace std;

#pragma commnet(lib,"winmm.lib")

class MobileSupervisor: public KuSingletone <MobileSupervisor>
{
private:
	GotoGoalBehavior m_GotoGoalBeh;
	MapBuildingBehavior m_MapBuildingBeh;
	PathTeachingBehavior m_PathTeachingBh;
	GlobalLocalizationBehavior m_GlobalLocalizationBh;
	GlobalMapBuilding m_GlobalMapBuildingBh;
	KuINIReadWriter* m_pINIReaderWriter;
	int m_nSetGoalPosID,m_nSetRobotPosID;
	int m_nBhID;
	KuThread m_KuThread;
	bool m_bBehaviorStates;
	vector<KuPose> m_vecWayPoint;
	int m_nTotalFrame;
	bool m_bAutoThread;
	bool m_bAvoidMode;
	bool m_binitflag;
	KuPose m_PreGoalPose;
	KuPose m_PreRobotPose;
	ofstream m_DataLog;

public:
	void execute(KuCommandMessage CMessage);
	bool loadMap();
	bool getBehaviorStates(KuCommandMessage CMessage);
	Localizer* getLocalizer(KuCommandMessage CMessage);
	static void AutonomousSupervisor(void* arg);
	bool Autonomousexecute(KuCommandMessage CMessage );
	double checkWayPoint(KuPose GoalPos);
	void setBehaviorStates(bool  bBehaviorStates);
	bool getBehaviorStates( );
	bool playMovie(string strNameNPath,int  nWayPointIdx );
	double waitWayPoint(KuPose wayPoint);
	void setTotalTime(int nTotalFrame );
	void startSavingPath();
	void terminateSavingPath();
	bool connectCommunication();
	void saveRobotTrajectory();
	MapBuildingBehavior* getMapBuildingBehavior(void);

public:
	MobileSupervisor();
	virtual ~MobileSupervisor();


};

#endif