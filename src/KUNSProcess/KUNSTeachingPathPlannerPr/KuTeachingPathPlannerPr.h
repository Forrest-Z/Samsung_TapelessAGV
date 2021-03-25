#ifndef CTEACHING_PATH_PLANNER_H
#define CTEACHING_PATH_PLANNER_H



#include <list>
#include <iostream>
#include<fstream>
#include <cmath>
#include "../../KUNSPose/KuPose.h"
#include "../../KUNSMap/KuMap.h"
#include "../../KUNSMath/KuMath.h"
#include "../../KUNSUtil/KuUtil.h"
#include "../../sensor/Sensor.h"
#include "../../KUNSUtil/KUNSSingletone/KuSingletone.h"


using namespace std;
class KuTeachingPathPlannerPr
{

private:
	static const int WAYPOINT_VALUE =2;
	list<KuPose> m_PathList; 
	list<KuPose> m_WayPointList;
	list<KuPose> m_SubGoalList; 
	KuMath m_math;
	KuPose m_RobotPos;
	KuPose m_WayPoint;



private:
	void extractWayPoint(int nPathCnt);
public:
	void clear(); //기존의 경로 및 경유점 정보를 삭제한다.
	list<KuPose> getPathList();   // 목표지점까지의 경로 list를 전달.
	list<KuPose> loadPath(string  strDataPath );
	list<KuPose> execute(KuPose RobotPos, bool *bsetWayPointflag=false);
	float getAngleDiff(const float& fAngle1, const float& fAngle2);
	void  savePath( string strDataPath);
	void  setWayPoint( );
	void  saveWayPointList(string strDataWaypoint );
	list<KuPose> getWayPointList();
	list<KuPose> loadWayPointList( string  strDataWaypoint);

	KuTeachingPathPlannerPr();
	~KuTeachingPathPlannerPr();
};

#endif

