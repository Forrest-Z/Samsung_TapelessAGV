#ifndef C_ZONE_CONTROL_H
#define C_ZONE_CONTROL_H

#include <list>
#include <iostream>
#include<fstream>
#include <cmath>
#include"../../KUNSPose/KuPose.h"
#include"../../KUNSMap/KuMap.h"
#include"../../KUNSMath/KuMath.h"
#include"../../KUNSUtil/KuUtil.h"
#include"../../Sensor/Sensor.h"
#include"../../KUNSUtil/KUNSSingletone/KuSingletone.h"
#include "../../KUNSProcess/KUNSTeachingPathPlannerPr/KuTeachingPathPlannerPr.h"
#include "../../KUNSUtil/KUNSINIReadWriter/KuINIReadWriter.h"
#include "../../KUNSMap/KuMapRepository.h"
#include "../../KUNSUtil/KUNSSingletone/KuSingletone.h"
#include "../../MobileSupervisor/KuRobotParameter.h"
#include "../../MobileSupervisor/KuCommandMessage.h"



using namespace std;
class KuZoneControlPr:public KuSingletone <KuZoneControlPr >
{

private:
	list<KuPose> m_PathList;
	vector<KuPose> m_PathList1;
	vector<KuPose> m_PathList2;
	vector<KuPose> m_PathList3;
	list<KuPose> m_WayPointList;
	list<KuPose> m_ZonePosition;
	KuMath m_math;
	vector<KuPose>m_vectorWayPoint;
	vector<KuPose>vectorWayPoint_temp;
	vector<KuPose>m_vectormediumPoint;
	KuMap* m_smtpMap;
	KuMap* m_dMap;
	vector<list<KuPose>> m_vecTotalPath;

private:
	void extractWayPoint(int nPathCnt);
	void compensateMap(int nSizeX, int nSizeY, int** nMap,int nZone);

public:
	void clear();
	list<KuPose> loadPath(string  strDataPath );
	vector<KuPose> extractWayPoint(list<KuPose> Pathlist);
	vector<KuPose> extractCrossPoint(list<KuPose> Pathlist_a,list<KuPose> Pathlist_b);
	vector<KuPose> extractmediumPoint(list<KuPose> Pathlist);
	KuPose distZone(KuPose RobotPos, list<KuPose> m_PathList);
	void LoadZoneMap();
	void  generateZoneMap(list<KuPose> ZonePointlist);
	void intersectionZoneMap(list<KuPose> ZonePointlist);

	int getZonedata(KuPose RobotPos);
	int getNextZonedata(KuPose RobotPos, KuPose TargetPos);

	void saveZoneMap(int nPathIdx);

	list<KuPose> ZonePoint(int nPathIdx);
	KuMap* getZoneMap();
	bool loadZoneMap(string strMapFilePath);
	void saveZoneMap(string strMapFilePath, KuMap* pMap);

	KuZoneControlPr();
	~KuZoneControlPr();
};

#endif
