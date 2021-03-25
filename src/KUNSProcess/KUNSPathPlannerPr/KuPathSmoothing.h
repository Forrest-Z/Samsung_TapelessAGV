#ifndef C_PATH_SMOOTHING_H
#define C_PATH_SMOOTHING_H

#include <list>

#include <iostream>
#include <vector>
#include "./matrix/MatrixAlgebra.h"
#include "../../KUNSPose/KuPose.h"
#include "../../KUNSMap/KuMap.h"
#include "../../KUNSMath/KuMath.h"
#include "../../KUNSMap/KuMapRepository.h"

using namespace std;

class KuPathSmoothing
{
static const int WAY_POINT_INTERVAL = 5;
static const int BIG_WAY_POINT_INTERVAL= 200;
static const int  WAY_POINT_VALUE=5;

private:
KuMath m_math;
list<KuPose> m_WayPointList;
list<KuPose> m_InflectionWayPointList;

private:
	list<KuPose> doCubicSplineInterpolation (double timeSlice, list<KuPose> stlWayPoint);
	bool Obstacleflag(int nmidPointX,int nmidPointY);
	list<KuPose>  extractWayPoint(list<KuPose>PathList, int nPathCnt);

public:
	list<KuPose> smoothingPath(list<KuPose> originalPathList);
	list<KuPose> smoothingCubicSplinePath(list<KuPose> originalPathList);
	list<KuPose> generateWaypointFromPath(list<KuPose> originalPathList,int ninterval);
	list<KuPose> doLeastSquare(list<KuPose> waypointlist);
	list<KuPose> getWayPointList();
	list<KuPose> getInflectionWayPointList();

	KuPathSmoothing();
	virtual ~KuPathSmoothing();
	
};

#endif 
