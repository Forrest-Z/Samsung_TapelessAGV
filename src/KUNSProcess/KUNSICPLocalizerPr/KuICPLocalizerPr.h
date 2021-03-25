#ifndef CICP_LOCALIZER_H
#define CICP_LOCALIZER_H

#include <list>
#include <iostream>
#include <cmath>
#include "../../Algorithm/KuICP/KuICP.h"
#include "../../KUNSPose/KuPose.h"
#include "../../KUNSMap/KuMap.h"
#include "../../KUNSMath/KuMath.h"
#include "../../KUNSUtil/KuUtil.h"
#include "../../KUNSUtil/KUNSSmartPointer/KuSmartPointer.h"
#include "../../sensor/Sensor.h"
#include "../../KUNSUtil/KUNSSingletone/KuSingletone.h"
#include "../../Localizer/Localizer.h"
#include "../../MobileSupervisor/KuRobotParameter.h"

using namespace std;

class KuICPLocalizerPr:public Localizer 
{
	static const int LASER_SCAN_IDX = 181;
private:
	KuMath m_math;
	KuICP m_pICP;

	double m_dMaxSensorRangeMM; //사용할 센서데이터의 최대 거리, unit mm
	double m_dMinSensorRangeMM;
	double m_dLaserSensorOffsetmm; //레이저 센서와 로봇 중심간의 거리, unit mm
	double m_dDistanceForICPExecution;

	KuPose m_RobotPos;
	KuPose m_DeltaPos;
	KuPose m_DeltaPosForICP;	
	KuPose m_LastRobotPosForICP;
	KuCartesianCoordinate2D m_CartesianCoordiNewRangeData[181];
	KuCartesianCoordinate2D m_CartesianCoordiLastRangeData[181];
	double m_dAccumulatedDeltaMovementForICP; // 일정 거리 이상 움직인 것을 파악하여 ICP를 수행하기 위해 이동거리를 계산하는 기능

	// -------------- ICP-based matching ------------------------ //

private:
	void initialize(); ///각종 변수등을 초기화 하는 함수		

	void calculateRobotPosByEncoderData();	//엔코더 데이터만을 이용한 로봇위치 계산 함수
	void computeRobotPoseByICPFor2D(double dX, double dY, double dTheta);
	void computeAccumulatedDeltaMovementForICP(KuPose DeltaPos);
	bool isAccDeltaMovementForICPOver(double dValue);	
	void copyRangeData(int_1DArray Data);  //poloar coordinate기준의 거리 센서 값을, cartesian coordinate기준의 값을 변경한다.
	bool setDeltaPose(KuPose DeltaPos);

	double calDeltaPosebyRangeData(KuCartesianCoordinate2D* CartesianCoordiLastRangeData, 
		KuCartesianCoordinate2D* CartesianCoordiNewRangeData,KuPose DeltaPos,float *fx,float *fy,float *ft);

	KuPose computeRobotPoseByICPFor2DfromRobotPos(double dX, double dY, double dTheta,KuPose DeltaPos,KuPose PreRobotPos);
	void calculateRobotPosByEncoderData(KuPose DeltaPos);

public:		
	KuPose estimateRobotPos(int_1DArray Data, KuPose DeltaEncoderData); //ICP 알고리듬을 이용하여 로봇의 위치를 추정하는 함수

	//로봇 위치 설정관련 함수----------------------------------------------------------
	void setInitRobotPos(KuPose RobotPos);	
	KuPose getRobotPos();
	void setRobotPos(KuPose RobotPos);
	double getRobotPosX();
	double getRobotPosY();
	double getRobotPosDeg();
	void setRobotPosX(double dRobotPosX);
	void setRobotPosY(double dRobotPosY);
	void setRobotPosDeg(double dRobotPosThetaDeg);
	//============================================================================
	KuICPLocalizerPr();
	~KuICPLocalizerPr();
};

#endif


