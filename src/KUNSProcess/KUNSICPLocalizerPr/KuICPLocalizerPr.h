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

	double m_dMaxSensorRangeMM; //����� ������������ �ִ� �Ÿ�, unit mm
	double m_dMinSensorRangeMM;
	double m_dLaserSensorOffsetmm; //������ ������ �κ� �߽ɰ��� �Ÿ�, unit mm
	double m_dDistanceForICPExecution;

	KuPose m_RobotPos;
	KuPose m_DeltaPos;
	KuPose m_DeltaPosForICP;	
	KuPose m_LastRobotPosForICP;
	KuCartesianCoordinate2D m_CartesianCoordiNewRangeData[181];
	KuCartesianCoordinate2D m_CartesianCoordiLastRangeData[181];
	double m_dAccumulatedDeltaMovementForICP; // ���� �Ÿ� �̻� ������ ���� �ľ��Ͽ� ICP�� �����ϱ� ���� �̵��Ÿ��� ����ϴ� ���

	// -------------- ICP-based matching ------------------------ //

private:
	void initialize(); ///���� �������� �ʱ�ȭ �ϴ� �Լ�		

	void calculateRobotPosByEncoderData();	//���ڴ� �����͸��� �̿��� �κ���ġ ��� �Լ�
	void computeRobotPoseByICPFor2D(double dX, double dY, double dTheta);
	void computeAccumulatedDeltaMovementForICP(KuPose DeltaPos);
	bool isAccDeltaMovementForICPOver(double dValue);	
	void copyRangeData(int_1DArray Data);  //poloar coordinate������ �Ÿ� ���� ����, cartesian coordinate������ ���� �����Ѵ�.
	bool setDeltaPose(KuPose DeltaPos);

	double calDeltaPosebyRangeData(KuCartesianCoordinate2D* CartesianCoordiLastRangeData, 
		KuCartesianCoordinate2D* CartesianCoordiNewRangeData,KuPose DeltaPos,float *fx,float *fy,float *ft);

	KuPose computeRobotPoseByICPFor2DfromRobotPos(double dX, double dY, double dTheta,KuPose DeltaPos,KuPose PreRobotPos);
	void calculateRobotPosByEncoderData(KuPose DeltaPos);

public:		
	KuPose estimateRobotPos(int_1DArray Data, KuPose DeltaEncoderData); //ICP �˰����� �̿��Ͽ� �κ��� ��ġ�� �����ϴ� �Լ�

	//�κ� ��ġ �������� �Լ�----------------------------------------------------------
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


