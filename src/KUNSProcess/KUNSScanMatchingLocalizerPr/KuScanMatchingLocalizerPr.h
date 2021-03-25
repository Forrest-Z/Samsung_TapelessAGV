#ifndef KUNS_SCANMATCHING_ICP_LOCALIZER_H
#define KUNS_SCANMATCHING_ICP_LOCALIZER_H

#include <list>
#include <iostream>
//#include <cmath>
#include "../../KUNSUtil/KUNSThread/KuThread.h"

#include "../../Algorithm/KuICP/KuICP.h"

#include "../../KUNSPose/KuPose.h"
#include "../../KUNSMath/KuMath.h"
#include "../../KUNSMap/KuMap.h"
#include "../../KUNSUtil/KuUtil.h"
#include "../../KUNSUtil/KUNSSingletone/KuSingletone.h"
#include "../../KUNSGUI/KuDrawingInfo.h"

#include "../../Localizer/Localizer.h"
#include "../../sensor/Sensor.h"
#include "../../MobileSupervisor/KuRobotParameter.h"
#include "../../KUNSProcess/KUNSImageLineBasedParticleFilterLocalizerPr/KuILBPFLocalizerPr.h"
using namespace std;

class KuScanMatchingLocalizerPr:public Localizer, public KuSingletone <KuScanMatchingLocalizerPr>
{
	static const int LASER_SCAN_IDX = 181;
private:
	KuMath m_math;
	KuICP m_pICP;
	KuUtil m_kuUtil;
	KuThread m_Thread;
	int_1DArray m_nLaserData;
	KuMap *m_pMap;
	int m_nMapSizeX;
	int m_nMapSizeY;
	bool m_bICPExecution;
	double m_dMaxSensorRangeMM; //사용할 센서데이터의 최대 거리, unit mm
	double m_dMinSensorRangeMM; //사용할 센서데이터의 최대 거리, unit mm
	double m_dLaserSensorOffsetmm; //레이저 센서와 로봇 중심간의 거리, unit mm
	double m_dDistanceForICPExecution;
	int_1DArray m_nVirtualLaserData;
	KuPose m_RobotPos;
	KuPose m_estimatedRobotPos;
	KuPose m_DeltaPos;
	KuPose m_DeltaPosForICP;	
	KuPose m_LastRobotPosForICP;
	KuCartesianCoordinate2D m_CartesianCoordiNewRangeData[181];
	KuCartesianCoordinate2D m_CartesianCoordiLastRangeData[181];
	KuCartesianCoordinate2D m_CartesianCoordiLastRangeDataT[181*2];

	double m_dAccumulatedDeltaMovementForICP; // 일정 거리 이상 움직인 것을 파악하여 ICP를 수행하기 위해 이동거리를 계산하는 기능
	CCriticalSection m_CriticalSection;
	int m_nNewRangeDataSize;
	int m_nLastRangeDataSize;
	double m_dError;
	double m_dPro;
	double m_dKXGain ;   //로봇의 전진방향 오차를 극복하는 게인. 실제 로봇의 경우 너무 크면 출렁이고, 너무 작으면 원하는 속도보다 느리게 움직임.
	double m_dKYGain ;   //로봇의 측면방향 오차를 극복하는 게인. 영향이 크지는 않지만, 크면 불안정하고, 작으면 측면방향 오차를 보정 못함.
	double m_dKThetaGain ;  //목적지를 향한 로봇의 방향(heading)을 극복하는 게인. 크면 매우 출렁이고(특히 초기에 제자리에서 목적지를 향해 휙 돔), 작으면 딴 방향을 향함.
	double m_dDesiredVel;
	double m_dRatioofRadius;
	int_1DArray m_nTVirtualLaserData;
	bool m_bcheck;
	// -------------- ICP-based matching ------------------------ //
	bool m_bMapping;
	int_1DArray m_ncopyLaserData;
	bool m_bThreadFlag;
	 bool m_bIsThreadFuncGenerated;
private:
	void initialize(); ///각종 변수등을 초기화 하는 함수		
	static void doThread(void* arg);

	void calculateRobotPosByEncoderData();	//엔코더 데이터만을 이용한 로봇위치 계산 함수
	void computeRobotPoseByICPFor2D(double dX, double dY, double dTheta);
	void computeAccumulatedDeltaMovementForICP(KuPose DeltaPos);
	bool isAccDeltaMovementForICPOver(double dValue);	
	void copyRangeData(int_1DArray Data);  //poloar coordinate기준의 거리 센서 값을, cartesian coordinate기준의 값을 변경한다.
	double setDeltaPose(KuPose DeltaPos);
	void  calculateLaserDatafromMap( KuPose RobotPos,KuMap* pMap);
	inline void copyRangeDatafromMap(int_1DArray  nData);
	inline double GetRandValue(double sigma);
	void  penetrateLaserDatafromMap( KuPose RobotPos,KuMap* pMap);
	void copyRangeDatafromMapT(int_1DArray  nData);
	double GetGaussianValue(double sigma, double x);

public:		
	double estimateRobotPos(int_1DArray Data, KuPose DeltaEncoderData,KuPose RobotPos,KuMap* pMap); //ICP 알고리듬을 이용하여 로봇의 위치를 추정하는 함수
	KuPose estimateRobotPosByDeadReckoning(KuPose RobotPos ,KuPose EncoderDelPos);
	KuPose estimateRobotPosByDeadReckoning(KuPose EncoderDelPos);
	KuPose computeDeltaPose( int_1DArray nPreLaserData, int_1DArray nNewLaserData);
	double  estimateRobotPosP(int_1DArray nData, KuPose DeltaEncoderData,KuPose RobotPos,KuMap* pMap,bool* bMapping);
	KuPose  getDeltaPos();


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

	KuPose getEstimatedRobotPos();
	KuPose computeDeltaPoseforNavi( int_1DArray nPreLaserData, int_1DArray nNewLaserData,KuPose DeltaEncoderData);
	void setParameter(double dKXGain, double dKYGain,double dKThetaGain,double dDesiredVel, double dWheelbase);
	void setDeviation(double dDeviationforTrans, double dDeviationforRotate,double dDeviationforTransRotate );
	
	void terminate();
	bool getThreadStates();

	KuScanMatchingLocalizerPr();
	~KuScanMatchingLocalizerPr();
};

#endif


