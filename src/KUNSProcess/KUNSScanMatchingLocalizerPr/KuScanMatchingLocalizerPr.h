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
	double m_dMaxSensorRangeMM; //����� ������������ �ִ� �Ÿ�, unit mm
	double m_dMinSensorRangeMM; //����� ������������ �ִ� �Ÿ�, unit mm
	double m_dLaserSensorOffsetmm; //������ ������ �κ� �߽ɰ��� �Ÿ�, unit mm
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

	double m_dAccumulatedDeltaMovementForICP; // ���� �Ÿ� �̻� ������ ���� �ľ��Ͽ� ICP�� �����ϱ� ���� �̵��Ÿ��� ����ϴ� ���
	CCriticalSection m_CriticalSection;
	int m_nNewRangeDataSize;
	int m_nLastRangeDataSize;
	double m_dError;
	double m_dPro;
	double m_dKXGain ;   //�κ��� �������� ������ �غ��ϴ� ����. ���� �κ��� ��� �ʹ� ũ�� �ⷷ�̰�, �ʹ� ������ ���ϴ� �ӵ����� ������ ������.
	double m_dKYGain ;   //�κ��� ������� ������ �غ��ϴ� ����. ������ ũ���� ������, ũ�� �Ҿ����ϰ�, ������ ������� ������ ���� ����.
	double m_dKThetaGain ;  //�������� ���� �κ��� ����(heading)�� �غ��ϴ� ����. ũ�� �ſ� �ⷷ�̰�(Ư�� �ʱ⿡ ���ڸ����� �������� ���� �� ��), ������ �� ������ ����.
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
	void initialize(); ///���� �������� �ʱ�ȭ �ϴ� �Լ�		
	static void doThread(void* arg);

	void calculateRobotPosByEncoderData();	//���ڴ� �����͸��� �̿��� �κ���ġ ��� �Լ�
	void computeRobotPoseByICPFor2D(double dX, double dY, double dTheta);
	void computeAccumulatedDeltaMovementForICP(KuPose DeltaPos);
	bool isAccDeltaMovementForICPOver(double dValue);	
	void copyRangeData(int_1DArray Data);  //poloar coordinate������ �Ÿ� ���� ����, cartesian coordinate������ ���� �����Ѵ�.
	double setDeltaPose(KuPose DeltaPos);
	void  calculateLaserDatafromMap( KuPose RobotPos,KuMap* pMap);
	inline void copyRangeDatafromMap(int_1DArray  nData);
	inline double GetRandValue(double sigma);
	void  penetrateLaserDatafromMap( KuPose RobotPos,KuMap* pMap);
	void copyRangeDatafromMapT(int_1DArray  nData);
	double GetGaussianValue(double sigma, double x);

public:		
	double estimateRobotPos(int_1DArray Data, KuPose DeltaEncoderData,KuPose RobotPos,KuMap* pMap); //ICP �˰����� �̿��Ͽ� �κ��� ��ġ�� �����ϴ� �Լ�
	KuPose estimateRobotPosByDeadReckoning(KuPose RobotPos ,KuPose EncoderDelPos);
	KuPose estimateRobotPosByDeadReckoning(KuPose EncoderDelPos);
	KuPose computeDeltaPose( int_1DArray nPreLaserData, int_1DArray nNewLaserData);
	double  estimateRobotPosP(int_1DArray nData, KuPose DeltaEncoderData,KuPose RobotPos,KuMap* pMap,bool* bMapping);
	KuPose  getDeltaPos();


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


