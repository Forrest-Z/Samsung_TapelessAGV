#ifndef KuFiducialbasedLocalizerPr_H
#define KuFiducialbasedLocalizerPr_H


#include <list>
#include <iostream>
#include "../../KUNSPose/KuPose.h"
#include "../../KUNSMap/KuMap.h"
#include "../../KUNSMath/KuMath.h"
#include "../../KUNSUtil/KuUtil.h"
#include "../../sensor/Sensor.h"
#include "../../KUNSUtil/KUNSSingletone/KuSingletone.h"
#include "../../Localizer/Localizer.h"
#include "../../KUNSUtil/KUNSThread/KuThread.h"
#include "../../KUNSUtil/KUNSCriticalSection/KuCriticalSection.h"
#include "../../MobileSupervisor/KuRobotParameter.h"
#include "../../Algorithm/FiducialMarkRecognizer/ALRecognizer.h"

using namespace std;

class KuFiducialbasedLocalizerPr : public KuSingletone <KuFiducialbasedLocalizerPr>
{

private:
	CCriticalSection m_CriticalSection;

private:
	KuThread m_KuThread;
	KuUtil m_KuUtil;
	CALRecognizer m_ALRecognizer; 

private:
	KuPose m_RobotPos;
	KuPose m_RobotPosbyFidicial;
	KuPose m_poseFiducialMark;
	vector<KuPose> m_vecFidicialPos;
	KuPose m_dDelEncoderData;
	IplImage * m_pIplCeilingGrayImage;
	bool m_bMapping;
	bool m_binitflag;

	// 엔코더에 기반하여 상대적인 이동량의 누적치를 계산하는 기능	
	double m_dAccumulatedDeltaMovement;
	double m_dAccumulatedDeltaAngle;

	//스레드 관련 변수----------------------
	bool m_doThreadFunc;
	int m_nThreadFuncPeriod;
	bool m_bIsThreadFuncGenerated;
	list<KuPose> m_FiducialMarkImgCoordList;
	list<KuPose> m_FiducialMarkList;
	list<KuPose> m_FiducialMarkDBList;
	KuPose m_FiducialMarkImgCoord;
	int m_nFiducialID;
	int m_nFiducialCnt;
	vector<KuPose> m_vecFiducialMark;

private:

	//스레드 관련 함수-----------------------------------------
	static void doThread(void* arg);
	//---------------------------------------------------------

	//---------------------------------------------------------
	void copyEncoderData(KuPose delEncoderData);
	void copyImageData(IplImage* pIplCeilingImage);
	void computeAccumulatedDeltaMovement(KuPose EncoderDelPos);


public:
	//스레드 관련 함수-----------------------------------------
	void start(int nPeriod);
	void terminate();
	void suspend();
	void resume();

	bool getThreadStates();

	//---------------------------------------------------------
	bool isAccDeltaMovementOver(double dMovement, double dAngle);
	void setRobotPos(KuPose RobotPos,bool bReliableflag);
	void setRobotPosX(double dPoseX);
	void setRobotPosY(double dPoseY);
	void setRobotPosDeg(double dPoseDeg);
	void setRobotPosRad(double dPoseRad);
	double getRobotPosX();
	double getRobotPosY();
	double getRobotPosDeg();
	double getRobotPosRad();
	KuPose getFiducialMarkPose(void);
	
	void init();

	KuPose getRobotPos();
	KuPose estimateRobotPosbyFiducialmark(KuPose RobotPos,IplImage* IplCeilingImage, bool bMapping);
	KuPose transferLocalPos2GlobalPos(KuPose FiducialLandMarkPos);
	void estimateRobotPosbyFiducialmark();
	void loadFiducialFeatureMap();
	list<KuPose> registerFiducialMark(KuPose RecogFiducialMark);
	list<KuPose> getFiducialMarkImgCoordList();
	list<KuPose> getFiducialMarkList();
	void saveFiducialFeatureMap();
	void initFiducialMark();
	KuPose getRobotPosbyFiducial();
	KuPose getFiducialMarkImgCoord();


	KuFiducialbasedLocalizerPr();
	~KuFiducialbasedLocalizerPr();

};

#endif 