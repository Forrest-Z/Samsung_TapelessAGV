
#ifndef KUNS_IMAGE_LINE_BASED_PARTICLE_FILTER_LOCALIZER_PROCESS_H
#define KUNS_IMAGE_LINE_BASED_PARTICLE_FILTER_LOCALIZER_PROCESS_H


#include <vector>
#include <iostream>
#include "../../Algorithm/ParticleFilter/ImagebasedParticleFilter .h"
#include "../../sensor/Sensor.h"
#include "../../KUNSPose/KuPose.h"
#include "../../KUNSUtil/KUNSThread/KuThread.h"
#include "../../KUNSUtil/KUNSSingletone/KuSingletone.h"
#include "../../KUNSUtil/KUNSSmartPointer/KuSmartPointer.h"
#include "../../KUNSGUI/KuDrawingInfo.h"
#include "../../MobileSupervisor/KuRobotParameter.h"
#include "../../Localizer/Localizer.h"

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/legacy/legacy.hpp>

#include "../../Algorithm/FeatureDetector/FeatureDetector.h"
#include "../../Algorithm/FeatureDetector/freak.h"
#include "../../Algorithm/FeatureDetector/hammingseg.h"

#include "../../Sensor/SiriusCameraInterface/KuSiriusCameraInterface.h"
#include <newmat.h>

#define MAX_CORNERS 500
#define NUMBER_CHOSEN 5

using namespace cv;
using namespace std;
class KuILBPFLocalizerPr : public KuSingletone <KuILBPFLocalizerPr>,public Localizer
{

private:
	CCriticalSection m_CriticalSection;

private:
	KuThread m_KuThread;
	KuThread m_KuThreadforHeight;

	KuUtil m_KuUtil;
	ImagebasedParticleFilter m_ParticleFilter;	
	CFeatureDetector m_FDetectorAlg;
private:
	KuPose m_RobotPos;
	int_1DArray m_nLaserData;
	double m_dEstimatedRangeData[Sensor::URG04LX_DATA_NUM181];

	KuPose m_dDelEncoderData;
	KuPose m_PreRobotpos;


	// 엔코더에 기반하여 상대적인 이동량의 누적치를 계산하는 기능
	double m_dAccumulatedDeltaMovement;
	double m_dAccumulatedDeltaAngle;



	//파티클 필터 관련---------------------------------------------------------
	//스레드 관련 변수----------------------
	bool m_bIsThreadFuncGenerated;
	bool m_doThreadFunc;
	int m_nThreadFuncPeriod;
	//--------------------------------------
	bool m_bIsNewSensorData,m_bfirst;
	//-----------------------------------------------------------------------

	int m_nPaticleFilterState; //파티클 필터의 상태저장 변수
	int m_nParticleNum; //파티클 갯수 저장 변수
	vector<Sample> m_vecParticle;// 파티클들을 저장할 vector형 변수

	bool m_bIsParticlConverged;
	bool m_bsaveFT;
	bool m_bsaveFR;



private:
	Mat m_cvMatImage;
	Mat m_cvMatImageBright;
	Mat m_cvMatImageT;
	int **m_nMap;
	double m_dHeight;
	bool m_bMapping;
	vector<CLAMPData> m_FRD;
	vector<CFREAKData> m_FTD;
	double m_dCam_f;


private:
	//스레드 관련 함수-----------------------------------------
	static void doThread(void* arg);
	//---------------------------------------------------------
	void copyRangeData(int_1DArray nLaserData);
	void copyCeilingImage(IplImage * IplCeilingImage);

	void computeAccumulatedDeltaMovement(KuPose EncoderDelPos);

	inline int max(int a, int b);
	inline int min(int a, int b);

	void startTimeCheck(LARGE_INTEGER& nStart);
	float finishTimeCheck(const LARGE_INTEGER nStart);

public:
	vector<CLAMPData>  getRegion();
	void saveFeatureData();
	vector<CLAMPData> loadFeatureData( );
	vector<CFREAKData> getTampletData();
	void copyEncoderData(KuPose delEncoderData);
	bool checkdoMapping();
	void initEncoderData();
	bool getThreadStates();

public:

	//파티클 필터 관련 함수------------------------------------
	void setRangeSensorParameter();
	void estimateRobotPosUsingParticleFilter();
	void setDeviation(double dDeviationforTrans, double dDeviationforRotate,double dDeviationforTransRotate );
	void setSampleNum(int nMaxSample,int nMinSample);
	bool getParticleConvergeStatus();
	void setSamples();
	void spreadParticleNearRobot(KuPose RobotPos, double dRegionSize); //로변 주변 영역에 파티클들을 뿌려준다.
	void updateMapData(int nMapSizeX, int nMapSizeY, int** nMap);
	//---------------------------------------------------------

	vector<Sample> getParticle();

	void saveParameter();
	
	vector<CFREAKData> loadTampleteData( );
	void saveTampleteData();
	void calLampHeight(KuPose RobotPos,vector<CLAMPData> *FRD);
	void registerLamp(KuPose RobotPos, vector<CLAMPData> *FRD);
	void symmetryTest(vector<DMatch>& matches1, vector<DMatch>& matches2, vector<DMatch>& symMatches);


	//스레드 관련 함수-----------------------------------------
	void start(int nPeriod, bool bMapping=false);
	void terminate();
	//---------------------------------------------------------
	bool isAccDeltaMovementOver(double dMovement, double dAngle);


	//-----------------------------------------------------------------
	void setRobotPos(KuPose RobotPos);
	void setRobotPosX(double dPoseX);
	void setRobotPosY(double dPoseY);
	void setRobotPosDeg(double dPoseDeg);
	void setRobotPosRad(double dPoseRad);
	double getRobotPosX();
	double getRobotPosY();
	double getRobotPosDeg();
	double getRobotPosRad();
	void setMap(int nMapSizeX, int nMapSizeY, int** nMap);
	//------------------------------------------------------------------

	void init();

	KuPose getRobotPos();
	KuPose estimateRobotPos(int_1DArray nRangeData, KuPose RobotPos, KuPose EncoderDelPos,IplImage * IplCeilingImage ,bool bMapping);
	KuPose estimateRobotPosByDeadReckoning(KuPose EncoderDelPos);

	void trans_global2img(const float& fRx, const float& fRy, const float& fRth, const float& fFx, const float& fFy, const float& fFz, int& nu, int& nv, const CamContext* pContext);

	void trans_img2global(const float& fRx, const float& fRy, const float& fRth, const int& nu, const int& nv, float& fFx, float& fFy, float& fFz, const CamContext* pContext, double dHeight);
	
	void initFREAKData(vector<CFREAKData> *CFData,KuPose RobotPos);
	void matchFREAKData(vector<DMatch> *matches, vector<CFREAKData> *mergeFTD,
		vector<CFREAKData> *tempFTD,vector<DMatch> *matchesA,vector<CFREAKData> CFData ,KuPose RobotPos);
	void KuILBPFLocalizerPr::checkAssociateFREAKData(vector<DMatch> *matches, vector<CFREAKData> *mergeFTD,
		vector<CFREAKData> *tempFTD,vector<DMatch> *matchesA,vector<CFREAKData> CFData ,KuPose RobotPos );
	void calFREAKHeight(int nIDXA,int nIDXB,KuPose RobotPos,double dLampH, vector<CFREAKData> *tempFTD,vector<CFREAKData>* CFData);
	bool exceptionmatchhandling(int nIDXA, int nIDXB, vector<CFREAKData>tempFTD,vector<CFREAKData>CFData,vector<CFREAKData> *mergeFTD);
	void eliminateFREAKData(KuPose RobotPos,vector<CFREAKData>*mergeFTD);
	void updateFREAKData(int nIDXA, int nIDXB,vector<CFREAKData>tempFTD,vector<CFREAKData> CFData,vector<CFREAKData>*mergeFTD);

	KuILBPFLocalizerPr();
	~KuILBPFLocalizerPr();

};

#endif 
