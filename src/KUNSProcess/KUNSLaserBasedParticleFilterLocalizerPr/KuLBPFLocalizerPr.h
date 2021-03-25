/*______________________________________________________________________________________________
PROJECT : Intelligent Robotics Lab. Department of Mechanical Engineering Korea University                                    
(c) Copyright 2012 KUME Intelligent Robotics Lab.                                             
All rights reserved                                                                         
$Directed by Jae-Bok Song                                                                     
$Designed by Joong-Tae Park                                                                  
$Description :레이저센서와 particle filter를 사용하여 위치추정을 수행하는 프로세스
$Created on: 2012. 6. 12.                                                                        
$Author: Joong-Tae Park      
$e-mail: jtpark1114@gmail.com
______________________________________________________________________________________________*/


#ifndef KUNS_LASER_BASED_PARTICLE_FILTER_LOCALIZER_PROCESS_H
#define KUNS_LASER_BASED_PARTICLE_FILTER_LOCALIZER_PROCESS_H


#include <vector>
#include <iostream>
#include "../../Algorithm/ParticleFilter/LaserBasedParticleFilter.h"
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
#include "../../Algorithm//KuICP/KuICP.h"

using namespace cv;
using namespace std;
class KuLBPFLocalizerPr : public KuSingletone <KuLBPFLocalizerPr>,public Localizer
{
private:
	CCriticalSection m_CriticalSection;
private:
	KuThread m_KuThread;
	KuUtil m_KuUtil;
	LaserBasedParticleFilter m_ParticleFilter;	

	KuPose m_RobotPos;
	int_1DArray m_nLaserData;
	double m_dEstimatedRangeData[Sensor::URG04LX_DATA_NUM181];

	KuPose m_dDelEncoderData;
	KuPose m_StartGazerData;

	//list<MatrixElement> m_lMatrixElement;

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

private:
	vector<Mat> m_descriptors;
	Mat m_cvMatDBImage;
	Mat m_cvMatImage;
	vector<KuPose> m_vecImagePath;
	int m_nImagePathSize;
	int m_nSelectPathIdx;
	Mat m_Ceilingdescriptors;
	int m_nSpreadIdx;
	vector<KeyPoint> m_keypoints;
	int **m_nMap;
	int m_nMapSizeX;
	int m_nMapSizeY;

private:

	//스레드 관련 함수-----------------------------------------
	static void doThread(void* arg);
	//---------------------------------------------------------
	void copyRangeData(int_1DArray nLaserData);
	void copyEncoderData(KuPose delEncoderData);
	void computeAccumulatedDeltaMovement(KuPose EncoderDelPos);

public:

	//파티클 필터 관련 함수------------------------------------
	void setRangeSensorParameter();
	void estimateRobotPosUsingParticleFilter();
	void setDeviation(double dDeviationforTrans, double dDeviationforRotate,double dDeviationforTransRotate );
	void setSampleNum(int nMaxSample,int nMinSample);
	//---------------------------------------------------------


	bool getParticleConvergeStatus();

	//스레드 관련 함수-----------------------------------------
	void start(int nPeriod);
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

	vector<Sample> getParticle();
	void init();

	void spreadParticleNearRobot(KuPose RobotPos, double dRegionSize); //로변 주변 영역에 파티클들을 뿌려준다.
	KuPose getRobotPos();
	//	KuPose estimateRobotPos(int_1DArray nRangeData, KuPose EncoderDelPos);
	KuPose estimateRobotPos(int_1DArray nRangeData, KuPose EncoderDelPos,IplImage * IplCeilingImage );

	KuPose estimateRobotPosByDeadReckoning(KuPose EncoderDelPos);
	void setSampleRegion(int minX, int maxX, int minY, int maxY){}
	void resetSamples();

	void loadDB();
	vector<KuPose> loadImagePath( );

	void copyCeilingImage(IplImage * IplCeilingImage);

		KuPose computeDeltaPose( double_1DArray dPreLaserData, double_1DArray dNewLaserData);
	void rangeData2Array(	double_1DArray  stRangaData, double * preRangeData);


	KuLBPFLocalizerPr();
	~KuLBPFLocalizerPr();



};

#endif /*KUNS_LASER_BASED_PARTICLE_FILTER_LOCALIZER_PROCESS_H*/

