#ifndef KUNS_GO_TO_GOALL_BEHAVIOR_H
#define KUNS_GO_TO_GOALL_BEHAVIOR_H

#include <iostream>
#include <conio.h>
#include <MMSystem.h>
#include "../../KUNSUtil/KUNSThread/KuThread.h"
#include "../../KUNSGUI/KuDrawingInfo.h"
#include "../../KUNSPose/KuPose.h"
#include "../../KUNSMath/KuMath.h"
#include "../../KUNSUtil/KUNSCriticalSection/KuCriticalSection.h"
#include "../../KUNSMap/KuMapRepository.h"
#include "../../KUNSProcess/KUNSPathPlannerPr/KuGradientPathPlannerPr.h"
#include "../../KUNSProcess/KUNSPathPlannerPr/KuPathSmoothing.h"
#include "../../Sensor/WheelActuatorInterface/SSAGVWheelActuatorInterface.h"
#include "../../KUNSProcess/KUNSKanayamaMotionControlPr/KuKanayamaMotionControlPr.h"
#include "../../KUNSUtil/KUNSINIReadWriter/KuINIReadWriter.h"
#include "../../KUNSProcess/KUNSLaserBasedParticleFilterLocalizerPr/KuLBPFLocalizerPr.h"
#include "../../Sensor/SensorSupervisor.h"
#include "../../KUNSUtil/KuUtil.h"
#include "../../KUNSProcess/KUNSTeachingPathPlannerPr/KuTeachingPathPlannerPr.h"
#include  "../../KUNSProcess/KUNSLaserMapBuilderPr/KuLaserMapBuilderPr.h"
#include "../../KUNSProcess/KUNSScanMatchingLocalizerPr/KuScanMatchingLocalizerPr.h"
#include "../../KUNSProcess/KUNSImageLineBasedParticleFilterLocalizerPr/KuILBPFLocalizerPr.h"
#include "../../KUNSProcess/KUNSICPLocalizerPr/KuICPLocalizerPr.h"
#include "../../KUNSProcess/KuFiducialbasedLocalizerPr/KuFiducialbasedLocalizerPr.h"
#include "../../KUNSProcess/KUNSPathBlockPr/KuPathBlockPlannerPr.h"
#include "../../KUNSProcess/KUNSPathBlockPr/KuPathBlockPr.h"
#include "../../Sensor/VirtualSensor/KuVrWheelActuatorInterface.h"
#include "../../AGVComm/AGVCommSupervisor.h"
#include "../../MultiRobotSupervisor/XmldataSetting.h"
#include "../../MultiRobotSupervisor/Clientpart.h"

using namespace std;
class GotoGoalBehavior 
{
	static const int INFINITY_VALUE = 100000;
	static const int GOAL_BOUNDARY = 2500;
	static const int LOCALGOAL_DISTANCE = 5000;

public:
	static const int JOB_START = 1;
	static const int JOB_STOP = 2;
	static const int JOB_PAUSE = 3;
	static const int JOB_RESUME = 4;
	static const int JOB_COMPLETE = 5;
	static const int JOB_CHANGE = 6;

private:
	CCriticalSection m_CriticalSection;
	KuKanayamaMotionControlPr m_KanayaMC;
	KuMath m_math;
	KuThread m_ObstacleThread;
	KuThread m_thread_sound_obstacle_detection;
	KuThread m_thread_sound_low_battery;
	KuThread m_Thread;
	KuTeachingPathPlannerPr m_KuTeachingPathPlannerPr;
	KuGradientPathPlannerPr m_KuPathPlanner; //경로계획기 인스턴스
	KuPathSmoothing m_KuPathSmoothing;
	KuUtil m_KuUtil;
	KuLaserMapBuilderPr m_LaserMapBuilder;
	KuGradientPathPlannerPr m_KuLocalPathPlanner; //지역 경로 계획기 생성.
	KuThread m_KuMappingThread;
	KuICPLocalizerPr m_ICPLocalizer;
	KuVrWheelActuatorInterface m_KuVrWheelActuator; 

private:
	bool m_bAbnormalState;
	KuPose m_RobotPos, m_GoalPos;		 
	bool m_bThreadFlag; 
	bool m_bPlaySoundObstacleDetection;
	bool m_bPlaySoundLowBattery;
	list<KuPose> m_listPath;
	list<KuPose> m_listWayPoint;
	vector<KuPose> m_vecPath;
	vector<KuPose> m_vecWayPoint;
	vector<KuPose> m_vecLocalPath;
	vector<KuPose> m_vecImagePath;
	KuSmartPointer<KuMap> m_smtpMap; //지도정보를 저장할 공간.
	KuINIReadWriter* m_pINIReaderWriter;
	int m_nSetGoalPosID,m_nSetRobotPosID;
	int m_nBhID;
	KuPose m_TargetPos;
	bool m_bWaitflag;
	int m_nselectIdx;
	int m_nSelectTargetIdx;
	int m_nSelectedMinIndx;
	KuPose m_CurWayPoint;
	int m_nWaitingTime;
	int m_nPrePathIndx;
	KuMap* m_pRefMap;
	KuMap *m_pLocalMap;
	KuMap *m_pOriginalMap; 
	KuPose m_LocalGoalPos;
	int m_nLocalGoalIndex;
	int m_nLocalGoalWayPointIndex;
	KuMap* m_pMap;
	int m_nMapSizeX;
	int m_nMapSizeY;
	int m_nObstacleDetectionTime;
	double m_dCheckObstacleDetectionTime;
	bool m_bMapping;
	bool m_binitAlignRobotAngle;
	ofstream m_DataLog;
	int m_nIDX;
	LARGE_INTEGER m_LITime;
	double m_dTimeA;
	double m_dTimeB;
	bool m_bterminate;
	KuPose m_StartPoint;
	KuPose m_WayPoint;
	int m_nIFDX;
	int m_nFCount;
	double m_dDistMarkfromRobot;
	bool m_bfirstLamp;
	bool m_bonLamp;
	int m_nTransVel; 
	int m_nRotVel;
public:
	double m_dPosDiff;
	KuPose m_RobotPosPrev;

private:
	list<KuPose>  m_DetourPathList;
	int m_nLocalMapSPosX, m_nLocalMapSPosY;
	int m_nGlobalMapX, m_nGlobalMapY;
	int m_nLocalMapX, m_nLocalMapY;
	int m_nBuildingMapX, m_nBuildingMapY;
	bool m_bAvoidModeflag;

private:
	IplImage* m_CeilingCamera;
	KuPose m_DelEncoderData;
	int_1DArray m_nLaserData181;
	int_1DArray m_nKinectLaserData;
	//int_1DArray m_nCombinateRangeData;	
	IplImage *m_IplKinectImage;
	int_1DArray m_nSonarObsState;
private:
	int m_nDistToTarget;
	int m_nDesiredVel ;
	int m_nGoalArea ;
	int m_nMaxTVel;
	int m_nMinTVel;

private://zigbee를위한 flag
	bool m_bOtherAGVPathFlag;

private:
	void initial();
	static void doThread(void* arg);
	static void doThreadwithAvoid(void* arg);
	static void doThreadForMapping(void* arg);
	static void doObstacleThread(void* arg);
	static void thread_sound_obstacle_detection(void* arg);
	static void thread_sound_low_battery(void* arg);
	void startLocalizer(KuPose RobotPos,bool blocalizer);
	bool initialize(KuCommandMessage CMessage);
	KuPose getTargetPosbyLocalPath(KuPose RobotPos,int nDistToTarget);
	KuPose getTargetPos(KuPose RobotPos,int nDistToTarget);
	KuPose getStaticTargetPos(KuPose RobotPos,int nDistToTarget);
	bool checkObstacles(KuPose RobotPos,KuPose TargetPos, int_1DArray nLaserData181, int_1DArray nKinectRnageData ,double dTargetDist);
	void initKanayamaProcess();
	void drawNaviData();
	void controlVelocityforMapInformation( KuPose PointPos, int** nMap);
	KuPose checkWayPoint(KuPose TargetPos,KuPose RobotPos, int *nselectIdx ,bool *bWaitflag);
	void initMapbuildingProcess(bool bLocalMapflag);
	bool initPathplanningProcess();
	int_1DArray combinateLaserNKinect(int_1DArray nLaserData181, int_1DArray nKinectRnageData);
	void generateLocalMap(KuPose RobotPos,KuPose DelEncoderData,int_1DArray nLaserData181, int_1DArray nKinectRnageData);
	void copyBuildingMapToGlobalMap(int** nBuildingMap, int** nGlobalMap, int** nOriginalMap, KuPose RobotPos);
	void copyGlobalMapToLocalMap(int**nGlobalMap, int** nLocalMap, KuPose RobotPos, int* nLocalMapSPosX, int* nLocalMapSPosY);
	void copyOriginalMapToGlobalMap(int** nOriginalMap, int** nGlobalMap, KuPose RobotPos);
	bool generateLocalPath();
	KuPose generateDetourGoalPos(KuPose RobotPos, vector<KuPose> vecPath, int nPathSize);
	bool generateDetourPath(KuPose RobotPos, KuPose DetourGoalPos, KuMap* pLocalMap, 
		int nLocalMapSPosX, int nLocalMapSPosY, list<KuPose> *DetourPathList);
	list<KuPose> transferLocaltoGlobalPath( list<KuPose> LocalPath,KuPose RobotPose,int nLocalMapSPosX, int nLocalMapSPosY);
	bool existObstaclePath(KuPose RobotPos, vector<KuPose> vecLocalPath, int nPathSize, KuMap* pLocalMap, int nLocalMapSPosX, int nLocalMapSPosY );
	bool generateLocalPathforObsAvoidance(KuPose RobotPos,KuPose DelEncoderData,int_1DArray nLaserData181, int_1DArray nKinectRnageData);
	bool checkObstacles(int_1DArray nLaserData181, int_1DArray nKinectRnageData);
	KuPose checkWayPointwithlocalpath(KuPose TargetPos,KuPose RobotPos, int *nselectIdx ,bool *bWaitflag);
	void initICPProcess();
	bool alignRobot(int_1DArray nData, KuPose GoalPos);
	void copyBuildingMapToGlobalMap(int** nBuildingMap, int** nGlobalMap,  KuPose RobotPos);
	void initAlignProcess();
	void playSound(string sFileName);
	void calStartPathPoint( );
	void startTimeCheck(LARGE_INTEGER& nStart);
	float finishTimeCheck(const LARGE_INTEGER nStart);
	void saveRobotData();
	int doObsDetection(double dRotVel);
	bool initPathBlock(KuPose RobotPos,KuPose GoalPos);
	double m_dRecognizingMarkDistTh;

public:	
	bool execute(KuCommandMessage CMessage);
	void terminate();
	bool getBehaviorStates();
	Localizer* getLocalizer();
	bool doGlobalLocaliztion();
	KuPose getCurWayPoint( );
	void setWaitTime(int nTime );
	void playSoundObstacleDetection(bool bPlay);
	void playSoundLowBattery(bool bPlay);
	void initPath(list<KuPose> Pathlist );
	int getTransVel();
	int getRotVel();
	bool initBlockPath(vector<PBlock> vecPathBlock);//0515
	bool getAbnormalState(void);
	void setAbnormalState(bool bState);
public:
	GotoGoalBehavior();
	~GotoGoalBehavior();

};

#endif 