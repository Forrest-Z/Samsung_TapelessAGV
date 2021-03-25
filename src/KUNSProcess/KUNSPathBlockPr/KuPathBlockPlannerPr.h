#ifndef KUNS_PATHBLOCK_PLANNER_PROC_H
#define KUNS_PATHBLOCK_PLANNER_PROC_H

#include <cmath>
#include <vector>
#include <list>
#include <fstream>
#include <MMSystem.h>
#include "KuPathBlockPr.h"
#include "../../KUNSPose/KuPose.h"
#include "../../Algorithm/PathBlock/PathBlock.h"
#include "../../KUNSUtil/KUNSSingletone/KuSingletone.h"
#include "../../KUNSUtil/KUNSThread/KuThread.h"
#include "../../MobileSupervisor/KuRobotParameter.h"
#include "../../KUNSMath/KuMath.h"
#include "../../Sensor/Sensor.h"
#include "../../Sensor/SwitchInterface/SwitchInterface.h"
#include "../../KUNSPRIMUSComm/KuPRIMUSCommSupervisor.h"

using namespace std;

class KuPathBlockPlannerPr: public KuSingletone <KuPathBlockPlannerPr>
{
private:
	vector<PBlock>m_vecPathBlockPos;
	vector<vector<PBlock>>m_vecGlobalPath;
	vector<PBlock>m_vecTotalPathBlockPos;
	KuPathBlockPr KPBPr;
	KuThread m_Thread;
	PBlock m_WayPointPathBlock;
	bool bTaskThreadflag;
	bool m_bOnWaypoint; // waypoint 위에 있는지 여부
	int m_nCurrentIDX;
	int m_nPreviousIDX;
	int m_nNextIDX;//0516
	int m_nWayPointIDX;
	int m_nWayPointIDXforTasks;
	int m_ncheckStep;
	KuPose m_RobotPose;
	KuPose m_CrtRobotPose;
	int m_nTransVel;
	int m_nRotateVel;
	bool m_bTerminateflag;
	bool m_bReversePathflag;
	bool m_bOtherAGVPathflag;
	bool m_bOpeningDoor; // 문을 열고 있는 중인지 여부
	bool m_bRobotPassedStartingPoint; // 처음 block을 지나갔는지 여부 (ISSAC)
	int m_nReturningBlockID; // 반환점 (ISSAC)
	bool m_bTaskRunning;
	bool m_bRunTaskThread;
	int m_nJobNum;

	int get_nearest_block_idx(KuPose poseRobot, int nStartIdx, int nEndIdx);
	int get_target_block_idx(int nCurrentBlockIdx, int nTargetOffset);
public:
	void initPreviousBlockIDX(void);
	void setPassedStartingPoint(bool bPassed);
	bool getPassedStartingPoint(void);
	void initialize(vector<PBlock> vecPathBlockPos);	
	bool generatePathBlock(KuPose RobotPose);	
	bool doMotion(int nMotion);
	bool doSound(int nSound);
	bool doDevice(int nDevice);
	bool doReadytoSignal(int nReadytoSignal);
	bool doTowerLamp(int nTowerLamp);
	static void doTaskThread(void* arg);
	bool doplayingSound(string strNameNPath,int  nIdx);
	double detectObstacles( KuPose RobotPos,int_1DArray nLaserData181, int_1DArray nKinectLaserData,int_1DArray nSonarObsState,double dRotVel);	
	bool doRotation(int nState);	
	void terminate();
	bool doFinemove();//123
	bool isOtherAGVPath();
	bool isEntryOtherAGVPath();
	bool isRobotOnWaypoint(); // 로봇이 waypoint 위에 있는지 여부
	bool isRobotOpeningDoor(); // 로봇이 현재 문을 열고 있는지 여부
	void setRobotOpeningDoor(bool bOpening);
	vector<PBlock>* getPathBlock(void); // 추가(140409)
	bool isTaskRunning(void);

	//set
	void setSequencePathblock(vector<PBlock> vecPathBlockPos, bool bFirstPath = false);
	void setGlobalPath(vector<vector<PBlock>> vecGlobalPath);
	void setReversePathflag(bool bReversePathflag);
	void setCurrentBlockIdx(int nCurrentIdx);
	void setNextBlockIdx(int nNextIdx);//0516

	//get
	KuPose getTargetPose(KuPose RobotPose);
	KuPose getTargetPoseISSAC(KuPose RobotPose);
	void  getPathBlockPos(vector<PBlock>*vecPathBlockPos);
	PBlock getCurrentPathBlock(vector<PBlock> vecPathBlockPos, KuPose RobotPose,int* IDX);

	PBlock getCurrentISSACPathBlock(vector<PBlock> vecPathBlockPos, KuPose RobotPose,int* IDX);//0517

	double getVelocityWeight();
	void getMotionData(int *nTransVel,int *nRotateVel);
	bool getTerminateState();
	void getGlobalPath(vector<vector<PBlock>>& vecGlobalPath);
	bool getReversePathflag();
	int getCurrentBlockIdx();
	int getNextBlockIdx();//0516
	void playSound(int nPath);
	void playSoundFailure(void);

public:
	KuPathBlockPlannerPr();
	~KuPathBlockPlannerPr();
};

#endif 

