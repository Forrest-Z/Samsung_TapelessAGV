#ifndef MULTI_ROBOT_SUPERVISOR_H
#define MULTI_ROBOT_SUPERVISOR_H

#include <conio.h>
#include <iostream>
#include "../KUNSUtil/KuUtil.h"
#include "../KUNSUtil/KUNSTimer/KuTimer.h"
#include "../KUNSUtil/KUNSSingletone/KuSingletone.h"

#include "../KUNSMap/KuMapRepository.h"
#include "../MobileSupervisor/KuCommandMessage.h"
#include "../MobileSupervisor/KuRobotParameter.h"

#include "MultiRobotActuatorInterface.h"
#include "../KUNSProcess/KUNSZoneControlPr/KuZoneControlPr.h"

#include "../KUNSProcess/KUNSPathPlannerPr/KuGradientPathPlannerPr.h"
#include "../KUNSBehavior/GotoGoalBehavior/GotoGoalBehavior.h"

#include "TotalTcpipCommunication.h"

using namespace std;               

class MultiRobotSupervisor: public KuSingletone <MultiRobotSupervisor>
{

	static const int DEFAULTST = 0;
	static const int BREAKDOWNAGV = 1;
	static const int COLLIOSION =2;
	static const int DEADLOCKTIME = 10000;
	static const int ROBOTBOUNDARYRADIUSLENGTH = 2000;//mm �κ����� �Ÿ�
	static const int ROBOTMOVEMENTDIST = 10;////mm ���峭 �κ� �Ǵ����� �κ���ġ�� �� �ֱ�� ���ϴ� ũ��

private:
	KuGradientPathPlannerPr m_KuPathPlanner; //��ΰ�ȹ�� �ν��Ͻ�
	KuThread m_KuThread;
	KuMap *m_pPbstacleMap;;//���峭�κ��� ��ġ�� ������ ��������
	
	KuPose* m_AGVRobotPos;
	list<KuPose> m_DetourPathList;
	int m_nThreadTime;
	double m_dTotalTime;

	KuPose m_PreRobotPos2;//���峭 �κ� �Ǵ����� ���� �ֱ��� �κ���ġ ����
	bool m_bDetourPathGenerated;
	bool m_bgeneraedpath;
	bool m_bWaitAGVflag; //setWaitforDiffAGV�� Flag �������
	int m_AGVTime[AGV_NUM];
	int m_nTransVel[AGV_NUM];
	int m_nRotVel[AGV_NUM];
	bool m_bThreadFlag;

	bool m_clientFlag;
public:
	GotoGoalBehavior m_GotoGoalBeh;

public:
	bool getBehaviorStates();
	void execute(KuCommandMessage CMessage);
	Localizer* getLocalizer();
	void Start(int nTime);
	bool loadZoneMap(string strMapFilePath);
	bool connectCommunication();
	KuPose* getRobotPos();
	void updateRobotPose();
	KuPose calNextAGVPose(KuPose RobotPos, int nTvel,int nRvel,double dTime);

	void setTransVel(int nID,int nTransVel);
	void setRotVel(int nID,int nRotVel);
	int getTransVel(int nID);
	int getRotVel(int nID);

private:
	static void doThread(void* arg);
	int Precon_AGVQuantity(KuPose RobotPos1, KuPose RobotPos2,int nAGVID,int nID);//�ηκ� ���̰� ���� �Ÿ����� �ִ��� �Ǻ�
	int  DetourforAGVQuantity(KuPose RobotPos2);//�κ� ���� �����Ǻ�
	list<KuPose> generateDetourPath(KuMap * pMap, KuPose RobotPos,KuPose Breakdownrobotpos,list<KuPose> CurPathlist,bool* bcheck  );//ȸ�ǰ�λ���
	bool checkBetweenAGVPose(KuPose RobotPos1,KuPose RobotPos2, int  nTVel1,int nRVel1,int nTVel2,int nRVel2);
	bool checkDeadAGV(int nID);


public:
	MultiRobotSupervisor();
	virtual ~MultiRobotSupervisor();


};

#endif