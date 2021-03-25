#ifndef CLIENT_PART_H
#define CLIENT_PART_H

#pragma once
//#include "stdio.h"
/*#include <string.h>*/
#include "../KUNSUtil/KUNSSocketComm/KUNSSocketComm.h"
#include "../KUNSMath/KuMath.h"
#include "../KUNSPose/KuPose.h"
#include "../KUNSUtil/KUNSSingletone/KuSingletone.h"
#include "../KUNSUtil/KUNSCriticalSection/KuCriticalSection.h"
#include "../KUNSUtil/KUNSSmartPointer/KuSmartPointer.h"
#include "../KUNSUtil/KUNSThread/KuThread.h"
#include "../KUNSGUI/KuDrawingInfo.h"
#include "../MobileSupervisor/KuRobotParameter.h"
#include "../MultiRobotSupervisor/MultiTcpipInterface.h"
#include "../MultiRobotSupervisor/XmldataSetting.h"

using namespace std;

class Clientpart : public KuSingletone <Clientpart> , public KuThread
{
public:

	static const int STATE_RUN = 1;
	static const int STATE_STOP = 2;
	static const int STATE_ALARM = 3;
	static const int STATE_PROGRAM_ON = 4;
	static const int STATE_PROGRAM_OFF = 5;
	static const int STATE_PM = 6;
	static const int STATE_UNKNOWN = 7;

private:
	KuThread m_Thread;
	bool m_sendflag;

	//스레드 관련 변수----------------------
	bool m_bIsThreadFuncGenerated;
	bool m_doThreadFunc;
	int m_nThreadFuncPeriod;
	//--------------------------------------

private:
	KuPose m_RobotPos[AGV_NUMBER];
	int m_nData;
	int m_nTransVel[AGV_NUMBER];
	int m_nRotVel[AGV_NUMBER];
	int m_nID[AGV_NUMBER];
	int m_nPrevBlockId;
	int m_nPrevState;
	int m_nCurrentState;
	int m_nCompleteCount;
	int m_nPrevCompleteCount;
	bool m_bJobComplete;
	bool m_bJobPause;
	char *m_temp;//demand data 
	char m_cSendDatatemp[500];
	bool m_bJobEnableReq;
	bool m_bJobListReq;

public:
	void terminate();

public:
	void setXmlmessage();
	void SetState(int nState);
	void SetCompleteCount(int nCnt);
	void setJobComplete(void);
	void setJobPause(bool bSet);
	bool getJobPause(void);
	void setJobEnableReq(bool bReq, int nJobNum = 0);
	bool getJobEnableReq(void);
	void setJobListReq(bool bReq);
	bool getJobListReq(void);

	//RSP_AGV
	bool ReqState();
	bool RspState();
	bool RspPara();
	bool RspMove();
	bool RspRotate();
	bool RspSound();
	bool RspDevice();
	bool RspTowerlamp();
	bool RspSensor();
	//RSP_AGV

	//RSP_JOB
	bool RspStart(string sJobCurrentInfo);
	bool RspStop(string sJobCurrentInfo);
	bool RspPause(string sJobCurrentInfo);
	bool RspCAPause(string sJobCurrentInfo);
	bool RspResume(string sJobCurrentInfo);
	bool RspCAResume(string sJobCurrentInfo);
	bool ReqComplete(string sJobCurrentInfo);
	bool RspChange(string sJobCurrentInfo, string sJobInfo);
	bool ReqCyclefinish(string sJobCurrentInfo);
	bool ReqOpercall();
	//RSP_JOB

	//RSP_Other
	bool RspPathdownload();
	bool RspMapdownload();
	bool RspFiledownload();

	//RSP_Other

	//REQ_AGV
	bool ReqLocation();
	bool ReqLive();
	bool ReqSensor();
	bool ReqJobEnable();
	bool ReqJobList();
	//REQ_AGV

	//양방향 xml data setting
	void SetXMLDuplex(string str_name);
	void DuplexClientSend();

	// IoT
	bool sendDataToIoTModule(void);

public:
	Clientpart();
	~Clientpart();
};
#endif