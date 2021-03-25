#ifndef XML_DATA_SETTING_H
#define XML_DATA_SETTING_H

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
#include "../KUNSProcess/KUNSPathBlockPr/KuPathBlockPlannerPr.h"
#include "../KUNSProcess/KUNSPathBlockPr/KuPathBlockPr.h"
#include "../MultiRobotSupervisor/MultiTcpipInterface.h"
#include "../MultiRobotSupervisor/tinyxml/tinyxml.h"

using namespace std;

class XmldataSetting : public KuSingletone <XmldataSetting> , public KuThread
{
public:
	/************************************************************************/
	/* 1	Ready
		2	Run
		3	Stop
		4	Reset
		5	Pause
		6	Complete
		7	CAPause
		8	CAPausing
		9	CAResuming
	/************************************************************************/
	static const int JOB_DEFAULT = 0; // for AGV only
	static const int JOB_READY = 1;
	static const int JOB_RUN = 2;
	static const int JOB_STOP = 3;
	static const int JOB_RESET = 4;
	static const int JOB_PAUSE = 5;
	static const int JOB_COMPLETE = 6;
//	static const int JOB_CAPAUSE = 7;
//	static const int JOB_CAPAUSING = 8;
//	static const int JOB_CARESUMING = 9; // 여기까지 ISSAC과 동일
	static const int JOB_RESUME = 10; // for AGV only
	static const int JOB_CHANGE = 11; // for AGV only
	static const int AGV_MODE = 0;

	static const int ISSAC_MODE = 1;//추후 ini로 뺄것
	
	typedef struct _tagJob
	{
		string job_id;
		string path_id;
		int job_status_id;
		int repeat_cnt;
		int complete_cnt;
	} Job;

private:
	string m_strsendXml;
	int m_nJobStatus;
	int m_nMode;
	string m_sPathID;
	int m_nCompleteCount;
	string m_strJobID;
	int m_nSendRepeatCounter;
	int m_nCurrentJobNum;
	int m_nPrevJobNum;
	vector<Job> m_vecJobList;
	int m_nNextJobNum;
	
private:
	KuPose m_RobotPos[AGV_NUMBER];
	vector<PBlock> m_vecPathBlock;
	char *m_temp;//demand data 
	char m_cSendDatatemp[500];

public:
	void terminate();

public:
	//set/get 함수
	void setPathBlock(vector<PBlock> vecPathBlcok);
	vector<PBlock> getPathBlock();

	void setJobStatus(int job_status);
	void setCompleteCount(int nCompCnt);
	int getJobStatus(void);
	int getCompleteCount(void);

	//set/get 함수
	bool loadISSACPath(int jobinfo, vector<PBlock>& vecPathBlock);

	void setMode(int nMode);
	int getMode();
	int getRepeatCount();
	void add_job_and_path(string sJobID, string sPathID, int nRepeatCnt, int nJobStatusID = JOB_DEFAULT, int nCompleteCnt = 0);
	string getJobID_PathID(string& sJobID);
	void setSendRepeatCounter(int nCount);
	int getSendRepeatCounter();
	string getCurrentJobID(void);
	string getNextJobID(void);
	string getPrevJobID(void);
	string getCurrentPathID(void);

	void selectNextJob(void);
	vector<XmldataSetting::Job>& getJobList(void);
	bool change_job_num(int nNum);
	void clear_job_list(void);
	bool selectJob(string sJobID);

public:
	XmldataSetting();
	~XmldataSetting();
};
#endif