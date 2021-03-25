#ifndef TOTAL_TCP_IP_COMMUNICATION_H
#define TOTAL_TCP_IP_COMMUNICATION_H

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
#include "MultiTcpipInterface.h"
#include "XmldataSetting.h"
#include "Clientpart.h"
#include "Serverpart.h"

using namespace std;

class TotalTcpipCommunication : public KuSingletone <TotalTcpipCommunication> , public KuThread
{
public:
	typedef struct _tagResponseData
	{
		string request_message;
		string job_id;
	} ResponseData;
	string m_strsendXml;
	CCriticalSection m_CriticalSection;

private:
	bool m_bClient;
	bool m_bServer;
	bool m_bISConnected;
	bool m_bTcpConnected;
	bool m_bISSACConnected;
	bool m_bIoTModuleConnected;
	KuThread m_ThreadServer;
	KuThread m_ThreadClient;
	bool m_sendflag;
	//스레드 관련 변수----------------------
	bool m_bIsThreadFuncGenerated;
	bool m_doThreadFunc;
	int m_nThreadFuncPeriod;
	//--------------------------------------

	bool m_bsendflag;
	bool m_bserverflag;
	bool m_bclientflag;
	bool m_bcontclientflag;

private:
	KuPose m_RobotPos[AGV_NUMBER];
	int m_nData;
	int m_nTransVel[AGV_NUMBER];
	int m_nRotVel[AGV_NUMBER];
	int m_nID[AGV_NUMBER];
	int m_nSendCnt;
	char *m_temp;//demand data 
	char m_cSendDatatemp[500];
	vector<ResponseData> m_vecResponseList;

public:
	void terminate();

//client/server thread
private:
	static void ThreadServer(void* arg);
	static void ThreadClient(void* arg);
	static void ThreadClientStatus(void* arg);
	void sendResponse(vector<ResponseData>& vecResponseList);
public:
	bool startServer(int nPeriod);
	bool startClient(int nPeriod);
//client/server thread

public:
	bool clientseq(ResponseData& rsp_data);//client
	bool serverseq();//server
	void Client();// Live 신호를 보냄
	bool isISSACConnected(void);
	bool isIoTModuleConnected(void);

public:
	TotalTcpipCommunication();
	~TotalTcpipCommunication();
};
#endif