#ifndef MULTI_TCPIP_INTERFACE_H
#define MULTI_TCPIP_INTERFACE_H

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

#define TCP_READ_BUFFER_SIZE 512 //3072
#define TCP_READ_HEADER_SIZE 10

using namespace std;

class MultiTcpipInterface : public KuSingletone <MultiTcpipInterface> , public KuThread
{
public:
	string m_strsendXml;
	string m_strreadXml;
	string m_strReaddata;
	string m_strReceiveData;
private:
	KuThread m_Thread;
	KUNSSocketComm m_sockServerISSAC;
	KUNSSocketComm m_sockClientISSAC;
	KUNSSocketComm m_sockClientIoT;
	HWND m_hWnd;
	bool m_sokectflag;
	//스레드 관련 변수----------------------
	bool m_bIsThreadFuncGenerated;
	bool m_doThreadFunc;
	int m_nThreadFuncPeriod;
	CCriticalSection m_CriticalSection;
	//--------------------------------------
	bool m_bsendflag;
	bool m_breadflag;
	// 
	bool m_bSending;

private:
	bool m_bTcpConnected;
	char m_cSendDatatemp[500];

private:
	static void Thread(void* arg);

public:
	bool start(int nPeriod);
	void terminate();

public:
	bool sendDataISSAC(string strMessage);
	bool sendDataIoT(string strMessage);
	bool bReadDataISSAC();
	void setReadData(char *c_ReceiveData);
	string* getReadData();
	bool isSending(void);

public:
	MultiTcpipInterface();
	~MultiTcpipInterface();
};
#endif