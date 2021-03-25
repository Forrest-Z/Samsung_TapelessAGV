#ifndef SERVER_PART_H
#define SERVER_PART_H

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
#include "../MultiRobotSupervisor/tinyxml/tinyxml.h"
#include "../KUNSProcess/KUNSPathBlockPr/KuPathBlockPr.h"
/*#pragma comment(lib, "tinyxmld.lib")*/
#include <stdio.h>
#include <string.h>
#include <math.h>
//base64 정의
const          char MYBASE64_keyE[]={'A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P','Q','R','S','T','U','V','W','X','Y','Z','a','b','c','d','e','f','g','h','i','j','k','l','m','n','o','p','q','r','s','t','u','v','w','x','y','z','0','1','2','3','4','5','6','7','8','9','+','/'};
const unsigned char MYBASE64_keyD[]={
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,62 ,0x80,0x80,0x80,63 ,
	52 ,53 ,54 ,55 ,56 ,57 ,58 ,59 ,60 ,61 ,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0  ,1  ,2  ,3  ,4  ,5  ,6  ,7  ,8  ,9  ,10 ,11 ,12 ,13 ,14 ,
	15 ,16 ,17 ,18 ,19 ,20 ,21 ,22 ,23 ,24 ,25 ,0x80,0x80,0x80,0x80,0x80,
	0x80,26 ,27 ,28 ,29 ,30 ,31 ,32 ,33 ,34 ,35 ,36 ,37 ,38 ,39 ,40 ,
	41 ,42 ,43 ,44 ,45 ,46 ,47 ,48 ,49 ,50 ,51 ,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80
};
//base64 정의

using namespace std;

class Serverpart : public KuSingletone <Serverpart> , public KuThread
{
public:
	string m_strsendXml;

private:
	KuThread m_Thread;
	bool m_sendflag;
	KuPathBlockPr m_KuPathBlockPr;
	//스레드 관련 변수----------------------
	bool m_bIsThreadFuncGenerated;
	bool m_doThreadFunc;
	int m_nThreadFuncPeriod;
	//--------------------------------------

	bool m_bsendflag;
	string m_strName;
	string m_strJobCurrentID;
	string m_strJobNewID;
private:
	KuPose m_RobotPos[AGV_NUMBER];
	int m_nData;
	int m_nTransVel[AGV_NUMBER];
	int m_nRotVel[AGV_NUMBER];
	int m_nID[AGV_NUMBER];
	char *m_temp;//demand data 
	char m_cSendDatatemp[500];
	int m_nJobChangeApproval; // -1: 응답 없음, 0: 주행 불가, 1: 주행 가능
	bool m_bJobReceivedOnce;
	bool m_bJobListReceivedOnce;

public:
	void terminate();

public:
	void setXmlmessage();

	//serverpart
	void XmlRspMessage(string& response_message, string& job_id);//rsp message name을 출력하는 함수
	void XmlDataParsing(string str_Name, string& job_id);//tinyxml써서 parsing하는 함수
	void XmlMapParsing(string str_Name);//tinyxml + base64 decoding 써서 parsing하는 함수
	void XmlPathParsing(TiXmlDocument* pDoc);
	bool Base64_Encode(unsigned char *data, int data_length, char *encodedStr);//64base encode
	bool Base64_Decode(char *encodedStr, int encodedStr_length, unsigned char *decodedStr, int *decodedStr_length);//64base decode
	void set_dataName(string str_name);
	string get_dataName();
	void savePathBlock();
	//serverpart
	PBlock PBlockdist(float nblocksize_x, float nblocksize_y, int nblocktype);//input=blocksize, blocktype
	void set_JOBCurrentName(char* pchJOBname);
	string get_JOBCurrentName();
	void set_JOBNextName(char* pchJOBname);
	string get_JOBNextName();
	int getJobChangeApproval(void);
	bool isJobListReceivedOnce(void);

public:
	Serverpart();
	~Serverpart();
};
#endif