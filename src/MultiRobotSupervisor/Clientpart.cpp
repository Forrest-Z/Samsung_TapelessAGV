#include "stdafx.h"
#include "Clientpart.h"
#include "Serverpart.h"
#include "../KUNSGUI/KuDrawingInfo.h"
#include "../KUNSPRIMUSComm/KuPRIMUSCommSupervisor.h"

Clientpart::Clientpart()
{
	m_bIsThreadFuncGenerated=false;
	m_sendflag=false;

	m_nPrevBlockId = -1;
	m_nPrevState = -1;
	m_nCurrentState = STATE_PROGRAM_ON;
	m_nCompleteCount = 0;
	m_nPrevCompleteCount = 0;
	m_bJobComplete = false;
	m_bJobPause = false;
	m_bJobEnableReq = false;
	m_bJobListReq = false;

	AllocConsole(); //콘솔 창을 만들어주는 함수.....
	freopen( "CONOUT$", "wt", stdout);
}

Clientpart::~Clientpart()
{
}

void Clientpart::setJobComplete(void)
{
	m_bJobComplete = true;
}

void Clientpart::setJobPause(bool bSet) // Job change 중일 때 기존에 수행 중이던 job을 마쳤을 경우 job_pause를 송신하기 위하여 flag 셋팅
{
	m_bJobPause = bSet;
}

bool Clientpart::getJobPause(void)
{
	return m_bJobPause;
}

bool Clientpart::RspPara()
{
	bool bSend(false);
	string strSendXml = "";

	string strAGVNumFinal = "000";
	char c[4];
	int nAGVID = KuRobotParameter::getInstance()->getRobotID();
	itoa(nAGVID, c, 10); // 10진수
	int nLength = strlen(c);
	strAGVNumFinal.replace(3 - nLength, nLength, c);
	
	strSendXml.clear();

	int nagvid=3;
	string agvid="";
	strSendXml += "0000000000";
	strSendXml += "<message>";
	strSendXml += "<header>";
	strSendXml += "<transid>0000001</transid>";
	strSendXml += "<name>RSP_AGV_PARA</name>";
	strSendXml += "</header>";
	strSendXml += "<body>";
	strSendXml += "<agvid>" + strAGVNumFinal + "</agvid>";
	strSendXml += "<map>";
	strSendXml += "</map>";
	strSendXml += "<path>";
	strSendXml += "</path>";
	strSendXml += "<robot>";
	strSendXml += "</robot>";
	strSendXml += "<sensor>";
	strSendXml += "</sensor>";
	strSendXml += "<kanayama>";
	strSendXml += "</kanayama>";
	strSendXml += "<particlefilter>";
	strSendXml += "</particlefilter>";
	strSendXml += "<sift>";
	strSendXml += "</sift>";
	strSendXml += "</body>";	
	strSendXml += "<return>";
	strSendXml += "<returncode>0</returncode>";
	strSendXml += "<returnmessage></returnmessage>";
	strSendXml += "</return>";
	strSendXml += "</message>";

	int length=strSendXml.size()-10;
	char buf[10];
	sprintf(buf, "%d", length);
	strSendXml.replace( 7,3, buf);
	bSend = MultiTcpipInterface::getInstance()->sendDataISSAC(strSendXml);

	return bSend;
}

bool Clientpart::RspMove()
{
	bool bSend(false);
	string strSendXml = "";
	string strAGVNumFinal = "000";
	char c[4];
	int nAGVID = KuRobotParameter::getInstance()->getRobotID();
	itoa(nAGVID, c, 10); // 10진수
	int nLength = strlen(c);
	strAGVNumFinal.replace(3 - nLength, nLength, c);
	
	strSendXml.clear();

	strSendXml += "0000000000";
	strSendXml += "<message>";
	strSendXml += "<header>";
	strSendXml += "<transid>0000001</transid>";
	strSendXml += "<name>RSP_AGV_MOVE</name>";
	strSendXml += "</header>";
	strSendXml += "<body>";
	strSendXml += "<agvid>" + strAGVNumFinal + "</agvid>";
	strSendXml += "</body>";	
	strSendXml += "<return>";
	strSendXml += "<returncode>0</returncode>";
	strSendXml += "<returnmessage></returnmessage>";
	strSendXml += "</return>";
	strSendXml += "</message>";

	int length=strSendXml.size()-10;
	char buf[10];
	sprintf(buf, "%d", length);
	strSendXml.replace( 7,3, buf);
	bSend = MultiTcpipInterface::getInstance()->sendDataISSAC(strSendXml);

	return bSend;
}

bool Clientpart::RspRotate()
{
	bool bSend(false);
	string strSendXml = "";
	string strAGVNumFinal = "000";
	char c[4];
	int nAGVID = KuRobotParameter::getInstance()->getRobotID();
	itoa(nAGVID, c, 10); // 10진수
	int nLength = strlen(c);
	strAGVNumFinal.replace(3 - nLength, nLength, c);
	
	strSendXml.clear();

	strSendXml += "0000000000";
	strSendXml += "<message>";
	strSendXml += "<header>";
	strSendXml += "<transid>0000001</transid>";
	strSendXml += "<name>RSP_AGV_ROTATE</name>";
	strSendXml += "</header>";
	strSendXml += "<body>";
	strSendXml += "<agvid>" + strAGVNumFinal + "</agvid>";
	strSendXml += "</body>";	
	strSendXml += "<return>";
	strSendXml += "<returncode>0</returncode>";
	strSendXml += "<returnmessage></returnmessage>";
	strSendXml += "</return>";
	strSendXml += "</message>";

	int length=strSendXml.size()-10;
	char buf[10];
	sprintf(buf, "%d", length);
	strSendXml.replace( 7,3, buf);
	bSend = MultiTcpipInterface::getInstance()->sendDataISSAC(strSendXml);

	return bSend;
}

bool Clientpart::RspSound()
{
	bool bSend(false);
	string strSendXml = "";
	string strAGVNumFinal = "000";
	char c[4];
	int nAGVID = KuRobotParameter::getInstance()->getRobotID();
	itoa(nAGVID, c, 10); // 10진수
	int nLength = strlen(c);
	strAGVNumFinal.replace(3 - nLength, nLength, c);
	
	strSendXml.clear();

	strSendXml += "0000000000";
	strSendXml += "<message>";
	strSendXml += "<header>";
	strSendXml += "<transid>0000001</transid>";
	strSendXml += "<name>RSP_AGV_SOUND</name>";
	strSendXml += "</header>";
	strSendXml += "<body>";
	strSendXml += "<agvid>" + strAGVNumFinal + "</agvid>";
	strSendXml += "</body>";	
	strSendXml += "<return>";
	strSendXml += "<returncode>0</returncode>";
	strSendXml += "<returnmessage></returnmessage>";
	strSendXml += "</return>";
	strSendXml += "</message>";

	int length=strSendXml.size()-10;
	char buf[10];
	sprintf(buf, "%d", length);
	strSendXml.replace( 7,3, buf);
	bSend = MultiTcpipInterface::getInstance()->sendDataISSAC(strSendXml);

	return bSend;
}

bool Clientpart::RspDevice()
{
	bool bSend(false);
	string strSendXml = "";
	string strAGVNumFinal = "000";
	char c[4];
	int nAGVID = KuRobotParameter::getInstance()->getRobotID();
	itoa(nAGVID, c, 10); // 10진수
	int nLength = strlen(c);
	strAGVNumFinal.replace(3 - nLength, nLength, c);
	
	strSendXml.clear();

	strSendXml += "0000000000";
	strSendXml += "<message>";
	strSendXml += "<header>";
	strSendXml += "<transid>0000001</transid>";
	strSendXml += "<name>RSP_AGV_DEVICE</name>";
	strSendXml += "</header>";
	strSendXml += "<body>";
	strSendXml += "<agvid>" + strAGVNumFinal + "</agvid>";
	strSendXml += "</body>";	
	strSendXml += "<return>";
	strSendXml += "<returncode>0</returncode>";
	strSendXml += "<returnmessage></returnmessage>";
	strSendXml += "</return>";
	strSendXml += "</message>";

	int length=strSendXml.size()-10;
	char buf[10];
	sprintf(buf, "%d", length);
	strSendXml.replace( 7,3, buf);
	bSend = MultiTcpipInterface::getInstance()->sendDataISSAC(strSendXml);

	return bSend;
}

bool Clientpart::RspTowerlamp()
{
	bool bSend(false);
	string strSendXml = "";
	string strAGVNumFinal = "000";
	char c[4];
	int nAGVID = KuRobotParameter::getInstance()->getRobotID();
	itoa(nAGVID, c, 10); // 10진수
	int nLength = strlen(c);
	strAGVNumFinal.replace(3 - nLength, nLength, c);
	
	strSendXml.clear();

	strSendXml += "0000000000";
	strSendXml += "<message>";
	strSendXml += "<header>";
	strSendXml += "<transid>0000001</transid>";
	strSendXml += "<name>RSP_AGV_TOWERLAMP</name>";
	strSendXml += "</header>";
	strSendXml += "<body>";
	strSendXml += "<agvid>" + strAGVNumFinal + "</agvid>";
	strSendXml += "</body>";	
	strSendXml += "<return>";
	strSendXml += "<returncode>0</returncode>";
	strSendXml += "<returnmessage></returnmessage>";
	strSendXml += "</return>";
	strSendXml += "</message>";

	int length=strSendXml.size()-10;
	char buf[10];
	sprintf(buf, "%d", length);
	strSendXml.replace( 7,3, buf);
	bSend = MultiTcpipInterface::getInstance()->sendDataISSAC(strSendXml);

	return bSend;
}

bool Clientpart::RspSensor()
{
	bool bSend(false);
	string strSendXml = "";
	string strAGVNumFinal = "000";
	char c[4];
	int nAGVID = KuRobotParameter::getInstance()->getRobotID();
	itoa(nAGVID, c, 10); // 10진수
	int nLength = strlen(c);
	strAGVNumFinal.replace(3 - nLength, nLength, c);
	
	strSendXml.clear();

	strSendXml += "0000000000";
	strSendXml += "<message>";
	strSendXml += "<header>";
	strSendXml += "<transid>0000001</transid>";
	strSendXml += "<name>RSP_AGV_SENSOR</name>";
	strSendXml += "</header>";
	strSendXml += "<body>";
	strSendXml += "<agvid>" + strAGVNumFinal + "</agvid>";
	strSendXml += "<sensor>";	
	strSendXml += "<id>1</id>";	
	strSendXml += "<value>26.4</value>";	
	strSendXml += "</sensor>";
	strSendXml += "<sensor>";	
	strSendXml += "<id>2</id>";	
	strSendXml += "<value>22</value>";	
	strSendXml += "</sensor>";
	strSendXml += "<sensor>";	
	strSendXml += "<id>3</id>";	
	strSendXml += "<value>100</value>";	
	strSendXml += "</sensor>";
	strSendXml += "<sensor>";	
	strSendXml += "<id>4</id>";	
	strSendXml += "<value>95</value>";	
	strSendXml += "</sensor>";
	strSendXml += "</body>";	
	strSendXml += "<return>";
	strSendXml += "<returncode>0</returncode>";
	strSendXml += "<returnmessage></returnmessage>";
	strSendXml += "</return>";
	strSendXml += "</message>";

	int length=strSendXml.size()-10;
	char buf[10];
	sprintf(buf, "%d", length);
	strSendXml.replace( 7,3, buf);
	bSend = MultiTcpipInterface::getInstance()->sendDataISSAC(strSendXml);

	return bSend;
}
//RSP_AGV

//RSP_JOB
bool Clientpart::RspStart(string sJobCurrentInfo)
{
	bool bSend(false);
	string strSendXml = "";
	string strAGVNumFinal = "000";
	char c[4];
	int nAGVID = KuRobotParameter::getInstance()->getRobotID();
	itoa(nAGVID, c, 10); // 10진수
	int nLength = strlen(c);
	strAGVNumFinal.replace(3 - nLength, nLength, c);
	
	strSendXml.clear();

	strSendXml += "0000000000";
	strSendXml += "<message>";
	strSendXml += "<header>";
	strSendXml += "<transid>0000001</transid>";
	strSendXml += "<name>RSP_JOB_START</name>";
	strSendXml += "</header>";
	strSendXml += "<body>";
	strSendXml += "<agvid>" + strAGVNumFinal + "</agvid>";
	strSendXml += "<jobid>"+sJobCurrentInfo+"</jobid>";
	strSendXml += "</body>";	
	strSendXml += "<return>";
	strSendXml += "<returncode>0</returncode>";
	strSendXml += "<returnmessage></returnmessage>";
	strSendXml += "</return>";
	strSendXml += "</message>";

	int length=strSendXml.size()-10;
	char buf[10];
	sprintf(buf, "%d", length);
	strSendXml.replace( 7,3, buf);
	bSend = MultiTcpipInterface::getInstance()->sendDataISSAC(strSendXml);

	return bSend;
}

bool Clientpart::RspStop(string sJobCurrentInfo)
{
	bool bSend(false);
	string strSendXml = "";
	string strAGVNumFinal = "000";
	char c[4];
	int nAGVID = KuRobotParameter::getInstance()->getRobotID();
	itoa(nAGVID, c, 10); // 10진수
	int nLength = strlen(c);
	strAGVNumFinal.replace(3 - nLength, nLength, c);

	string strCompleteCount;
	strCompleteCount.clear();
	itoa(m_nCompleteCount, c, 10); // 10진수
	strCompleteCount = c;
	
	strSendXml.clear();

	strSendXml += "0000000000";
	strSendXml += "<message>";
	strSendXml += "<header>";
	strSendXml += "<transid>0000001</transid>";
	strSendXml += "<name>RSP_JOB_STOP</name>";
	strSendXml += "</header>";
	strSendXml += "<body>";
	strSendXml += "<agvid>" + strAGVNumFinal + "</agvid>";
	strSendXml += "<jobid>" + sJobCurrentInfo + "</jobid>";
	strSendXml += "<completecount>" + strCompleteCount + "</completecount>";
	strSendXml += "</body>";	
	strSendXml += "<return>";
	strSendXml += "<returncode>0</returncode>";
	strSendXml += "<returnmessage></returnmessage>";
	strSendXml += "</return>";
	strSendXml += "</message>";

	int length=strSendXml.size()-10;
	char buf[10];
	sprintf(buf, "%d", length);
	strSendXml.replace( 7,3, buf);
	bSend = MultiTcpipInterface::getInstance()->sendDataISSAC(strSendXml);

	return bSend;
}

bool Clientpart::RspPause(string sJobCurrentInfo)
{
	bool bSend(false);
	string strSendXml = "";
	string strAGVNumFinal = "000";
	char c[4];
	int nAGVID = KuRobotParameter::getInstance()->getRobotID();
	itoa(nAGVID, c, 10); // 10진수
	int nLength = strlen(c);
	strAGVNumFinal.replace(3 - nLength, nLength, c);

	string strCompleteCount;
	strCompleteCount.clear();
	itoa(m_nCompleteCount, c, 10); // 10진수
	strCompleteCount = c;
	
	strSendXml.clear();

	strSendXml += "0000000000";
	strSendXml += "<message>";
	strSendXml += "<header>";
	strSendXml += "<transid>0000001</transid>";
	strSendXml += "<name>RSP_JOB_PAUSE</name>";
	strSendXml += "</header>";
	strSendXml += "<body>";
	strSendXml += "<agvid>" + strAGVNumFinal + "</agvid>";
	strSendXml += "<jobid>" + sJobCurrentInfo + "</jobid>";
	strSendXml += "<completecount>"+strCompleteCount+"</completecount>";
	strSendXml += "</body>";	
	strSendXml += "<return>";
	strSendXml += "<returncode>0</returncode>";
	strSendXml += "<returnmessage></returnmessage>";
	strSendXml += "</return>";
	strSendXml += "</message>";

	int length=strSendXml.size()-10;
	char buf[10];
	sprintf(buf, "%d", length);
	strSendXml.replace( 7,3, buf);
	bSend = MultiTcpipInterface::getInstance()->sendDataISSAC(strSendXml);

	return bSend;
}

bool Clientpart::RspCAPause(string sJobCurrentInfo)
{
	bool bSend(false);
	string strSendXml = "";
	string strAGVNumFinal = "000";
	char c[4];
	int nAGVID = KuRobotParameter::getInstance()->getRobotID();
	itoa(nAGVID, c, 10); // 10진수
	int nLength = strlen(c);
	strAGVNumFinal.replace(3 - nLength, nLength, c);

	string strCompleteCount;
	strCompleteCount.clear();
	itoa(m_nCompleteCount, c, 10); // 10진수
	strCompleteCount = c;
	
	strSendXml.clear();

	strSendXml += "0000000000";
	strSendXml += "<message>";
	strSendXml += "<header>";
	strSendXml += "<transid>0000001</transid>";
	strSendXml += "<name>RSP_JOB_CAPAUSE</name>";
	strSendXml += "</header>";
	strSendXml += "<body>";
	strSendXml += "<agvid>" + strAGVNumFinal + "</agvid>";
	strSendXml += "<jobid>" + sJobCurrentInfo + "</jobid>";
	strSendXml += "<completecount>"+strCompleteCount+"</completecount>";
	strSendXml += "</body>";	
	strSendXml += "<return>";
	strSendXml += "<returncode>0</returncode>";
	strSendXml += "<returnmessage></returnmessage>";
	strSendXml += "</return>";
	strSendXml += "</message>";

	int length=strSendXml.size()-10;
	char buf[10];
	sprintf(buf, "%d", length);
	strSendXml.replace( 7,3, buf);
	bSend = MultiTcpipInterface::getInstance()->sendDataISSAC(strSendXml);

	return bSend;
}

bool Clientpart::RspResume(string sJobCurrentInfo)
{
	bool bSend(false);
	string strSendXml = "";
	string strAGVNumFinal = "000";
	char c[4];
	int nAGVID = KuRobotParameter::getInstance()->getRobotID();
	itoa(nAGVID, c, 10); // 10진수
	int nLength = strlen(c);
	strAGVNumFinal.replace(3 - nLength, nLength, c);

	string strCompleteCount;
	strCompleteCount.clear();
	itoa(m_nCompleteCount, c, 10); // 10진수
	strCompleteCount = c;
	
	strSendXml.clear();

	strSendXml += "0000000000";
	strSendXml += "<message>";
	strSendXml += "<header>";
	strSendXml += "<transid>0000001</transid>";
	strSendXml += "<name>RSP_JOB_RESUME</name>";
	strSendXml += "</header>";
	strSendXml += "<body>";
	strSendXml += "<agvid>" + strAGVNumFinal + "</agvid>";
	strSendXml += "<jobid>" + sJobCurrentInfo + "</jobid>";
	strSendXml += "<completecount>"+strCompleteCount+"</completecount>";
	strSendXml += "</body>";	
	strSendXml += "<return>";
	strSendXml += "<returncode>0</returncode>";
	strSendXml += "<returnmessage></returnmessage>";
	strSendXml += "</return>";
	strSendXml += "</message>";
	
	int length=strSendXml.size()-10;
	char buf[10];
	sprintf(buf, "%d", length);
	strSendXml.replace( 7,3, buf);
	bSend = MultiTcpipInterface::getInstance()->sendDataISSAC(strSendXml);

	return bSend;
}

bool Clientpart::RspCAResume(string sJobCurrentInfo)
{
	bool bSend(false);
	string strSendXml = "";
	string strAGVNumFinal = "000";
	char c[4];
	int nAGVID = KuRobotParameter::getInstance()->getRobotID();
	itoa(nAGVID, c, 10); // 10진수
	int nLength = strlen(c);
	strAGVNumFinal.replace(3 - nLength, nLength, c);

	string strCompleteCount;
	strCompleteCount.clear();
	itoa(m_nCompleteCount, c, 10); // 10진수
	strCompleteCount = c;
	
	strSendXml.clear();

	strSendXml += "0000000000";
	strSendXml += "<message>";
	strSendXml += "<header>";
	strSendXml += "<transid>0000001</transid>";
	strSendXml += "<name>RSP_JOB_CARESUME</name>";
	strSendXml += "</header>";
	strSendXml += "<body>";
	strSendXml += "<agvid>" + strAGVNumFinal + "</agvid>";
	strSendXml += "<jobid>" + sJobCurrentInfo + "</jobid>";
	strSendXml += "<completecount>"+strCompleteCount+"</completecount>";
	strSendXml += "</body>";	
	strSendXml += "<return>";
	strSendXml += "<returncode>0</returncode>";
	strSendXml += "<returnmessage></returnmessage>";
	strSendXml += "</return>";
	strSendXml += "</message>";
	
	int length=strSendXml.size()-10;
	char buf[10];
	sprintf(buf, "%d", length);
	strSendXml.replace( 7,3, buf);
	bSend = MultiTcpipInterface::getInstance()->sendDataISSAC(strSendXml);

	return bSend;
}

bool Clientpart::ReqComplete(string sJobCurrentInfo)
{
	bool bSend(false);

	if(m_bJobComplete)
	{
		string strSendXml = "";
		string strAGVNumFinal = "000";
		char c[4];
		int nAGVID = KuRobotParameter::getInstance()->getRobotID();
		itoa(nAGVID, c, 10); // 10진수
		int nLength = strlen(c);
		strAGVNumFinal.replace(3 - nLength, nLength, c);

		string strCompleteCount;
		strCompleteCount.clear();
		int completecount = m_nCompleteCount;//XmldataSetting::getInstance()->getRepeatCounter();
		itoa(completecount, c, 10); // 10진수
		strCompleteCount = c;

		strSendXml.clear();

		strSendXml += "0000000000";
		strSendXml += "<message>";
		strSendXml += "<header>";
		strSendXml += "<transid>0000001</transid>";
		strSendXml += "<name>REQ_JOB_COMPLETE</name>";
		strSendXml += "</header>";
		strSendXml += "<body>";
		strSendXml += "<agvid>" + strAGVNumFinal + "</agvid>";
		strSendXml += "<jobid>" + sJobCurrentInfo + "</jobid>";
		strSendXml += "<completecount>" + strCompleteCount + "</completecount>";
		strSendXml += "</body>";	
		strSendXml += "</message>";

		int length=strSendXml.size()-10;
		char buf[10];
		sprintf(buf, "%d", length);
		strSendXml.replace( 7,3, buf);
		bSend = MultiTcpipInterface::getInstance()->sendDataISSAC(strSendXml);

		m_bJobComplete = false; // 초기화

		XmldataSetting::getInstance()->setJobStatus(XmldataSetting::JOB_COMPLETE);
	}

	return bSend;
}

bool Clientpart::RspChange(string sJobCurrentInfo, string sJobInfo)
{
	bool bSend(false);
	string strSendXml = "";
	string strAGVNumFinal = "000";
	char c[4];
	int nAGVID = KuRobotParameter::getInstance()->getRobotID();
	itoa(nAGVID, c, 10); // 10진수
	int nLength = strlen(c);
	strAGVNumFinal.replace(3 - nLength, nLength, c);

	strSendXml += "0000000000";
	strSendXml += "<message>";
	strSendXml += "<header>";
	strSendXml += "<transid>0000001</transid>";
	strSendXml += "<name>RSP_JOB_CHANGE</name>";
	strSendXml += "</header>";
	strSendXml += "<body>";
	strSendXml += "<agvid>" + strAGVNumFinal + "</agvid>";
	strSendXml += "<jobinfo>";	
	strSendXml += "<from>"+sJobCurrentInfo+"</from>";	
	strSendXml += "<to>"+sJobInfo+"</to>";	
	strSendXml += "</jobinfo>";	
	strSendXml += "</body>";	
	strSendXml += "<return>";	
	strSendXml += "<returncode>0</returncode>";	
	strSendXml += "<returnmessage></returnmessage>";	
	strSendXml += "</return>";	
	strSendXml += "</message>";

	int length=strSendXml.size()-10;
	char buf[10];
	sprintf(buf, "%d", length);
	strSendXml.replace( 7,3, buf);
	bSend = MultiTcpipInterface::getInstance()->sendDataISSAC(strSendXml);

	return bSend;
}
//RSP_JOB
//REQ_JOB
bool Clientpart::ReqCyclefinish(string sJobCurrentInfo)
{
	bool bSend(false);

	if(m_nCompleteCount != m_nPrevCompleteCount)
	{
		string strSendXml = "";
		string strAGVNumFinal = "000";
		char c[4];
		int nAGVID = KuRobotParameter::getInstance()->getRobotID();
		itoa(nAGVID, c, 10); // 10진수
		int nLength = strlen(c);
		strAGVNumFinal.replace(3 - nLength, nLength, c);
	
		string strCompleteCount;
		strCompleteCount.clear();
		itoa(m_nCompleteCount, c, 10); // 10진수
		strCompleteCount = c;

		strSendXml.clear();

		strSendXml += "0000000000";
		strSendXml += "<message>";
		strSendXml += "<header>";
		strSendXml += "<transid>0000001</transid>";
		strSendXml += "<name>REQ_JOB_CYCLEFINISH</name>";
		strSendXml += "</header>";
		strSendXml += "<body>";
		strSendXml += "<agvid>" + strAGVNumFinal + "</agvid>";
		strSendXml += "<jobid>"+sJobCurrentInfo+"</jobid>";
		strSendXml += "<completecount>"+strCompleteCount+"</completecount>";
		strSendXml += "</body>";	
		strSendXml += "</message>";

		int length=strSendXml.size()-10;
		char buf[10];
		sprintf(buf, "%d", length);
		strSendXml.replace( 7,3, buf);
		bSend = MultiTcpipInterface::getInstance()->sendDataISSAC(strSendXml);

		m_nPrevCompleteCount = m_nCompleteCount;
	}

	return bSend;
}

bool Clientpart::ReqOpercall()
{
	bool bSend(false);
	string strSendXml = "";
	string strAGVNumFinal = "000";
	char c[4];
	int nAGVID = KuRobotParameter::getInstance()->getRobotID();
	itoa(nAGVID, c, 10); // 10진수
	int nLength = strlen(c);
	strAGVNumFinal.replace(3 - nLength, nLength, c);
	
	strSendXml.clear();

	strSendXml += "0000000000";
	strSendXml += "<message>";
	strSendXml += "<header>";
	strSendXml += "<transid>0000001</transid>";
	strSendXml += "<name>REQ_OPER_CALL</name>";
	strSendXml += "</header>";
	strSendXml += "<body>";
	strSendXml += "<operid>3</operid>";
	strSendXml += "</body>";	
	strSendXml += "<return>";
	strSendXml += "<returncode>0</returncode>";
	strSendXml += "<returnmessage></returnmessage>";
	strSendXml += "</return>";
	strSendXml += "</message>";

	int length=strSendXml.size()-10;
	char buf[10];
	sprintf(buf, "%d", length);
	strSendXml.replace( 7,3, buf);
	bSend = MultiTcpipInterface::getInstance()->sendDataISSAC(strSendXml);

	return bSend;
}
//REQ_JOB

//RSP_Other
bool Clientpart::RspPathdownload()
{
	bool bSend(false);
	string strSendXml = "";
	string strAGVNumFinal = "000";
	char c[4];
	int nAGVID = KuRobotParameter::getInstance()->getRobotID();
	itoa(nAGVID, c, 10); // 10진수
	int nLength = strlen(c);
	strAGVNumFinal.replace(3 - nLength, nLength, c);
	
	strSendXml.clear();

	strSendXml += "0000000000";
	strSendXml += "<message>";
	strSendXml += "<header>";
	strSendXml += "<transid>0000001</transid>";
	strSendXml += "<name>RSP_PATH_DOWNLOAD</name>";
	strSendXml += "</header>";
	strSendXml += "<body>";
	strSendXml += "<agvid>" + strAGVNumFinal + "</agvid>";
	strSendXml += "</body>";	
	strSendXml += "<return>";
	strSendXml += "<returncode>0</returncode>";
	strSendXml += "<returnmessage></returnmessage>";
	strSendXml += "</return>";
	strSendXml += "</message>";

	int length=strSendXml.size()-10;
	char buf[10];
	sprintf(buf, "%d", length);
	strSendXml.replace( 7,3, buf);
	bSend = MultiTcpipInterface::getInstance()->sendDataISSAC(strSendXml);

	return bSend;
}

bool Clientpart::RspMapdownload()
{
	bool bSend(false);
	string strSendXml = "";
	string strAGVNumFinal = "000";
	char c[4];
	int nAGVID = KuRobotParameter::getInstance()->getRobotID();
	itoa(nAGVID, c, 10); // 10진수
	int nLength = strlen(c);
	strAGVNumFinal.replace(3 - nLength, nLength, c);
	
	strSendXml.clear();

	strSendXml += "0000000000";
	strSendXml += "<message>";
	strSendXml += "<header>";
	strSendXml += "<transid>0000001</transid>";
	strSendXml += "<name>RSP_MAP_DOWNLOAD</name>";
	strSendXml += "</header>";
	strSendXml += "<body>";
	strSendXml += "<agvid>" + strAGVNumFinal + "</agvid>";
	strSendXml += "</body>";	
	strSendXml += "<return>";
	strSendXml += "<returncode>0</returncode>";
	strSendXml += "<returnmessage></returnmessage>";
	strSendXml += "</return>";
	strSendXml += "</message>";

	int length=strSendXml.size()-10;
	char buf[10];
	sprintf(buf, "%d", length);
	strSendXml.replace( 7,3, buf);
	bSend = MultiTcpipInterface::getInstance()->sendDataISSAC(strSendXml);

	return bSend;
}

bool Clientpart::RspFiledownload()
{
	bool bSend(false);
	string strSendXml = "";
	string strAGVNumFinal = "000";
	char c[4];
	int nAGVID = KuRobotParameter::getInstance()->getRobotID();
	itoa(nAGVID, c, 10); // 10진수
	int nLength = strlen(c);
	strAGVNumFinal.replace(3 - nLength, nLength, c);

	strSendXml.clear();

	strSendXml += "0000000000";
	strSendXml += "<message>";
	strSendXml += "<header>";
	strSendXml += "<transid>0000001</transid>";
	strSendXml += "<name>RSP_FILE_DOWNLOAD</name>";
	strSendXml += "</header>";
	strSendXml += "<body>";
	strSendXml += "<agvid>" + strAGVNumFinal + "</agvid>";
	strSendXml += "</body>";	
	strSendXml += "<return>";
	strSendXml += "<returncode>0</returncode>";
	strSendXml += "<returnmessage></returnmessage>";
	strSendXml += "</return>";
	strSendXml += "</message>";

	int length=strSendXml.size()-10;
	char buf[10];
	sprintf(buf, "%d", length);
	strSendXml.replace( 7,3, buf);
	bSend = MultiTcpipInterface::getInstance()->sendDataISSAC(strSendXml);

	return bSend;
}

//RSP_Other

//REQ_AGV
bool Clientpart::ReqLocation()
{
	bool bSend(false);

	// 다른 block에 진입한 경우에만 전송
	if(m_nPrevBlockId != KuPathBlockPlannerPr::getInstance()->getCurrentBlockIdx() &&
		KuPathBlockPlannerPr::getInstance()->getCurrentBlockIdx() >= 0)
	{
		string strSendXml = "";
		string strAGVNumFinal = "000";
		char c[4];
		int nAGVID = KuRobotParameter::getInstance()->getRobotID();
		itoa(nAGVID, c, 10); // 10진수
		int nLength = strlen(c);
		strAGVNumFinal.replace(3 - nLength, nLength, c);

		// Current block id
		string strCurrentBlockID;// = "000";
		strCurrentBlockID.clear();
		int nCurrentBlockID = KuPathBlockPlannerPr::getInstance()->getCurrentBlockIdx();

		strCurrentBlockID = (*KuPathBlockPlannerPr::getInstance()->getPathBlock())[nCurrentBlockID].block_id_for_ISSAC;

		m_nPrevBlockId = nCurrentBlockID;

		// Next block id
		string strNextBlockID;// = "000";
		strNextBlockID.clear();
		int nNextBlockID=KuPathBlockPlannerPr::getInstance()->getNextBlockIdx();
		strNextBlockID = (*KuPathBlockPlannerPr::getInstance()->getPathBlock())[nNextBlockID].block_id_for_ISSAC;

		strSendXml.clear();

		strSendXml += "0000000000";
		strSendXml += "<message>";
		strSendXml += "<header>";
		strSendXml += "<transid>0000001</transid>";
		strSendXml += "<name>REQ_AGV_LOCATION</name>";
		strSendXml += "</header>";
		strSendXml += "<body>";
		strSendXml += "<agv>";
		strSendXml += "<agvid>" + strAGVNumFinal + "</agvid>";
		strSendXml += "<pathblockid>";
		strSendXml += "<current>" + strCurrentBlockID + "</current>";
		strSendXml += "<next>" + strNextBlockID + "</next>";
		strSendXml += "</pathblockid>";
		strSendXml += "</agv>";
		strSendXml += "</body>";
		strSendXml += "</message>";

		int length=strSendXml.size()-10;
		char buf[10];
		sprintf(buf, "%d", length);
		strSendXml.replace( 7,3, buf);
		bSend = MultiTcpipInterface::getInstance()->sendDataISSAC(strSendXml);

		//cout << "current block " << strCurrentBlockID << " next block " << strNextBlockID << endl;
	}

	return bSend;
}

bool Clientpart::ReqLive()
{
	bool bSend(false);
	string strSendXml = "";
	string strAGVNumFinal = "000";
	char c[4];
	int nAGVID = KuRobotParameter::getInstance()->getRobotID();
	itoa(nAGVID, c, 10); // 10진수
	int nLength = strlen(c);
	strAGVNumFinal.replace(3 - nLength, nLength, c);

	strSendXml.clear();

	strSendXml += "0000000000";
	strSendXml += "<message>";
	strSendXml += "<header>";
	strSendXml += "<transid>0000001</transid>";
	strSendXml += "<name>REQ_AGV_LIVE</name>";
	strSendXml += "</header>";
	strSendXml += "<body>";
	strSendXml += "<agvid>" + strAGVNumFinal + "</agvid>";
	strSendXml += "</body>";
	strSendXml += "</message>";

	int length=strSendXml.size()-10;
	char buf[10];
	sprintf(buf, "%d", length);
	strSendXml.replace( 7,3, buf);
	
 	bSend = MultiTcpipInterface::getInstance()->sendDataISSAC(strSendXml); // 3DDlg thread와 충돌 문제 발생, m_strsendXml 엑세스 할 경우 발생

	return bSend;
}

bool Clientpart::ReqJobEnable()
{
	bool bSend(false);
	string sSendXml = "";
	string sAGVNumFinal = "000";
	char c[4];
	int nAGVID = KuRobotParameter::getInstance()->getRobotID();
	itoa(nAGVID, c, 10); // 10진수
	int nLength = strlen(c);
	sAGVNumFinal.replace(3 - nLength, nLength, c);

	sSendXml.clear();

	sSendXml += "0000000000";
	sSendXml += "<message>";
	sSendXml += "<header>";
	sSendXml += "<transid>0000001</transid>";
	sSendXml += "<name>REQ_JOB_ENABLE</name>";
	sSendXml += "</header>";
	sSendXml += "<body>";
	sSendXml += "<agvid>" + sAGVNumFinal + "</agvid>";
	sSendXml += "<pathid>" + XmldataSetting::getInstance()->getCurrentPathID() + "</pathid>";
	sSendXml += "</body>";
	sSendXml += "</message>";

	int length=sSendXml.size()-10;
	char buf[10];
	sprintf(buf, "%d", length);
	sSendXml.replace( 7,3, buf);

	bSend = MultiTcpipInterface::getInstance()->sendDataISSAC(sSendXml);

	return bSend;
}

bool Clientpart::ReqJobList()
{
	bool bSend(false);
	string strSendXml = "";
	string strAGVNumFinal = "000";
	char c[4];
	int nAGVID = KuRobotParameter::getInstance()->getRobotID();
	itoa(nAGVID, c, 10); // 10진수
	int nLength = strlen(c);
	strAGVNumFinal.replace(3 - nLength, nLength, c);

	strSendXml.clear();

	strSendXml += "0000000000";
	strSendXml += "<message>";
	strSendXml += "<header>";
	strSendXml += "<transid>0000001</transid>";
	strSendXml += "<name>REQ_JOB_LIST</name>";
	strSendXml += "</header>";
	strSendXml += "<body>";
	strSendXml += "<agvid>" + strAGVNumFinal + "</agvid>";
	strSendXml += "</body>";
	strSendXml += "</message>";

	int length=strSendXml.size()-10;
	char buf[10];
	sprintf(buf, "%d", length);
	strSendXml.replace( 7,3, buf);

	bSend = MultiTcpipInterface::getInstance()->sendDataISSAC(strSendXml);

	return bSend;
}

void Clientpart::SetState(int nState)
{
	m_nCurrentState = nState;
}

void Clientpart::SetCompleteCount(int nCnt)
{
	m_nCompleteCount = nCnt;

	XmldataSetting::getInstance()->setCompleteCount(nCnt);
}

bool Clientpart::ReqState()
{
	bool bSend(false);

//	if(m_nPrevState != m_nCurrentState) // state가 변경되었을 경우
	{
		string strSendXml = "";
		string strAGVNumFinal = "000";
		char c[4];
		int nAGVID = KuRobotParameter::getInstance()->getRobotID();
		itoa(nAGVID, c, 10); // 10진수
		int nLength = strlen(c);
		strAGVNumFinal.replace(3 - nLength, nLength, c);

		//state 정보 수정필요
		string strAGVState = "";
		itoa(m_nCurrentState, c, 10); // 10진수
// 		int nStateLength = strlen(c);
		strAGVState = c;//.replace(3 - nStateLength, nStateLength, c);
		//state 정보 수정필요

		strSendXml.clear();

		strSendXml += "0000000000";
		strSendXml += "<message>";
		strSendXml += "<header>";
		strSendXml += "<transid>0000001</transid>";
		strSendXml += "<name>REQ_AGV_STATE</name>";
		strSendXml += "</header>";
		strSendXml += "<body>";
		strSendXml += "<agvid>" + strAGVNumFinal + "</agvid>";
		strSendXml += "<state>"+strAGVState+"</state>";
		strSendXml += "</body>";
		strSendXml += "</message>";

		int length=strSendXml.size()-10;
		char buf[10];
		sprintf(buf, "%d", length);
		strSendXml.replace( 7,3, buf);
		bSend = MultiTcpipInterface::getInstance()->sendDataISSAC(strSendXml);

		m_nPrevState = m_nCurrentState;
	}

	return bSend;
}

bool Clientpart::RspState()//전체적으로 수정필요
{
	bool bSend(false);
	string strSendXml = "";
	string strAGVNumFinal = "000";
	char c[4];
	int nAGVID = KuRobotParameter::getInstance()->getRobotID();
	itoa(nAGVID, c, 10); // 10진수
	int nLength = strlen(c);
	strAGVNumFinal.replace(3 - nLength, nLength, c);

	//state 정보 수정필요
	string strAGVState = "000";
	int nAGVState = KuRobotParameter::getInstance()->getRobotID();
	itoa(nAGVState, c, 10); // 10진수
	int nStateLength = strlen(c);
	strAGVState.replace(3 - nStateLength, nStateLength, c);
	//state 정보 수정필요

	strSendXml.clear();

	strSendXml += "0000000000";
	strSendXml += "<message>";
	strSendXml += "<header>";
	strSendXml += "<transid>0000001</transid>";
	strSendXml += "<name>REQ_AGV_STATE</name>";
	strSendXml += "</header>";
	strSendXml += "<body>";
	strSendXml += "<agvid>" + strAGVNumFinal + "</agvid>";
	strSendXml += "<state>"+strAGVState+"</state>";
	strSendXml += "</body>";
	strSendXml += "<return>";
	strSendXml += "<returncode>0</returncode>";
	strSendXml += "<returnmessage></returnmessage>";
	strSendXml += "</return>";
	strSendXml += "</message>";

	int length=strSendXml.size()-10;
	char buf[10];
	sprintf(buf, "%d", length);
	strSendXml.replace( 7,3, buf);
	bSend = MultiTcpipInterface::getInstance()->sendDataISSAC(strSendXml);

	return bSend;
}//전체적으로 수정필요

bool Clientpart::ReqSensor()
{
	bool bSend(false);
	string strSendXml = "";
	string strAGVNumFinal = "000";
	char c[4];
	int nAGVID = KuRobotParameter::getInstance()->getRobotID();
	itoa(nAGVID, c, 10); // 10진수
	int nLength = strlen(c);
	strAGVNumFinal.replace(3 - nLength, nLength, c);

	//sensor정보 수정필요
	//sensor정보 수정필요

	strSendXml.clear();

	strSendXml += "0000000000";
	strSendXml += "<message>";
	strSendXml += "<header>";
	strSendXml += "<transid>0000001</transid>";
	strSendXml += "<name>REQ_AGV_SENSOR</name>";
	strSendXml += "</header>";
	strSendXml += "<body>";
	strSendXml += "<agvid>" + strAGVNumFinal + "</agvid>";
	strSendXml += "<sensor>";
	strSendXml += "<id>4</id>";
	strSendXml += "<value>26.4</value>";
	strSendXml += "</sensor>";	
	strSendXml += "<sensor>";
	strSendXml += "<id>5</id>";
	strSendXml += "<value>22</value>";
	strSendXml += "</sensor>";
	strSendXml += "</body>";
	strSendXml += "</message>";

	int length=strSendXml.size()-10;
	char buf[10];
	sprintf(buf, "%d", length);
	strSendXml.replace( 7,3, buf);
	bSend = MultiTcpipInterface::getInstance()->sendDataISSAC(strSendXml);

	return bSend;
}
//REQ_AGV

void Clientpart::setJobEnableReq(bool bReq, int nJobNum)
{
	if(bReq == false)
	{
		m_bJobEnableReq = bReq;
		return;
	}
	else if(nJobNum < XmldataSetting::getInstance()->getJobList().size())
	{
		XmldataSetting::getInstance()->selectJob(XmldataSetting::getInstance()->getJobList()[nJobNum].job_id);
		m_bJobEnableReq = bReq;
	}
}

bool Clientpart::getJobEnableReq(void)
{
	return m_bJobEnableReq;
}

void Clientpart::setJobListReq(bool bReq)
{
	m_bJobListReq = bReq;
}

bool Clientpart::getJobListReq(void)
{
	return m_bJobListReq;
}

bool Clientpart::sendDataToIoTModule(void)
{
	bool bSend(false);
	string strSendXml = "";
	
	// Current time
	char cTime[15];
	time_t now;
	time(&now); // Seconds since 1970

	struct tm timeCurrent;
	localtime_s(&timeCurrent, &now);

	strftime(cTime, sizeof(cTime), "%Y%m%d%H%M%S", &timeCurrent); // %Y, %m, %d, %I, %M, %S

	string sTime = cTime;
	stringstream ssRobotPosX;
	stringstream ssRobotPosY;
	stringstream ssBatteryVoltage;
	string sState;

	ssBatteryVoltage.precision(1);
	ssBatteryVoltage.setf(std::ios::fixed);

	ssRobotPosX << (int)(KuDrawingInfo::getInstance()->getRobotPos().getXm() * 100); // cm
	ssRobotPosY << (int)(KuDrawingInfo::getInstance()->getRobotPos().getYm() * 100); // cm
	ssBatteryVoltage << KuPRIMUSCommSupervisor::getInstance()->getExternalBatteryVoltage(); // V

	if(KuPathBlockPlannerPr::getInstance()->isRobotOnWaypoint())
	{
		sState = "Stop";
	}
	else
	{
		sState = "Run";
	}

	strSendXml.clear();

	strSendXml += "<Root>";
	strSendXml += "<Eqp Code = \"1\" Process = \"\">";
	strSendXml += "<Data Type = \"ERH\" Send = \"" + sTime + "\">";
	strSendXml += "<R name = \"PosX\">";
	strSendXml += "<C>" + ssRobotPosX.str() + "</C>"; // cm
	strSendXml += "</R>";
	strSendXml += "<R name = \"PosY\">";
	strSendXml += "<C>" + ssRobotPosY.str() + "</C>"; // cm
	strSendXml += "</R>";
	strSendXml += "<R name = \"BattRemain\">";
	strSendXml += "<C>" + ssBatteryVoltage.str() + "</C>"; // Voltage
	strSendXml += "</R>";
	strSendXml += "<R name = \"State\">"; // run or stop
	strSendXml += "<C>" + sState + "</C>";
	strSendXml += "</R>";
	strSendXml += "<R name = \"ErrMsg\">";
	strSendXml += "<C></C>";
	strSendXml += "</R>";
	strSendXml += "</Data>";
	strSendXml += "</Eqp>";
	strSendXml += "</Root>";

	bSend = MultiTcpipInterface::getInstance()->sendDataIoT(strSendXml);

	strSendXml.clear();

	return bSend;
}