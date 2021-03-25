#ifndef MULTI_ROBOT_ACTUATOR_INTERFACE_H
#define MULTI_ROBOT_ACTUATOR_INTERFACE_H

#include <limits>
#include "../KUNSUtil/KUNSSerialComm/KuSerialComm.h"
#include "../KUNSMath/KuMath.h"
#include "../KUNSPose/KuPose.h"
#include "../KUNSUtil/KUNSSingletone/KuSingletone.h"
#include "../KUNSUtil/KUNSCriticalSection/KuCriticalSection.h"
#include "../KUNSUtil/KUNSSmartPointer/KuSmartPointer.h"
#include "../KUNSUtil/KUNSThread/KuThread.h"
#include "../KUNSGUI/KuDrawingInfo.h"
#include "../MobileSupervisor/KuRobotParameter.h"
#include "../KUNSGUI/KuDrawingInfo.h"


using namespace std;

static const int AGV_NUM = 2;


class MultiRobotActuatorInterface : public KuSingletone <MultiRobotActuatorInterface> , public KuThread
{
public:
	static const int AGV_DATA_NUM =5;
	static const int DATA_X =0;
	static const int DATA_Y =1;
	static const int DATA_T =2;
	static const int DATA_VT=3;//추가
	static const int DATA_VR=4;//추가
	static const int FULL_SIZE =9;
	static const int COMMAND_SEND =103;

private:
	KuThread m_SendThread;
	KuThread m_ReadThread;
	KuSerialComm m_KuSerialComm;
private:
	bool m_bISConnected;
	KuPose m_RobotPos[AGV_NUM];
	int m_nData;
	int m_nTransVel[AGV_NUM];
	int m_nRotVel[AGV_NUM];
private:
	static void doSendThread(void* arg);
	static void doReadThread(void* arg);
	bool readData();
	void sendData();
	void setCommandMsg(int nID,int nDataType);

public:
	//-------AGV 초기화----------
	void initialize();	
	bool connect(string strSerialPort);
	void setRobotPos(KuPose RobotPos);
	KuPose* getRobotPos();

	void setTransVel(int nID,int nTransVel);
	void setRotVel(int nID,int nRotVel);
	int getTransVel(int nID);
	int getRotVel(int nID);

	MultiRobotActuatorInterface();
	~MultiRobotActuatorInterface();
};
#endif





