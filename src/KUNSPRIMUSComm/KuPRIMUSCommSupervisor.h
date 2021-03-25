#ifndef KUNS_PRIMUS_COMM_SUPERVISOR_H
#define KUNS_PRIMUS_COMM_SUPERVISOR_H

#include "../KUNSUtil/KUNSSingletone/KuSingletone.h"
#include "../KUNSUtil/KUNSThread/KuThread.h"
#include "../MobileSupervisor/KuRobotParameter.h"
#include "../sensor/WheelActuatorInterface/SSAGVWheelActuatorInterface.h"
#include "PRIMUSCommunication.h"
#include "CheckPC.h"
#include "afxdialogex.h"
#include "debug.h"
#include "PRIMUSVariable.h"
#include "Global.h"

using namespace std;

class KuPRIMUSCommSupervisor : public KuSingletone <KuPRIMUSCommSupervisor>
{
private:
	CPRIMUSCommunication		cPRIMUSCom;
	CCheckPC					cCheckPC;

private:
	KuThread m_checkStateThread;
	bool m_bConnection;

private:
	void setSerialDef();
	void terminateSerialComm();

public:
	void init();
	bool checkConnection();
	void getCommParam();
	void checkSBCState();
	void checkPRIMUSState();
	void sendDisconnecTime();
	void sendDoorOpen1();
	void sendDoorOpen2();

	void showCommParam();
	void showPRIMUSState();

	static void checkStateThread(void* arg);
	void startcheckStateThread();
	void terminatecheckStateThread();

	float getExternalBatteryVoltage(void);
public:
	KuPRIMUSCommSupervisor();
	~KuPRIMUSCommSupervisor();
};

#endif 
