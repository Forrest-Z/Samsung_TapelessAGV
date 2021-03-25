	/*______________________________________________________________________________________________
	PROJECT : Intelligent Robotics Lab. Department of Mecanical Engineering Korea University                                    
	(c) Copyright 2007 KUME Intelligent Robotics Lab.                                             
	All rights reserved                                                                         
	$Directed by Jae-Bok Song                                                                     
	$Designed by Joong-Tae Park                                                                  
	$Description : samsung agv의 속도 및 엔코도등을 제어하는 기능을 제공하는 클래스
	$Data: 2007/09                                                                           
	$Author: Joong-Tae Park                                                                      
	______________________________________________________________________________________________*/

#ifndef SWITCH_INTERFACE_H
#define SWITCH_INTERFACE_H

#include <limits>
#include "../../KUNSUtil/KUNSSerialComm/KuSerialComm.h"
#include "../../KUNSMath/KuMath.h"
#include "../../KUNSPose/KuPose.h"
#include "../../KUNSUtil/KUNSSingletone/KuSingletone.h"
#include "../../KUNSUtil/KUNSCriticalSection/KuCriticalSection.h"
#include "../../KUNSUtil/KUNSSmartPointer/KuSmartPointer.h"
#include "../../KUNSUtil/KUNSThread/KuThread.h"
#include "../../KUNSUtil/KuUtil.h"
#include "../Sensor.h"

	using namespace std;
class SwitchInterface : public KuSingletone <SwitchInterface>
{

	static const int RECV_SWITCH_PROTOCOL_SIZE = 5;//8;//kebi

private:
	KuSerialComm m_KuSerialComm;
	KuUtil m_KuUtil;
	bool m_bISConnected;
	int m_nPort;
	KuThread m_KuThread;
	int m_nJobNum;
	bool m_bgetDataflag;
	bool m_binitflag;

	bool m_b_s_Checkflag;
	bool m_b_d_Checkflag;
	bool m_b_c_Checkflag;

	int_1DArray m_nObsState;
	int m_nButtonState;

private:
	static void doThread(void* arg);
	bool checkState();

public:
	bool connect(string strSerialPort);
	bool disConnect();
	int getState();
	void setState(int State);
	int getJobNum(void);
	bool initStart();
	bool execute();
	void initLampflag();
	void offLampflag();
	int_1DArray getObsData();

	SwitchInterface();
	~SwitchInterface();
};
#endif