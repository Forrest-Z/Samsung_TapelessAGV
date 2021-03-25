#ifndef C_SICK_LMS100_INTERFACE_H
#define C_SICK_LMS100_INTERFACE_H

#include <iostream>

#include <winsock2.h>
#pragma comment(lib, "ws2_32")

#include "./SickLMS100/LMS100Comm.h"
#include "./SickLMS100/LMSManager.h"


#include "../../KUNSUtil/KUNSThread/KuThread.h"
#include "../../KUNSUtil/KUNSSingletone/KuSingletone.h"
#include "../../KUNSUtil/KUNSCriticalSection/KuCriticalSection.h"
#include "../../KUNSUtil/KUNSSmartPointer/KuSmartPointer.h"
#include "../../KUNSUtil/KuUtil.h"
#include "../../KUNSMath/KuMath.h"
#include "../Sensor.h"

using namespace std;
class SickLMS100Interface
{

private:
	bool m_bIntialized;
	bool m_isSockEstablised;
	SOCKET m_socketFrontSick;
	KuUtil m_KuUtil;
	CLMSManager m_lms_manager;

private:
	int_1DArray m_LRFRangeData_181;// 181개 레이저 데이터


public:
	bool init();
	bool connect(char * cIP, int nPort);
	int_1DArray getData( );
	void startTimeCheck(LARGE_INTEGER& nStart);
	float finishTimeCheck(const LARGE_INTEGER nStart);
	void disconnectLaserScanner();

public:

	SickLMS100Interface();
	~SickLMS100Interface();

};

#endif