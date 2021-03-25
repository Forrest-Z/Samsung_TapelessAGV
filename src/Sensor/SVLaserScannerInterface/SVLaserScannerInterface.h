#pragma once

#include "../../KUNSUtil/KUNSSocketComm/KUNSSocketComm.h"
#include "./SVLaserScannerSerialComm.h"
#include "../../KUNSUtil/KUNSSingletone/KuSingletone.h"
#include "../../KUNSUtil/KUNSThread/KuThread.h"
#include "../../KUNSUtil/KuUtil.h"
#include <vector>

#define SV_USE_ETHERNET

#define SV_PACKET_SIZE 369 // 362 (data) + 7 (head, length, sum, and tail) bytes
#define SV_DATA_NUM 181 // 181 laser data
#define SV_TCP_BUFFER_SIZE_MAX 20480

using namespace std;

class CSVLaserScannerInterface : public KuSingletone <CSVLaserScannerInterface>
{
public:
	/* Variables */

	/* Functions */
	CSVLaserScannerInterface();
	~CSVLaserScannerInterface();
	bool connect(string strPort);
	bool disconnect(void);
	bool execute_serial(void);
	bool execute_ethernet(void);
	int_1DArray get_data( );

private:
	/* Variables */
	KuUtil m_KuUtil;
	SVLaserScannerSerialComm m_KuSerialComm;
	KUNSSocketComm m_sockClient;
	int m_nPort;
	bool m_bConnected;
	bool m_bInitialized;
	bool m_bGetDataFlag;
	KuThread m_KuThread;
	unsigned char* m_pchLaserScanData;
	vector<double> m_LaserScanData;
	int_1DArray m_LRFRangeData_181;// 181개 레이저 데이터

	/* Functions */
	static void do_thread(void* arg);
	void send_run_command_serial(void);
	bool send_run_command_ethernet(void);
	void send_stop_command_serial(void);
	bool send_stop_command_ethernet(void);
	bool set_angle_resolution_ethernet(void);
	void receive_data_serial(void);
	bool receive_data_ethernet(void);
};