#ifndef GPIO_COMM_H
#define GPIO_COMM_H

#include "../../KUNSUtil/KUNSSerialComm/KuSerialComm.h"
#include "../../KUNSUtil/KUNSSingletone/KuSingletone.h"
#include "../../KUNSUtil/KUNSThread/KuThread.h"

#define GPIO_PACKET_SIZE 1 // bytes

class CGPIOComm : public KuSingletone <CGPIOComm>
{
public:
	/* Variables */

	/* Functions */
	CGPIOComm();
	~CGPIOComm();
	bool connect(string strPort);
	bool disconnect(void);
	bool execute(void);
	void sendData(char* pchData);

private:
	/* Variables */
	KuSerialComm m_KuSerialComm;
	int m_nPort;
	bool m_bConnected;
	bool m_bInitialized;
	bool m_bGetDataFlag;
	KuThread m_KuThread;

	/* Functions */
	static void doThread(void* arg);
	void receiveData(void);
};

#endif