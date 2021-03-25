#ifndef ZIGBEE_COMM_H
#define ZIGBEE_COMM_H

#include "../../KUNSUtil/KUNSSerialComm/KuSerialComm.h"
#include "../../KUNSUtil/KUNSSingletone/KuSingletone.h"
#include "../../KUNSUtil/KUNSThread/KuThread.h"

#define ZIGBEE_PACKET_SIZE 12 // bytes

class CZigbeeComm : public KuSingletone <CZigbeeComm>
{
public:
	/* Variables */
	std::string m_sSendDataPrev;

	/* Functions */
	CZigbeeComm();
	~CZigbeeComm();
	bool connect(string strPort);
	bool disconnect(void);
	bool execute(void);
	void sendData(char* pchData, char chAreaChar1, char chAreaChar2, char chAreaChar3);
	int getOtherAGVState(void);
	void setSendData(char* pchData); // 전송할 데이터를 저장
	char* getSendData(void);

private:
	/* Variables */
	KuSerialComm m_KuSerialComm;
	int m_nPort;
	int m_nOtherAGVState;
	bool m_bConnected;
	bool m_bInitialized;
	bool m_bGetDataFlag;
	KuThread m_KuThread;
	char m_chSendData[ZIGBEE_PACKET_SIZE - 3]; // 다른 AGV들에게 보내는 데이터

	/* Functions */
	static void doThread(void* arg);
	void receiveData(void);
};

#endif