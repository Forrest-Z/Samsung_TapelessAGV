#ifndef AGV_COMM_SUPERVISOR_H
#define AGV_COMM_SUPERVISOR_H

#include "ZigbeeComm/ZigbeeComm.h"
#include "GPIOComm/GPIOComm.h"
#include "../KUNSUtil/KUNSSingletone/KuSingletone.h"
#include "../KUNSGUI/KuDrawingInfo.h"
#include <iostream>
#include "../MultiRobotSupervisor/Clientpart.h"

class CAGVCommSupervisor : public KuSingletone <CAGVCommSupervisor>
{
public:
	/* Variables */
	enum AGV_STATE {ST_NONE, ST_WAIT, ST_CHECK, ST_ENTRY, ST_GETOUT, ST_WAITENTRY};

	/* Functions */
	CAGVCommSupervisor();
	~CAGVCommSupervisor();
	bool initZigbeeComm(void);
	bool initGPIOComm(void);
	bool checkAbnormalState(int nPathBlockID, double dCurrentVel); // 문제 발생 여부 판단
	void sendArrivalSignalAtStartingPoint(bool send); // 시작 지점에서 신호를 보냄
	void initAllRelays(void); // 모든 relay off
	void setEntryState(bool bState);
	void setCheckState(void);
	bool isSafeToPass(void);
	void enableZigbeeComm(bool bEnable);
	void enableGPIOComm(bool bEnable);
	bool isZigbeeConnected(void);

private:
	/* Variables */
	int m_nCurrentBlockCnt;
	int m_nPathBlockIDPrev;
	int m_nSelectRelayPosNum;
	//bool m_bEntryState; // 위험지역 진입 여부
	bool m_bAbnormalState;
	bool m_bEnableZigbeeComm;
	bool m_bEnableGPIOComm;
	bool m_bZigbeeConnected;
	vector<KuPose> m_vecKuPose;

	/* Functions */
	void loadRelayPoint();
	void findRelayPosNum();
	void sendAGVState(AGV_STATE state); // Zigbee comm.
	AGV_STATE receiveOtherAGVState(void); // Zigbee comm.
};

#endif