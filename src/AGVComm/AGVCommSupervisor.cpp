#include "stdafx.h"
#include "AGVCommSupervisor.h"
#include "../MobileSupervisor/KuRobotParameter.h"

CAGVCommSupervisor::CAGVCommSupervisor()
: m_nCurrentBlockCnt(0)
, m_nPathBlockIDPrev(0)
, m_bEnableGPIOComm(false)
, m_bEnableZigbeeComm(false)
, m_bZigbeeConnected(false)
{
	sendAGVState(ST_NONE); // 기본 상태
	m_bAbnormalState=false;//AGV이상상태 초기화
	loadRelayPoint();//Relay signal을 줄 위치 불러오기
}

CAGVCommSupervisor::~CAGVCommSupervisor()
{
	initAllRelays();
}

bool CAGVCommSupervisor::initZigbeeComm(void)
{
	char  chZigbeeCom[10];

	KuRobotParameter::getInstance()->getZigbeeComport(chZigbeeCom);

	if(CZigbeeComm::getInstance()->connect(chZigbeeCom))
	{
		m_bEnableZigbeeComm = true;
		m_bZigbeeConnected = true;

		CZigbeeComm::getInstance()->execute();

		return true;
	}

	printf("Failed to connect Zigbee COM port.\n");

	return false;
}

bool CAGVCommSupervisor::isZigbeeConnected(void)
{
	return m_bZigbeeConnected;
}

bool CAGVCommSupervisor::initGPIOComm(void)
{
	m_bAbnormalState=false;//AGV이상상태 초기화

	char  chGPIOCom[10];

	KuRobotParameter::getInstance()->getGPIOComport(chGPIOCom);

	if(CGPIOComm::getInstance()->connect(chGPIOCom))
	{
		m_bEnableGPIOComm = true;

		CGPIOComm::getInstance()->execute();

		Sleep(100);

		initAllRelays(); // 모든 relay off

		return true;
	}

	printf("Failed to connect GPIO COM port.\n");

	return false;
}

CAGVCommSupervisor::AGV_STATE CAGVCommSupervisor::receiveOtherAGVState(void)
{
	AGV_STATE agv_state(ST_NONE);

	switch(CZigbeeComm::getInstance()->getOtherAGVState())
	{
	case 0: agv_state = ST_NONE; break;
	case 1: agv_state = ST_WAIT; break;
	case 2: agv_state = ST_CHECK; break;
	case 3: agv_state = ST_ENTRY; break;
	case 4: agv_state = ST_GETOUT; break;
	case 5: agv_state = ST_WAITENTRY; break;
	}

	return agv_state;
}

void CAGVCommSupervisor::sendAGVState(AGV_STATE state)
{
	char chState[9];

	chState[0] = 'F';
	chState[1] = 'F';
	chState[2] = '0';
	chState[3] = '1';
	chState[4] = 'T';

	switch(state)
	{
	case ST_NONE: chState[5] = '0'; break;
	case ST_WAIT: chState[5] = '1'; break;
	case ST_CHECK: chState[5] = '2'; break;
	case ST_ENTRY: chState[5] = '3'; break;
	case ST_GETOUT: chState[5] = '4'; break;
	case ST_WAITENTRY: chState[5] = '5'; break;
	}

	chState[6] = '0'; // CZigbeeComm 클래스의 sendData() 함수에서 값을 넣어줌
	chState[7] = '0'; // CZigbeeComm 클래스의 sendData() 함수에서 값을 넣어줌
	chState[8] = '0'; // CZigbeeComm 클래스의 sendData() 함수에서 값을 넣어줌

	CZigbeeComm::getInstance()->setSendData(chState);
}

void CAGVCommSupervisor::setEntryState(bool bState) // 충돌지역 진입 여부
{
	if(bState == false) // 진입 상태가 아님
	{
		sendAGVState(ST_GETOUT);
	}
	else // 진입 상태
	{
		if(isSafeToPass())
		{
			sendAGVState(ST_ENTRY);
		}
	}
}

void CAGVCommSupervisor::setCheckState(void) // 충돌지역 진입 여부
{
	sendAGVState(ST_CHECK);
}

bool CAGVCommSupervisor::isSafeToPass(void)//궤도 AGV의 상태를 받아오는 함수
{
	int nOtherAGVState(CZigbeeComm::getInstance()->getOtherAGVState());

	// SGEC Cond. AGV용 프로토콜 값(6) 반영
	if(nOtherAGVState == 6)
	{
		return true; // 진입 가능
	}

	if(nOtherAGVState == 1 || // 충돌지역 진입 전 대기
		nOtherAGVState == 2 || // 충돌지역 동시 진행 check 상태
		nOtherAGVState == 3 || // 충돌지역 진입 상태
		nOtherAGVState == 5) // 충돌지역 진입 후 대기
	{
		return false; // 진입 불가
	}

	return true; // 진입 가능
}

bool CAGVCommSupervisor::checkAbnormalState(int nPathBlockID, double dCurrentVel)
{
	bool bAbnormal(false);

	if(nPathBlockID != m_nPathBlockIDPrev)
	{
		m_nCurrentBlockCnt = 1;
	}
	else
	{
		m_nCurrentBlockCnt++;
	}

	//printf("current block id: %d\n", m_nCurrentBlockCnt);

	if(m_nCurrentBlockCnt > 200/*value : n*/)//특정 블락 정보가 n회(n*100ms) 들어올때 
	{
		bAbnormal = true;

		// Abnormal state : relay on
		findRelayPosNum();
		if(!(m_nSelectRelayPosNum<0))
		{
			m_bAbnormalState = true;

			char chData[20]; //20byte 전송 (1byte씩 20회)
			memset(chData,0,sizeof(chData));
			sprintf(chData,"relay on %d\r",m_nSelectRelayPosNum);

			for(int i=0; i<5/*value : k*/; i++)//데이터 누락 경우를 대비하여 k번 send
			{
				for(int j=0; j<strlen(chData); j++)
				{
					CGPIOComm::getInstance()->sendData(chData+j); // Send
				}
			}
		}
	}
	else if(m_bAbnormalState)//Abnormal state --> normal state 로 바뀐경우 : relay off
	{
		m_bAbnormalState = false;

		char chData[20]; //20byte 전송 (1byte씩 20회)
		memset(chData,0,sizeof(chData));
		sprintf(chData,"relay off %d\r",m_nSelectRelayPosNum);

		for(int i=0; i<5/*value : k*/; i++)//데이터 누락 경우를 대비하여 k번 send
		{
			for(int j=0; j<strlen(chData); j++)
			{
				CGPIOComm::getInstance()->sendData(chData+j); // Send
			}
		}
	}

	m_nPathBlockIDPrev = nPathBlockID;

	return bAbnormal;
}

// AGV가 시작 지점에 도착했을 때 relay 2번으로 신호를 보냄
void CAGVCommSupervisor::sendArrivalSignalAtStartingPoint(bool bSend)
{
	if(bSend)
	{
		char chData[20]; //20byte 전송 (1byte씩 20회)
		memset(chData,0,sizeof(chData));
		sprintf(chData,"relay on %d\r", 2); // 2번 relay로 전송

		for(int i=0; i<5/*value : k*/; i++)//데이터 누락 경우를 대비하여 k번 send
		{
			for(int j=0; j<strlen(chData); j++)
			{
				CGPIOComm::getInstance()->sendData(chData+j); // Send
			}
		}
	}
	else
	{
		char chData[20]; //20byte 전송 (1byte씩 20회)
		memset(chData,0,sizeof(chData));
		sprintf(chData,"relay off %d\r", 2); // 2번 relay로 전송

		for(int i=0; i<5/*value : k*/; i++)//데이터 누락 경우를 대비하여 k번 send
		{
			for(int j=0; j<strlen(chData); j++)
			{
				CGPIOComm::getInstance()->sendData(chData+j); // Send
			}
		}
	}
}

void CAGVCommSupervisor::initAllRelays(void)
{
	char chData[20]; //20byte 전송 (1byte씩 20회)
	memset(chData,0,sizeof(chData));

	// 0번 relay off
	sprintf(chData,"relay off %d\r", 0); // 0번 relay로 전송

	for(int i=0; i<5/*value : k*/; i++)//데이터 누락 경우를 대비하여 k번 send
	{
		for(int j=0; j<strlen(chData); j++)
		{
			CGPIOComm::getInstance()->sendData(chData+j); // Send
		}
	}

	// 1번 relay off
	sprintf(chData,"relay off %d\r", 1); // 0번 relay로 전송

	for(int i=0; i<5/*value : k*/; i++)//데이터 누락 경우를 대비하여 k번 send
	{
		for(int j=0; j<strlen(chData); j++)
		{
			CGPIOComm::getInstance()->sendData(chData+j); // Send
		}
	}

	// 2번 relay off
	sprintf(chData,"relay off %d\r", 2); // 0번 relay로 전송

	for(int i=0; i<5/*value : k*/; i++)//데이터 누락 경우를 대비하여 k번 send
	{
		for(int j=0; j<strlen(chData); j++)
		{
			CGPIOComm::getInstance()->sendData(chData+j); // Send
		}
	}

	// 3번 relay off
	sprintf(chData,"relay off %d\r", 3); // 0번 relay로 전송

	for(int i=0; i<5/*value : k*/; i++)//데이터 누락 경우를 대비하여 k번 send
	{
		for(int j=0; j<strlen(chData); j++)
		{
			CGPIOComm::getInstance()->sendData(chData+j); // Send
		}
	}
}

void CAGVCommSupervisor::findRelayPosNum()//가까운 경보등의 relay number를 찾는 함수
{
	KuPose RobotPose = KuDrawingInfo::getInstance()->getRobotPos();
	m_nSelectRelayPosNum = -1;
	double dMinDist = DBL_MAX;
	for(int i=0; i<m_vecKuPose.size(); i++)
	{
		double dDelX = RobotPose.getX() - m_vecKuPose[i].getX();
		double dDelY = RobotPose.getY() - m_vecKuPose[i].getY();
		double dDist = sqrt(dDelX*dDelX+dDelY*dDelY);
		if(dDist<dMinDist)
		{
			dMinDist = dDist;
			m_nSelectRelayPosNum = i;
		}
	}
}

void CAGVCommSupervisor::loadRelayPoint()//data/path/RelayPoint에 저장해둔 경보등 전역 좌표를 불러오는 함수
{
	char cFilePathName[300];
	string strDataImagPath =KuRobotParameter::getInstance()->getTeachingPathNameNPath();
	string strNewPath;
	memset(cFilePathName,0,sizeof(cFilePathName));
	sprintf(cFilePathName,"./data/path/RelayPoint.txt");
	strNewPath=cFilePathName;

	ifstream DataLog;
	DataLog.open(strNewPath);
	double dx, dy;
	KuPose RobotPose;
	m_vecKuPose.clear();
	while(!DataLog.eof()){
		KuPose PathPos;
		DataLog >> dx >> dy;
		RobotPose.setX(dx);
		RobotPose.setY(dy);
		m_vecKuPose.push_back(RobotPose);
	}
	DataLog.close();
	m_vecKuPose.pop_back();
}

// Zigbee 통신을 설정/해제
void CAGVCommSupervisor::enableZigbeeComm(bool bEnable)
{
	m_bEnableZigbeeComm = bEnable;

	if(bEnable)
	{
		initZigbeeComm();
	}
	else
	{
		CZigbeeComm::getInstance()->disconnect();
	}
}

// GPIO 통신을 설정/해제
void CAGVCommSupervisor::enableGPIOComm(bool bEnable)
{
	m_bEnableGPIOComm = bEnable;

	if(bEnable)
	{
		initGPIOComm();
	}
	else
	{
		CGPIOComm::getInstance()->disconnect();
	}
}
