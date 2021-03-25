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
	sendAGVState(ST_NONE); // �⺻ ����
	m_bAbnormalState=false;//AGV�̻���� �ʱ�ȭ
	loadRelayPoint();//Relay signal�� �� ��ġ �ҷ�����
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
	m_bAbnormalState=false;//AGV�̻���� �ʱ�ȭ

	char  chGPIOCom[10];

	KuRobotParameter::getInstance()->getGPIOComport(chGPIOCom);

	if(CGPIOComm::getInstance()->connect(chGPIOCom))
	{
		m_bEnableGPIOComm = true;

		CGPIOComm::getInstance()->execute();

		Sleep(100);

		initAllRelays(); // ��� relay off

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

	chState[6] = '0'; // CZigbeeComm Ŭ������ sendData() �Լ����� ���� �־���
	chState[7] = '0'; // CZigbeeComm Ŭ������ sendData() �Լ����� ���� �־���
	chState[8] = '0'; // CZigbeeComm Ŭ������ sendData() �Լ����� ���� �־���

	CZigbeeComm::getInstance()->setSendData(chState);
}

void CAGVCommSupervisor::setEntryState(bool bState) // �浹���� ���� ����
{
	if(bState == false) // ���� ���°� �ƴ�
	{
		sendAGVState(ST_GETOUT);
	}
	else // ���� ����
	{
		if(isSafeToPass())
		{
			sendAGVState(ST_ENTRY);
		}
	}
}

void CAGVCommSupervisor::setCheckState(void) // �浹���� ���� ����
{
	sendAGVState(ST_CHECK);
}

bool CAGVCommSupervisor::isSafeToPass(void)//�˵� AGV�� ���¸� �޾ƿ��� �Լ�
{
	int nOtherAGVState(CZigbeeComm::getInstance()->getOtherAGVState());

	// SGEC Cond. AGV�� �������� ��(6) �ݿ�
	if(nOtherAGVState == 6)
	{
		return true; // ���� ����
	}

	if(nOtherAGVState == 1 || // �浹���� ���� �� ���
		nOtherAGVState == 2 || // �浹���� ���� ���� check ����
		nOtherAGVState == 3 || // �浹���� ���� ����
		nOtherAGVState == 5) // �浹���� ���� �� ���
	{
		return false; // ���� �Ұ�
	}

	return true; // ���� ����
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

	if(m_nCurrentBlockCnt > 200/*value : n*/)//Ư�� ��� ������ nȸ(n*100ms) ���ö� 
	{
		bAbnormal = true;

		// Abnormal state : relay on
		findRelayPosNum();
		if(!(m_nSelectRelayPosNum<0))
		{
			m_bAbnormalState = true;

			char chData[20]; //20byte ���� (1byte�� 20ȸ)
			memset(chData,0,sizeof(chData));
			sprintf(chData,"relay on %d\r",m_nSelectRelayPosNum);

			for(int i=0; i<5/*value : k*/; i++)//������ ���� ��츦 ����Ͽ� k�� send
			{
				for(int j=0; j<strlen(chData); j++)
				{
					CGPIOComm::getInstance()->sendData(chData+j); // Send
				}
			}
		}
	}
	else if(m_bAbnormalState)//Abnormal state --> normal state �� �ٲ��� : relay off
	{
		m_bAbnormalState = false;

		char chData[20]; //20byte ���� (1byte�� 20ȸ)
		memset(chData,0,sizeof(chData));
		sprintf(chData,"relay off %d\r",m_nSelectRelayPosNum);

		for(int i=0; i<5/*value : k*/; i++)//������ ���� ��츦 ����Ͽ� k�� send
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

// AGV�� ���� ������ �������� �� relay 2������ ��ȣ�� ����
void CAGVCommSupervisor::sendArrivalSignalAtStartingPoint(bool bSend)
{
	if(bSend)
	{
		char chData[20]; //20byte ���� (1byte�� 20ȸ)
		memset(chData,0,sizeof(chData));
		sprintf(chData,"relay on %d\r", 2); // 2�� relay�� ����

		for(int i=0; i<5/*value : k*/; i++)//������ ���� ��츦 ����Ͽ� k�� send
		{
			for(int j=0; j<strlen(chData); j++)
			{
				CGPIOComm::getInstance()->sendData(chData+j); // Send
			}
		}
	}
	else
	{
		char chData[20]; //20byte ���� (1byte�� 20ȸ)
		memset(chData,0,sizeof(chData));
		sprintf(chData,"relay off %d\r", 2); // 2�� relay�� ����

		for(int i=0; i<5/*value : k*/; i++)//������ ���� ��츦 ����Ͽ� k�� send
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
	char chData[20]; //20byte ���� (1byte�� 20ȸ)
	memset(chData,0,sizeof(chData));

	// 0�� relay off
	sprintf(chData,"relay off %d\r", 0); // 0�� relay�� ����

	for(int i=0; i<5/*value : k*/; i++)//������ ���� ��츦 ����Ͽ� k�� send
	{
		for(int j=0; j<strlen(chData); j++)
		{
			CGPIOComm::getInstance()->sendData(chData+j); // Send
		}
	}

	// 1�� relay off
	sprintf(chData,"relay off %d\r", 1); // 0�� relay�� ����

	for(int i=0; i<5/*value : k*/; i++)//������ ���� ��츦 ����Ͽ� k�� send
	{
		for(int j=0; j<strlen(chData); j++)
		{
			CGPIOComm::getInstance()->sendData(chData+j); // Send
		}
	}

	// 2�� relay off
	sprintf(chData,"relay off %d\r", 2); // 0�� relay�� ����

	for(int i=0; i<5/*value : k*/; i++)//������ ���� ��츦 ����Ͽ� k�� send
	{
		for(int j=0; j<strlen(chData); j++)
		{
			CGPIOComm::getInstance()->sendData(chData+j); // Send
		}
	}

	// 3�� relay off
	sprintf(chData,"relay off %d\r", 3); // 0�� relay�� ����

	for(int i=0; i<5/*value : k*/; i++)//������ ���� ��츦 ����Ͽ� k�� send
	{
		for(int j=0; j<strlen(chData); j++)
		{
			CGPIOComm::getInstance()->sendData(chData+j); // Send
		}
	}
}

void CAGVCommSupervisor::findRelayPosNum()//����� �溸���� relay number�� ã�� �Լ�
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

void CAGVCommSupervisor::loadRelayPoint()//data/path/RelayPoint�� �����ص� �溸�� ���� ��ǥ�� �ҷ����� �Լ�
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

// Zigbee ����� ����/����
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

// GPIO ����� ����/����
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
