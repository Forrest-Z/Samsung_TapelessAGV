#include "stdafx.h"
#include "ZigbeeComm.h"

CZigbeeComm::CZigbeeComm()
: m_nPort(0)
, m_bConnected(false)
, m_bInitialized(false)
, m_bGetDataFlag(false)
, m_nOtherAGVState(0)
{
	m_sSendDataPrev = "000000000";
}

CZigbeeComm::~CZigbeeComm()
{

}

bool CZigbeeComm::connect(string strPort)
{
	m_nPort = atoi(&strPort.c_str()[3]);
	m_KuSerialComm.SetComport(m_nPort, 9600, 8, '1', 0);		//port, baudrate, databit, stopbit, paritybit
	m_KuSerialComm.CreateCommInfo();
	m_bConnected = m_KuSerialComm.OpenComport();
	m_KuSerialComm.m_bStartFlag = m_bConnected;

	return m_bConnected;
}

bool CZigbeeComm::disconnect(void)
{
	m_KuThread.terminate();
	m_KuSerialComm.CloseConnection();

	m_bConnected = false;

	return true;
}

bool CZigbeeComm::execute()
{
	if(m_bConnected)
	{
		m_bGetDataFlag=true;
		m_bInitialized=true;
		m_KuThread.start(doThread,this, 100, "CZigbeeComm::execute()"); // Thread ���ο����� sleep�� ��

		return true;
	}

	return false;
}

void CZigbeeComm::doThread(void* arg)
{
	CZigbeeComm* pZC = (CZigbeeComm*)arg;

	if(pZC->m_bInitialized==true)
	{
//		pZC->sendData(pZC->getSendData(), '0', '0', '5'); // 5�� �浹����

		Sleep(1000);
/*
		char chSendDataPrev = pZC->m_sSendDataPrev.c_str()[5];
		char chSendData = pZC->getSendData()[5];

		if(chSendDataPrev != chSendData)
		{
			pZC->sendData(pZC->getSendData(), '0', '0', '8'); // 8�� �浹����

			pZC->m_sSendDataPrev = pZC->getSendData();
		}
*/
		Sleep(1300); // Sleep �� ���� 2��~3�ʰ� �Ǿ�� ��(SGEC)

		pZC->receiveData(); // ����
	}
}

char* CZigbeeComm::getSendData(void)
{
	return m_chSendData;
}

void CZigbeeComm::sendData(char* pchData, char chAreaChar1, char chAreaChar2, char chAreaChar3)
{
	if(m_bConnected)
	{
		char chSendData[ZIGBEE_PACKET_SIZE]; // 12 byte
		int nSum(0);
		char chCheckSum[sizeof(int)];

		// STX
		chSendData[0] = 0x02;

		// Data
		memcpy(&chSendData[1], pchData, ZIGBEE_PACKET_SIZE - 3); // copy 9 bytes
		chSendData[7] = chAreaChar1; // �浹���� ù��° ����
		chSendData[8] = chAreaChar2; // �浹���� �ι�° ����
		chSendData[9] = chAreaChar3; // �浹���� ����° ����

		// Checksum
		for(int i=1; i < ZIGBEE_PACKET_SIZE - 2; i++)
		{
			nSum += chSendData[i];
		}
	
		sprintf(chCheckSum, "%x", (char)(nSum & 0x0f)); // 16������ ��ȯ
		chSendData[10] = chCheckSum[0];

		// ETX
		chSendData[11] = 0x03;

		m_KuSerialComm.sendData(chSendData,ZIGBEE_PACKET_SIZE); // Send

		/**/
		printf("[Zigbee : Send] ");
		for(int k = 0; k < 12; k++)
		{
			printf("%c ", chSendData[k]);
		}
		printf("\n");
		/**/
	}
}

void CZigbeeComm::receiveData(void)
{
	int i, j;
	unsigned char chReceiveData[MAXBLOCK + 1];
	const int nLength = m_KuSerialComm.readData(chReceiveData, MAXBLOCK);
	int nSelBit = -1;
	int nSum(0);
	int nOtherAGVState(0);
	char chCheckSum[sizeof(int)];
	unsigned char chDataReceiveID[2];
	unsigned char chDataSendID[2];
	unsigned char chDataCommand;
	unsigned char chDataStation[2];
	unsigned char chDataSubNumber[2];

	for(i = 0; i < nLength - ZIGBEE_PACKET_SIZE + 1;)
	{
		if(chReceiveData[i] == 0x02 && // STX
			chReceiveData[i + ZIGBEE_PACKET_SIZE - 1] == 0x03) // ETX
		{
			nSelBit = i; // starting position of the packet

			// for testing
			/**/
			if(
				chReceiveData[nSelBit + 5] == 'T' /*&& // �浹����
				chReceiveData[nSelBit + 9] != '5' &&
				chReceiveData[nSelBit + 9] != '6'*/
				)
			{
				printf("[Zigbee : Recv] ");
				for(int k = 0; k < ZIGBEE_PACKET_SIZE; k++)
				{
					printf("%c ", chReceiveData[nSelBit + k]);
				}
				printf("\n");
			}
			/**/

			// Checksum
			for(j = 1; j < ZIGBEE_PACKET_SIZE - 2; j++)
			{
				nSum += chReceiveData[nSelBit + j];
			}

			sprintf(chCheckSum, "%x", (char)(nSum & 0x0f)); // 16������ ��ȯ

			if(1)//(unsigned char)(chCheckSum[0]) == chReceiveData[nSelBit + 10])
			{
				// Copy data
				chDataReceiveID[0] = chReceiveData[nSelBit + 1];
				chDataReceiveID[1] = chReceiveData[nSelBit + 2];
				chDataSendID[0] = chReceiveData[nSelBit + 3];
				chDataSendID[1] = chReceiveData[nSelBit + 4];
				chDataCommand = chReceiveData[nSelBit + 5];
				chDataStation[0] = chReceiveData[nSelBit + 6];
				chDataStation[1] = chReceiveData[nSelBit + 7];
				chDataSubNumber[0] = chReceiveData[nSelBit + 8];
				chDataSubNumber[1] = chReceiveData[nSelBit + 9];

				//m_nOtherAGVState = (int)((chDataStation[0] << 8) | chDataStation[1]);
				if(
				//	(chDataStation[1] == '0' && chDataSubNumber[0] == '0' && chDataSubNumber[1] == '5') ||
				//	(chDataStation[1] == '0' && chDataSubNumber[0] == '0' && chDataSubNumber[1] == '6') ||
					(chDataStation[1] == '0' && chDataSubNumber[0] == '0' && chDataSubNumber[1] == '8')) // 5��(Comp.), 6��(Comp.), 8��(Cond.) �浹������ ���
				{
					switch(chDataStation[0]) // �ٸ� AGV�� ���� Ȯ��
					{
					case '0': nOtherAGVState = 0; break;
					case '1': nOtherAGVState = 1; break;
					case '2': nOtherAGVState = 2; break;
					case '3': 
						if(nOtherAGVState != 6)
						{
							nOtherAGVState = 3;
						}
						break;
					case '4':
						if(nOtherAGVState != 3 && nOtherAGVState != 6)
						{
							nOtherAGVState = 4;
						}
						break;
					case '5': nOtherAGVState = 5; break;
					case '6': nOtherAGVState = 6; break;
					}
					/*
					if(chDataStation[0] == '3')
					{
						nOtherAGVState = 3; // �ٸ� AGV�� ���� Ȯ��
					}
					if(chDataStation[0] == '4')
					{
						nOtherAGVState = 4; // �ٸ� AGV�� ���� Ȯ��
					}
					*/
					//printf("other agv state1 = %d\n", nOtherAGVState);
				}

				//printf("Zigbee receive: valid checksum\n");

				m_nOtherAGVState = nOtherAGVState;
				
				i = i + ZIGBEE_PACKET_SIZE; // packet size ��ŭ index�� ���� ��

				continue;
			}
			else
			{
				printf("[CZigbeeComm] Checksum error\n");
			}
		}

		i++; // �������� packet�� �ƴ� ��� index�� 1 �������� ��
	}

//	printf("[CZigbeeComm] nOtherAGVState = %d\n", nOtherAGVState);

//	m_nOtherAGVState = nOtherAGVState;
}

int CZigbeeComm::getOtherAGVState(void)
{
	return m_nOtherAGVState;
}

void CZigbeeComm::setSendData(char* pchData)
{
	memcpy(m_chSendData, pchData, ZIGBEE_PACKET_SIZE - 3); // copy 9 bytes
}