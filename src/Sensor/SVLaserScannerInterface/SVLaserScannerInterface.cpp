#include "stdafx.h"
#include "../../ANSCommon/ANSCommon.h"
#include "../../MobileSupervisor/KuRobotParameter.h"
#include "SVLaserScannerInterface.h"

CSVLaserScannerInterface::CSVLaserScannerInterface()
	: m_nPort(0)
	, m_bConnected(false)
	, m_bInitialized(false)
	, m_bGetDataFlag(false)
	, m_pchLaserScanData(0)
{
	m_pchLaserScanData = new unsigned char [SV_DATA_NUM * 2]; // 2 bytes for each ray

	m_LaserScanData.reserve(SV_DATA_NUM); // 1024 data

	m_LRFRangeData_181=	 m_KuUtil.generateIntType1DArray(SV_DATA_NUM ,0);
}

CSVLaserScannerInterface::~CSVLaserScannerInterface()
{
	if(m_pchLaserScanData)
	{
		delete [] m_pchLaserScanData;
	}

	m_LaserScanData.clear();
}

bool CSVLaserScannerInterface::connect(string strPort)
{
#ifdef SV_USE_ETHERNET

	bool bSocketConnection(false);

	bSocketConnection = m_sockClient.initClient((char*)KuRobotParameter::getInstance()->getSVLaserIP().c_str(), KuRobotParameter::getInstance()->getSVLaserPort());

	if(bSocketConnection)
	{
		execute_ethernet();

		m_bConnected = true;
	}
	else
	{
		printf("[SV Laser Scanner] Connection lost\n");
		ANS_LOG_WRITE("[SV Laser Scanner] Connection lost");
	}

#else

	m_nPort = atoi(&strPort.c_str()[3]);
	m_KuSerialComm.SetComport(m_nPort, 230400, 8, '1', 0);		//port, baudrate, databit, stopbit, paritybit
	m_KuSerialComm.CreateCommInfo();
	m_bConnected = m_KuSerialComm.OpenComport();
	m_KuSerialComm.m_bStartFlag = m_bConnected;

	execute_serial();

#endif

	return m_bConnected;
}

bool CSVLaserScannerInterface::disconnect(void)
{
#ifdef SV_USE_ETHERNET

	send_stop_command_ethernet();

	m_sockClient.close();

#else

	send_stop_command_serial();

	m_KuThread.terminate();
	m_KuSerialComm.CloseConnection();

#endif

	return true;
}

bool CSVLaserScannerInterface::execute_ethernet(void)
{
/*
	if(set_angle_resolution_ethernet())
	{
		m_bInitialized = true;

		return false;
	}
*/

	if(!send_run_command_ethernet())
	{
		m_bInitialized = false;

		return false;
	}

	m_KuThread.start(do_thread, this, 50, "CSVLaserScannerInterface::execute_ethernet()");

	m_bInitialized = true;

	return true;
}

bool CSVLaserScannerInterface::execute_serial(void)
{
	if(m_bConnected)
	{
		m_bGetDataFlag=true;
		m_bInitialized=true;
		send_run_command_serial(); // 데이터 전송 시작 명령(LRF -> KUNS)
		m_KuThread.start(do_thread, this, 50, "CSVLaserScannerInterface::execute_serial()");

		return true;
	}

	return false;
}

void CSVLaserScannerInterface::do_thread(void* arg)
{
	CSVLaserScannerInterface* pSV = (CSVLaserScannerInterface*)arg;

	if(pSV->m_bInitialized==true)
	{
#ifdef SV_USE_ETHERNET

		pSV->receive_data_ethernet();

#else

		pSV->receive_data_serial(); // 수신

#endif
	}
}

bool CSVLaserScannerInterface::send_run_command_ethernet(void)
{
	stringstream ssData;
	bool bRes(false);

	// Send
	ssData << (char)0x55 // stx
		<< (char)0xF2 // run device
		<< (char)0x00 << (char)0x01 // length
		<< (char)0x00
		<< (char)((0xF2 + 0x01 + 0x00) & 0xFF) // checksum
		<< (char)0x55; // etx

	m_sockClient.sendData((void*)ssData.str().c_str(), ssData.str().length());

// 	Sleep(100);

	// Receive
	char cReceiveData[8];

	int nRecvSize = m_sockClient.receiveData(&cReceiveData, 8);

	if(nRecvSize > 0)
	{
		if(cReceiveData[0] == 0x55 &&cReceiveData[6] == 0x55)
		{
			if(cReceiveData[4] == 0x01)
			{
				bRes = true;
			}
		}
	}

	return bRes;
}

void CSVLaserScannerInterface::send_run_command_serial(void)
{
	char chSendData[8]; // 8 byte
	int nSum(0);

	// Header
	chSendData[0] = 0x02;
	chSendData[1] = 0x02;

	// Data
	chSendData[2] = 0x01;
	chSendData[3] = 0x01;

	// Dummy
	chSendData[4] = 0x00;
	chSendData[5] = 0x00;

	// Tail
	chSendData[6] = 0x03;
	chSendData[7] = 0x03;

	m_KuSerialComm.sendData(chSendData, 8); // Send
}

bool CSVLaserScannerInterface::send_stop_command_ethernet(void)
{
	stringstream ssData;
	bool bRes(false);

	// Send
	ssData << (char)0x55 // stx
		<< (char)0xF4 // stop device
		<< (char)0x00 << (char)0x01 // length
		<< (char)0x00
		<< (char)((0xF4 + 0x01 + 0x00) & 0xFF) // checksum
		<< (char)0x55; // etx

	m_sockClient.sendData((void*)ssData.str().c_str(), ssData.str().length());

	// 	Sleep(100);

	// Receive
	char cReceiveData[8];

	int nRecvSize = m_sockClient.receiveData(&cReceiveData, 8);

	if(nRecvSize > 0)
	{
		if(cReceiveData[0] == 0x55 &&cReceiveData[6] == 0x55)
		{
			if(cReceiveData[4] == 0x01)
			{
				bRes = true;
			}
		}
	}

	return bRes;
}

bool CSVLaserScannerInterface::set_angle_resolution_ethernet(void)
{
	stringstream ssData;
	bool bRes(false);

	// Send
	ssData << (char)0x55 // stx
		<< (char)0xF6 // set angle resolution
		<< (char)0x00 << (char)0x01 // length
		<< (char)0x01 // 1 deg step
		<< (char)((0xF6 + 0x01 + 0x01) & 0xFF) // checksum
		<< (char)0x55; // etx

	m_sockClient.sendData((void*)ssData.str().c_str(), ssData.str().length());

	// 	Sleep(100);

	// Receive
	char cReceiveData[8];

	int nRecvSize = m_sockClient.receiveData(&cReceiveData, 8);

	if(nRecvSize > 0)
	{
		if(cReceiveData[0] == 0x55 &&cReceiveData[6] == 0x55)
		{
			if(cReceiveData[4] == 0x01)
			{
				bRes = true;
			}
		}
	}

	return bRes;
}

void CSVLaserScannerInterface::send_stop_command_serial(void)
{
	char chSendData[8]; // 8 byte
	int nSum(0);

	// Header
	chSendData[0] = 0x02;
	chSendData[1] = 0x02;

	// Data
	chSendData[2] = 0x02;
	chSendData[3] = 0x01;

	// Dummy
	chSendData[4] = 0x00;
	chSendData[5] = 0x00;

	// Tail
	chSendData[6] = 0x03;
	chSendData[7] = 0x03;

	m_KuSerialComm.sendData(chSendData, 8); // Send
}

bool CSVLaserScannerInterface::receive_data_ethernet(void)
{
	if(m_bInitialized)
	{
		stringstream ssData;
		bool bRes(false);

		// Send
		ssData << (char)0x55 // stx
			<< (char)0xF3 // get data
			<< (char)0x00 << (char)0x01 // length
			<< (char)0x00
			<< (char)((0xF3 + 0x01 + 0x00) & 0xFF) // checksum
			<< (char)0x55; // etx

		m_sockClient.sendData((void*)ssData.str().c_str(), ssData.str().length());

		// Receive
		char chBuffer[SV_TCP_BUFFER_SIZE_MAX];
		int nRecvSize(0);

		nRecvSize = m_sockClient.receiveData((void*)chBuffer, SV_TCP_BUFFER_SIZE_MAX);

		if(nRecvSize > 0)
		{
			if(chBuffer[0] == (char)0x55 && chBuffer[1] == (char)0xF3) // STX
			{
				int i(1);
				int nScanCnt(0); // number of scan data
				int nDist(0); // mm
				const int nAmount((int)(((chBuffer[3] << 8) | chBuffer[2]) & 0xFFFF));

				while(i < SV_TCP_BUFFER_SIZE_MAX - 1)
				{
						nDist = (int)((chBuffer[4 + 2 * nScanCnt + 1] << 8) | chBuffer[4 + 2 * nScanCnt]); // mm

						int nIdx = SV_DATA_NUM - nScanCnt - 1;

						if(nIdx >= 0 && nIdx < SV_DATA_NUM)
						{
							m_LRFRangeData_181[nIdx] = nDist; // mm
						}

						nScanCnt++;

						if(nScanCnt >= SV_DATA_NUM)
						{
							return true;
						}
				}
			}
		}
	}
}

void CSVLaserScannerInterface::receive_data_serial(void)
{
	int i;
	unsigned char chReceiveData[SV_MAXBLOCK + 1];
	const int nLength = m_KuSerialComm.readData(chReceiveData, SV_MAXBLOCK);
	int nLaserScanData;
// 	int nSelBit = -1;
// 	bool bReceiveData(false);

	for(i = 0; i < nLength - SV_PACKET_SIZE + 1; i++)
	{
		if(chReceiveData[i] == 0x02 && chReceiveData[i + 1] == 0x02 &&
// 			(int)((chReceiveData[i + 2] << 8) | chReceiveData[i + 3]) == 2048) // Header and data length
			(int)(chReceiveData[i + 2] | chReceiveData[i + 3] << 8) == SV_DATA_NUM * 2) // 362 bytes
		{
//			if(chReceiveData[i + SV_PACKET_SIZE - 2] == 0x03 && chReceiveData[i + SV_PACKET_SIZE - 1] == 0x03) // Tail
			{
				// Receive data
				memcpy(m_pchLaserScanData, &chReceiveData[i + 4], SV_DATA_NUM * 2); // 181 * 2 = 362 bytes
			}
		}
	}

	for(i = 0; i < SV_DATA_NUM * 2; i += 2) // 2 bytes step
	{
		nLaserScanData = (int)((m_pchLaserScanData[i + 1]  << 8) | m_pchLaserScanData[i]); // 16 bit (2 bytes) for each scan data, cm

		if(nLaserScanData < 50) nLaserScanData = 0; // lower than 50 cm (unstable data) -> 0 cm

		m_LRFRangeData_181[SV_DATA_NUM - i / 2 - 1] = nLaserScanData * 10; // mm
	}
}

int_1DArray CSVLaserScannerInterface::get_data()
{
	return m_LRFRangeData_181;
}
