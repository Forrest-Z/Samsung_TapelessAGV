#include "stdafx.h"
#include "GPIOComm.h"

CGPIOComm::CGPIOComm()
	: m_nPort(0)
	, m_bConnected(false)
	, m_bInitialized(false)
	, m_bGetDataFlag(false)
{

}

CGPIOComm::~CGPIOComm()
{

}

bool CGPIOComm::connect(string strPort)
{
	m_nPort = atoi(&strPort.c_str()[3]);
	m_KuSerialComm.SetComport(m_nPort, 9600, 8, '1', 0);		//port, baudrate, databit, stopbit, paritybit
	m_KuSerialComm.CreateCommInfo();
	m_bConnected = m_KuSerialComm.OpenComport();
	m_KuSerialComm.m_bStartFlag = m_bConnected;

	return m_bConnected;
}

bool CGPIOComm::disconnect(void)
{
	m_KuThread.terminate();
	m_KuSerialComm.CloseConnection();

	m_bConnected = false;

	return true;
}

bool CGPIOComm::execute()
{
	if(m_bConnected)
	{
		m_bGetDataFlag=true;
		m_bInitialized=true;
		m_KuThread.start(doThread,this,100, "CGPIOComm::execute()");

		return true;
	}

	return false;
}

void CGPIOComm::doThread(void* arg)
{
	CGPIOComm* pZC = (CGPIOComm*)arg;

	if(pZC->m_bInitialized==true)
	{
	}
}

void CGPIOComm::sendData(char* pchData)
{
	if(m_bConnected)
	{
		m_KuSerialComm.sendData(pchData,GPIO_PACKET_SIZE); // Send
	}
}

void CGPIOComm::receiveData(void)
{
/*
	int i;
	unsigned char chReceiveData[MAXBLOCK + 1];
	const int nLength = m_KuSerialComm.readData(chReceiveData, MAXBLOCK);
	int nSelBit = -1;
	int nSum(0);
	unsigned char chDataReceiveID[2];
	unsigned char chDataSendID[2];
	unsigned char chDataCommand;
	unsigned char chDataStation[2];
	unsigned char chDataSubNumber[2];

	for(i = 0; i < nLength - ZIGBEE_PACKET_SIZE + 1; i++)
	{
		if(chReceiveData[i] == 0x02 && // STX
			chReceiveData[i + ZIGBEE_PACKET_SIZE - 1] == 0x03) // ETX
		{
			nSelBit = i; // position of the packet
			break;
		}
	}

	if(nSelBit != -1)
	{
		// Checksum
		for(i = 1; i < ZIGBEE_PACKET_SIZE - 2; i++)
		{
			nSum += chReceiveData[nSelBit + i];
		}

		if((unsigned char)nSum == (chReceiveData[nSelBit + 10] & 0xf))
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
		}
	}
*/
}
