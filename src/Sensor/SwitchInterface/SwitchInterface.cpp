#include "stdafx.h"
#include "SwitchInterface.h"


SwitchInterface::SwitchInterface()
{
	m_bISConnected = false;
	m_nButtonState=0;
	m_bgetDataflag=false;
	m_nObsState = m_KuUtil.generateIntType1DArray(Sensor::SONAR_NUM,0); //거리센서 정보를 저장하는 변수 초기화
	m_binitflag=false;
	m_nJobNum = 0; // 0 ~ 2
}

SwitchInterface::~SwitchInterface()
{

}
bool SwitchInterface::connect(string strSerialPort)
{
	m_nPort= atoi(&strSerialPort.c_str()[3]);
	m_KuSerialComm.SetComport(m_nPort, 115200, 8, '1', 0);		//port, baudrate, databit, stopbit, paritybit
	m_KuSerialComm.CreateCommInfo();
	m_bISConnected = m_KuSerialComm.OpenComport();
	m_KuSerialComm.m_bStartFlag = m_bISConnected;
	
	return m_bISConnected;
}

/**
@brief Korean: 로봇과의 연결을 끝는다.
@brief English: 
*/
bool SwitchInterface::disConnect()
{
	m_KuThread.terminate();
	m_KuSerialComm.CloseConnection();
	return true;
}

void SwitchInterface::doThread(void* arg)
{
	SwitchInterface* pSI = (SwitchInterface*)arg;

	if(pSI->m_binitflag==true)
	{   
		if(pSI->initStart())
		{
			pSI->m_binitflag=false;
		}
	}
	else
	{
		pSI->m_binitflag=false;
		pSI->checkState();
	}
}

bool SwitchInterface::execute()
{
	if(m_bISConnected)
	{
		m_bgetDataflag=true;
		//m_binitflag=true;//kebi
		m_KuThread.start(doThread,this,500, "SwitchInterface::execute()");
		return true;
	}

	return false;
}

int SwitchInterface::getState()
{
	return  m_nButtonState;
}

void SwitchInterface::setState(int State)
{
	m_nButtonState=State;
}
bool SwitchInterface::checkState()
{
	char charSendProtocol[1];//
	charSendProtocol[0] = 0x64; // d 
	m_KuSerialComm.sendData(charSendProtocol,1);// 보냄
	Sleep(10);

	unsigned char ucharReadProtocol[MAXBLOCK+1];//demand data 
	int nLength = m_KuSerialComm.readData(ucharReadProtocol,MAXBLOCK);
	int nSelBit=-1;

	//char값들을 초기화하는부분이 필요
	for(int nBit=0; nBit<nLength-RECV_SWITCH_PROTOCOL_SIZE+1; nBit++)
	{
		char cStart[10]={0};
		char cEnd[10]={0};
		char cSecond[10]={0};
		char cThird[10]={0};
		char cFourth[10]={0};

		sprintf(cStart,"%X",ucharReadProtocol[0+nBit]);
		sprintf(cEnd,"%X",ucharReadProtocol[RECV_SWITCH_PROTOCOL_SIZE+nBit-1]);

		if(cStart[0]=='2'&&cEnd[0]=='3')
		{
			nSelBit=nBit;
			break;
		}
	}

	if(nSelBit!=-1)
	{
		char cData[3]={0};
		char cData1[2]={0};
		char cData2[2]={0};
		sprintf(cData, "%c", ucharReadProtocol[nSelBit+1]);
		sprintf(cData1, "%c", ucharReadProtocol[nSelBit+2]);
		sprintf(cData2, "%c", ucharReadProtocol[nSelBit+3]);

		if(cData[0]=='1')
		{
			printf("User pressed button 1 (path %c)\n", cData2[0]);
// 			offLampflag();
			m_nButtonState = 1; // pressed
			
			switch(cData2[0])
			{
			case '0': m_nJobNum = 0; break;
			case '1': m_nJobNum = 1; break;
			case '2': m_nJobNum = 2; break;
			default: m_nJobNum = 0; break;
			}
		}
		
/*
		if(cData[0]=='1'&&cData1[0]=='0'&&cData2[0]=='0')
		{
			printf("User pressed button 1\n");
			offLampflag();
			m_nState=1;
			m_nButtonState=1;
		}
		else if(cData1[0]=='0'&&cData[0]=='0'&&cData2[0]=='0')
		{
			//printf("no input!!!\n");
			m_nButtonState=0;
		}
		else if(cData1[0]=='2'&&cData[0]=='0'&&cData2[0]=='0')
		{
			//printf("no input!!!\n");
			m_nButtonState=0;
		}
		else if(cData2[0]=='3'&&cData[0]=='0'&&cData1[0]=='0')
		{
			printf("User pressed button 3(call)\n");
			offLampflag();
			m_nState=2;		
			m_nButtonState=2;
		}
		else if(cData2[0]=='3'&&cData[0]=='0'&&cData1[0]=='2')
		{
			offLampflag();
			printf("User pressed button 3(call)\n");
			m_nState=2;		
		}
*/

		/*
		for(int i=0; i<4;i++)
		{
			sprintf(cData,"%c",ucharReadProtocol[nSelBit+3+i]);
			if(cData[0]=='S')
			{
				m_nObsState[i]=Sensor::SAFE;
			}
			else if(cData[0]=='W')
			{
				m_nObsState[i]=Sensor::WARNING;
			}
			if(cData[0]=='D')
			{
				m_nObsState[i]=Sensor::DANGER;
			}
		}*/
	}

	Sleep(20);

	return false;
}

int SwitchInterface::getJobNum(void)
{
	return m_nJobNum; // 0 ~ 2
}

void SwitchInterface::initLampflag()
{
	m_binitflag=true;
}

void SwitchInterface::offLampflag()
{
	m_binitflag=false;
}

bool SwitchInterface::initStart()
{
	char charSendProtocol[1];//
	charSendProtocol[0] = 0x73; // s 

	m_KuSerialComm.sendData(charSendProtocol,1);// 보냄
	Sleep(20);

	unsigned char ucharReadProtocol[MAXBLOCK+1];//demand data 
	int nLength = m_KuSerialComm.readData(ucharReadProtocol,MAXBLOCK);
	int nSelBit=-1;

	for(int nBit=0; nBit<nLength-RECV_SWITCH_PROTOCOL_SIZE+1; nBit++)
	{
		char cStart[10]={0};
		char cEnd[10]={0};

		sprintf(cStart,"%X",ucharReadProtocol[0+nBit]);
		sprintf(cEnd,"%X",ucharReadProtocol[RECV_SWITCH_PROTOCOL_SIZE+nBit-1]);
	
		if(cStart[0]=='2'&&cEnd[0]=='3')
		{
			nSelBit=nBit;
			break;
		}
	}

	if(nSelBit!=-1)
	{
		char cData[2]={0};
		sprintf(cData,"%c",ucharReadProtocol[nSelBit+2]);

		if(cData[0]=='2')
		{
			printf("LED ON!!!\n");
			//m_nState=5;
			return true;
		}		
	}

	Sleep(20);

	printf("Button Box disconnection!!!\n");
	m_nButtonState=0;

	return false;
}

int_1DArray SwitchInterface::getObsData()
{

	int_1DArray nObsState;
	nObsState = m_nObsState;
	return nObsState;
}