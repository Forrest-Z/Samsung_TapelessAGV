#include "stdafx.h"
#include "SwitchInterface.h"


SwitchInterface::SwitchInterface()
{
	m_bISConnected = false;
	m_nState=0;
	m_bgetDataflag=false;
	m_nObsState = m_KuUtil.generateIntType1DArray(Sensor::SONAR_NUM,0); //거리센서 정보를 저장하는 변수 초기화

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
			if(pSI->checkState())
			{
				Sleep(50);
				pSI->checkObstacle();
			}
		}
	}
	else
	{
		if(pSI->checkState())
		{
			Sleep(50);
			pSI->checkObstacle();
		}
	}


	
}

bool SwitchInterface::execute()
{
	if(m_bISConnected)
	{
		m_bgetDataflag=true;
		m_binitflag=true;
		m_KuThread.start(doThread,this,500);
	}

	return m_binitflag;
}

int SwitchInterface::getState()
{
	return  m_nState;
}

bool SwitchInterface::checkState()
{
	char charSendProtocol[1];//
	charSendProtocol[0] = 0x64; // d 
	m_KuSerialComm.sendData(charSendProtocol,1);// 보냄
	Sleep(20);

	unsigned char ucharReadProtocol[MAXBLOCK+1];//demand data 
	int nLength = m_KuSerialComm.readData(ucharReadProtocol,MAXBLOCK);

	char cFirst[2]={0};
	sprintf(cFirst,"%c",ucharReadProtocol[0]);

	if(cFirst[0]=='0')
	{
		printf("0!!!\n");
		m_nState=0;
		return true;
	}
	else if(cFirst[0]=='1')
	{
		printf("1!!!\n");
		m_nState=1;
		return true;
	}

	Sleep(20);

	return false;
}

void SwitchInterface::initLampflag()
{
	m_binitflag=true;
}

bool SwitchInterface::initStart()
{
	if(m_nState==2||m_nState==1)
	{
		return true;
	}

	char charSendProtocol[1];//
	charSendProtocol[0] = 0x73; // s 

	m_KuSerialComm.sendData(charSendProtocol,1);// 보냄
	Sleep(20);

	unsigned char ucharReadProtocol[MAXBLOCK+1];//demand data 
	int nLength = m_KuSerialComm.readData(ucharReadProtocol,MAXBLOCK);

	char cFirst[2]={0};
	sprintf(cFirst,"%c",ucharReadProtocol[0]);

	if(cFirst[0]=='2')//youngho
	{
		printf("2!!!\n");
		m_nState=2;
		return true;
	}

	Sleep(20);

	return false;
}

bool SwitchInterface::checkObstacle()
{

	char charSendProtocol[1];//
	charSendProtocol[0] = 0x6F; // o 
	m_KuSerialComm.sendData(charSendProtocol,1);// 보냄
	Sleep(20);

	unsigned char ucharReadProtocol[MAXBLOCK+1];//demand data 
	int nLength = m_KuSerialComm.readData(ucharReadProtocol,MAXBLOCK);

	char cFirst[2]={0};
	char cScnd[2]={0};

	sprintf(cFirst,"%c",ucharReadProtocol[0]);
	sprintf(cScnd,"%c",ucharReadProtocol[1]);


	if(cFirst[0]=='T'&&cScnd[0]=='t')
	{
		for(int i=0; i<4;i++)
		{
			char cData[2]={0};
			sprintf(cData,"%c",ucharReadProtocol[2+i]);
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
			
		}
		return true;
	}

	Sleep(20);

	return false;
}
int_1DArray SwitchInterface::getObsData()
{

	int_1DArray nObsState;
	nObsState = m_nObsState;
	return nObsState;
}