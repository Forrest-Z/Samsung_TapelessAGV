#include "stdafx.h"
#include "MultiTcpipInterface.h"
#include "../ANSCommon/ANSCommon.h"

MultiTcpipInterface::MultiTcpipInterface()
{
	m_bIsThreadFuncGenerated=false;
	m_bsendflag=false;
	m_breadflag=false;
	m_bTcpConnected=false;
	m_strReaddata="";
	m_bSending = false;

	AllocConsole(); //콘솔 창을 만들어주는 함수.....
	freopen( "CONOUT$", "wt", stdout);
}

MultiTcpipInterface::~MultiTcpipInterface()
{
}

bool MultiTcpipInterface::isSending(void)
{
	return m_bSending;
}

bool MultiTcpipInterface::start(int nPeriod)
{
	if(m_bIsThreadFuncGenerated==false){
		m_doThreadFunc = true; //스레드 함수가 반복동작 할 수 있도록 플래그 설정.
		m_bIsThreadFuncGenerated = true; //스레드 함수가 생성되었다는 플래그 설정.
		m_nThreadFuncPeriod = nPeriod; //스레드 함수 실행주기 입력..
		m_bTcpConnected=true;
		DWORD dwThreadID;
		m_Thread.start(Thread,this,nPeriod, "MultiTcpipInterface::start()"); //메인 스레드 시작	
		//HANDLE  hThreadID = CreateThread(NULL,0,Thread,this,0,&dwThreadID);
	}
	return m_bTcpConnected;
}

void  MultiTcpipInterface::Thread( void* arg)
{
	MultiTcpipInterface* pMRTI = (MultiTcpipInterface*)arg;
	//  pMRTI->setXmlmessage();
/*	pMRTI->RecSendData();*/

	printf("fail_loadXml\n");
	pMRTI->terminate();
}

bool MultiTcpipInterface::sendDataISSAC(string strMessage)
{
	bool bSend(false);
	bool bSocketConnection(false);

	while(m_bSending) // 동시에 보내는 것을 방지(가끔 에러 발생함)
	{
		printf("[AGV -> ISSAC] Waiting for sending previous data.\n");
		Sleep(10);
	}

	m_bSending = true;

	//while(bSocketConnection == false)
	{
		//printf("Client initialized\n");
		bSocketConnection=m_sockClientISSAC.initClient((char*)KuRobotParameter::getInstance()->getISSACServerIP().c_str(),
																	KuRobotParameter::getInstance()->getISSACServerPort()); //엄책임님 IP:10.240.143.105 김책임님 IP:168.219.68.162

		if(bSocketConnection)
		{
			//printf("[ISSAC comm.] Connected to the server.\n");
		}
		else
		{
			printf("[AGV -> ISSAC] Connection lost\n");
			ANS_LOG_WRITE("[AGV -> ISSAC] Connection lost");
		}

		Sleep(100);
	}
	//통신연결(Client)

	//데이터전송
	while(bSocketConnection && !bSend)
	{
 		int nRes = m_sockClientISSAC.sendData((void*)strMessage.c_str(), strMessage.size());
	
// 		Sleep(100);

		if(nRes >= 0)
		{
			bSend=true;
		}
	}

	//데이터전송
	if(bSocketConnection)
	{
		m_sockClientISSAC.close();
	}
	
// 	Sleep(500);

	m_bSending = false;

	if(bSend)
	{
		ANS_LOG_WRITE("[TCP/IP Send (ISSAC)] " + strMessage);
	}

	return bSend;
}

bool MultiTcpipInterface::sendDataIoT(string strMessage)
{
	bool bSend(false);
	bool bSocketConnection(false);

	while(m_bSending) // 동시에 보내는 것을 방지(가끔 에러 발생함)
	{
		printf("[AGV -> IoT_module] Waiting for sending previous data.\n");
		Sleep(10);
	}

	m_bSending = true;

	//while(bSocketConnection == false)
	{
		//printf("Client initialized\n");
		bSocketConnection=m_sockClientIoT.initClient((char*)KuRobotParameter::getInstance()->getIoTServerIP().c_str(),
			KuRobotParameter::getInstance()->getIoTServerPort());

		if(bSocketConnection)
		{
			//printf("[IoT comm.] Connected to the server.\n");
		}
		else
		{
			printf("[AGV -> IoT_module] Connection lost\n");
			ANS_LOG_WRITE("[AGV -> IoT_module] Connection lost");
		}

		Sleep(100);
	}
	//통신연결(Client)

	//데이터전송
	while(bSocketConnection && !bSend)
	{
		int nRes = m_sockClientIoT.sendData((void*)strMessage.c_str(), strMessage.size());

		// 		Sleep(100);

		if(nRes >= 0)
		{
			bSend=true;
		}
	}

	//데이터전송
	if(bSocketConnection)
	{
		m_sockClientIoT.close();
	}

	// 	Sleep(500);

	m_bSending = false;

	if(bSend)
	{
		ANS_LOG_WRITE("[TCP/IP Send (IoT)] " + strMessage);
	}

	return bSend;
}

bool MultiTcpipInterface::bReadDataISSAC()
{
	int i;
	int nTotalLength(0);
	int nReadLength(0);
	int nRemainingLength(-1);
	int nRes(-1);
	int nReadStart(0);
	int nRecvLength;
	bool bReceived(false);
	string strRecv;
	string str_readLength="";
	char cReadDatatemp[TCP_READ_BUFFER_SIZE]; // null 문자가 들어갈 수 있도록 1 byte 늘려줌
	m_sokectflag=false;
	m_breadflag=false;

	//통신연결(Server)
	while (true)
	{
		if(m_sockServerISSAC.initServer(KuRobotParameter::getInstance()->getISSACClientPort())) // Port no.: 9001
		{
			printf("Connected to the client\n");

			memset(cReadDatatemp, 0, TCP_READ_BUFFER_SIZE);

			while(nRemainingLength == -1 || nRemainingLength > 0)
			{
				nRes = m_sockServerISSAC.receiveData(&cReadDatatemp, TCP_READ_BUFFER_SIZE - 1);
				string strRecvTemp;

				strRecvTemp.clear();

				if(nTotalLength == 0)
				{
					for(i = 0; i < TCP_READ_HEADER_SIZE; i++)
					{
						str_readLength+=cReadDatatemp[i];
					}

					nTotalLength=stoi(str_readLength);// readData의 전체 길이

					nRemainingLength = nTotalLength + TCP_READ_HEADER_SIZE;

					nReadStart = TCP_READ_HEADER_SIZE; // header를 제외하고 읽음
				}

				strRecvTemp = cReadDatatemp;

				if(nRemainingLength > nRes)
				{
					nRecvLength = nRes;
				}
				else
				{
					nRecvLength = nRemainingLength;

					bReceived = true;
				}

				nRecvLength -= nReadStart;

				strRecv += strRecvTemp.substr(nReadStart, nRecvLength);

				nRemainingLength -= nRes;

				nReadStart = 0; // 첫 번째 byte부터 읽음

				printf("[ISSAC -> AGV] Downloading.... %3d%%\r", (int)((nTotalLength - nRemainingLength) / nTotalLength * 100));

// 				Sleep(1);
			}

			printf("\n");

			setReadData((char*)strRecv.c_str()); // 받은 데이터를 저장

			if(bReceived)
			{
				m_breadflag=true;
				break;
			}
		}
		else
		{
			printf("Failed to connect\n");
			Sleep(1000);
		}
	}

	m_sockServerISSAC.close();

	if(bReceived)
	{
		stringstream ss;
		ss << "[TCP/IP Read] " << str_readLength << strRecv;
		ANS_LOG_WRITE(ss.str());
	}

//	Sleep(500);

	return m_breadflag;
}

void MultiTcpipInterface::setReadData(char *c_ReceiveData)
{
	//m_strReaddata.clear();
	m_strReceiveData=c_ReceiveData;
}

string* MultiTcpipInterface::getReadData()
{
	return &m_strReceiveData;
}

void MultiTcpipInterface::terminate()
{
	m_bIsThreadFuncGenerated = false; //스레드가 생성되지 않았다는 플래그
	m_doThreadFunc = false; //스레드 함수가 실행되도록 하는 플래그
}
