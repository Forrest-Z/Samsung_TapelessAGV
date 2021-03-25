#include "stdafx.h"
#include "TotalTcpipCommunication.h"
#include "../ANSCommon/ANSCommon.h"

TotalTcpipCommunication::TotalTcpipCommunication()
{
	m_bIsThreadFuncGenerated=false;
	m_bsendflag=false;
	m_sendflag=false;
	m_bClient=false;
	m_bServer=false;
	m_bcontclientflag=false;
	m_bISSACConnected = false; // ISSAC alive ��ȣ �۽� ���� ���� (ISSAC ���� ����)
	m_bIoTModuleConnected = false; // IoT ��� ���� ���� ����
	m_nSendCnt = 1;
	AllocConsole(); //�ܼ� â�� ������ִ� �Լ�.....
	freopen( "CONOUT$", "wt", stdout);

	m_vecResponseList.clear();
}

TotalTcpipCommunication::~TotalTcpipCommunication()
{
	Clientpart::getInstance()->SetState(Clientpart::STATE_PROGRAM_OFF);
	Clientpart::getInstance()->ReqState();

	Sleep(500); // ���α׷� ���� state �޼����� ������ ���� ��� ���

	m_ThreadServer.terminate();
	m_ThreadClient.terminate();
}

bool TotalTcpipCommunication::startServer(int nPeriod)
{
	if(m_bIsThreadFuncGenerated==false){
		m_doThreadFunc = true; //������ �Լ��� �ݺ����� �� �� �ֵ��� �÷��� ����.
		m_bIsThreadFuncGenerated = true; //������ �Լ��� �����Ǿ��ٴ� �÷��� ����.
		m_nThreadFuncPeriod = nPeriod; //������ �Լ� �����ֱ� �Է�..
		m_bTcpConnected=true;
		DWORD dwThreadID;
		m_ThreadServer.start(ThreadServer,this,nPeriod, "TotalTcpipCommunication::startServer()"); //���� ������ ����	
		//HANDLE  hThreadID = CreateThread(NULL,0,Thread,this,0,&dwThreadID);
	}
	return m_bTcpConnected;
}

void  TotalTcpipCommunication::ThreadServer( void* arg)
{
	TotalTcpipCommunication* pMRTI = (TotalTcpipCommunication*)arg;

	pMRTI->serverseq(); // �޼��� ����
}

bool TotalTcpipCommunication::startClient(int nPeriod)
{
	m_doThreadFunc = true; //������ �Լ��� �ݺ����� �� �� �ֵ��� �÷��� ����.
	m_bIsThreadFuncGenerated = true; //������ �Լ��� �����Ǿ��ٴ� �÷��� ����.
	m_nThreadFuncPeriod = nPeriod; //������ �Լ� �����ֱ� �Է�..
	m_bTcpConnected=true;
	m_ThreadClient.start(ThreadClient, this, nPeriod, "TotalTcpipCommunication::startClient()"); //���� ������ ����

	return m_bTcpConnected;
}

void  TotalTcpipCommunication::ThreadClient(void* arg)
{
	TotalTcpipCommunication* pMRTI = (TotalTcpipCommunication*)arg;

	pMRTI->Client();
}

void TotalTcpipCommunication::Client()
{
	// IoT ////////////////////////////////////////////////////////////////////////////////
	if(m_nSendCnt % 15 == 0)
	{
		// Approx. 1 sec duration
		if(KuRobotParameter::getInstance()->getUsingIoT())
		{
			if(Clientpart::getInstance()->sendDataToIoTModule()) // ���� ������ ��� 1�� ���� �߻�
			{
				m_bIoTModuleConnected = true;

				printf("[AGV -> IoT_module] Successfully sent AGV data.\n");
			}
			else
			{
				m_bIoTModuleConnected = false;

				//printf("[AGV -> IoT_module] Connection lost\n");
			}
		}
	}

	// ISSAC ////////////////////////////////////////////////////////////////////////////

	// Live signal
	if(m_nSendCnt % 30 == 0)
	{
		// Approx. 5 sec duration
		if(Clientpart::getInstance()->ReqLive()) // ���� ������ ��� 1�� ���� �߻�
		{
			m_bISSACConnected = true;

			printf("[AGV -> ISSAC] REQ_AGV_LIVE\n");
		}
		else // Alive ��ȣ �۽ſ� ������ ���(ISSAC ������ ���� ���) -> �κ� ����
		{
			m_bISSACConnected = false;

			//printf("[AGV -> ISSAC] REQ_AGV_LIVE - Connection lost\n");
		}
	}

	if(m_bISSACConnected) // Alive ��ȣ �۽ſ� ������ ��쿡�� ����
	{
		if(m_nSendCnt % 15 == 0)
		{
			// Approx. 1 sec duration
			// AGV state
			if(Clientpart::getInstance()->ReqState())
			{
				printf("[AGV -> ISSAC] REQ_AGV_STATE\n");
			}
		}

		// Response
		sendResponse(m_vecResponseList);

		// States & AGV location
		// 100 ms duration
	//	if(Clientpart::getInstance()->ReqState())
	//	{
	//		printf("[AGV -> ISSAC] REQ_AGV_STATE\n");
	//	}

		// Job cycle finish
		string sJobID = XmldataSetting::getInstance()->getCurrentJobID();//Serverpart::getInstance()->get_JOBCurrentName();

		if(Clientpart::getInstance()->ReqCyclefinish(sJobID))
		{
			printf("[AGV -> ISSAC] REQ_JOB_CYCLEFINISH\n");
		}

		// Job complete
		if(Clientpart::getInstance()->ReqComplete(sJobID))
		{
			printf("[AGV -> ISSAC] REQ_JOB_COMPLETE\n");
		}

		// Job pause (job change ����� �������� �� ������ ���� ���̴� job�� �Ϸ���� �ʾ��� ��� job pause ��ȣ�� ����)
		if(Clientpart::getInstance()->getJobPause())
		{
			if(Clientpart::getInstance()->RspPause(sJobID))
			{
				printf("[AGV -> ISSAC] RSP_JOB_PAUSE: %s\n", sJobID);

				Clientpart::getInstance()->setJobPause(false);
			}
		}

		// AGV location
		if(1)//XmldataSetting::getInstance()->getJobStatus()==XmldataSetting::JOB_START ||
			//XmldataSetting::getInstance()->getJobStatus()==XmldataSetting::JOB_RESUME ||
			//XmldataSetting::getInstance()->getJobStatus()==XmldataSetting::JOB_STOP)
		{
			if(Clientpart::getInstance()->ReqLocation()) // current block ID + next block ID
			{
				printf("[AGV -> ISSAC] REQ_AGV_LOCATION\n");
			}
		}

		// Job change approval
		if(Clientpart::getInstance()->getJobEnableReq())
		{
			if(Clientpart::getInstance()->ReqJobEnable())
			{
				printf("[AGV -> ISSAC] REQ_JOB_ENABLE\n");

				Clientpart::getInstance()->setJobEnableReq(false);
			}
		}

		// Request job list
		if(Clientpart::getInstance()->getJobListReq())
		{
			if(Clientpart::getInstance()->ReqJobList())
			{
				printf("[AGV -> ISSAC] REQ_JOB_LIST\n");

				Clientpart::getInstance()->setJobListReq(false);
			}
		}
	}
	///////////////////////////////////////////////////////////////////////////////////////

	// Count
	if(m_nSendCnt % 30 == 0)
	{
		m_nSendCnt = 1;
	}
	else
	{
		m_nSendCnt++;
	}
}

bool TotalTcpipCommunication::isISSACConnected(void)
{
	return m_bISSACConnected;
}

bool TotalTcpipCommunication::isIoTModuleConnected(void)
{
	return m_bIoTModuleConnected;
}

void TotalTcpipCommunication::sendResponse(vector<ResponseData>& vecResponseList)
{
	vector<ResponseData> vecResponseListCopy;

	// Copy ����
	m_CriticalSection.Lock();
	vecResponseListCopy = vecResponseList;
	vecResponseList.clear();
	m_CriticalSection.Unlock();

	// Response �۽�
	for(int i = 0; i < vecResponseListCopy.size(); i++)
	{
		clientseq(vecResponseListCopy[i]);
	}
}

bool TotalTcpipCommunication::clientseq(ResponseData& rsp_data)
{
	// 	string str_name="";
	// 
	//	str_name=Serverpart::getInstance()->get_dataName();

	string str_name = rsp_data.request_message;
/*
	if(str_name == "REQ_JOB_START")
	{
		if(XmldataSetting::getInstance()->getJobStatus() != XmldataSetting::JOB_DEFAULT &&
			XmldataSetting::getInstance()->getJobStatus() != XmldataSetting::JOB_COMPLETE)
		{
			rsp_data.request_message = "REQ_JOB_PAUSE"; // Job pause�� ������ �ٲ�
		}
	}
*/
	if(str_name=="REQ_PATH_DOWNLOAD")
	{
		if(Clientpart::getInstance()->RspPathdownload())
		{
			printf("[AGV -> ISSAC] RSP_PATH_DOWNLOAD\n");
		}

		return m_bClient;
	}
	else if(str_name=="REQ_JOB_START")
	{
//		string str_JOBCurrentName=XmldataSetting::getInstance()->getCurrentJobID();//Serverpart::getInstance()->get_JOBCurrentName();

		if(Clientpart::getInstance()->RspStart(rsp_data.job_id))
		{
			printf("[AGV -> ISSAC] RSP_JOB_START\n");
		}

		XmldataSetting::getInstance()->setJobStatus(XmldataSetting::JOB_STOP);

		if(Clientpart::getInstance()->RspStop(rsp_data.job_id))
		{
			printf("[AGV -> ISSAC] RSP_JOB_STOP\n");
		}

// 		XmldataSetting::getInstance()->setJobStatus(XmldataSetting::JOB_START);
		XmldataSetting::getInstance()->setMode(XmldataSetting::ISSAC_MODE);

		return m_bClient;
	}
	else if(str_name=="REQ_JOB_STOP")
	{
//		string str_JOBCurrentName=XmldataSetting::getInstance()->getCurrentJobID();//Serverpart::getInstance()->get_JOBCurrentName();

		if(Clientpart::getInstance()->RspStop(XmldataSetting::getInstance()->getCurrentJobID()))
		{
			printf("[AGV -> ISSAC] RSP_JOB_STOP\n");
		}

		XmldataSetting::getInstance()->setJobStatus(XmldataSetting::JOB_STOP);

		return m_bClient;
	}
	else if(str_name=="REQ_JOB_PAUSE")
	{
//		string str_JOBCurrentName=XmldataSetting::getInstance()->getCurrentJobID();//Serverpart::getInstance()->get_JOBCurrentName();

		XmldataSetting::getInstance()->setJobStatus(XmldataSetting::JOB_PAUSE);

		if(Clientpart::getInstance()->RspPause(XmldataSetting::getInstance()->getCurrentJobID()))
		{
			printf("[AGV -> ISSAC] RSP_JOB_PAUSE\n");
		}

		return m_bClient;
	}
	else if(str_name=="REQ_JOB_CAPAUSE") // Pause for collision avoidance
	{
//		string str_JOBCurrentName=XmldataSetting::getInstance()->getCurrentJobID();//Serverpart::getInstance()->get_JOBCurrentName();

		if(Clientpart::getInstance()->RspCAPause(XmldataSetting::getInstance()->getCurrentJobID()))
		{
			printf("[AGV -> ISSAC] RSP_JOB_CAPAUSE\n");
		}

		XmldataSetting::getInstance()->setJobStatus(XmldataSetting::JOB_PAUSE);

		return m_bClient;
	}
	else if(str_name=="REQ_JOB_RESUME")
	{
//		string str_JOBCurrentName=XmldataSetting::getInstance()->getCurrentJobID();//Serverpart::getInstance()->get_JOBCurrentName();

		if(Clientpart::getInstance()->RspResume(XmldataSetting::getInstance()->getCurrentJobID()))
		{
			printf("[AGV -> ISSAC] RSP_JOB_RESUME\n");
		}

		XmldataSetting::getInstance()->setJobStatus(XmldataSetting::JOB_RESUME);

		return m_bClient;
	}
	else if(str_name=="REQ_JOB_CARESUME")
	{
//		string str_JOBCurrentName=XmldataSetting::getInstance()->getCurrentJobID();//Serverpart::getInstance()->get_JOBCurrentName();

		if(Clientpart::getInstance()->RspCAResume(XmldataSetting::getInstance()->getCurrentJobID()))
		{
			printf("[AGV -> ISSAC] RSP_JOB_CARESUME\n");
		}

		XmldataSetting::getInstance()->setJobStatus(XmldataSetting::JOB_RESUME);

		return m_bClient;
	}
	else if(str_name=="REQ_JOB_COMPLETE")
	{
//		string str_JOBCurrentName=XmldataSetting::getInstance()->getCurrentJobID();//Serverpart::getInstance()->get_JOBCurrentName();

		if(Clientpart::getInstance()->ReqComplete(XmldataSetting::getInstance()->getCurrentJobID()))
		{
			printf("[AGV -> ISSAC] RSP_JOB_COMPLETE\n");
		}

		XmldataSetting::getInstance()->setJobStatus(XmldataSetting::JOB_COMPLETE);

		return m_bClient;
	}
	else if(str_name=="REQ_JOB_CHANGE")
	{
		string str_JOBCurrentName=XmldataSetting::getInstance()->getCurrentJobID();//Serverpart::getInstance()->get_JOBCurrentName();
		string str_JOBNextName=XmldataSetting::getInstance()->getNextJobID();//Serverpart::getInstance()->get_JOBNextName();

		if(Clientpart::getInstance()->RspChange(str_JOBCurrentName, str_JOBNextName))
		{
			printf("[AGV -> ISSAC] RSP_JOB_CHANGE\n");
		}

		XmldataSetting::getInstance()->setJobStatus(XmldataSetting::JOB_CHANGE);

		return m_bClient;
	}
	else if(str_name=="RSP_JOB_LIST")
	{
		return m_bClient;
	}
	else if(str_name=="REQ_MAP_DOWNLOAD")//update
	{
		if(Clientpart::getInstance()->RspMapdownload())
		{
			printf("[AGV -> ISSAC] RSP_MAP_DOWNLOAD\n");
		}

		return m_bClient;
	}
	else if(str_name=="REQ_AGV_STATE")//update
	{
		if(Clientpart::getInstance()->RspState())
		{
			printf("[AGV -> ISSAC] RSP_AGV_STATE\n");
		}

		return m_bClient;
	}
	else if(str_name=="REQ_AGV_SENSOR")//update
	{
		if(Clientpart::getInstance()->RspSensor())
		{
			printf("[AGV -> ISSAC] RSP_AGV_SENSOR\n");
		}

		return m_bClient;
	}
	else if(str_name=="REQ_AGV_PARA")//update
	{
		if(Clientpart::getInstance()->RspPara())
		{
			printf("[AGV -> ISSAC] RSP_AGV_PARA\n");
		}

		return m_bClient;
	}
	else if(str_name=="REQ_AGV_MOVE")//update
	{
		if(Clientpart::getInstance()->RspMove())
		{
			printf("[AGV -> ISSAC] RSP_AGV_MOVE\n");
		}

		return m_bClient;
	}
	else if(str_name=="REQ_AGV_ROTATE")//update
	{
		if(Clientpart::getInstance()->RspRotate())
		{
			printf("[AGV -> ISSAC] RSP_AGV_ROTATE\n");
		}

		return m_bClient;
	}
	else if(str_name=="REQ_AGV_SOUND")//update
	{
		if(Clientpart::getInstance()->RspSound())
		{
			printf("[AGV -> ISSAC] RSP_AGV_SOUND\n");
		}

		return m_bClient;
	}
	else if(str_name=="REQ_AGV_DEVICE")//update
	{
		if(Clientpart::getInstance()->RspDevice())
		{
			printf("[AGV -> ISSAC] RSP_AGV_DEVICE\n");
		}

		return m_bClient;
	}
	else if(str_name=="REQ_AGV_TOWERLAMP")//update
	{
		if(Clientpart::getInstance()->RspTowerlamp())
		{
			printf("[AGV -> ISSAC] RSP_AGV_TOWERLAMP\n");
		}

		return m_bClient;
	}
	else if(str_name=="REQ_FILE_DOWNLOAD")//update
	{
		if(Clientpart::getInstance()->RspFiledownload())
		{
			printf("[AGV -> ISSAC] RSP_FILE_DOWNLOAD\n");
		}

		return m_bClient;
	}
	else
	{
		printf("[AGV -> ISSAC] Unknown data\n");
		return m_bClient;
	}
	return m_bClient;
}

bool TotalTcpipCommunication::serverseq()
{
	bool bReadxml;
	string str_Name;
	string str_xmldata;
	ResponseData rsp_data;

	str_Name.clear();
	str_xmldata.clear();

	//SERVER START!! data read
	bReadxml=MultiTcpipInterface::getInstance()->bReadDataISSAC();
	//SERVER START!! data read

	if(bReadxml==true)
	{
		Serverpart::getInstance()->XmlRspMessage(rsp_data.request_message, rsp_data.job_id);
		str_Name = rsp_data.request_message;

		//printf("################### recieved message = %s\n", str_Name.c_str());

		m_CriticalSection.Lock();
		m_vecResponseList.push_back(rsp_data); // ���� ����Ʈ
		m_CriticalSection.Unlock();

		if(str_Name=="REQ_PATH_DOWNLOAD")
		{
			printf("[ISSAC -> AGV] RSP_PATH_DOWNLOAD\n");
		}
		else if(str_Name=="REQ_JOB_START")
		{
			printf("[ISSAC -> AGV] RSP_JOB_START\n");
		}
		else if(str_Name=="REQ_JOB_STOP")
		{
			printf("[ISSAC -> AGV] RSP_JOB_STOP\n");
		}
		else if(str_Name=="REQ_JOB_PAUSE")
		{
			printf("[ISSAC -> AGV] RSP_JOB_PAUSE\n");
		}
		else if(str_Name=="REQ_JOB_CAPAUSE")
		{
			printf("[ISSAC -> AGV] REQ_JOB_CAPAUSE\n");
		}
		else if(str_Name=="REQ_JOB_RESUME")
		{
			printf("[ISSAC -> AGV] REQ_JOB_RESUME\n");
		}
		else if(str_Name=="REQ_JOB_CARESUME")
		{
			printf("[ISSAC -> AGV] REQ_JOB_CARESUME\n");
		}
		else if(str_Name=="REQ_JOB_CHANGE")
		{
			printf("[ISSAC -> AGV] RSP_JOB_CHANGE\n");
		}
		else if(str_Name=="RSP_JOB_ENABLE")
		{
			printf("[ISSAC -> AGV] RSP_JOB_ENABLE\n");
		}
		else if(str_Name=="RSP_JOB_LIST")
		{
			printf("[ISSAC -> AGV] RSP_JOB_LIST\n");
		}
		else if(str_Name=="RSP_OPER_CALL")
		{
			printf("[ISSAC -> AGV] RSP_OPER_CALL\n");
		}
		else if(str_Name=="REQ_MAP_DOWNLOAD")//update
		{
			printf("[ISSAC -> AGV] REQ_MAP_DOWNLOAD\n");
		}
		else if(str_Name=="REQ_AGV_STATE")//update
		{
			printf("[ISSAC -> AGV] REQ_MAP_DOWNLOAD\n");
		}
		else if(str_Name=="REQ_AGV_SENSOR")//update
		{
			printf("[ISSAC -> AGV] REQ_AGV_SENSOR\n");
		}
		else if(str_Name=="REQ_AGV_PARA")//update
		{
			printf("[ISSAC -> AGV] REQ_AGV_PARA\n");
		}
		else if(str_Name=="REQ_AGV_MOVE")//update
		{
			printf("[ISSAC -> AGV] REQ_AGV_MOVE\n");
		}
		else if(str_Name=="REQ_AGV_ROTATE")//update
		{
			printf("[ISSAC -> AGV] REQ_AGV_ROTATE\n");
		}
		else if(str_Name=="REQ_AGV_SOUND")//update
		{
			printf("[ISSAC -> AGV] REQ_AGV_SOUND\n");
		}
		else if(str_Name=="REQ_AGV_DEVICE")//update
		{
			printf("[ISSAC -> AGV] REQ_AGV_DEVICE\n");
		}
		else if(str_Name=="REQ_AGV_TOWERLAMP")//update
		{
			printf("[ISSAC -> AGV] REQ_AGV_TOWERLAMP\n");
		}
		else if(str_Name=="REQ_FILE_DOWNLOAD")//update
		{
			printf("[ISSAC -> AGV] REQ_FILE_DOWNLOAD\n");
		}
		else if(str_Name=="REQ_AGV_LOCATION")//update
		{
			printf("[ISSAC -> AGV] REQ_AGV_LOCATION\n");
		}
		else
		{
			printf("[ISSAC -> AGV] Unknown message\n");
		}
	}
	else
	{
		printf("Server read fail\n");
	}
	return bReadxml;
}
void TotalTcpipCommunication::terminate()
{
	m_bIsThreadFuncGenerated = false; //�����尡 �������� �ʾҴٴ� �÷���
	m_doThreadFunc = false; //������ �Լ��� ����ǵ��� �ϴ� �÷���
}
