#include "stdafx.h"
#include "Serverpart.h"
#include "../MobileSupervisor/KuRobotParameter.h"
#include "../ANSCommon/ANSCommon.h"
#include "Clientpart.h"

Serverpart::Serverpart()
{
	m_bIsThreadFuncGenerated=false;
	m_bsendflag=false;
	m_sendflag=false;
	m_strName.clear();
	m_strJobCurrentID.clear();
	m_strJobNewID.clear();
	m_nJobChangeApproval = -1; // 응답 없음
	m_bJobReceivedOnce = false;
	m_bJobListReceivedOnce = false;

	AllocConsole(); //콘솔 창을 만들어주는 함수.....
	freopen( "CONOUT$", "wt", stdout);
}

Serverpart::~Serverpart()
{
}

void Serverpart::XmlRspMessage(string& response_message, string& job_id)//rsp message name을 출력하는 함수
{
	string sJobID;
	char *c_ReceiveData=(char*)MultiTcpipInterface::getInstance()->getReadData()->c_str();
	TiXmlDocument tinyxmlDoc;

// 	char *c_RealData = c_ReceiveData + 10;
	string str_Name="";
	const char* chRes = tinyxmlDoc.Parse((const char*)c_ReceiveData, 0);//, TIXML_ENCODING_UTF8);
	TiXmlElement* pRoot = tinyxmlDoc.FirstChildElement("message");
	//if (!pRoot);

	TiXmlElement* pElement_name = pRoot->FirstChildElement("header")->FirstChildElement("name");
	//char* pName=(char*)pElement->Value();
	char* pName = (char*)pElement_name->GetText();
	str_Name=pName;
	set_dataName(str_Name);
	if(str_Name=="REQ_MAP_DOWNLOAD")
	{
		XmlMapParsing(str_Name);
	}
	else if(str_Name=="REQ_PATH_DOWNLOAD")
	{
		TiXmlDocument tinyxmlDoc;

		tinyxmlDoc.Parse((const char*)MultiTcpipInterface::getInstance()->getReadData()->c_str(), 0, TIXML_ENCODING_UTF8);

		XmlPathParsing(&tinyxmlDoc);
	}
	else
	{
		XmlDataParsing(str_Name, sJobID);
	}
	//tinyxml 써서 name parsing해야 되는 부분

	response_message = str_Name;
	job_id = sJobID;
}

void Serverpart::XmlDataParsing(string str_Name, string& job_id)//tinyxml써서 parsing하는 함수
{
	job_id.clear();
	char *c_ReceiveData=(char*)MultiTcpipInterface::getInstance()->getReadData()->c_str();
// 	char *c_RealData = c_ReceiveData + 10;
	TiXmlDocument tinyxmlDoc;

	tinyxmlDoc.Parse((const char*)c_ReceiveData, 0, TIXML_ENCODING_UTF8);

	TiXmlElement* pRoot = tinyxmlDoc.FirstChildElement("message");
	TiXmlElement* pBody= pRoot->FirstChildElement("body");
	TiXmlElement* pReturn= pRoot->FirstChildElement("return");

	if(str_Name=="REQ_JOB_START")
	{
		TiXmlElement* pElement_agvid = pBody->FirstChildElement("agvid");
		TiXmlElement* pElement_jobid = pBody->FirstChildElement("jobid");
		TiXmlElement* pElement_pathid = pBody->FirstChildElement("pathid");
		TiXmlElement* pElement_repeatcount = pBody->FirstChildElement("repeatcount");
		TiXmlElement* pElement_timeout = pBody->FirstChildElement("timeout");

		if(pElement_agvid && pElement_jobid && pElement_pathid && pElement_repeatcount && pElement_timeout)
		{
			char* pagvid = (char*)pElement_agvid->GetText();
			char* pjobid = (char*)pElement_jobid->GetText();
			char* ppathid = (char*)pElement_pathid->GetText();
			char* prepeatcount = (char*)pElement_repeatcount->GetText();
			char* ptimeout = (char*)pElement_timeout->GetText();

			int nagvid = atoi(pagvid);
// 			set_JOBCurrentName(pjobid);
			m_strJobCurrentID = pjobid;
			string strPathID = ppathid;
 			int npathid = atoi(strPathID.substr(9, 4).c_str());
			int nrepeatcount = atoi(prepeatcount);
			int ntimeout = atoi(ptimeout);
			job_id = pjobid;

			ofstream path_list_file;

 			if(!m_bJobReceivedOnce) // 프로그램을 실행한 이후 처음 받는 job이면
			{
				XmldataSetting::getInstance()->clear_job_list();
			}

//			XmldataSetting::getInstance()->setJobID_PathID(m_strJobCurrentID, npathid);
			XmldataSetting::getInstance()->add_job_and_path(m_strJobCurrentID, strPathID, nrepeatcount);

// 			{
				path_list_file.open("./data/path/path_list.txt", ios::out); // 새로운 파일 생성
// 			}
// 			else
// 			{
// 				path_list_file.open("./data/path/path_list.txt", ios::app); // 이어서 데이타 추가
// 			}

			if(path_list_file.is_open())
			{
				for(int i = 0; i < XmldataSetting::getInstance()->getJobList().size(); i++)
				{
					path_list_file << XmldataSetting::getInstance()->getJobList()[i].job_id << " "
									<< XmldataSetting::getInstance()->getJobList()[i].path_id << " "
									<< XmldataSetting::getInstance()->getJobList()[i].job_status_id << " "
									<< XmldataSetting::getInstance()->getJobList()[i].repeat_cnt << " "
									<< XmldataSetting::getInstance()->getJobList()[i].complete_cnt << endl;
				}

				path_list_file.close();
			}

			m_bJobReceivedOnce = true;

			// Log
			stringstream ss;
			string sJobID;
			ss << "Received a new job " << sJobID;
			ANS_LOG_WRITE(ss.str());
		}
	}
	else if(str_Name=="REQ_JOB_STOP")
	{
		TiXmlElement* pElement_agvid = pBody->FirstChildElement("agvid");
		TiXmlElement* pElement_jobid = pBody->FirstChildElement("jobid");

		if(pElement_agvid && pElement_jobid)
		{
			char* pagvid = (char*)pElement_agvid->GetText();
			char* pjobid = (char*)pElement_jobid->GetText();

			int nagvid = atoi(pagvid);
// 			set_JOBCurrentName(pjobid);
			job_id = pjobid;
		}
	}
	else if(str_Name=="REQ_JOB_PAUSE")
	{
		TiXmlElement* pElement_agvid = pBody->FirstChildElement("agvid");
		TiXmlElement* pElement_jobid = pBody->FirstChildElement("jobid");

		if(pElement_agvid && pElement_jobid)
		{
			char* pagvid = (char*)pElement_agvid->GetText();
			char* pjobid = (char*)pElement_jobid->GetText();

			int nagvid = atoi(pagvid);
// 			set_JOBCurrentName(pjobid);
			job_id = pjobid;
		}
	}
	else if(str_Name=="REQ_JOB_RESUME")
	{
		TiXmlElement* pElement_agvid = pBody->FirstChildElement("agvid");
		TiXmlElement* pElement_jobid = pBody->FirstChildElement("jobid");

		if(pElement_agvid && pElement_jobid)
		{
			char* pagvid = (char*)pElement_agvid->GetText();
			char* pjobid = (char*)pElement_jobid->GetText();

			int nagvid = atoi(pagvid);
// 			set_JOBCurrentName(pjobid);
			job_id = pjobid;
		}
	}
	else if(str_Name=="REQ_JOB_CHANGE")
	{
		TiXmlElement* pElement_agvid = pBody->FirstChildElement("agvid");
		TiXmlElement* pElement_from = pBody->FirstChildElement("jobinfo")->FirstChildElement("from");
		TiXmlElement* pElement_to = pElement_from->FirstChildElement("to");

		if(pElement_agvid && pElement_from && pElement_to)
		{
			m_strJobNewID.clear();

			char* pagvid = (char*)pElement_agvid->GetText();
			char* pfrom = (char*)pElement_from->GetText();
			char* pto = (char*)pElement_to->GetText();

			int nagvid = atoi(pagvid);
 			set_JOBCurrentName(pfrom);
 			set_JOBNextName(pto);
		}
	}
	else if(str_Name=="REQ_JOB_COMPLETE")
	{
		TiXmlElement* pElement_agvid = pBody->FirstChildElement("agvid");
		TiXmlElement* pElement_jobid = pBody->FirstChildElement("jobid");

		if(pElement_agvid && pElement_jobid)
		{
			char* pagvid = (char*)pElement_agvid->GetText();
			char* pjobid = (char*)pElement_jobid->GetText();

			int nagvid = atoi(pagvid);
// 			set_JOBCurrentName(pjobid);
			job_id = pjobid;
		}
	}
	else if(str_Name=="RSP_JOB_ENABLE")
	{
		TiXmlElement* pElement_agvid = pBody->FirstChildElement("agvid");
		TiXmlElement* pElement_pathid = pBody->FirstChildElement("pathid");
		TiXmlElement* pElement_returncode = pReturn->FirstChildElement("returncode");

		if(pElement_agvid && pElement_pathid)
		{
			char* pagvid = (char*)pElement_agvid->GetText();
			char* ppathid = (char*)pElement_pathid->GetText();
			string sPathID = ppathid;
			string sJobID = "";

			int nagvid = atoi(pagvid);

			char* preturncode = (char*)pElement_returncode->GetText();

			if(*preturncode == '0') // Approval
			{
				for(int i = 0; i < XmldataSetting::getInstance()->getJobList().size();i++)
				{
					if(XmldataSetting::getInstance()->getJobList()[i].path_id == sPathID)
					{
						sJobID = XmldataSetting::getInstance()->getJobList()[i].job_id;
						break;
					}
				}

				if(sJobID != "") // 해당 path에 대한 job id가 존재할 때
				{
					//string sJobIDPrev = XmldataSetting::getInstance()->getCurrentJobID();
					string sJobIDPrev = XmldataSetting::getInstance()->getPrevJobID();
					if(sJobIDPrev != "")
					{
						Clientpart::getInstance()->RspStop(sJobIDPrev);
					}

					if(Clientpart::getInstance()->RspStart(sJobID))
					{
						if(XmldataSetting::getInstance()->selectJob(sJobID))
						{
							XmldataSetting::getInstance()->setJobStatus(XmldataSetting::JOB_RUN);

							printf("[AGV -> ISSAC] RSP_JOB_START\n");
						}
					}

					printf("[ISSAC -> AGV] Job change approval\n");

					m_nJobChangeApproval = 1; // 주행 가능
				}
			}
			else // -1
			{
				m_nJobChangeApproval = 0; // 주행 불가

				printf("[ISSAC -> AGV] Unable to change the job. (RSP_JOB_ENABLE)\n");
			}
		}
	}
	else if(str_Name=="RSP_JOB_LIST")
	{
		TiXmlElement* pElement_agvid = pBody->FirstChildElement("agvid");
		char* pagvid=(char*)pElement_agvid->GetText();
		int nagvid = atoi(pagvid);

		if(nagvid == KuRobotParameter::getInstance()->getRobotID()) // AGV ID가 같으면
		{
			TiXmlNode* pJobInfo = pRoot->FirstChild("body")->FirstChild("jobinfo");
			int nCnt(1);

			XmldataSetting::getInstance()->clear_job_list(); // 모든 job을 삭제


			for(pJobInfo; pJobInfo; pJobInfo = pJobInfo->NextSibling())
			{
				TiXmlElement* pElement_jobid = pJobInfo->FirstChildElement("jobid");
				TiXmlElement* pElement_carrierid = pJobInfo->FirstChildElement("carrierid");
				TiXmlElement* pElement_repeatcount = pJobInfo->FirstChildElement("repeatcount");
				TiXmlElement* pElement_completecount = pJobInfo->FirstChildElement("completecount");
				TiXmlElement* pElement_jobstatusid = pJobInfo->FirstChildElement("jobstatusid");
				TiXmlElement* pElement_pathid = pJobInfo->FirstChildElement("pathid");

				if(pElement_jobid && pElement_carrierid && pElement_repeatcount && pElement_completecount && pElement_jobstatusid && pElement_pathid)
				{
					char* pJobID = (char*)pElement_jobid->GetText();
					string sJobID = pJobID;
					char* pRepeatCnt = (char*)pElement_repeatcount->GetText();
					int nRepeatCnt = atoi(pRepeatCnt);
					char* pCompleteCnt = (char*)pElement_completecount->GetText();
					int nCompleteCnt = atoi(pCompleteCnt);
					char* pJobStatusID = (char*)pElement_jobstatusid->GetText();
					int nJobStatusID = atoi(pJobStatusID);
					char* pPathID = (char*)pElement_pathid->GetText();
					string sPathID = pPathID;

					XmldataSetting::getInstance()->add_job_and_path(sJobID, sPathID, nRepeatCnt, nJobStatusID, nCompleteCnt);
					Clientpart::getInstance()->RspStop(sJobID);

					printf("[Job %d] ID: %s, Path: %s, Repeat cnt.: %d, Comp. cnt.: %d, Job status: %d\n", nCnt, sJobID.c_str(), sPathID.c_str(), nRepeatCnt, nCompleteCnt, nJobStatusID);

					//m_bJobListReceivedOnce = true;

					nCnt++;
				}
			}

			// 파일에 저장
			ofstream path_list_file;

			path_list_file.open("./data/path/path_list.txt", ios::out); // 새로운 파일 생성

			if(path_list_file.is_open())
			{
				for(int i = 0; i < XmldataSetting::getInstance()->getJobList().size(); i++)
				{
					path_list_file << XmldataSetting::getInstance()->getJobList()[i].job_id << " "
									<< XmldataSetting::getInstance()->getJobList()[i].path_id << " "
									<< XmldataSetting::getInstance()->getJobList()[i].job_status_id << " "
									<< XmldataSetting::getInstance()->getJobList()[i].repeat_cnt << " "
									<< XmldataSetting::getInstance()->getJobList()[i].complete_cnt << endl;
				}

				path_list_file.close();
			}

			//m_bJobReceivedOnce = true;

			m_bJobListReceivedOnce = true;
		}
	}
	else if(str_Name=="RSP_OPER_CALL")
	{
		TiXmlElement* pElement_operid = pBody->FirstChildElement("operid");

		if(pElement_operid)
		{
			char* poperid = (char*)pElement_operid->GetText();

			int noperid = atoi(poperid);
		}
	}
	else if(str_Name=="REQ_AGV_LOCATION")
	{
		TiXmlNode* pAGVLocationID = pRoot->FirstChild("body")->FirstChild("agv");

		if(pAGVLocationID != 0)
		{
			for(pAGVLocationID; pAGVLocationID; pAGVLocationID = pAGVLocationID->NextSibling())
			{	
				TiXmlElement* pElement_agvid = pAGVLocationID->FirstChildElement("agvid");
				TiXmlElement* pElement_currentid = pAGVLocationID->FirstChildElement("pathblockid")->FirstChildElement("current");
				TiXmlElement* pElement_nextid = pAGVLocationID->FirstChildElement("pathblockid")->FirstChildElement("next");

				if(pElement_agvid && pElement_currentid && pElement_nextid)
				{
					char* pAGVID = (char*)pElement_agvid->GetText();
					int nAGVID = atoi(pAGVID);

					char* pCurrentBlockID = (char*)pElement_currentid->GetText();
					string strCurrentBlockID;

					if(pCurrentBlockID) // Next block 정보가 수신되지 않는 경우 예외처리
					{
						strCurrentBlockID = pCurrentBlockID;
					}
					else
					{
						strCurrentBlockID.clear();
					}

					char* pNextBlockID = (char*)pElement_nextid->GetText();
					string strNextBlockID;

					if(pNextBlockID) // Next block 정보가 수신되지 않는 경우 예외처리
					{
						strNextBlockID = pNextBlockID;
					}
					else
					{
						strNextBlockID.clear();
					}

					//vector<string> block_id;
					//block_id.push_back(strCurrentBlockID); // 추후 for문으로 n개의 block을 등록해야 함
					//block_id.push_back(strNextBlockID); // 추후 for문으로 n개의 block을 등록해야 함

					//saveMultiAGVLocation(nAGVID, block_id);

					printf("[AGV%d] CurrentID: %s, Next ID: %s\n", nAGVID, strCurrentBlockID.c_str(), strNextBlockID.c_str());
				}
			}
		}
	}
	else
	{
		printf("[ISSAC -> AGV] Received unknown data\n");
	}
}

int Serverpart::getJobChangeApproval(void) // -1: 응답 없음, 0: 주행 불가, 1: 주행 가능
{
	int nApproval(m_nJobChangeApproval);

	m_nJobChangeApproval = -1; // 초기화

	return nApproval;
}

bool Serverpart::isJobListReceivedOnce(void)
{
	return m_bJobListReceivedOnce;
}

void Serverpart::XmlPathParsing(TiXmlDocument* pDoc)
{
	vector<PBlock> vecPathblock;
	PBlock pathblock;
	char c[100];
	int nCnt(0);

	ofstream path_list_file("./data/path/path_list.txt", ios::out);

// 	char *c_RealData = c_ReceiveData + 10;

	pDoc->Parse((const char*)pDoc, 0, TIXML_ENCODING_UTF8);
	TiXmlElement* pRoot = pDoc->FirstChildElement("message");
	TiXmlElement* pElement_agvid = pRoot->FirstChildElement("body")->FirstChildElement("agv")->FirstChildElement("agvid");
	
	char* pagvid=(char*)pElement_agvid->GetText();
	int nagvid = atoi(pagvid);

	TiXmlNode* pPath = pRoot->FirstChild("body")->FirstChild("pathlist")->FirstChild("path");

	for(pPath; pPath; pPath = pPath->NextSibling())
	{
 		TiXmlElement* pElement_pathid = pPath->FirstChildElement("pathid");

		char* ppathid=(char*)pElement_pathid->GetText();
		string strPathID = ppathid;
 		int npathid = atoi(strPathID.substr(9, 4).c_str());

		TiXmlNode* pBlock = pPath->FirstChild("block");
		vecPathblock.clear();

		for(pBlock; pBlock; pBlock = pBlock->NextSibling())
		{
			TiXmlElement* pElement_id = pBlock->FirstChildElement("id");
			TiXmlElement* pElement_blocktype = pBlock->FirstChildElement("type");
			TiXmlElement* pElement_waypoint = pBlock->FirstChildElement("waypoint");
			TiXmlElement* pElement_obstacle = pBlock->FirstChildElement("obstacle");
			TiXmlElement* pElement_position_x = pBlock->FirstChildElement("position")->FirstChildElement("x");
			TiXmlElement* pElement_position_y = pBlock->FirstChildElement("position")->FirstChildElement("y");
			TiXmlElement* pElement_blocksize_x = pBlock->FirstChildElement("blocksize")->FirstChildElement("y");
			TiXmlElement* pElement_blocksize_y = pBlock->FirstChildElement("blocksize")->FirstChildElement("y");
			TiXmlElement* pElement_velocity = pBlock->FirstChildElement("velocity");
// 			TiXmlElement* pElement_task = pBlock->FirstChildElement("task");//task 부분 추가 필요

			char* pId=(char*)pElement_id->GetText();
			string strBlockID = pId;
			int nId = atoi(strBlockID.substr(9, 4).c_str());

			char* pBlocktype = (char*)pElement_blocktype->GetText();
			int nBlocktype = atoi(pBlocktype);

			char* pWaypoint = (char*)pElement_waypoint->GetText();
			int nWaypoint = atoi(pWaypoint);

			char* pObstacle = (char*)pElement_obstacle->GetText();
			int nObstacle = atoi(pObstacle);

			char* pPosition_x = (char*)pElement_position_x->GetText();
			float nPosition_x = atof(pPosition_x);

			char* pPosition_y = (char*)pElement_position_y->GetText();
			float nPosition_y = atof(pPosition_y);

			char* pBlocksize_x = (char*)pElement_blocksize_x->GetText();
			float nBlocksize_x = atof(pBlocksize_x);

			char* pBlocksize_y = (char*)pElement_blocksize_y->GetText();
			float nBlocksize_y = atof(pBlocksize_y);

			char* pVelocity = (char*)pElement_velocity->GetText();
			int nVelocity = atoi(pVelocity);

			// Path block 내용 채우기 ///////////////////////////////////////////////////////////////
			m_KuPathBlockPr.setBlockSize(nBlocksize_x / 1000, nBlocksize_y / 1000);
			pathblock = m_KuPathBlockPr.getPathBlock(nBlocktype); // 초기화. dist1, dist2 값을 부여함.

			pathblock.id = nId;
			pathblock.block_id_for_ISSAC = strBlockID;
			pathblock.pathdata = nBlocktype;
			pathblock.x = nPosition_x/1000;
			pathblock.y = nPosition_y/1000;
			pathblock.sizex = nBlocksize_x / 1000;
			pathblock.sizey = nBlocksize_y / 1000;
			pathblock.velocity = nVelocity;
			pathblock.task = -1; // 수정 필요할 수도 있음
			pathblock.state = 0; // 수정 필요할 수도 있음
			pathblock.nRouteOrder = 0; // default (via)
			pathblock.check = 0; // default
			pathblock.overlap = false; // default
			pathblock.waypoint = nWaypoint;
// 			pathblock.dist1.start_x = 0.; // getPathBlock() 함수에서 이미 값을 부여함.
// 			pathblock.dist1.start_y = 0.; // getPathBlock() 함수에서 이미 값을 부여함.
// 			pathblock.dist1.end_x = 0.; // getPathBlock() 함수에서 이미 값을 부여함.
// 			pathblock.dist1.end_y = 0.; // getPathBlock() 함수에서 이미 값을 부여함.
// 			pathblock.dist2.start_x = 0.; // getPathBlock() 함수에서 이미 값을 부여함.
// 			pathblock.dist2.start_y = 0.; // getPathBlock() 함수에서 이미 값을 부여함.
// 			pathblock.dist2.end_x = 0.; // getPathBlock() 함수에서 이미 값을 부여함.
// 			pathblock.dist2.end_y = 0.; // getPathBlock() 함수에서 이미 값을 부여함.
			pathblock.detectObstacle = (bool)nObstacle;
			pathblock.TaskList.clear(); // 수정 필요
			
			// Task
			TiXmlNode* pTask = pBlock->FirstChild("task")->FirstChild();

			for(pTask; pTask; pTask = pTask->NextSibling())
			{
				const char* node_name = pTask->Value(); // for debugging
				char* pTaskType = (char*)pTask->ToElement()->GetText();
				int nTaskType = atoi(pTaskType);

				if(nTaskType == PathBlock::DEVICE5)
				{
					pathblock.check = 1; // Device5를 충돌방지 용으로 사용(SGEC)
					continue; // Task에 등록하지 않음
				}

				pathblock.TaskList.push_back(nTaskType);
			}

			if(pathblock.TaskList.size() == 0)
			{
				pathblock.waypoint = 0; // Waypoint 해제
			}

			vecPathblock.push_back(pathblock);
		}

		// Path block 파일로 저장
//		sprintf(c, "./data/path/LoadPathBlock%d.txt", npathid);
		sprintf(c, "./data/path/%s.txt", ppathid);
		string strFilePath = c;
		m_KuPathBlockPr.savePathBlock(vecPathblock, strFilePath);

		if(path_list_file.is_open())
		{
			path_list_file << nCnt << " " << ppathid << " " << XmldataSetting::JOB_DEFAULT << " " << -1 << " " << 0 << endl;
		}

		printf("Successfully saved the path to %s file.\n", strFilePath.c_str());

		nCnt++;
	}

	if(path_list_file.is_open())
	{
		path_list_file.close();
	}
}

void Serverpart::XmlMapParsing(string str_Name)//tinyxml + base64 decoding 써서 parsing하는 함수
{
	int nMapSizeX = KuRobotParameter::getInstance()->getMapSizeXm()*10;
	int nMapSizeY = KuRobotParameter::getInstance()->getMapSizeYm()*10;
	int nBitmapHeader = 54; // bytes

	unsigned char *decodedStr = new unsigned char [nMapSizeX * nMapSizeY * 3 + nBitmapHeader];//nMapSizeX * nMapSizeY * 3]; // decode 된 데이터의 사이즈가 12000054이므로 임시로 넣어줌. "nMapSizeX * nMapSizeY * 3"은 에러 발생.
	int decodedStr_length;
	int pencode_mapdata_length;
	
	char *c_ReceiveData=(char*)MultiTcpipInterface::getInstance()->getReadData()->c_str();
	TiXmlDocument tinyxmlDoc;

	tinyxmlDoc.Parse((const char*)c_ReceiveData, 0, TIXML_ENCODING_UTF8);
	TiXmlElement* pRoot = tinyxmlDoc.FirstChildElement("message");
	TiXmlElement* pElement_agvid = pRoot->FirstChildElement("body")->FirstChildElement("agv")->FirstChildElement("agvid");

	char* pagvid=(char*)pElement_agvid->GetText();
	int nagvid = atoi(pagvid);

	TiXmlElement* pElement_encode_mapdata = pRoot->FirstChildElement("body")->FirstChildElement("mapdata");
//	char* pencode_mapdata=(char*)pElement_encode_mapdata->GetText();

	pencode_mapdata_length = strlen((char*)pElement_encode_mapdata->GetText());

	bool bMapdata=Base64_Decode((char*)pElement_encode_mapdata->GetText(), pencode_mapdata_length, decodedStr, &decodedStr_length);
	
	if(bMapdata==true)
	{
		printf("[ISSAC -> AGV] Decoding complete.\n");
		printf("[ISSAC -> AGV] Now saving the map data to a file... ");
		
		// BMP로 저장
		IplImage* IplMapImage = cvCreateImage(cvSize (nMapSizeX, nMapSizeY), IPL_DEPTH_8U, 3);

		for(int LoopX=0 ; LoopX < nMapSizeX ; LoopX++){
			for(int LoopY=0 ; LoopY < nMapSizeY; LoopY++){
					IplMapImage->imageData[(LoopX+LoopY*nMapSizeX)*3]=(char)decodedStr[(LoopX+LoopY*nMapSizeX)*3 + nBitmapHeader]; //b
					IplMapImage->imageData[(LoopX+LoopY*nMapSizeX)*3+1]=(char)decodedStr[(LoopX+LoopY*nMapSizeX)*3+1 + nBitmapHeader]; //g
					IplMapImage->imageData[(LoopX+LoopY*nMapSizeX)*3+2]=(char)decodedStr[(LoopX+LoopY*nMapSizeX)*3+2 + nBitmapHeader]; //r
			}
		}
		cvFlip(IplMapImage);
		cvSaveImage(KuRobotParameter::getInstance()->getMapNameNPath().c_str(),IplMapImage);
		cvReleaseImage(&IplMapImage);

		printf("done\n");
	}
	else
	{
		printf("[ISSAC -> AGV] Failed to read map data.\n");
	}

	delete []  decodedStr;
}

void Serverpart::set_dataName(string str_name)
{
	m_strName=str_name;
}

string Serverpart::get_dataName()
{
	return m_strName;
}

void Serverpart::set_JOBCurrentName(char* pchJOBname)
{
	m_strJobCurrentID.clear();

	m_strJobCurrentID = pchJOBname;
}

string Serverpart::get_JOBCurrentName()
{
	return m_strJobCurrentID;
}

void Serverpart::set_JOBNextName(char* pchJOBname)
{
	m_strJobNewID = pchJOBname;
}

string Serverpart::get_JOBNextName()
{
	return m_strJobNewID;
}

PBlock Serverpart::PBlockdist(float nblocksize_x, float nblocksize_y, int nblocktype)//input=blocksize, blocktype
{
	LinePoint Lpoint;
	PBlock Ppathblock;
	
	Ppathblock.x=0;
	Ppathblock.y=0;
	Ppathblock.id=-1;
	Ppathblock.velocity= 0;
	Ppathblock.task=-1;
	Ppathblock.waypoint=0;
	Ppathblock.state=0;
	Ppathblock.nRouteOrder=0;
	Ppathblock.pathdata=-1;
	Ppathblock.sizex=1;
	Ppathblock.sizey=1;//초기화..
	Ppathblock.check=false;
	Ppathblock.overlap=false;
	Lpoint.start_x=0.0;
	Lpoint.start_y=0.0;
	Lpoint.end_x=0.0;
	Lpoint.end_y=0.0;
	Ppathblock.dist1=Lpoint;
	Lpoint.start_x=0.0;
	Lpoint.start_y=0.0;
	Lpoint.end_x=0.0;
	Lpoint.end_y=0.0;
	Ppathblock.dist2=Lpoint;
	Ppathblock.detectObstacle=true;
	Ppathblock.TaskList.clear();

	Ppathblock.sizex=nblocksize_x;
	Ppathblock.sizey=nblocksize_y;//초기화..
	Ppathblock= m_KuPathBlockPr.getPathBlock(nblocktype, false);
	return Ppathblock;
}

//----------------------------------------base64-------------------------------------------

bool Serverpart::Base64_Encode(unsigned char *data, int data_length, char *encodedStr)
{
	int i, j;
	unsigned char tmpBytes[3];

	if((data_length>0) && (data==NULL)) return false;
	if(data_length<0)		    return false;
	if(data_length==0)		    return true;

	for(i=0,j=0;i<(data_length/3);i++){
		tmpBytes[0] = data[i*3];
		tmpBytes[1] = data[i*3+1];
		tmpBytes[2] = data[i*3+2];
		encodedStr[j++] = MYBASE64_keyE[(unsigned char)((tmpBytes[0]>>2)&0x3F)];
		encodedStr[j++] = MYBASE64_keyE[(unsigned char)(((tmpBytes[0]<<4)&0x30) | ((tmpBytes[1]>>4)&0x0F))];
		encodedStr[j++] = MYBASE64_keyE[(unsigned char)(((tmpBytes[1]<<2)&0x3C) | ((tmpBytes[2]>>6)&0x03))];
		encodedStr[j++] = MYBASE64_keyE[(unsigned char)(tmpBytes[2]&0x3F)];
	}
	switch(data_length%3){
	case 0:
		break;
	case 1:
		tmpBytes[0] = data[i*3];
		encodedStr[j++] = MYBASE64_keyE[(unsigned char)((tmpBytes[0]>>2)&0x3F)];
		encodedStr[j++] = MYBASE64_keyE[(unsigned char)((tmpBytes[0]<<4)&0x30)];
		encodedStr[j++] = '=';
		encodedStr[j++] = '=';
		break;
	case 2:
		tmpBytes[0] = data[i*3];
		tmpBytes[1] = data[i*3+1];
		encodedStr[j++] = MYBASE64_keyE[(tmpBytes[0]>>2)&0x3F];
		encodedStr[j++] = MYBASE64_keyE[(unsigned char)(((tmpBytes[0]<<4)&0x30) | ((tmpBytes[1]>>4)&0x0F))];
		encodedStr[j++] = MYBASE64_keyE[(unsigned char)((tmpBytes[1]<<2)&0x3C)];
		encodedStr[j++] = '=';
		break;
	};

	return true;
}
bool Serverpart::Base64_Decode(char *encodedStr, int encodedStr_length, unsigned char *decodedStr, int *decodedStr_length)
{
	int i, x, n;
	unsigned char tmpBytes[4];

	if((encodedStr_length>0) && (encodedStr==NULL)) return false;
	if(encodedStr_length<0)				return true;
	if(encodedStr_length==0)			return true;
	if((encodedStr_length%4))			return false;

	if(encodedStr[encodedStr_length-3]=='=') *decodedStr_length=encodedStr_length/4*3-3;
	else if(encodedStr[encodedStr_length-2]=='=') *decodedStr_length=encodedStr_length/4*3-2;
	else if(encodedStr[encodedStr_length-1]=='=') *decodedStr_length=encodedStr_length/4*3-1;
	else *decodedStr_length=encodedStr_length/4*3;

	n = (encodedStr_length/4);

	for(i=0;i<n;i++)
	{
		if(encodedStr[i*4+0]=='='){ tmpBytes[0]=0; }else{ tmpBytes[0] = MYBASE64_keyD[(unsigned char)encodedStr[i*4+0]]; }
		if(encodedStr[i*4+1]=='='){ tmpBytes[1]=0; }else{ tmpBytes[1] = MYBASE64_keyD[(unsigned char)encodedStr[i*4+1]]; }
		if(encodedStr[i*4+2]=='='){ tmpBytes[2]=0; }else{ tmpBytes[2] = MYBASE64_keyD[(unsigned char)encodedStr[i*4+2]]; } 
		if(encodedStr[i*4+3]=='='){ tmpBytes[3]=0; }else{ tmpBytes[3] = MYBASE64_keyD[(unsigned char)encodedStr[i*4+3]]; }
		if((tmpBytes[0]&0x80)||(tmpBytes[1]&0x80)||(tmpBytes[2]&0x80)||(tmpBytes[3]&0x80)) return false;
		x =            ( (int)(tmpBytes[0] & 0x3F) );
		x = (x << 6) | ( (int)(tmpBytes[1] & 0x3F) );
		x = (x << 6) | ( (int)(tmpBytes[2] & 0x3F) );
		x = (x << 6) | ( (int)(tmpBytes[3] & 0x3F) );
		if((i+1)<n){
			decodedStr[i*3+0] = (unsigned char)((x>>16)&0xFF);
			decodedStr[i*3+1] = (unsigned char)((x>> 8)&0xFF);
			decodedStr[i*3+2] = (unsigned char)((x>> 0)&0xFF);
		}else{
			if((i*3+0)<encodedStr_length) decodedStr[i*3+0] = (unsigned char)((x>>16)&0xFF);
			if((i*3+1)<encodedStr_length) decodedStr[i*3+1] = (unsigned char)((x>> 8)&0xFF);
			if((i*3+2)<encodedStr_length) decodedStr[i*3+2] = (unsigned char)((x>> 0)&0xFF);
		}

		printf("[ISSAC -> AGV] Decoding map data.... %3d%%\r", (int)((float)(i + 1) / n * 100));
	}

	printf("\n");

	return true;
}

//----------------------------------------base64-------------------------------------------
