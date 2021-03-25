#include "stdafx.h"
#include "MobileSupervisor.h"
#include "../ANSCommon/ANSCommon.h"

MobileSupervisor::MobileSupervisor()
{
//	ANS_LOG_WRITE("Algorithm start ==================================================");

	m_bBehaviorStates=false;
	m_bAutoThread=false;
	m_bAvoidMode=false;
	m_nTotalFrame=0;
}

MobileSupervisor::~MobileSupervisor()
{
	ANS_LOG_WRITE("Algorithm end ====================================================");	
}

/**
@brief Korean: 초기지도 정보를 불러오는 함수
@brief English: 
*/
bool MobileSupervisor::loadMap()
{
	//지도를 불러 온다--------------------------------------------------------------------------
	string strMapNameNPath = KuRobotParameter::getInstance()->getMapNameNPath();
	bool bRes = kuMapRepository::getInstance()->loadMap(strMapNameNPath); ///지도 로딩

	//속도 지도를 불러 온다--------------------------------------------------------------------------
	strMapNameNPath = KuRobotParameter::getInstance()->getVelocityMapNameNPath();
	kuMapRepository::getInstance()->loadVelocityMap(strMapNameNPath); ///지도 로딩

	//티칭한 경로를 불러 온다--------------------------------------------------------------------------
	string strDataPath =KuRobotParameter::getInstance()->getTeachingPathNameNPath();
	string strNewPath;
	char cFilePathName[300];
	memset(cFilePathName,0,sizeof(cFilePathName));
	sprintf(cFilePathName,"%s/path.txt",strDataPath.c_str());
	strNewPath=cFilePathName;//char 포인터값을 string에 대입
	KuTeachingPathPlannerPr m_KuTeachingPathPlannerPr;
	list <KuPose> Pathlist=m_KuTeachingPathPlannerPr.loadPath(strNewPath);
	KuDrawingInfo::getInstance()->setPath(Pathlist);

	//티칭한 경유점을 불러 온다--------------------------------------------------------------------------
	int nAGVID=KuRobotParameter::getInstance()->getRobotID();
	string strDataWaypoint =KuRobotParameter::getInstance()->getTeachingPathNameNPath();
	memset(cFilePathName,0,sizeof(cFilePathName));
	sprintf(cFilePathName,"%s/waypoint_%d.txt",strDataPath.c_str(),nAGVID);
	strNewPath=cFilePathName;//char 포인터값을 string에 대입
	list <KuPose> WayPointList=m_KuTeachingPathPlannerPr.loadWayPointList(strNewPath);
	list<KuPose>::iterator itway;
	for(itway=WayPointList.begin(); itway!=WayPointList.end(); itway++){
		m_vecWayPoint.push_back(*itway);
	}
	KuDrawingInfo::getInstance()->setWayPointList(WayPointList);	

	//zone Map 불러 온다--------------------------------------------------------------------------
	// 	strNewPath= KuRobotParameter::getInstance()->getZoneMapNameNPath();
	// 	MultiRobotSupervisor::getInstance()->loadZoneMap(strNewPath);

	vector<CLAMPData> FRData= KuILBPFLocalizerPr::getInstance()->loadFeatureData( );
	KuDrawingInfo::getInstance()->setRegion(FRData);

	vector<CFREAKData> FTData= KuILBPFLocalizerPr::getInstance()->loadTemplateData( );
	KuDrawingInfo::getInstance()->setTemplateData(FTData);

	KuFiducialbasedLocalizerPr::getInstance()->loadFiducialFeatureMap( );
	list<KuPose> FiducialMarkList = KuFiducialbasedLocalizerPr::getInstance()->getFiducialMarkList();
	KuDrawingInfo::getInstance()->setFiducialMarkList(FiducialMarkList);

	//전역위치 파라미터를 불러 온다--------------------------------------------------------------------------
	if(KuRobotParameter::getInstance()->getGlobalLocalization()=="yes") {		
		string strDataImagPath =KuRobotParameter::getInstance()->getTeachingPathNameNPath();
		memset(cFilePathName,0,sizeof(cFilePathName));
		sprintf(cFilePathName,"%s/Imagepath.txt",strDataImagPath.c_str());
		strNewPath=cFilePathName;//char 포인터값을 string에 대입
		vector<KuPose> Landmark=GlobalLocalizationSupervisor::getInstance()->loadImagePath(strNewPath);
		string strDataImagPixcelPath =KuRobotParameter::getInstance()->getTeachingPathNameNPath();
		memset(cFilePathName,0,sizeof(cFilePathName));
		sprintf(cFilePathName,"%s/Imagepixel.txt",strDataImagPixcelPath.c_str());
		strNewPath=cFilePathName;//char 포인터값을 string에 대입
		GlobalLocalizationSupervisor::getInstance()->loadImagePixcelPath(strNewPath);
		KuDrawingInfo::getInstance()->setvecLandmarkPos(Landmark);
		GlobalLocalizationSupervisor::getInstance()->loadDB(Landmark.size());	
	}

	//로봇 초기 위치를 불러온다-----------------------------------------------------------------------
	string strLastRobotpos  =KuRobotParameter::getInstance()->getLastRobotPoseNameNPath();
	int nUsingLastPose = KuRobotParameter::getInstance()->getUsingLastPose();
	KuPose LastRobotPos;
	if(nUsingLastPose==1)	LastRobotPos = GlobalLocalizationSupervisor::getInstance()->loadLastRobotPos(strLastRobotpos);
	else LastRobotPos = KuRobotParameter::getInstance()->getInitRobotPoseForNav();
	KuDrawingInfo::getInstance()->setRobotPos(LastRobotPos);

	// 	cvNamedWindow("Demo",0);
	// 	cvSetWindowProperty("Demo",CV_WND_PROP_FULLSCREEN,CV_WINDOW_FULLSCREEN);
	// 	cvResizeWindow("Demo", 1024, 768);

	return bRes;
}

/**
@brief Korean: 현재 수행중인  Behavior의 상태를 나타내는 함수
@brief English: 
*/
bool MobileSupervisor::getBehaviorStates(KuCommandMessage CMessage)
{
	bool bBehaviorStates=false;
	int nBehaviorName = CMessage.getBehaviorName(); //행위에 대한 이름을 인덱스로 가지고 온다.
	switch(nBehaviorName)
	{	
	case KuCommandMessage::HYBRID_MAP_BUILDING_BH:
		bBehaviorStates = m_MapBuildingBeh.getBehaviorStates();
		break;
	case KuCommandMessage::GOTOGOAL_BH:
		bBehaviorStates=m_GotoGoalBeh.getBehaviorStates();
		break;
	case KuCommandMessage::PATH_TEACHING_BH:
		bBehaviorStates=m_PathTeachingBh.getBehaviorStates();
		break;
	case KuCommandMessage::AUTONOMOUS_GOTOGOAL:
		bBehaviorStates=m_bAutoThread;
		break;
	case KuCommandMessage::GLOBAL_LOCALIZATION_BH:
		bBehaviorStates=m_GlobalLocalizationBh.getBehaviorStates();
		break;
	case KuCommandMessage::GLOBAL_MAP_BUILDING_BH:
		bBehaviorStates=m_GlobalMapBuildingBh.getBehaviorStates();
	default:break;
	case KuCommandMessage::MULTI_GOTOGOAL_BH:
		bBehaviorStates=MultiRobotSupervisor::getInstance()->getBehaviorStates();
		break;
	}

	return bBehaviorStates;
}

/**
@brief Korean: 사용자의 명령에 따라 behavior에세 명령을 내리는 함수
@brief English: 
*/
void MobileSupervisor::execute(KuCommandMessage CMessage)
{
	int nBehaviorName = CMessage.getBehaviorName(); //행위에 대한 이름을 인덱스로 가지고 온다.
	int nBehaviorPeriod = CMessage.getBehaviorPeriod();
	KuPose GoalPos = CMessage.getGoalPos();
	KuPose RobotPos = CMessage.getRobotPos();

	switch(nBehaviorName){	
	case KuCommandMessage::GOTOGOAL_BH:
		m_GotoGoalBeh.execute(CMessage);
		break;
	case KuCommandMessage::HYBRID_MAP_BUILDING_BH:
		m_MapBuildingBeh.execute(CMessage);	
		break;
	case KuCommandMessage::PATH_TEACHING_BH:
		m_PathTeachingBh.execute(CMessage);	
		break;
	case KuCommandMessage::AUTONOMOUS_GOTOGOAL:
		Autonomousexecute(CMessage);	
		break;
	case KuCommandMessage::GLOBAL_LOCALIZATION_BH:
		m_GlobalLocalizationBh.execute(CMessage);
		break;
	case KuCommandMessage::GLOBAL_MAP_BUILDING_BH:
		m_GlobalMapBuildingBh.execute(CMessage);
		break;
	case KuCommandMessage::MULTI_GOTOGOAL_BH:
		MultiRobotSupervisor::getInstance()->execute(CMessage);
		break;
	default:break;
	}
}

/**
@brief Korean: 현재 사용중인  Behavior의 Localizer를 나타내는 함수
@brief English: 
*/
Localizer* MobileSupervisor::getLocalizer(KuCommandMessage CMessage)
{
	Localizer* pLocalizer=NULL;
	int nBehaviorName = CMessage.getBehaviorName(); //행위에 대한 이름을 인덱스로 가지고 온다.

	switch(nBehaviorName){

	case KuCommandMessage::GOTOGOAL_BH:
		pLocalizer=m_GotoGoalBeh.getLocalizer();
		break;
	case KuCommandMessage::HYBRID_MAP_BUILDING_BH:
		pLocalizer=m_MapBuildingBeh.getLocalizer();	
		break;
	case KuCommandMessage::PATH_TEACHING_BH:
		pLocalizer=m_PathTeachingBh.getLocalizer();	
		break;
	case KuCommandMessage::AUTONOMOUS_GOTOGOAL:
		pLocalizer=m_GotoGoalBeh.getLocalizer();
		break;
	case KuCommandMessage::GLOBAL_LOCALIZATION_BH:
		pLocalizer=m_GlobalLocalizationBh.getLocalizer();
		break;
	case KuCommandMessage::GLOBAL_MAP_BUILDING_BH:
		pLocalizer=m_GlobalMapBuildingBh.getLocalizer();
		break;
	case KuCommandMessage::MULTI_GOTOGOAL_BH:
		pLocalizer=MultiRobotSupervisor::getInstance()->getLocalizer();
		break;
	default:break;
	}

	return pLocalizer;
}

/**
@brief Korean: Autonomous를 실행시키는 함수
@brief English: 
*/
bool MobileSupervisor::Autonomousexecute(KuCommandMessage CMessage )
{

	switch(CMessage.getCommandName()){

	case KuCommandMessage::START_THREAD:	
		if(m_bAutoThread==false)
		{
			//SensorSupervisor::getInstance()->executeSwitch( );
			//m_binitflag=true;

			// ISSAC communication start
			if(KuRobotParameter::getInstance()->getUsingISSAC() == 1) // 0: ISSAC 미사용, 1: ISSAC 사용
			{
				connectCommunication();

				Sleep(6000); // 서버에 연결될 때까지 기다림

				// Sound
				if(TotalTcpipCommunication::getInstance()->isISSACConnected() == true) // 연결됨
				{
					string strNameNPath;
					strNameNPath=KuRobotParameter::getInstance()->getMovieNameNPath();

					WCHAR wcharFilePathName[100];
					stringstream ss;
					ss << strNameNPath << "/issac_connected.wav";

					MultiByteToWideChar(0,0,ss.str().c_str(),100,wcharFilePathName,100);
					LPCWSTR lpcwstrFilePathName=wcharFilePathName;
					PlaySound(lpcwstrFilePathName,NULL, SND_SYNC);
				}
				else
				{
					string strNameNPath;
					strNameNPath=KuRobotParameter::getInstance()->getMovieNameNPath();

					WCHAR wcharFilePathName[100];
					stringstream ss;
					ss << strNameNPath << "/stand-alone_mode.wav";

					MultiByteToWideChar(0,0,ss.str().c_str(),100,wcharFilePathName,100);
					LPCWSTR lpcwstrFilePathName=wcharFilePathName;
					PlaySound(lpcwstrFilePathName,NULL, SND_SYNC);
				}

				// Job list 요청
				Clientpart::getInstance()->setJobListReq(true);

				// Job list를 수신할때까지 기다림
				if(TotalTcpipCommunication::getInstance()->isISSACConnected() == true)
				{
					while(!Serverpart::getInstance()->isJobListReceivedOnce())
					{
						Sleep(100);
					}

					ifstream file_last_job("./data/ISSAC/last_job.txt");
					string sJobID;
					file_last_job >> sJobID;
					XmldataSetting::getInstance()->selectJob(sJobID);
				}
			}
			else
			{
				//SwitchInterface::getInstance()->initLampflag(); // Switch 그린라이트를 켬
			}

			//SwitchInterface::getInstance()->initLampflag(); // Switch 그린라이트를 켬

			//wait to signal 
			m_PreGoalPose=KuDrawingInfo::getInstance()->getGoalPos();
			m_PreRobotPose=KuDrawingInfo::getInstance()->getRobotPos();

			m_bAutoThread=true;
			m_bAvoidMode=CMessage.getAvoidMode();
			m_KuThread.start(AutonomousSupervisor,this,100, "AutonomousSupervisor main thread"); //메인 스레드 시작

			char cData[300]; 
			memset(cData,0,sizeof(cData));
			string strDataPath = KuRobotParameter::getInstance()->getTeachingPathNameNPath();
			sprintf(cData,"%s/RobotTrajectory.log",strDataPath.c_str());
			m_DataLog.open(cData);
		}
		break;
	case KuCommandMessage::TERMINATE_THREAD:
		{
			m_DataLog.close();
			m_KuThread.terminate();
			KuCommandMessage CMessage;
			CMessage.setCommandName(KuCommandMessage::TERMINATE_THREAD);
			CMessage.setBehaviorName(KuCommandMessage::GOTOGOAL_BH);
			CMessage.setGoalPos(KuDrawingInfo::getInstance()->getGoalPos());
			CMessage.setRobotPos(KuDrawingInfo::getInstance()->getRobotPos());	
			MobileSupervisor::getInstance()->execute(CMessage);
			m_bAutoThread=false;
		}
		break;
	case KuCommandMessage::SUSPEND_THREAD:
		m_KuThread.suspend();
		break;
	case KuCommandMessage::RESUME_THREAD:
		m_KuThread.resume();
		break;
	default:break;
	}

	return true;
}
/**
@brief Korean: 정해져 있는 구간간의 이동을 명령하는 함수.
@brief English: 
*/

void MobileSupervisor::AutonomousSupervisor(void* arg)
{
	char cFilePathName[300];
	string sDataPath;

	// ISSAC /////////////////////////////////////////////////////////////////////////
	int nRepeatIdx = XmldataSetting::getInstance()->getCompleteCount(); // 현재까지 job 반복 횟수
	// 	int nMode = XmldataSetting::getInstance()->getMode();
	int nRepeat = XmldataSetting::getInstance()->getRepeatCount(); // 총 job 반복 횟수
	int nJobInfoPrev = XmldataSetting::getInstance()->getJobStatus();
	int nJobNumISSAC = XmldataSetting::getInstance()->getJobList().size();
	bool bJobCompleteSent(false);
	string sPathIDISSAC = XmldataSetting::getInstance()->getCurrentPathID();
//	string sJobCurrentID = XmldataSetting::getInstance()->getCurrentJobID();

	ifstream path_file;
	sprintf(cFilePathName, "./data/path/%s.txt", sPathIDISSAC.c_str());
	sDataPath=cFilePathName;
	path_file.open(sDataPath);

	if(path_file.is_open()) // path file이 존재하는지 확인
	{
		path_file.close();

		while(nRepeatIdx < nRepeat || nRepeat == -1) // ISSAC
		{
			MobileSupervisor* pMS = (MobileSupervisor*)arg;
			vector<vector<PBlock>> vecGlobalPath; // global path
			SwitchInterface::getInstance()->setState(0);
			Sleep(200);
			int nSwitch = SensorSupervisor::getInstance()->getSwitchState();
			sPathIDISSAC = XmldataSetting::getInstance()->getCurrentPathID();

			KuPathBlockPr PB;
			vector<PBlock> vecPath;
			vecPath.clear();

			// ISSAC //////////////////////////////////////////////////////////////////
			int nJobStatus = XmldataSetting::getInstance()->getJobStatus();
			nRepeat = XmldataSetting::getInstance()->getRepeatCount(); // 반복횟수 업데이트
			bJobCompleteSent = false;

			if(nJobStatus == XmldataSetting::JOB_RUN && nJobInfoPrev != XmldataSetting::JOB_RUN) // 새로운 job이 주어질 경우
			{
				nRepeatIdx = 0; // 카운트 초기화
			}
			nJobInfoPrev = nJobStatus;

			bool bRun(true);

			// ISSAC /////////////////////////////////////////////////////////////////////////
			if(KuRobotParameter::getInstance()->getUsingISSAC() == 1) // 0: ISSAC 미사용, 1: ISSAC 사용
			{
				/*
				if(TotalTcpipCommunication::getInstance()->failedToSendAliveSignal() == true) // ISSAC과 통신이 안될 때
				{
					bRun = true;
				}
				else
				{
					if(nJobStatus != XmldataSetting::JOB_RUN &&
						nJobStatus != XmldataSetting::JOB_RESUME)
					{
						bRun = false;
					}
					else
					{
						bRun = true;
					}
				}
				*/

				bRun = true;
			}
			// ISSAC /////////////////////////////////////////////////////////////////////////

			if(/*SensorSupervisor::getInstance()->getSwitchState() > 0 && nSwitch <= nPathNum  && */bRun)
			{
				/////////////////////리모트 신호에 따른 경로////////////////////////
				vecGlobalPath.clear();
				SwitchInterface::getInstance()->setState(0);
				int nFileIdx;

				// ISSAC //////////////////
				if(KuRobotParameter::getInstance()->getUsingISSAC() == 1) // 0: ISSAC 미사용, 1: ISSAC 사용
				{
					// 					nFileIdx = sPathIDISSAC;
				}
				// ISSAC //////////////////
				else
				{
					nFileIdx = nSwitch;
				}

				// 				sprintf(cFilePathName, "./data/path/LoadPathBlock%d.txt", nFileIdx);

				if(sPathIDISSAC != "")
				{
					sprintf(cFilePathName, "./data/path/%s.txt", sPathIDISSAC.c_str());
					sDataPath = cFilePathName;//char 포인터값을 string에 대입
					vecPath = PB.loadPathBlock(sDataPath);
					KuPathBlockPlannerPr::getInstance()->initialize(vecPath);

					// Initial robot pose
					KuPose poseRobotInit = KuRobotParameter::getInstance()->getInitRobotPoseForNav();
					if(vecPath.size() > 0)
					{
						poseRobotInit.setXm(vecPath[0].x);
						poseRobotInit.setYm(vecPath[0].y);
						poseRobotInit.setThetaRad(KuDrawingInfo::getInstance()->getRobotPos().getThetaRad());
					}
					KuDrawingInfo::getInstance()->setRobotPos(poseRobotInit);
				}

				// ISSAC /////////////////////////////////////////////////////////////////////////
				if(KuRobotParameter::getInstance()->getUsingISSAC() == 1) // 0: ISSAC 미사용, 1: ISSAC 사용
				{
					if(vecPath.size() > 0)
					{
						vecPath[vecPath.size() - 1].waypoint = 0; // 마지막 block의 waypoint 제거
						vecGlobalPath.push_back(vecPath); // 다른 처리 없이 그대로의 block 리스트를 사용
					}
				}
				else
				{
					KuPathBlockPlannerPr::getInstance()->generatePathBlock(KuDrawingInfo::getInstance()->getRobotPos());
					KuPathBlockPlannerPr::getInstance()->getGlobalPath(vecGlobalPath);
				}

				vecPath.clear();

				for(int i=0; i<vecGlobalPath.size(); i++)
				{
					SwitchInterface::getInstance()->setState(0);
					if(KuPathBlockPlannerPr::getInstance()->getReversePathflag()) continue;//회기경로 flag가 true이면 무시
					KuPathBlockPlannerPr::getInstance()->setSequencePathblock(vecGlobalPath[i]);
					pMS->m_GotoGoalBeh.initBlockPath(vecGlobalPath[i]);
					MobileSupervisor::getInstance()->setBehaviorStates(true);
					KuCommandMessage CMessage;
					CMessage.setAvoidMode(MobileSupervisor::getInstance()->m_bAvoidMode);
					CMessage.setCommandName(KuCommandMessage::START_THREAD);
					CMessage.setBehaviorName(KuCommandMessage::GOTOGOAL_BH);

					//CMessage.setBehaviorName(KuCommandMessage::MULTI_GOTOGOAL_BH);

					CMessage.setGoalPos(KuDrawingInfo::getInstance()->getGoalPos());
					CMessage.setRobotPos(KuDrawingInfo::getInstance()->getRobotPos());	
					MobileSupervisor::getInstance()->execute(CMessage);

					while(MobileSupervisor::getInstance()->getBehaviorStates(CMessage))
					{
						//if(KuDrawingInfo::getInstance()->getDirectionofPathflag()==true)
						//MobileSupervisor::getInstance()->waitWayPoint(MultiRobotSupervisor::getInstance()->m_GotoGoalBeh.getCurWayPoint());
						MobileSupervisor::getInstance()->saveRobotTrajectory();
						Sleep(100);
					}
				}

				SwitchInterface::getInstance()->setState(0);

				if(!(KuPathBlockPlannerPr::getInstance()->getReversePathflag()))
				{
					//SensorSupervisor::getInstance()->initLampflag();
				}

				// ISSAC /////////////////////////////////////////////////////////////////////////
				nRepeatIdx++;

				Clientpart::getInstance()->SetCompleteCount(nRepeatIdx); // 프로그램을 정상적으로 종료할 때 GotoGoalBehavior thread가 종료되면서 이 부분이 실행됨.
				// 비정상적으로 nRepeatIdx가 증가하여 SetCompleteCount() 함수가 정보를 파일에 저장함.
				Clientpart::getInstance()->SetState(Clientpart::STATE_STOP);

				if(XmldataSetting::getInstance()->getJobStatus() == XmldataSetting::JOB_STOP)
				{
					break;
				}
				// ISSAC /////////////////////////////////////////////////////////////////////////
			}

			SwitchInterface::getInstance()->setState(0);

			vecGlobalPath.clear();
		}

		// ISSAC /////////////////////////////////////////////////////////////////////////
		if(nRepeatIdx > 0)
		{
			// 현재 job을 수행하는 동안 새로운 job이 수신되었는지 판단
			if(1)//nJobNumISSAC == XmldataSetting::getInstance()->getJobList().size()) // 새로운 job이 없다면
			{
				if(bJobCompleteSent == false)
				{
					// Log
					stringstream ss;
					string sJobID;
					string sPathID = XmldataSetting::getInstance()->getJobID_PathID(sJobID);
					ss << "Completed the job " << sJobID << " (path " <<  sPathID << ")";
					ANS_LOG_WRITE(ss.str());

	//				XmldataSetting::getInstance()->selectNextJob();

					Clientpart::getInstance()->setJobComplete();

					bJobCompleteSent = true;

					ss.clear();
					sJobID.clear();
					sPathID.clear();
				}
			}
			else // Path가 변경되었을 경우(Job change)
			{
				// Log
				stringstream ss;
				string sJobIDCurr;
				string sPathIDCurr = XmldataSetting::getInstance()->getJobID_PathID(sJobIDCurr);
				ss << "Job change: " << sJobIDCurr << " (path " <<  sPathIDISSAC << " -> " << sPathIDCurr << ")";
				ANS_LOG_WRITE(ss.str());

				ss.clear();
				sJobIDCurr.clear();
				sPathIDCurr.clear();

				Clientpart::getInstance()->setJobPause(true);
			}

			XmldataSetting::getInstance()->setJobStatus(XmldataSetting::JOB_COMPLETE);
			printf("Job complete (%d/%d)\n", nRepeatIdx, nRepeat);
		}

		SwitchInterface::getInstance()->setState(0);
		// ISSAC /////////////////////////////////////////////////////////////////////////
	}
	else
	{
		printf("[MobileSupervisor] Unable to open the path file: %s\n", cFilePathName);
	}

	XmldataSetting::getInstance()->selectNextJob(); // 현재 job 반복이 끝나면 다음 job으로 변경

	sDataPath.clear();
	sPathIDISSAC.clear();
}

void MobileSupervisor::saveRobotTrajectory()
{		
	KuPose RobotPos= KuDrawingInfo::getInstance()->getRobotPos();
	m_DataLog<<RobotPos.getX()<<" "<<RobotPos.getY()<<" "<<RobotPos.getThetaDeg()<<";"<<endl;
}
/**
@brief Korean: 현재 수행중인  Behavior의 상태를 나타내는 함수
@brief English: 
*/
bool MobileSupervisor::getBehaviorStates( )
{
	m_bBehaviorStates=m_GotoGoalBeh.getBehaviorStates();
	return m_bBehaviorStates;
}
/**
@brief Korean: 현재 수행중인  Behavior의 상태를 나타내는 함수
@brief English: 
*/
void MobileSupervisor::setBehaviorStates( bool bBehaviorStates)
{
	m_bBehaviorStates=bBehaviorStates;
}

void MobileSupervisor::setTotalTime(int nTotalFrame )
{
	m_GotoGoalBeh.setWaitTime(nTotalFrame);
}

/**
@brief Korean: goal 지점에서 얼마나 경유할지 구하는 함수
@brief English: 
*/
double MobileSupervisor::waitWayPoint(KuPose wayPoint)
{
	int nWayPointIdx=0;
	double dWayPointX;
	double dWayPointY;
	double dDist=10000000;

	int nWayPointSize = m_vecWayPoint.size();

	KuPose selectPos;

	while( true ) {

		if(nWayPointIdx >= nWayPointSize) {
			nWayPointIdx = nWayPointSize-1; //배열인덱스 이기 떄문에 1을 빼줘야 한다.			
			return 1000;
		}

		dWayPointX = m_vecWayPoint[nWayPointIdx].getX();
		dWayPointY = m_vecWayPoint[nWayPointIdx].getY();

		dDist=hypot(wayPoint.getX()-dWayPointX, wayPoint.getY()- dWayPointY);

		if(dDist<100&&wayPoint.getX()!=0&&wayPoint.getY()!=0)
		{
			//string strNameNPath;
			//strNameNPath=KuRobotParameter::getInstance()->getMovieNameNPath();		 
			//playMovie(strNameNPath, nWayPointIdx );	
			return 1000*m_vecWayPoint[nWayPointIdx].getPro();
		}	

		nWayPointIdx++;
	}
}

/**
@brief Korean: goal 지점에서 얼마나 경유할지 구하는 함수
@brief English: 
*/
double MobileSupervisor::checkWayPoint(KuPose GoalPos)
{

	int nWayPointIdx=0;
	double dWayPointX;
	double dWayPointY;
	double dDist=10000000;

	int nWayPointSize = m_vecWayPoint.size();

	KuPose selectPos;

	while( true ) {

		if(nWayPointIdx >= nWayPointSize) {
			nWayPointIdx = nWayPointSize-1; //배열인덱스 이기 떄문에 1을 빼줘야 한다.
			return 1000;
		}
		dWayPointX = m_vecWayPoint[nWayPointIdx].getX();
		dWayPointY = m_vecWayPoint[nWayPointIdx].getY();

		dDist=hypot(GoalPos.getX()-dWayPointX, GoalPos.getY()- dWayPointY);

		if(dDist<200)
		{
			string strNameNPath;

			strNameNPath=KuRobotParameter::getInstance()->getMovieNameNPath();		 
			playMovie(strNameNPath, nWayPointIdx );			

			return 1000*m_vecWayPoint[nWayPointIdx].getPro();
		}	

		nWayPointIdx++;
	}

}

bool MobileSupervisor::playMovie(string strNameNPath,int  nWayPointIdx)
{


	char cFilePathName[100];	
	char cFilePathName2[100];	
	WCHAR wcharFilePathName[100];
	double fps;
	IplImage* frame;

	sprintf_s(cFilePathName,"%s/movie_%d.wmv",strNameNPath.c_str(), nWayPointIdx);
	sprintf_s(cFilePathName2,"%s/movie_%d.wav",strNameNPath.c_str(), nWayPointIdx);


	//	CvCapture* capture = cvCreateFileCapture(cFilePathName);
	CvCapture* capture = cvCaptureFromAVI(cFilePathName);
	fps = cvGetCaptureProperty(capture, CV_CAP_PROP_FPS);//초당프레임
	int nTotalFrame= (int)cvGetCaptureProperty(capture,CV_CAP_PROP_FRAME_COUNT);

	setTotalTime(nTotalFrame*fps);
	//CvVideoWriter* writer ; 
	MultiByteToWideChar(0,0,cFilePathName2,100,wcharFilePathName,100);
	LPCWSTR lpcwstrFilePathName=wcharFilePathName;
	PlaySound(lpcwstrFilePathName,NULL, SND_SYNC);
	int nframeCnt=0;

	while(1)
	{
		cvGrabFrame(capture);

		frame = cvRetrieveFrame(capture);
		fps = cvGetCaptureProperty(capture, CV_CAP_PROP_FPS);//초당프레임
		nframeCnt++;

		if(!frame||nframeCnt>nTotalFrame){
			PlaySound(0,0, 0);
			break;
		}
		cvShowImage("Demo", frame);
		char chKey = cvWaitKey(fps);
		if( chKey == 27) break;
	}

	cvReleaseCapture(&capture);

	//cvDestroyAllWindows();


	return true;
}

/**
@brief Korean: 현재 위치 저장 스레드 시작
@brief English: 
*/
void MobileSupervisor::startSavingPath()
{
	GlobalLocalizationSupervisor::getInstance()->startSavingPathThrerad();
}

/**
@brief Korean: 현재 위치 저장 스레드 종료
@brief English: 
*/
void MobileSupervisor::terminateSavingPath()
{
	GlobalLocalizationSupervisor::getInstance()->terminateSavingPathThread();

}

/**
@brief Korean: 
@brief English: 
*/
bool MobileSupervisor::connectCommunication()
{
	return MultiRobotSupervisor::getInstance()->connectCommunication();
}

/**
@brief Korean: MapBuildingBehavior 포인터를 리턴
@brief English: 
*/
MapBuildingBehavior* MobileSupervisor::getMapBuildingBehavior(void)
{
	return &m_MapBuildingBeh;
}
