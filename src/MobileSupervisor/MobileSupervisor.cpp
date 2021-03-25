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
@brief Korean: �ʱ����� ������ �ҷ����� �Լ�
@brief English: 
*/
bool MobileSupervisor::loadMap()
{
	//������ �ҷ� �´�--------------------------------------------------------------------------
	string strMapNameNPath = KuRobotParameter::getInstance()->getMapNameNPath();
	bool bRes = kuMapRepository::getInstance()->loadMap(strMapNameNPath); ///���� �ε�

	//�ӵ� ������ �ҷ� �´�--------------------------------------------------------------------------
	strMapNameNPath = KuRobotParameter::getInstance()->getVelocityMapNameNPath();
	kuMapRepository::getInstance()->loadVelocityMap(strMapNameNPath); ///���� �ε�

	//ƼĪ�� ��θ� �ҷ� �´�--------------------------------------------------------------------------
	string strDataPath =KuRobotParameter::getInstance()->getTeachingPathNameNPath();
	string strNewPath;
	char cFilePathName[300];
	memset(cFilePathName,0,sizeof(cFilePathName));
	sprintf(cFilePathName,"%s/path.txt",strDataPath.c_str());
	strNewPath=cFilePathName;//char �����Ͱ��� string�� ����
	KuTeachingPathPlannerPr m_KuTeachingPathPlannerPr;
	list <KuPose> Pathlist=m_KuTeachingPathPlannerPr.loadPath(strNewPath);
	KuDrawingInfo::getInstance()->setPath(Pathlist);

	//ƼĪ�� �������� �ҷ� �´�--------------------------------------------------------------------------
	int nAGVID=KuRobotParameter::getInstance()->getRobotID();
	string strDataWaypoint =KuRobotParameter::getInstance()->getTeachingPathNameNPath();
	memset(cFilePathName,0,sizeof(cFilePathName));
	sprintf(cFilePathName,"%s/waypoint_%d.txt",strDataPath.c_str(),nAGVID);
	strNewPath=cFilePathName;//char �����Ͱ��� string�� ����
	list <KuPose> WayPointList=m_KuTeachingPathPlannerPr.loadWayPointList(strNewPath);
	list<KuPose>::iterator itway;
	for(itway=WayPointList.begin(); itway!=WayPointList.end(); itway++){
		m_vecWayPoint.push_back(*itway);
	}
	KuDrawingInfo::getInstance()->setWayPointList(WayPointList);	

	//zone Map �ҷ� �´�--------------------------------------------------------------------------
	// 	strNewPath= KuRobotParameter::getInstance()->getZoneMapNameNPath();
	// 	MultiRobotSupervisor::getInstance()->loadZoneMap(strNewPath);

	vector<CLAMPData> FRData= KuILBPFLocalizerPr::getInstance()->loadFeatureData( );
	KuDrawingInfo::getInstance()->setRegion(FRData);

	vector<CFREAKData> FTData= KuILBPFLocalizerPr::getInstance()->loadTemplateData( );
	KuDrawingInfo::getInstance()->setTemplateData(FTData);

	KuFiducialbasedLocalizerPr::getInstance()->loadFiducialFeatureMap( );
	list<KuPose> FiducialMarkList = KuFiducialbasedLocalizerPr::getInstance()->getFiducialMarkList();
	KuDrawingInfo::getInstance()->setFiducialMarkList(FiducialMarkList);

	//������ġ �Ķ���͸� �ҷ� �´�--------------------------------------------------------------------------
	if(KuRobotParameter::getInstance()->getGlobalLocalization()=="yes") {		
		string strDataImagPath =KuRobotParameter::getInstance()->getTeachingPathNameNPath();
		memset(cFilePathName,0,sizeof(cFilePathName));
		sprintf(cFilePathName,"%s/Imagepath.txt",strDataImagPath.c_str());
		strNewPath=cFilePathName;//char �����Ͱ��� string�� ����
		vector<KuPose> Landmark=GlobalLocalizationSupervisor::getInstance()->loadImagePath(strNewPath);
		string strDataImagPixcelPath =KuRobotParameter::getInstance()->getTeachingPathNameNPath();
		memset(cFilePathName,0,sizeof(cFilePathName));
		sprintf(cFilePathName,"%s/Imagepixel.txt",strDataImagPixcelPath.c_str());
		strNewPath=cFilePathName;//char �����Ͱ��� string�� ����
		GlobalLocalizationSupervisor::getInstance()->loadImagePixcelPath(strNewPath);
		KuDrawingInfo::getInstance()->setvecLandmarkPos(Landmark);
		GlobalLocalizationSupervisor::getInstance()->loadDB(Landmark.size());	
	}

	//�κ� �ʱ� ��ġ�� �ҷ��´�-----------------------------------------------------------------------
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
@brief Korean: ���� ��������  Behavior�� ���¸� ��Ÿ���� �Լ�
@brief English: 
*/
bool MobileSupervisor::getBehaviorStates(KuCommandMessage CMessage)
{
	bool bBehaviorStates=false;
	int nBehaviorName = CMessage.getBehaviorName(); //������ ���� �̸��� �ε����� ������ �´�.
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
@brief Korean: ������� ��ɿ� ���� behavior���� ����� ������ �Լ�
@brief English: 
*/
void MobileSupervisor::execute(KuCommandMessage CMessage)
{
	int nBehaviorName = CMessage.getBehaviorName(); //������ ���� �̸��� �ε����� ������ �´�.
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
@brief Korean: ���� �������  Behavior�� Localizer�� ��Ÿ���� �Լ�
@brief English: 
*/
Localizer* MobileSupervisor::getLocalizer(KuCommandMessage CMessage)
{
	Localizer* pLocalizer=NULL;
	int nBehaviorName = CMessage.getBehaviorName(); //������ ���� �̸��� �ε����� ������ �´�.

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
@brief Korean: Autonomous�� �����Ű�� �Լ�
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
			if(KuRobotParameter::getInstance()->getUsingISSAC() == 1) // 0: ISSAC �̻��, 1: ISSAC ���
			{
				connectCommunication();

				Sleep(6000); // ������ ����� ������ ��ٸ�

				// Sound
				if(TotalTcpipCommunication::getInstance()->isISSACConnected() == true) // �����
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

				// Job list ��û
				Clientpart::getInstance()->setJobListReq(true);

				// Job list�� �����Ҷ����� ��ٸ�
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
				//SwitchInterface::getInstance()->initLampflag(); // Switch �׸�����Ʈ�� ��
			}

			//SwitchInterface::getInstance()->initLampflag(); // Switch �׸�����Ʈ�� ��

			//wait to signal 
			m_PreGoalPose=KuDrawingInfo::getInstance()->getGoalPos();
			m_PreRobotPose=KuDrawingInfo::getInstance()->getRobotPos();

			m_bAutoThread=true;
			m_bAvoidMode=CMessage.getAvoidMode();
			m_KuThread.start(AutonomousSupervisor,this,100, "AutonomousSupervisor main thread"); //���� ������ ����

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
@brief Korean: ������ �ִ� �������� �̵��� ����ϴ� �Լ�.
@brief English: 
*/

void MobileSupervisor::AutonomousSupervisor(void* arg)
{
	char cFilePathName[300];
	string sDataPath;

	// ISSAC /////////////////////////////////////////////////////////////////////////
	int nRepeatIdx = XmldataSetting::getInstance()->getCompleteCount(); // ������� job �ݺ� Ƚ��
	// 	int nMode = XmldataSetting::getInstance()->getMode();
	int nRepeat = XmldataSetting::getInstance()->getRepeatCount(); // �� job �ݺ� Ƚ��
	int nJobInfoPrev = XmldataSetting::getInstance()->getJobStatus();
	int nJobNumISSAC = XmldataSetting::getInstance()->getJobList().size();
	bool bJobCompleteSent(false);
	string sPathIDISSAC = XmldataSetting::getInstance()->getCurrentPathID();
//	string sJobCurrentID = XmldataSetting::getInstance()->getCurrentJobID();

	ifstream path_file;
	sprintf(cFilePathName, "./data/path/%s.txt", sPathIDISSAC.c_str());
	sDataPath=cFilePathName;
	path_file.open(sDataPath);

	if(path_file.is_open()) // path file�� �����ϴ��� Ȯ��
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
			nRepeat = XmldataSetting::getInstance()->getRepeatCount(); // �ݺ�Ƚ�� ������Ʈ
			bJobCompleteSent = false;

			if(nJobStatus == XmldataSetting::JOB_RUN && nJobInfoPrev != XmldataSetting::JOB_RUN) // ���ο� job�� �־��� ���
			{
				nRepeatIdx = 0; // ī��Ʈ �ʱ�ȭ
			}
			nJobInfoPrev = nJobStatus;

			bool bRun(true);

			// ISSAC /////////////////////////////////////////////////////////////////////////
			if(KuRobotParameter::getInstance()->getUsingISSAC() == 1) // 0: ISSAC �̻��, 1: ISSAC ���
			{
				/*
				if(TotalTcpipCommunication::getInstance()->failedToSendAliveSignal() == true) // ISSAC�� ����� �ȵ� ��
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
				/////////////////////����Ʈ ��ȣ�� ���� ���////////////////////////
				vecGlobalPath.clear();
				SwitchInterface::getInstance()->setState(0);
				int nFileIdx;

				// ISSAC //////////////////
				if(KuRobotParameter::getInstance()->getUsingISSAC() == 1) // 0: ISSAC �̻��, 1: ISSAC ���
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
					sDataPath = cFilePathName;//char �����Ͱ��� string�� ����
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
				if(KuRobotParameter::getInstance()->getUsingISSAC() == 1) // 0: ISSAC �̻��, 1: ISSAC ���
				{
					if(vecPath.size() > 0)
					{
						vecPath[vecPath.size() - 1].waypoint = 0; // ������ block�� waypoint ����
						vecGlobalPath.push_back(vecPath); // �ٸ� ó�� ���� �״���� block ����Ʈ�� ���
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
					if(KuPathBlockPlannerPr::getInstance()->getReversePathflag()) continue;//ȸ���� flag�� true�̸� ����
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

				Clientpart::getInstance()->SetCompleteCount(nRepeatIdx); // ���α׷��� ���������� ������ �� GotoGoalBehavior thread�� ����Ǹ鼭 �� �κ��� �����.
				// ������������ nRepeatIdx�� �����Ͽ� SetCompleteCount() �Լ��� ������ ���Ͽ� ������.
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
			// ���� job�� �����ϴ� ���� ���ο� job�� ���ŵǾ����� �Ǵ�
			if(1)//nJobNumISSAC == XmldataSetting::getInstance()->getJobList().size()) // ���ο� job�� ���ٸ�
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
			else // Path�� ����Ǿ��� ���(Job change)
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

	XmldataSetting::getInstance()->selectNextJob(); // ���� job �ݺ��� ������ ���� job���� ����

	sDataPath.clear();
	sPathIDISSAC.clear();
}

void MobileSupervisor::saveRobotTrajectory()
{		
	KuPose RobotPos= KuDrawingInfo::getInstance()->getRobotPos();
	m_DataLog<<RobotPos.getX()<<" "<<RobotPos.getY()<<" "<<RobotPos.getThetaDeg()<<";"<<endl;
}
/**
@brief Korean: ���� ��������  Behavior�� ���¸� ��Ÿ���� �Լ�
@brief English: 
*/
bool MobileSupervisor::getBehaviorStates( )
{
	m_bBehaviorStates=m_GotoGoalBeh.getBehaviorStates();
	return m_bBehaviorStates;
}
/**
@brief Korean: ���� ��������  Behavior�� ���¸� ��Ÿ���� �Լ�
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
@brief Korean: goal �������� �󸶳� �������� ���ϴ� �Լ�
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
			nWayPointIdx = nWayPointSize-1; //�迭�ε��� �̱� ������ 1�� ����� �Ѵ�.			
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
@brief Korean: goal �������� �󸶳� �������� ���ϴ� �Լ�
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
			nWayPointIdx = nWayPointSize-1; //�迭�ε��� �̱� ������ 1�� ����� �Ѵ�.
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
	fps = cvGetCaptureProperty(capture, CV_CAP_PROP_FPS);//�ʴ�������
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
		fps = cvGetCaptureProperty(capture, CV_CAP_PROP_FPS);//�ʴ�������
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
@brief Korean: ���� ��ġ ���� ������ ����
@brief English: 
*/
void MobileSupervisor::startSavingPath()
{
	GlobalLocalizationSupervisor::getInstance()->startSavingPathThrerad();
}

/**
@brief Korean: ���� ��ġ ���� ������ ����
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
@brief Korean: MapBuildingBehavior �����͸� ����
@brief English: 
*/
MapBuildingBehavior* MobileSupervisor::getMapBuildingBehavior(void)
{
	return &m_MapBuildingBeh;
}
