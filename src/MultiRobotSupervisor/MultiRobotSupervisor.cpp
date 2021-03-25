#include "stdafx.h"
#include "MultiRobotSupervisor.h"

MultiRobotSupervisor::MultiRobotSupervisor()
{
	m_dTotalTime=0;
	m_bDetourPathGenerated=false;
	m_pPbstacleMap=NULL;
	m_bWaitAGVflag=true;
	m_bThreadFlag=false;
	m_clientFlag=false;

}

MultiRobotSupervisor::~MultiRobotSupervisor()
{

}

/**
@brief Korean: Thread 돌리는 함수
*/
void MultiRobotSupervisor::Start(int nTime)
{
	m_dTotalTime=0;
	m_nThreadTime=nTime;
	m_bDetourPathGenerated=false;
	m_bgeneraedpath=false;
	m_bWaitAGVflag=true;

	for(int i=0; i<AGV_NUM;i++)
	{
		m_AGVTime[i]=0;
	}
	if(m_bThreadFlag==false)
	{
		m_bThreadFlag=true;
		m_KuThread.start(doThread,this,m_nThreadTime, "MultiRobotSupervisor::Start()"); //메인 스레드 시작			

	}

}

/**
@brief Korean: Thread
*/
void MultiRobotSupervisor::doThread(void* arg)
{
	MultiRobotSupervisor* pMRS = (MultiRobotSupervisor*)arg;

	KuPose RobotPos1;
	KuPose RobotPos2;

	//실제 AGV 위치 값을 받음------------------------------------------------
	pMRS->updateRobotPose();
	//실제 AGV 위치 값을 받음=================================================

	KuMap* pMap=kuMapRepository::getInstance()->getMap();//KuZoneControlPr::getInstance()->getZoneMap();//zone map 가져오는거 

	int nAGVID = KuRobotParameter::getInstance()->getRobotID()-1;

	if(pMRS->m_GotoGoalBeh.getBehaviorStates()==false)
	{
		for(int i=0; i<AGV_NUM;i++)
		{
			pMRS->m_AGVTime[i]=0;
		}
	}

	for(int nID=0;nID<AGV_NUM;nID++ )
	{
		if(nAGVID==nID) continue;

		RobotPos1= KuDrawingInfo::getInstance()->getRobotPos();//자기 위치
		RobotPos2 = pMRS->m_AGVRobotPos[nID];//통신위치

		int nTVel1 = pMRS->m_GotoGoalBeh.getTransVel();//자신의 속도
		int nRVel1 = pMRS->m_GotoGoalBeh.getRotVel();//자신의 속도

		int nTVel2 = MultiRobotActuatorInterface::getInstance()->getTransVel(nID);
		int nRVel2 = MultiRobotActuatorInterface::getInstance()->getRotVel(nID);

		if(pMRS->checkBetweenAGVPose(RobotPos1,RobotPos2, nTVel1,nRVel1,nTVel2,nRVel2)&&pMRS->m_AGVTime[nID]<DEADLOCKTIME)
		{
			if (nAGVID >nID) //m_AGV2일 때만 if문 실행(수정)
			{
				SSAGVWheelActuatorInterface::getInstance()->stop();
				KuDrawingInfo::getInstance()->setWaitforDiffAGV(true);
			}
			else
			{
				KuDrawingInfo::getInstance()->setWaitforDiffAGV(false);
			}

			if(pMRS->checkDeadAGV(nID))
			{
				list<KuPose> CurPathlist = KuDrawingInfo::getInstance()->getPath();
				RobotPos1 = KuDrawingInfo::getInstance()->getRobotPos();
				RobotPos2 = pMRS->m_AGVRobotPos[nID];
				KuDrawingInfo::getInstance()->setWaitforDiffAGV(true);
				Sleep(1000);
				bool bcheck=false;

				list<KuPose>  DetourPathList = pMRS->generateDetourPath(pMap, RobotPos1, RobotPos2, CurPathlist,&bcheck);//우회경로 만드는 함수
	
				if(bcheck==true)
				{
					pMRS->m_GotoGoalBeh.initPath(DetourPathList);
					KuDrawingInfo::getInstance()->setPath(DetourPathList);	
					KuDrawingInfo::getInstance()->setWaitforDiffAGV(false);
					Sleep(1000);					
				}
			}
		}
		else
		{
			for(int i=0; i<AGV_NUM;i++)
			{
				pMRS->m_AGVTime[i]=0;
			}
		}
	}
}

bool MultiRobotSupervisor::checkDeadAGV(int nID)
{
	m_AGVTime[nID]+=m_nThreadTime;

	if(m_AGVTime[nID]>DEADLOCKTIME)
	{
		return true;
	}

	return false;
}

bool MultiRobotSupervisor::checkBetweenAGVPose(KuPose RobotPos1,KuPose RobotPos2, int  nTVel1,int nRVel1,int nTVel2,int nRVel2)
{
	double dDist[10]={0};
	int i=0;
	for(double dTime=0;dTime<10*0.5;dTime+=0.5,i++)
	{
		KuPose CurRobotPos1=calNextAGVPose(RobotPos1, nTVel1, nRVel1,dTime);
		KuPose CurRobotPos2=calNextAGVPose(RobotPos2, nTVel2, nRVel2,dTime);
		double dRobotDist = _hypot((double)(CurRobotPos1.getX() - CurRobotPos2.getX()), (double)(CurRobotPos1.getY() - CurRobotPos2.getY()));
		dDist[i]=dRobotDist;
	}
	
	bool bflag=false;

	for(int i=0;i<10;i++)
	{
		if(dDist[0]>dDist[i]&&dDist[0]<2500)
		{
			bflag=true;
		}
		else
		{
			bflag=false;
		}
	}

	if(bflag==true)	return true;
	else return false;
}
KuPose MultiRobotSupervisor::calNextAGVPose(KuPose RobotPos, int nTvel,int nRvel,double dTime)
{
	double dBetweenWheel=KuRobotParameter::getInstance()->getWheelBaseofRobot();
	double dRightWheelDistance=(nTvel+(dBetweenWheel*nRvel)/2.0)*dTime;
	double dLeftWheelDistance=(nTvel-(dBetweenWheel*nRvel)/2.0)*dTime;

	double dAverageWheelDistance = (dLeftWheelDistance + dRightWheelDistance)/2;

	double dDeltaT = (dRightWheelDistance - dLeftWheelDistance) / dBetweenWheel;

	double dDistance2RobotCenter=0.0;
	double dDeltaY=0.0;
	double dDeltaX=0.0;


	if(fabs(dDeltaT) >= 0.0017f ) {
		dDistance2RobotCenter = dAverageWheelDistance / dDeltaT;
		dDeltaY = dDistance2RobotCenter - ( dDistance2RobotCenter * cos(dDeltaT) );
		dDeltaX = dDistance2RobotCenter * sin(dDeltaT);
	}
	else {
		dDistance2RobotCenter = 0.0f;
		dDeltaY = 0;
		dDeltaX = dAverageWheelDistance;
	}

	//상대좌표를 절대좌표로 변환하는것.
	double dReferenceX =  RobotPos.getX() + dDeltaX * cos( RobotPos.getThetaRad() ) - dDeltaY * sin( RobotPos.getThetaRad() );
	double dReferenceY =  RobotPos.getY() + dDeltaX * sin( RobotPos.getThetaRad() ) + dDeltaY * cos( RobotPos.getThetaRad()  );
	double dReferenceT =  RobotPos.getThetaRad() + dDeltaT;


	if(dReferenceT > M_PI){
		dReferenceT -= 2*M_PI;
	}
	else if(dReferenceT < -M_PI){
		dReferenceT += 2*M_PI;
	}

	RobotPos.setX(dReferenceX); 
	RobotPos.setY(dReferenceY);
	RobotPos.setThetaRad(dReferenceT); 
	
	return RobotPos;
}

void MultiRobotSupervisor::setTransVel(int nID,int nTransVel)
{
	m_nTransVel[nID]=nTransVel;
	MultiRobotActuatorInterface::getInstance()->setTransVel(nID,nTransVel);
	int nT= MultiRobotActuatorInterface::getInstance()->getTransVel(nID);

}

void MultiRobotSupervisor::setRotVel(int nID,int nRotVel)
{	
	m_nRotVel[nID]=nRotVel;
	MultiRobotActuatorInterface::getInstance()->setRotVel(nID,nRotVel);

}

int MultiRobotSupervisor::getTransVel(int nID)
{
	return m_nTransVel[nID];
}

int MultiRobotSupervisor::getRotVel(int nID)
{
	return m_nRotVel[nID];
}

/**
@brief Korean: [전제조건] '하나의 zone에는 하나의 AGV만 존재한다'는 조건 설정하는 함수
*/
int  MultiRobotSupervisor::Precon_AGVQuantity(KuPose Robot1TargetPos, KuPose RobotPos2,int nAGVID, int nID)
{
	double dRobotDist = _hypot((double)(Robot1TargetPos.getX() - RobotPos2.getX()), (double)(Robot1TargetPos.getY() - RobotPos2.getY()));

	// 다음 Zone에 AGV가 대기

	if (dRobotDist <= ROBOTBOUNDARYRADIUSLENGTH)//자기로봇의 타겟점과 상대 로봇간의 거리가 1800 이하이면 충돌 가능성 COLLIOSION 리턴
	{
		if (nAGVID >nID) //m_AGV2일 때만 if문 실행(수정)
		{
			SSAGVWheelActuatorInterface::getInstance()->stop();
			printf("stop!!!!!!!!!\n");			
			KuDrawingInfo::getInstance()->setWaitforDiffAGV(m_bWaitAGVflag);
		}
		else
		{
			m_bWaitAGVflag=false;
			KuDrawingInfo::getInstance()->setWaitforDiffAGV(m_bWaitAGVflag);
		}
		return COLLIOSION;
	}
	else
		{
			m_bWaitAGVflag=false;
			KuDrawingInfo::getInstance()->setWaitforDiffAGV(m_bWaitAGVflag);
		}

	return DEFAULTST;
}

/**
@brief Korean: 고장판별하는 함수
*/
int  MultiRobotSupervisor::DetourforAGVQuantity(KuPose RobotPos2)
{
	double dRobotMovementDist = _hypot((double)(RobotPos2.getX() - m_PreRobotPos2.getX()), (double)(RobotPos2.getY() - m_PreRobotPos2.getY()));
	double dRobotMovementRad = fabs(RobotPos2.getThetaDeg() - m_PreRobotPos2.getThetaDeg());

	if ((dRobotMovementDist <= ROBOTMOVEMENTDIST)&&(dRobotMovementRad <= 3))//상대 로봇의 이전루프의 로봇위치랑 현재 로봇위치랑 비교해서 이동량이 10이하이고 각도가 3도이하면 고장BREAKDOWNAGV플래그리턴
	{
		return BREAKDOWNAGV;
	}

	return DEFAULTST;
}

/**
@brief Korean:우회경로 만드는 함수
*/
list<KuPose> MultiRobotSupervisor::generateDetourPath(KuMap * pMap, KuPose RobotPos,KuPose Breakdownrobotpos,list<KuPose> CurPathlist,bool* bcheck  ) 
{
	int nSizeX = pMap->getX();
	int nSizeY = pMap->getY();
	list<KuPose> Pathlist;
	list<KuPose> tempPathlist;//임시path

	if(m_pPbstacleMap==NULL)
		m_pPbstacleMap = new KuMap(nSizeX, nSizeY);

	//============================================
	// 메모리 공간 할당
	//============================================
	int** nMap = m_pPbstacleMap->getMap();
	int** nRefMap = kuMapRepository::getInstance()->getMap()->getMap();

	for(int i = 0 ; i < nSizeX ; i++){
		for(int j = 0 ; j < nSizeY ; j++){
			nMap[i][j] = nRefMap[i][j];
		}
	}

	int nRobotX=Breakdownrobotpos.getX()/100;
	int nRobotY=Breakdownrobotpos.getY()/100;

	double dRadiusofRobot_cell = (int)(KuRobotParameter::getInstance()->getRobotRadius()/((double)100.0))*4.0;//로봇셀 사이즈 따라 나누어서//점유영역 사이즈 조정하고 싶으면, 줄일라면 -2를 -5이런식으로 숫자크게빼

	for(int i=-dRadiusofRobot_cell; i<dRadiusofRobot_cell;i++)//고장난 로봇 주변 반경 점유영역으로 변경 
	{
		for(int j=-dRadiusofRobot_cell; j<dRadiusofRobot_cell;j++)
		{
			if (nRobotX+i<0 || nRobotY+j<0 || nRobotX+i>=pMap->getX() || nRobotY+j>=pMap->getY()) {continue;}	// 맵사이즈를 벗어날 경우 예외 처리
			nMap[nRobotX+i][nRobotY+j] = KuMap::OCCUPIED_AREA;	
		}
	}

	KuDrawingInfo::getInstance()->setMap(m_pPbstacleMap);//그냥 유아이에 점유된거 보여줄라고 지워도되

	list<KuPose>::iterator iteratorPath;
	int nCnt=0;
	KuPose selPathpoint;
	int nselCnt=-1;
	double dMinDist=100000000000;
	int nMinpointNum=0;
	KuPose LastPos;

	for(iteratorPath = CurPathlist.begin();  iteratorPath != CurPathlist.end(); iteratorPath++)//현재 경로점들과 로봇 위치간의 유클리디언 거리비교해서 가장 가까운 점뽑음
	{
		KuPose Pathpoint;

		Pathpoint=(*iteratorPath);

		double dRobotPathDist = _hypot((double)(RobotPos.getX() - Pathpoint.getX()), (double)(RobotPos.getY() - Pathpoint.getY()));

		if(dRobotPathDist<dMinDist)
		{
			dMinDist=dRobotPathDist;
			nMinpointNum=nCnt;
		}
		nCnt++;
	}

	nCnt=0;

	for(iteratorPath = CurPathlist.begin();  iteratorPath != CurPathlist.end(); iteratorPath++)
	{
		nCnt++;

		if(nCnt < nMinpointNum)//위함수에서 뽑은 가까운점까지는 날리고
			continue;

		KuPose Pathpoint;

		Pathpoint=(*iteratorPath);

		double dRobotPathDist = _hypot((double)(RobotPos.getX() - Pathpoint.getX()), (double)(RobotPos.getY() - Pathpoint.getY()));
		double dRobottoBreakdownRobotDist = _hypot((double)(Breakdownrobotpos.getX() - Pathpoint.getX()), (double)(Breakdownrobotpos.getY() - Pathpoint.getY()));

		if(dRobottoBreakdownRobotDist>ROBOTBOUNDARYRADIUSLENGTH&&dRobotPathDist>ROBOTBOUNDARYRADIUSLENGTH)//자기로봇위치와 패스포인트들과의 거리를 비교해서 1800보다 긴쪽
		{																								//그리고 고장난로봇위치와 패스포인트들과의 거리를 비교해서 1800보다 긴쪽 
			                         //두조건을 만족할라면 로봇바운더리인 1800보다 큰 자기로봇반대쪽 패스위에포인트가 뽑히겠지
			//뽑힌 포인트를 타겟 포즈가 된다//(질문)
			nselCnt=nCnt;
			selPathpoint=Pathpoint;
			LastPos=Pathpoint;
			break;
		}
		LastPos=Pathpoint;
	}

	if(nselCnt+1>CurPathlist.size()-1||nselCnt==-1)
	{
		nselCnt=CurPathlist.size()-1;
		selPathpoint=LastPos;
	}

	

	nCnt = 0;
	for(iteratorPath = CurPathlist.begin();  iteratorPath != CurPathlist.end(); iteratorPath++)//기존 패스에서 회피경로 이후에 기존경로를 붙여주기 위해서
	{
		nCnt++;

		if(nCnt < nselCnt+1)
			continue;

		KuPose Pathpoint;

		Pathpoint=(*iteratorPath);

		tempPathlist.push_back(Pathpoint);
	}

	m_KuPathPlanner.initializeMapSize(kuMapRepository::getInstance()->getMap()->getX(),
		kuMapRepository::getInstance()->getMap()->getY()	);
	m_KuPathPlanner.setMap(m_pPbstacleMap); 
	m_KuPathPlanner.initIntCost((int)(KuRobotParameter::getInstance()->getRobotRadius()/((double)100.0))+1 );
	m_KuPathPlanner.generatePath(selPathpoint, RobotPos); //경로 생성
	Pathlist = m_KuPathPlanner.getPath();

	if(Pathlist.size()==0){
		(*bcheck)=false;
	}//경로생성 	
	else 
	{
		(*bcheck)=true;
	}

	for(iteratorPath = tempPathlist.begin();  iteratorPath != tempPathlist.end(); iteratorPath++)//생성된 회피경로에 아까 만든 기존 패스를 붙이는 함수
	{
		KuPose Pathpoint;
		Pathpoint=(*iteratorPath);
		Pathlist.push_back(Pathpoint);
	}


	return Pathlist;
}

/**
@brief Korean: 현재 사용중인  Behavior의 state를 나타내는 함수
*/
bool MultiRobotSupervisor::getBehaviorStates()
{
	bool bBehaviorStates=false;
	bBehaviorStates=m_GotoGoalBeh.getBehaviorStates();
	return bBehaviorStates;
}

/**
@brief Korean: 
*/
void MultiRobotSupervisor::execute(KuCommandMessage CMessage)
{
	int nBehaviorName = CMessage.getBehaviorName(); //행위에 대한 이름을 인덱스로 가지고 온다.
	int nBehaviorPeriod = CMessage.getBehaviorPeriod();
	KuPose GoalPos = CMessage.getGoalPos();
	KuPose RobotPos = CMessage.getRobotPos();

	Start(100);

	m_GotoGoalBeh.execute(CMessage);
}

/**
@brief Korean: 현재 사용중인  Behavior의 Localizer를 나타내는 함수
*/
Localizer* MultiRobotSupervisor::getLocalizer()
{
	Localizer* pLocalizer=NULL;
	pLocalizer=m_GotoGoalBeh.getLocalizer();
	return pLocalizer;
}

bool MultiRobotSupervisor::loadZoneMap(string strMapFilePath)
{
	return KuZoneControlPr::getInstance()->loadZoneMap(strMapFilePath);
}
/**
 @brief Korean: 
 @brief English: 
*/
bool MultiRobotSupervisor::connectCommunication()
{
// 	char  cSerialPort[10];
// 	KuRobotParameter::getInstance()->getCommunicationComport(cSerialPort);
// 	if(MultiRobotActuatorInterface::getInstance()->connect(cSerialPort))
// 		return true;
// 	return false;
	TotalTcpipCommunication::getInstance()->startServer(200);
	TotalTcpipCommunication::getInstance()->startClient(100); // Live 신호: 5 sec, Location & state: 100 msec
	//TotalTcpipCommunication::getInstance()->clientseq();

	return true;
}
/**
 @brief Korean: ROBOT pose 받아옴
 @brief English: 
*/
void MultiRobotSupervisor::updateRobotPose()
{
	m_AGVRobotPos = MultiRobotActuatorInterface::getInstance()->getRobotPos();
}
/**
 @brief Korean: 
 @brief English: 
*/
KuPose* MultiRobotSupervisor::getRobotPos()
{
	return m_AGVRobotPos;
}

