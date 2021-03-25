#include "stdafx.h"
#include "GotoGoalBehavior.h"
#include "../../MultiRobotSupervisor/TotalTcpipCommunication.h"
#include "../../ANSCommon/ANSCommon.h"

GotoGoalBehavior::GotoGoalBehavior()
{
	m_vecPath.clear();
	m_listPath.clear();
	m_vecWayPoint.clear();
	m_listWayPoint.clear();
	m_bThreadFlag = false;
	m_bWaitflag=false;
	m_nselectIdx=0;
	m_nSelectTargetIdx=0;
	m_CurWayPoint.init();
	m_nWaitingTime=0;
	m_pLocalMap=NULL;
	m_pOriginalMap=NULL;
	m_pMap= NULL;
	m_nObstacleDetectionTime=3;
	m_dRecognizingMarkDistTh = KuRobotParameter::getInstance()->getRecognizingDistTh();
	cout<<"[GotoGoalBehavior]: Instance is created!!!"<<endl;

	char cData[300]; 
	memset(cData,0,sizeof(cData));
	string strDataPath = KuRobotParameter::getInstance()->getTeachingPathNameNPath();
	sprintf(cData,"%s/RobotLog.log",strDataPath.c_str());
	m_DataLog.open(cData);
	m_nIDX=0;
	m_nFCount=0;
	m_dDistMarkfromRobot=DBL_MAX;

	m_bOtherAGVPathFlag=false;//zigbee를 위한 flag

	m_bPlaySoundObstacleDetection = false;
	m_bPlaySoundLowBattery = true; // 항상 출력
	m_bAbnormalState = false;

	m_dPosDiff = 0; // 추정된 위치의 변화량

	m_thread_sound_low_battery.start(thread_sound_low_battery, this, 1000, "sound_low_battery"); // 배터리 부족 음성 안내
}

GotoGoalBehavior::~GotoGoalBehavior()
{
	if(m_pLocalMap!=NULL)
	{
		delete [] m_pLocalMap;
		m_pLocalMap=NULL;
	}
	if(m_pOriginalMap!=NULL)
	{
		delete [] m_pOriginalMap;
		m_pOriginalMap=NULL;
	}
	if(m_pMap!=NULL)
	{
		delete [] m_pMap;
		m_pMap=NULL;
	}
	m_DataLog.close();

	cout<<"[GotoGoalBehavior]: Instance is destroyed!!!"<<endl;
}
void GotoGoalBehavior::initial()
{
	m_bThreadFlag = true;
	m_vecPath.clear();
	m_listPath.clear();
	m_vecWayPoint.clear();
	m_listWayPoint.clear();
	m_CurWayPoint.init();
	m_nWaitingTime=0;
	m_nselectIdx=-1;
	m_bWaitflag=false;
	m_nSelectTargetIdx=0;
	m_bAvoidModeflag=false;
	m_nPrePathIndx=-1;
	m_nLocalGoalIndex=3;
	m_nLocalGoalWayPointIndex=-1;
	m_dCheckObstacleDetectionTime=0;
	m_nObstacleDetectionTime=KuRobotParameter::getInstance()->getObstacleDetectionTime();
	m_dRecognizingMarkDistTh = KuRobotParameter::getInstance()->getRecognizingDistTh();
	m_bMapping=false;
	m_binitAlignRobotAngle=true;
	m_bterminate=true;
	m_dTimeA=0.0;
	m_dTimeB=0.0;
	m_nIFDX=-1;
	m_nFCount=0;
	m_dDistMarkfromRobot=DBL_MAX;
	m_RobotPos=KuDrawingInfo::getInstance()->getRobotPos();
	m_bfirstLamp= true;
	m_bonLamp=false;

	m_bOtherAGVPathFlag=false;//zigbee를 위한 flag
}
/**
 @brief Korean: 초기화 작업을 수행하는 함수.
 @brief English: 
*/
bool GotoGoalBehavior::initialize(KuCommandMessage CMessage)
{
	m_RobotPos=KuDrawingInfo::getInstance()->getRobotPos();
	m_GoalPos=KuDrawingInfo::getInstance()->getGoalPos();	

	printf("GotoGoalBehavior_initialize\n");
	if(m_bThreadFlag==true) return false;
	initial();
	m_bAvoidModeflag=CMessage.getAvoidMode();
	initKanayamaProcess();
	startLocalizer(m_RobotPos,KuRobotParameter::getInstance()->getLocalization()); 

	if(m_bAvoidModeflag)
	{
		initMapbuildingProcess(true);	
		if(!initPathplanningProcess()) return false;	
		if(!generateLocalPath()) return false;	
		calStartPathPoint();
		initAlignProcess();
	}
	else
	{
		initMapbuildingProcess(false);	
		//if(!initPathBlock( m_RobotPos, m_GoalPos)) return false;	
		//if(!initPathplanningProcess()) return false;	
	}
	
	//m_KuVrWheelActuator.setRobotPos( m_RobotPos );
	
	m_StartPoint=KuDrawingInfo::getInstance()->getRobotPos();
	
	return true;
}
bool GotoGoalBehavior::initPathBlock(KuPose RobotPos,KuPose GoalPos)
{
	vector<PBlock> vecPathBlock;
	vector<PBlock> vecTotalPathBlock;

	KuDrawingInfo::getInstance()->getPathBlockPos(&vecTotalPathBlock);
	KuPathBlockPlannerPr::getInstance()->initialize(vecTotalPathBlock);
	if(!KuPathBlockPlannerPr::getInstance()->generatePathBlock(RobotPos)) return false;
	KuPathBlockPlannerPr::getInstance()->getPathBlockPos(&vecPathBlock);

	KuPathBlockPr KPBPr;
	list<KuPose> PathList=KPBPr.generatePathList(vecPathBlock);
	KuDrawingInfo::getInstance()->setPath(PathList);
	if(PathList.size()<1) return false;

	return true;
}


bool GotoGoalBehavior::initBlockPath(vector<PBlock> vecPathBlock)
{
	KuPathBlockPr KPBPr;
	list<KuPose> PathList=KPBPr.generatePathList(vecPathBlock);
	KuDrawingInfo::getInstance()->setPath(PathList);
	if(PathList.size()<1) return false;

	return true;
}

/**
@brief Korean: ICP 프로세스를 수행하기 위해 필요한 초기화 작업을 수행하는 함수.
@brief English: 
*/
void GotoGoalBehavior::initAlignProcess()
{
	KuPose initRobotPos;
	double dPathX = m_vecPath[0].getX();
	double dPathY = m_vecPath[0].getY();
	double dPathT=m_vecPath[0].getThetaDeg();
	initRobotPos.setX(dPathX);
	initRobotPos.setY(dPathY);
	initRobotPos.setThetaDeg(dPathT);
	KuDrawingInfo::getInstance()->setAuxiliaryRobotPos(initRobotPos);
	
	SensorSupervisor::getInstance()->readSensorData();
	m_nLaserData181 = SensorSupervisor::getInstance()->getLaserDataFront();

	if(!alignRobot(m_nLaserData181, initRobotPos))
		m_RobotPos =KuDrawingInfo::getInstance()->getRobotPos();

}
/**
@brief Korean: ICP 프로세스를 수행하기 위해 필요한 초기화 작업을 수행하는 함수.
@brief English: 
*/
void GotoGoalBehavior::initICPProcess()
{
	double dDesiredVel = KuRobotParameter::getInstance()->getDesiedVel();
	double dKX =KuRobotParameter::getInstance()->getdKX();
	double dKY =KuRobotParameter::getInstance()->getdKY();
	double dKT = KuRobotParameter::getInstance()->getdKT();
	double dWheelbase = KuRobotParameter::getInstance()->getWheelBaseofRobot();

	double dDeiviationforTrans = KuRobotParameter::getInstance()->getDeviationforTrans();
	double dDeiviationforRotae = KuRobotParameter::getInstance()->getDeviationforRotate();
	double dDeiviationforTransRotae = KuRobotParameter::getInstance()->getDeviationforTransRotate();
	//m_ICPLocalizer.setInitRobotPos(m_RobotPos);

	KuScanMatchingLocalizerPr::getInstance()->setDeviation(dDeiviationforTrans, dDeiviationforRotae, dDeiviationforTransRotae );
	KuScanMatchingLocalizerPr::getInstance()->setParameter(dKX,dKY,dKT,dDesiredVel,dWheelbase);
	if(KuScanMatchingLocalizerPr::getInstance()->getThreadStates()==false)
	KuScanMatchingLocalizerPr::getInstance()->setInitRobotPos(m_RobotPos);
}
/**
@brief Korean: 
@brief English: 
*/
bool GotoGoalBehavior::initPathplanningProcess()
{	
	m_listWayPoint = KuDrawingInfo::getInstance()->getWayPointList();
	m_listPath = KuDrawingInfo::getInstance()->getPath();
	//load path & waypoint -----------------------------------------------------------------------------
	/*
	if(KuDrawingInfo::getInstance()->getDirectionofPathflag()==false)
	{
		m_listPath.reverse();
		m_listWayPoint.reverse();
		KuDrawingInfo::getInstance()->setPath(m_listPath);
		KuDrawingInfo::getInstance()->setWayPointList(m_listWayPoint);	
		//KuDrawingInfo::getInstance()->setDirectionofPathflag(false);
	}
	*/
	//path planning start-----------------------------------------------------------------------------
	if(m_listPath.size()==0)
	{
		m_RobotPos =KuDrawingInfo::getInstance()->getRobotPos();
		m_GoalPos = KuDrawingInfo::getInstance()->getGoalPos();
		m_KuPathPlanner.initializeMapSize(kuMapRepository::getInstance()->getMap()->getX(),
			kuMapRepository::getInstance()->getMap()->getY()	);
		m_KuPathPlanner.setMap(kuMapRepository::getInstance()->getMap()); 
		m_KuPathPlanner.initIntCost((int)(KuRobotParameter::getInstance()->getRobotRadius()/((double)100.0))+1 );
		m_KuPathPlanner.generatePath(m_GoalPos, m_RobotPos); //경로 생성
		m_listPath = m_KuPathPlanner.getPath();
		if(m_listPath.size()==0){
			m_bThreadFlag = false;	
			return false; 
		}//경로생성 실패의 경우
		m_listPath=m_KuPathSmoothing.smoothingPath(m_listPath);
	}
	//path planning  end-----------------------------------------------------------------------------

	list<KuPose>::iterator itway;

	for(itway=m_listWayPoint.begin(); itway!=m_listWayPoint.end(); itway++){
		m_vecWayPoint.push_back(*itway);
	}

	list<KuPose>::iterator it;
	for(it=m_listPath.begin(); it!=m_listPath.end(); it++){
		m_vecPath.push_back(*it);
	}

	calStartPathPoint();

	//Goal Drawing start-----------------------------------------------------------------------------
	int nPathSize = m_vecPath.size()-1;
	double dPathX = m_vecPath[nPathSize].getX();
	double dPathY = m_vecPath[nPathSize].getY();
	m_GoalPos.setX(dPathX);
	m_GoalPos.setY(dPathY);
	KuDrawingInfo::getInstance()->setGoalPos(m_GoalPos);
	//Goal Drawing start-----------------------------------------------------------------------------


	//경로생성 완료==================================================================
	m_RobotPos =KuDrawingInfo::getInstance()->getRobotPos();
	KuDrawingInfo::getInstance()->setPath(m_listPath); //경로 화면에 표시
	
	return true; 
}

void GotoGoalBehavior::calStartPathPoint( )
{
	KuPose RobotPos=KuDrawingInfo::getInstance()->getRobotPos();
	int nDistToTarget =KuRobotParameter::getInstance()->getTargetDistance();

	int  nMinIdx=-1;
	double dMinPathX = 0.0;
	double dMinPathY = 0.0;
	double dDist=0;
	bool bcheck= false;

	//로봇에서 부터의 최소가 되는 지점 및 거리값
	while( true ) {
		nMinIdx++;
		if(nMinIdx >= m_vecPath.size()) {
			nMinIdx = m_vecPath.size(); //배열인덱스 이기 떄문에 1을 빼줘야 한다.	
			break;
		}
		double dMinPathX = m_vecPath[nMinIdx].getX();
		double dMinPathY = m_vecPath[nMinIdx].getY();

		dDist=hypot(RobotPos.getX()-dMinPathX, RobotPos.getY()- dMinPathY);

		if(dDist>nDistToTarget&& bcheck== true)
		{
			m_nPrePathIndx = nMinIdx;
			m_nSelectedMinIndx = nMinIdx;
			break;
		}	
		else if(dDist<nDistToTarget)
		{
			bcheck= true;
		}
	}
	if(nMinIdx==-1) {
		m_nPrePathIndx = m_vecPath.size()-1;
		m_nSelectedMinIndx = m_vecPath.size()-1;
	}

	int nWayPointSize = m_vecWayPoint.size();
	double dWayPointX=0.0;
	double dWayPointY=0.0;

	for(int nPathIdx=m_nSelectedMinIndx; nPathIdx>1;nPathIdx--)
	{
		for(int nWayPointIdx=1; nWayPointIdx<nWayPointSize;nWayPointIdx++)
		{
			dWayPointX = m_vecWayPoint[nWayPointIdx].getX();
			dWayPointY = m_vecWayPoint[nWayPointIdx].getY();

			dDist=hypot(m_vecPath[nPathIdx].getX()-dWayPointX, m_vecPath[nPathIdx].getY()- dWayPointY);

			if(dDist < 500&&m_vecWayPoint[nWayPointIdx].getID()!=1)
			{
				m_vecWayPoint[nWayPointIdx].setID(1);
			}	
		}
	}
}
/**
@brief Korean: 우회 경로를 생성하기 위한 목적지를 설정하는 함수
@brief English: Sets the goal position to generate detour path
*/
KuPose GotoGoalBehavior::generateDetourGoalPos(KuPose RobotPos, vector<KuPose> vecPath, int nPathSize)
{
	KuPose LocalGoalPos;	

	if(nPathSize<3) return m_GoalPos;

	for(int i=m_nLocalGoalIndex-3; i<nPathSize; i++)
	{
		if(i<0) continue;
		
		for(int j=1; j<m_vecWayPoint.size(); j++){
	
			if(hypot((m_vecWayPoint[j].getX()- vecPath[i].getX()),(m_vecWayPoint[j].getY() - vecPath[i].getY()))< 300
				&&m_vecWayPoint[j].getID()!=1)
			{
				LocalGoalPos.setX(m_vecWayPoint[j].getX());
				LocalGoalPos.setY(m_vecWayPoint[j].getY());
				if(hypot((vecPath[i].getX()- RobotPos.getX()),(vecPath[i].getY() - RobotPos.getY()))< 500)
				{
					 m_vecWayPoint[j].setID(1);					
				}
				m_nLocalGoalIndex=i;
				m_nLocalGoalWayPointIndex=j;

				return LocalGoalPos;
			}
		}

		if(hypot((vecPath[i].getX()- RobotPos.getX()),(vecPath[i].getY() - RobotPos.getY()))> LOCALGOAL_DISTANCE){		
			LocalGoalPos.setX(vecPath[i].getX());
			LocalGoalPos.setY(vecPath[i].getY());
			m_nLocalGoalIndex=i;
			return LocalGoalPos;
		}
	}

	return m_GoalPos;
}
// KuPose GotoGoalBehavior::generateDetourGoalPos(KuPose RobotPos, vector<KuPose> vecPath, int nPathSize)
// {
// 	KuPose LocalGoalPos;	
// 
// 	if(nPathSize<3) return m_GoalPos;
// 
// 	for(int i=m_nLocalGoalIndex-3; i<nPathSize; i++)
// 	{
// 	if(hypot((vecPath[i].getX()- RobotPos.getX()),(vecPath[i].getY() - RobotPos.getY()))> LOCALGOAL_DISTANCE){		
// 			LocalGoalPos.setX(vecPath[i].getX());
// 			LocalGoalPos.setY(vecPath[i].getY());
// 			m_nLocalGoalIndex=i;
// 			return LocalGoalPos;
// 		}
// 	}
// 
// 	return m_GoalPos;
// }

/**
@brief Korean: 지역 지도 상의 우회 경로 생성 함수
@brief English: Creates detour path in local map
*/
bool GotoGoalBehavior::generateDetourPath(KuPose RobotPos, KuPose DetourGoalPos, KuMap* pLocalMap, 
	int nLocalMapSPosX, int nLocalMapSPosY, list<KuPose> *DetourPathList)
{
	KuPose LocalRobotPos, LocalGoalPos;
	LocalRobotPos.setX(RobotPos.getX()- nLocalMapSPosX*100);
	LocalRobotPos.setY(RobotPos.getY()- nLocalMapSPosY*100);
	LocalGoalPos.setX( DetourGoalPos.getX()- nLocalMapSPosX*100);
	LocalGoalPos.setY( DetourGoalPos.getY()- nLocalMapSPosY*100);
	m_KuLocalPathPlanner.setMap(pLocalMap);

	int nResult =m_KuLocalPathPlanner.generatePath(LocalGoalPos,	LocalRobotPos);//경로를 생성시키는 함수.

	if(nResult==0){ //경로가 생성된 경우.
		*DetourPathList = m_KuLocalPathPlanner.getPath();//실질적으로 경로를 리스트 형태로 받아오는 함수.
		*DetourPathList = m_KuPathSmoothing.smoothingPath(*DetourPathList);
		//cout<<"우회경로 생성 성공"<<endl;
		return true; 
	}
	//cout<<"우회경로 생성 실패~~~~~!!!!!!!!!!"<<endl;
	return false;
}

/**
@brief Korean: 생성된 지역지도 상의 경로를 전역 위치상의 경로로 바꿔주는 함수
@brief English: Changes path created by global position into path created by local map
*/
list<KuPose> GotoGoalBehavior::transferLocaltoGlobalPath( list<KuPose> LocalPath,KuPose RobotPose,int nLocalMapSPosX, int nLocalMapSPosY)
{
	list<KuPose>  GlobalDetourPathList;
	list<KuPose>::iterator it;
	KuPose GlobalDetourPath;
	if(LocalPath.size()>0){
		it = LocalPath.begin();
		it++;
		while(it != LocalPath.end()){
			GlobalDetourPath.setX(it->getX() + nLocalMapSPosX*100);
			GlobalDetourPath.setY(it->getY() + nLocalMapSPosY*100);
			GlobalDetourPathList.push_back(GlobalDetourPath);
			it++;
		}

	}
	return GlobalDetourPathList;
}

/**
@brief Korean: 지역 경로 생성 함수
@brief English: Executes local path planning
*/
bool GotoGoalBehavior::generateLocalPath()
{
	m_DetourPathList.clear();
	copyGlobalMapToLocalMap( m_pRefMap->getMap(), m_pLocalMap->getMap(), m_RobotPos, &m_nLocalMapSPosX, &m_nLocalMapSPosY);
	m_LocalGoalPos = generateDetourGoalPos(m_RobotPos, m_vecPath,m_vecPath.size()-1);
	generateDetourPath(m_RobotPos, m_LocalGoalPos, m_pLocalMap, m_nLocalMapSPosX, m_nLocalMapSPosY, &m_DetourPathList);
	m_DetourPathList= transferLocaltoGlobalPath(m_DetourPathList,m_RobotPos,m_nLocalMapSPosX, m_nLocalMapSPosY);
	m_vecLocalPath.clear();
	list<KuPose>::iterator it;
	for(it=m_DetourPathList.begin(); it!=m_DetourPathList.end(); it++){
		m_vecLocalPath.push_back(*it);
	}

	if(m_vecLocalPath.size()==0) return false;	
	return true; 
}
/**
@brief Korean: 
@brief English: 
*/
void GotoGoalBehavior::initMapbuildingProcess(bool bLocalMapflag)
{
	double dThicknessofWall=50;
	string strUpdateSpeed="no";

	m_pRefMap = kuMapRepository::getInstance()->getMap(); 
	m_nMapSizeX =m_nGlobalMapX=m_nBuildingMapX=m_pRefMap->getX();
	m_nMapSizeY =m_nGlobalMapY=m_nBuildingMapY=m_pRefMap->getY();

	
	if(NULL==m_pMap)
	{
		m_pMap = new KuMap(m_nMapSizeX, m_nMapSizeY); 
	}

	int** nMap = m_pMap->getMap();
	int** nRefMap = m_pRefMap->getMap();

	for(int i=0; i<m_nMapSizeX;i++)
	{
		for(int j=0; j<m_nMapSizeY;j++)
		{
			nMap[i][j] = nRefMap[i][j];
		}
	}


	KuMapBuilderParameter InputParamFront, InputParamRear;

	InputParamFront.setMapSizeXmYm(m_nMapSizeX/10, m_nMapSizeY/10);
	InputParamFront.setLaserScanIdx(Sensor::URG04LX_DATA_NUM181);
	InputParamFront.setMinDistofSensorData(KuRobotParameter::getInstance()->getURG04LXLaserMinDist()); // unit mm
	InputParamFront.setMaxDistofSensorData(KuRobotParameter::getInstance()->getURG04LXLaserMaxDist()); // unit mm
	InputParamFront.setLaserXOffset(KuRobotParameter::getInstance()->getFrontLaserXOffset());
	InputParamFront.setRadiusofRobot(KuRobotParameter::getInstance()->getRobotRadius());	
	InputParamFront.setRobotPos(m_RobotPos);
	InputParamFront.setSigma(100);	
	InputParamFront.setThicknessofWall(dThicknessofWall);

	InputParamRear.setMapSizeXmYm(m_nMapSizeX/10, m_nMapSizeY/10);
	InputParamRear.setLaserScanIdx(Sensor::URG04LX_DATA_NUM181);
	InputParamRear.setMinDistofSensorData(KuRobotParameter::getInstance()->getURG04LXLaserMinDist()); // unit mm
	InputParamRear.setMaxDistofSensorData(KuRobotParameter::getInstance()->getURG04LXLaserMaxDist()); // unit mm
	InputParamRear.setLaserXOffset(KuRobotParameter::getInstance()->getRearLaserXOffset());
	InputParamRear.setRadiusofRobot(KuRobotParameter::getInstance()->getRobotRadius());	
	InputParamRear.setRobotPos(m_RobotPos);
	InputParamRear.setSigma(100);	
	InputParamRear.setThicknessofWall(dThicknessofWall);

	if(strUpdateSpeed == "yes"){InputParamFront.setLaserUpdateSpeedflag(true);}
	m_LaserMapBuilder.initialize(InputParamFront, InputParamRear);

	if(true==bLocalMapflag)
	{
		//지역 지도 생성
		if(NULL==m_pLocalMap)
		{
			int nLocalMapSize = (LOCALGOAL_DISTANCE*2)/100; //*2는 LOCALGOAL_DISTANCE가 지도의 반이라서, /100은 mm단위는 격자단위로하기 위해
			m_pLocalMap = new KuMap(nLocalMapSize+30, nLocalMapSize+30);  //10은 마진 
			m_nLocalMapX=m_pLocalMap->getX();
			m_nLocalMapY=m_pLocalMap->getY();
			//지역 경로계획
			m_KuLocalPathPlanner.initIntCost(7);
			m_KuLocalPathPlanner.initializeMapSize(m_nLocalMapX,m_nLocalMapY);	

			m_pOriginalMap = new KuMap(m_nLocalMapX, m_nLocalMapY); 
		}
	}

}
/**
@brief Korean: Kanayama  프로세스를 수행하기 위해 필요한 초기화 작업을 수행하는 함수.
@brief English: 
*/
void GotoGoalBehavior::initKanayamaProcess()
{
	//INI 파일로부터 kanayama motion controㅣ을 수행하기 위해 필요한 정보를 얻어온다.------------------------------------
	m_nDistToTarget =KuRobotParameter::getInstance()->getTargetDistance();
	m_nDesiredVel = KuRobotParameter::getInstance()->getDesiedVel();
	m_nGoalArea =KuRobotParameter::getInstance()->getGoalArea();
	double dKX =KuRobotParameter::getInstance()->getdKX();
	double dKY =KuRobotParameter::getInstance()->getdKY();
	double dKT = KuRobotParameter::getInstance()->getdKT();
	m_nMaxTVel=KuRobotParameter::getInstance()->getMaxRobotVelocity();
	m_nMinTVel=KuRobotParameter::getInstance()->getMinRobotVelocity();
	bool bDirectionofRotationflag=(bool)KuRobotParameter::getInstance()->getDirectionofRotation();
	//==================================================================================================================
	m_KanayaMC.init();
	m_KanayaMC.setDirectionofRotation(bDirectionofRotationflag );
	m_KanayaMC.setGain(dKX,dKY,dKT);	//게인 설정, 설정하지 않더라고 기본 게인값이 사용된다.
	m_KanayaMC.setMaxTRVel(m_nMaxTVel,30);
	m_KanayaMC.setMinTRVel(m_nMinTVel,0);
}
/**
@brief Korean: 작성중인 지도를 전역 지도에 복사하는 함수.
@brief English: Copies the building map to global map
*/
void GotoGoalBehavior::copyBuildingMapToGlobalMap(int** nBuildingMap, int** nGlobalMap,  KuPose RobotPos)
{
	int nRobotX = (int)RobotPos.getX()/100.;
	int nRobotY = (int)RobotPos.getY()/100.;
	int nRange= 50;

	for(int i=nRobotX -nRange; i<nRobotX+nRange; i++){
		for(int j=nRobotY -nRange; j<nRobotY+nRange; j++){
			if(m_nMapSizeX-1<i||i<1||m_nMapSizeY-1<j||j<1) continue;
			if(m_nMapSizeX-1<i-nRobotX+nRange||i-nRobotX+nRange<1
				||m_nMapSizeY-1<j-nRobotY+nRange||j-nRobotY+nRange<1) continue;
			if(KuMap::UNKNOWN_AREA == nBuildingMap[i][j]) continue;

			nGlobalMap[i][j] = nBuildingMap[i][j];

		}
	}
}

/**
@brief Korean: 작성중인 지도를 전역 지도에 복사하는 함수.
@brief English: Copies the building map to global map
*/
void GotoGoalBehavior::copyBuildingMapToGlobalMap(int** nBuildingMap, int** nGlobalMap, int** nOriginalMap, KuPose RobotPos)
{
	int nRobotX = (int)RobotPos.getX()/100.;
	int nRobotY = (int)RobotPos.getY()/100.;

	for(int i=nRobotX -50; i<nRobotX+50; i++){
		for(int j=nRobotY -50; j<nRobotY+50; j++){
			if(m_nBuildingMapX-1<i||i<1||m_nBuildingMapY-1<j||j<1) continue;
			if(m_nGlobalMapX-1<i||i<1||m_nGlobalMapY-1<j||j<1) continue;
			if(110-1<i-nRobotX+50||i-nRobotX+50<1||110-1<j-nRobotY+50||j-nRobotY+50<1) continue;

			nOriginalMap[i-nRobotX+50][j-nRobotY+50] =nGlobalMap[i][j]; 

			if(KuMap::UNKNOWN_AREA == nBuildingMap[i][j]) continue;
			if(KuMap::OCCUPIED_AREA == nGlobalMap[i][j]) continue;
			nGlobalMap[i][j] = nBuildingMap[i][j];
		}
	}
}

/**
@brief Korean: 전역 지도를 지역 지도에 복사하는 함수.
@brief English: Copies the global map to local map
*/
void GotoGoalBehavior::copyGlobalMapToLocalMap(int**nGlobalMap, int** nLocalMap, KuPose RobotPos, int* nLocalMapSPosX, int* nLocalMapSPosY)

{
	//--------초기화----------------------------------//
	for(int i=0;i<m_nLocalMapX; i++){
		for(int j=0; j<m_nLocalMapY; j++){
			nLocalMap[i][j] = KuMap::UNKNOWN_AREA;
		}
	}
	//=================================================

	int nRobotX = (int)RobotPos.getX()/100.;
	int nRobotY = (int)RobotPos.getY()/100.;
	int nX=0., nY=0;
	int nHalfSizeOfLocalMapX =m_nLocalMapX/2;
	int nHalfSizeOfLocalMapY =m_nLocalMapY/2;

	for(int i=0;i<m_nLocalMapX; i++){
		for(int j=0; j<m_nLocalMapY; j++){
			nX = nRobotX - nHalfSizeOfLocalMapX + i;
			nY = nRobotY - nHalfSizeOfLocalMapY + j;
			if(m_nLocalMapX<i||i<1||m_nLocalMapY<j||j<1) continue;
			if(m_nGlobalMapX<nX||nX<1||m_nGlobalMapY<nY||nY<1) continue;	
			nLocalMap[i][j]=nGlobalMap[nX][nY];

		}
	}
	*nLocalMapSPosX = nRobotX - nHalfSizeOfLocalMapX;
	*nLocalMapSPosY = nRobotY - nHalfSizeOfLocalMapY;
}

/**
@brief Korean: 전역지도를 원래의 상태로 바꾸는 함수
@brief English: Changes the global map to original map
*/
void GotoGoalBehavior::copyOriginalMapToGlobalMap(int** nOriginalMap, int** nGlobalMap, KuPose RobotPos)
{

	int nRobotX = (int)RobotPos.getX()/100.;
	int nRobotY = (int)RobotPos.getY()/100.;

	for(int i=nRobotX -50; i<nRobotX+50; i++){
		for(int j=nRobotY -50; j<nRobotY+50; j++){
			if(m_nGlobalMapX<i||i<1||m_nGlobalMapY<j||j<1) continue;
			if(m_nLocalMapX<i-nRobotX+50||i-nRobotX+50<1||m_nLocalMapY<j-nRobotY+50||j-nRobotY+50<1) continue;

			nGlobalMap[i][j]=nOriginalMap[i-nRobotX+50][j-nRobotY+50] ; 
		}
	}

}
/**
@brief Korean: 
@brief English: 
*/
int_1DArray GotoGoalBehavior::combinateLaserNKinect(int_1DArray nLaserData181, int_1DArray nKinectRnageData)
{
	int nLaserMinDis=KuRobotParameter::getInstance()->getURG04LXLaserMinDist();
	int nKinectMinDis=KuRobotParameter::getInstance()->getKinectMinDist();
	
	int_1DArray nCombinateRangeData=m_KuUtil.generateIntType1DArray(Sensor::URG04LX_DATA_NUM181,0);
	int nStart=(Sensor::URG04LX_DATA_NUM181-Sensor::KINECT_SENSOR_FOV)/2.0;
	int nEnd=(Sensor::URG04LX_DATA_NUM181+Sensor::KINECT_SENSOR_FOV)/2.0;

	for(int i=0;i<Sensor::URG04LX_DATA_NUM181;i++)
	{
		nCombinateRangeData[i]=m_nLaserData181[i];

		if(i>nStart&&i<nEnd)
		{
			if(nLaserData181[i]>nKinectRnageData[i-nStart]&& nKinectRnageData[i-nStart]>nKinectMinDis)
			{
				nCombinateRangeData[i]=nKinectRnageData[i-nStart];
			}
		}
	}

	return nCombinateRangeData;
}
void GotoGoalBehavior::generateLocalMap(KuPose RobotPos,KuPose DelEncoderData,int_1DArray nLaserData181, int_1DArray nKinectRnageData)
{
	KuMapBuilderParameter InputParam; 		

	int_1DArray nCombinateRangeData;

	nCombinateRangeData =combinateLaserNKinect(nLaserData181, nKinectRnageData);

	if(DelEncoderData.getThetaDeg()<3.0)
	{
		//지도 작성 시작---------------------------------------------------------------------
		InputParam.setDelRobotPos(DelEncoderData); 
		InputParam.setRobotPos(RobotPos);
		InputParam.setLaserData(nCombinateRangeData);
		InputParam.setLaserUpdateSpeedflag(true);
		m_LaserMapBuilder.buildMapFront(InputParam);		
		//지도 작성 종료---------------------------------------------------------------------

		//Map copy  start------------------------------------------------------------------------------------------------------------------------	
		copyBuildingMapToGlobalMap(m_LaserMapBuilder.getMap(), m_pRefMap->getMap(),m_pOriginalMap->getMap(), m_RobotPos);
//		copyGlobalMapToLocalMap( m_pMap->getMap(), m_pLocalMap->getMap(), m_RobotPos, &m_nLocalMapSPosX, &m_nLocalMapSPosY);
		//Map copy  end------------------------------------------------------------------------------------------------------------------------	
	}

}
/**
@brief Korean: 경로에 장애물의 존재 여부를 판단하고 장애물의 크기를 확장하는 함수
@brief English: Checks the presence of obstacles in path and extends the size of obstacles
*/
bool GotoGoalBehavior::existObstaclePath(KuPose RobotPos, vector<KuPose> vecLocalPath, int nPathSize, KuMap* pLocalMap, int nLocalMapSPosX, int nLocalMapSPosY )
{
	int nX=0, nY=1;
	int **nMap=m_pRefMap->getMap();
	bool bObsCSpaceflag=false;
	int nObsCSpace=2;
	int ninterval=2;//검사할 영역 간격
	int nObnCnt=3;
	int nObstVeticlaBnd=3;//30cm
	int nRX=RobotPos.getXm()*10;
	int nRY=RobotPos.getYm()*10;

	if(nPathSize<nObnCnt){return false;}


	for(int i=nObnCnt; i<nPathSize; i++ )
	{
		int nGridPathX = vecLocalPath[i].getXm()*10;
		int nGridPathY = vecLocalPath[i].getYm()*10;

		for(int n=-ninterval;n<(ninterval+1);n+=1){
			for(int m=-ninterval;m<(ninterval+1);m+=1){

				if(nGridPathX+n>m_pRefMap->getX()-1||nGridPathX+n< 1
					||nGridPathY+m>m_pRefMap->getY()-1||nGridPathY+m<1){continue;}

				if(m_pRefMap->getMap()[nGridPathX+n][nGridPathY+m]==KuMap::OCCUPIED_AREA){		
					
					bObsCSpaceflag=true; 

					for(int q=-nObsCSpace;q<nObsCSpace+1;q++)
					{
						for(int p=-nObsCSpace;p<nObsCSpace+1;p++)
						{

							if(nGridPathX+n+q>m_pRefMap->getX()-1||nGridPathX+n+q< 1
								||nGridPathY+m+p>m_pRefMap->getY()-1||nGridPathY+m+p<1){continue;}

							if(m_pRefMap->getMap()[nGridPathX+n+q][nGridPathY+m+p]==KuMap::OCCUPIED_AREA) {continue;}


							double dDelX=((double)(nGridPathX-nRX))/10.0;
							double dDelY=((double)(nGridPathY-nRY))/10.0;
							double dAccDelX=dDelX;
							double dAccDelY=dDelY;
							
							while(hypot(dAccDelX,dAccDelY)<nObstVeticlaBnd)
							{
								if(nGridPathX+n+q+(int)dAccDelX>m_pRefMap->getX()-1||nGridPathX+n+q+(int)dAccDelX<1
									||nGridPathY+m+p+(int)dAccDelY>m_pRefMap->getY()-1||nGridPathY+m+p+(int)dAccDelY<1){
										break;
								}

								m_pRefMap->getMap()[nGridPathX+n+q+(int)dAccDelX][nGridPathY+m+p+(int)dAccDelY]=KuMap::WARNING_AREA;
							
								dAccDelX+=dDelX;
								dAccDelY+=dDelY;
							}
						}
					}
					
				}
			}
		}

	}
	if(bObsCSpaceflag){
		return true;
	}
	return false;
}
// 
// bool GotoGoalBehavior::existObstaclePath(KuPose RobotPos, vector<KuPose> vecLocalPath, int nPathSize, KuMap* pLocalMap, int nLocalMapSPosX, int nLocalMapSPosY )
// {
// 	int nX=0, nY=1;
// 	int **nMap=m_pMap->getMap();
// 	int **nLocalMap=pLocalMap->getMap();
// 	bool bObsCSpaceflag=false;
// 	int nObsCSpace=1;
// 	int ninterval=2;//검사할 영역 간격
// 	int nObnCnt=3;
// 
// 	if(nPathSize<nObnCnt)
// 	{
// 		return false;
// 	}
// 
// 	for(int i=nObnCnt; i<nPathSize; i++ )
// 	{
// 		int nGridPathX =vecLocalPath[i].getXm()*10;
// 		int nGridPathY = vecLocalPath[i].getYm()*10;
// 
// 		if(nGridPathX>m_pMap->getX()-(ninterval+1)||nGridPathX<(ninterval+1)
// 			||nGridPathY>m_pMap->getY()-(ninterval+1)||nGridPathY<(ninterval+1)){continue;}
// 
// 		int nGridLocalPathX=(int)(nGridPathX- nLocalMapSPosX);
// 		int nGridLocalPathY=(int)(nGridPathY- nLocalMapSPosY);
// 
// 		for(int n=-ninterval;n<(ninterval+1);n+=1){
// 			for(int m=-ninterval;m<(ninterval+1);m+=1){
// 
// 				if(nGridLocalPathX+n>nLocalMapSPosX-1||nGridLocalPathX+n<1
// 					||nGridLocalPathY+m>nLocalMapSPosY-1||nGridLocalPathY+m<1){continue;}
// 				if(nGridPathX>m_pMap->getX()-(ninterval+1)||nGridPathX<(ninterval+1)
// 					||nGridPathY>m_pMap->getY()-(ninterval+1)||nGridPathY<(ninterval+1)){continue;}
// 
// 
// 				if(m_pMap->getMap()[nGridPathX+n][nGridPathY+m]==KuMap::OCCUPIED_AREA){		
// 
// 					bObsCSpaceflag=true;
// 
// 					for(int q=-nObsCSpace;q<nObsCSpace+1;q++)
// 					{
// 						for(int p=-nObsCSpace;p<nObsCSpace+1;p++)
// 						{
// 							if(nGridLocalPathX+n+q>nLocalMapSPosX-1||nGridLocalPathX+n+q<1||nGridLocalPathY+m+p>nLocalMapSPosY-1||nGridLocalPathY+m+p<1){
// 								continue;
// 							}	
// 							if(pLocalMap->getMap()[nGridLocalPathX+n+q][nGridLocalPathY+m+p]==KuMap::OCCUPIED_AREA)continue;
// 
// 							if(abs(p)+abs(q)<(nObsCSpace)*2){
// 								//m_pMap->getMap()[nGridPathX+n+q][nGridPathY+m+p]=KuMap::FRONTIER_GRID;
// 								pLocalMap->getMap()[nGridLocalPathX+n+q][nGridLocalPathY+m+p]=KuMap::WARNING_AREA;
// 
// 							}
// 						}
// 					}
// 				}
// 			}
// 		}
// 
// 	}
// 	if(bObsCSpaceflag){
// 		return true;
// 	}
// 	return false;
// }
/**
 @brief Korean: 
 @brief English: 
*/
bool GotoGoalBehavior::generateLocalPathforObsAvoidance(KuPose RobotPos,KuPose DelEncoderData,int_1DArray nLaserData181, int_1DArray nKinectRnageData)
{
	bool bDoLocalPathPlanning=false;
	bool bexistObstaclePath=false;
	bool breturn=true;
	generateLocalMap(RobotPos,DelEncoderData,nLaserData181, nKinectRnageData);

	if((fabs(RobotPos.getX()-m_LocalGoalPos.getX())<2000) &&( fabs(RobotPos.getY()-m_LocalGoalPos.getY())<2000) &&		
		(hypot(m_GoalPos.getX()-m_LocalGoalPos.getX(),m_GoalPos.getY()-m_LocalGoalPos.getY())>500))
	{
	
		bDoLocalPathPlanning = true;
		if(m_nLocalGoalWayPointIndex!=-1)
			if((fabs(m_vecWayPoint[m_nLocalGoalWayPointIndex].getX()-m_LocalGoalPos.getX())<100) 
				&&( fabs(m_vecWayPoint[m_nLocalGoalWayPointIndex].getY()-m_LocalGoalPos.getY())<100)
				&&hypot((m_LocalGoalPos.getX()- RobotPos.getX()),(m_LocalGoalPos.getY() - RobotPos.getY()))> 500				
				)
			{
				bDoLocalPathPlanning = false;
			}	
	}
	if(existObstaclePath(RobotPos, m_vecLocalPath, m_vecLocalPath.size()-1,  m_pLocalMap,  m_nLocalMapSPosX, m_nLocalMapSPosY ))//로컬패스의 장애물 검사.
	{
		bDoLocalPathPlanning = true;
		bexistObstaclePath =true;
	}
	
	if(DelEncoderData.getThetaDeg()>0&&hypot(DelEncoderData.getX(),DelEncoderData.getY())<50)
	{
		bDoLocalPathPlanning = false;
		bexistObstaclePath =false;
	}

	//Path planning-----------------------------------------------------------------------
	if(bDoLocalPathPlanning == true||m_vecLocalPath.size()==0)
	{
		if(bexistObstaclePath==true){
			SSAGVWheelActuatorInterface::getInstance()->stop();
			m_KanayaMC.init();
			Sleep(50);
		}
		bDoLocalPathPlanning=false;
		breturn=generateLocalPath();
		if(!breturn) bDoLocalPathPlanning=true;
	}
		m_LaserMapBuilder.initMap();
		return breturn;
}

/**
 @brief Korean: 스레드로 돌아가는 함수
 @brief English: 
*/
void GotoGoalBehavior::doThreadwithAvoid(void* arg)
{
	GotoGoalBehavior* pGGB = (GotoGoalBehavior*)arg;
	KuVelocity generatedVel;

	double dTargetDist=pGGB->m_nDistToTarget;
	double dDistfromRobottoWayPoint=INFINITY_VALUE;

	if(SensorSupervisor::getInstance()->readSensorData() ==false) return;
	pGGB->m_nLaserData181 = SensorSupervisor::getInstance()->getLaserDataFront();
	pGGB->m_nKinectLaserData=SensorSupervisor::getInstance()->getKinectRangeData();
	pGGB->m_DelEncoderData =SensorSupervisor::getInstance()->getEncoderDelPos();
	pGGB->m_CeilingCamera=SensorSupervisor::getInstance()->getCeilingImageData();
	pGGB->m_IplKinectImage=SensorSupervisor::getInstance()->getKinectImageData();
	double dGyro = SensorSupervisor::getInstance()->getGyroData();
	pGGB->m_DelEncoderData.setThetaDeg(dGyro);
	
	if(KuRobotParameter::getInstance()->getLocalization()== Localizer::PARTICLEFILTER)
	{
		pGGB->m_RobotPos = KuLBPFLocalizerPr::getInstance()->estimateRobotPos(pGGB->m_nLaserData181, pGGB->m_DelEncoderData,pGGB->m_CeilingCamera);	

	}
	else if(KuRobotParameter::getInstance()->getLocalization()== Localizer::SCANMATCHING)
	{
		KuScanMatchingLocalizerPr::getInstance()->estimateRobotPos(pGGB->m_nLaserData181, pGGB->m_DelEncoderData,pGGB->m_RobotPos,pGGB->m_pMap);
		pGGB->m_RobotPos =KuScanMatchingLocalizerPr::getInstance()->getRobotPos();
	}

	if(pGGB->m_math.calcDistBetweenPoses(pGGB->m_GoalPos,pGGB->m_RobotPos) <pGGB->m_nGoalArea){
		pGGB->terminate();
		return;
	}
	else if(pGGB->m_math.calcDistBetweenPoses(pGGB->m_GoalPos,pGGB->m_RobotPos) <GOAL_BOUNDARY){
		pGGB->m_KanayaMC.setMaxTRVel(200,30);	//goal 주변 속도 감소
	}


	if(!pGGB->generateLocalPathforObsAvoidance(pGGB->m_RobotPos,pGGB->m_DelEncoderData,pGGB->m_nLaserData181, pGGB->m_nKinectLaserData))
	{
		SSAGVWheelActuatorInterface::getInstance()->stop();
		pGGB->m_CurWayPoint.init();
		pGGB->m_nWaitingTime=0;
		pGGB->copyOriginalMapToGlobalMap(pGGB->m_pOriginalMap->getMap(), pGGB->m_pRefMap->getMap(), pGGB->m_RobotPos);
		pGGB->drawNaviData();

		return;
	}

	pGGB->m_TargetPos= pGGB->getTargetPosbyLocalPath(pGGB->m_RobotPos, pGGB->m_nDistToTarget);

	//pGGB->m_TargetPos=pGGB->getStaticTargetPos( pGGB->m_RobotPos, pGGB->m_nDistToTarget);
	
	pGGB->m_TargetPos=pGGB->checkWayPointwithlocalpath(pGGB->m_TargetPos,pGGB->m_RobotPos, &pGGB->m_nselectIdx ,&pGGB->m_bWaitflag);
	
	if(pGGB->m_nselectIdx!=-1)
		dDistfromRobottoWayPoint=hypot(pGGB->m_RobotPos.getX()-pGGB->m_vecWayPoint[pGGB->m_nselectIdx].getX(), pGGB->m_RobotPos.getY()- pGGB->m_vecWayPoint[pGGB->m_nselectIdx].getY());
	
	if(pGGB->m_bWaitflag&&dDistfromRobottoWayPoint<pGGB->m_nGoalArea)
	{
		double dSleepTime=1000*pGGB->m_vecWayPoint[pGGB->m_nselectIdx].getPro();
		pGGB->m_bWaitflag=false;
		SSAGVWheelActuatorInterface::getInstance()->stop();
		pGGB->m_CurWayPoint=pGGB->m_vecWayPoint[pGGB->m_nselectIdx];
		
		Sleep(dSleepTime);	

		if(KuDrawingInfo::getInstance()->getDirectionofPathflag()==true)
		{
			Sleep(pGGB->m_nWaitingTime);	
		}

		pGGB->m_KanayaMC.init();

		return;
	}

	pGGB->m_CurWayPoint.init();
	pGGB->m_nWaitingTime=0;
	pGGB->copyOriginalMapToGlobalMap(pGGB->m_pOriginalMap->getMap(), pGGB->m_pRefMap->getMap(), pGGB->m_RobotPos);

// 	if(pGGB->checkObstacles( pGGB->m_nLaserData181, pGGB->m_nKinectLaserData))
// 	{
// 		SSAGVWheelActuatorInterface::getInstance()->stop();
// 	}
//	else {	
		generatedVel =  pGGB->m_KanayaMC.generateTRVelocity(pGGB->m_TargetPos, pGGB->m_RobotPos, pGGB->m_nDesiredVel );
		SSAGVWheelActuatorInterface::getInstance()->moveByTRVelocity(generatedVel.m_nTranslationVel, generatedVel.m_nRotationDegVel);
//	}

	pGGB->controlVelocityforMapInformation( pGGB->m_RobotPos, kuMapRepository::getInstance()->getVelocityMap());

	pGGB->drawNaviData();

}

/**
 @brief Korean: 스레드로 돌아가는 함수
 @brief English: 
*/
void GotoGoalBehavior::doThread(void* arg)
{
//	KuPRIMUSCommSupervisor::getInstance()->showPRIMUSState(); // PRIMUS 상태 출력

	GotoGoalBehavior* pGGB = (GotoGoalBehavior*)arg;
//	LARGE_INTEGER TotalTime;
//	pGGB->startTimeCheck(TotalTime);	
	KuVelocity generatedVel;
	double dDistfromRobottoWayPoint=INFINITY_VALUE;
	double dTargetDist=pGGB->m_nDistToTarget;
	static bool bRobotMoving(false);

	// ISSAC ////////////////////////////////////
	int nMode = XmldataSetting::getInstance()->getMode();

	if(nMode==XmldataSetting::ISSAC_MODE)
	{
		pGGB->m_TargetPos=KuPathBlockPlannerPr::getInstance()->getTargetPoseISSAC(pGGB->m_RobotPos);
	}
	else
	{
		pGGB->m_TargetPos=KuPathBlockPlannerPr::getInstance()->getTargetPose(pGGB->m_RobotPos);//로봇의 위치와 블록경로를 기반으로 target을 뽑아옴
	}

	if(pGGB->getAbnormalState())
	{
		Clientpart::getInstance()->SetState(Clientpart::STATE_ALARM);

//		ANS_LOG_WRITE("Abnormal state (alarm).");
	}
	else
	{
		if(!bRobotMoving)//SSAGVWheelActuatorInterface::getInstance()->getTVel() < 10) // mm, 로봇 모터 전원이 켜져 있을 경우에만 정상적인 값이 들어옴
		{
			Clientpart::getInstance()->SetState(Clientpart::STATE_STOP);
		}
		else
		{
			Clientpart::getInstance()->SetState(Clientpart::STATE_RUN);
		}
	}
	
	//JOB에 따른 주행부분(START, RESUME일땐 주행, STOP, PAUSE일땐 정지)--------------------------------------------------------------------------------------------------------------------------------------------------0516
	
	//JOB 정보 READ--------------------------------------------------------------------
	int nJobinfo = XmldataSetting::getInstance()->getJobStatus();//job의 종류를 받아옴(START=1/STOP=2/PAUSE=3/RESUME=4/JOB_COMPLETE=5/JOB_CHANGE=6)
	//JOB 정보 READ--------------------------------------------------------------------
	if(nMode==XmldataSetting::ISSAC_MODE)
	{
		while(nJobinfo == XmldataSetting::JOB_PAUSE)
		{
			SSAGVWheelActuatorInterface::getInstance()->stop();
			nJobinfo=XmldataSetting::getInstance()->getJobStatus();

			if(nJobinfo == XmldataSetting::JOB_RESUME)
			{
				break;
			}

			Sleep(100);
		}
/*
		// Alive 신호 송신 실패 시 로봇 정지
		while(TotalTcpipCommunication::getInstance()->failedToSendAliveSignal())
		{
			SSAGVWheelActuatorInterface::getInstance()->stop();
			Clientpart::getInstance()->SetState(Clientpart::STATE_STOP);

			Sleep(100);
		}
/**/
	}

	//JOB에 따른 주행부분(START, RESUME일땐 주행, STOP, PAUSE일땐 정지)--------------------------------------------------------------------------------------------------------------------------------------------------0516

	if(SensorSupervisor::getInstance()->readSensorData() ==false) return;
	pGGB->m_nLaserData181 = SensorSupervisor::getInstance()->getLaserDataFront();
	pGGB->m_nKinectLaserData=SensorSupervisor::getInstance()->getKinectRangeData();
	pGGB->m_DelEncoderData =SensorSupervisor::getInstance()->getEncoderDelPos();
	pGGB->m_CeilingCamera=SensorSupervisor::getInstance()->getCeilingImageData();
	pGGB->m_IplKinectImage=SensorSupervisor::getInstance()->getKinectImageData();
	pGGB->m_nSonarObsState = SensorSupervisor::getInstance()->getObsData();

	double dGyro = SensorSupervisor::getInstance()->getGyroData();

	double dDistEc=hypot(pGGB->m_DelEncoderData.getX(),pGGB->m_DelEncoderData.getY())*10.0;
	if(dDistEc>1500){pGGB->m_DelEncoderData.init();	}
	else pGGB->m_DelEncoderData.setThetaDeg(dGyro);
/*	pGGB->m_RobotPos  = pGGB->m_KuVrWheelActuator.getRobotPos();*/

	if(KuDrawingInfo::getInstance()->getWaitforDiffAGV()==true)
	{
		pGGB->drawNaviData();
		return;
	}
	

	if(KuRobotParameter::getInstance()->getLocalization()== Localizer::PARTICLEFILTER)
	{
		pGGB->m_RobotPosPrev = pGGB->m_RobotPos; 
		pGGB->m_RobotPos = KuLBPFLocalizerPr::getInstance()->estimateRobotPos(pGGB->m_nLaserData181, pGGB->m_DelEncoderData,pGGB->m_CeilingCamera);	
		
		pGGB->m_dPosDiff = hypot(pGGB->m_RobotPos.getXm() - pGGB->m_RobotPosPrev.getXm(),
								pGGB->m_RobotPos.getYm() - pGGB->m_RobotPosPrev.getYm());

		if(pGGB->m_dPosDiff > 0.0001)
		{
			bRobotMoving = true;
		}
		else
		{
			bRobotMoving = false;
			//printf("not moving: poseDiff = %f\n", pGGB->m_dPosDiff);
		}

		KuFiducialbasedLocalizerPr::getInstance()->estimateRobotPosbyFiducialmark(pGGB->m_RobotPos,pGGB->m_CeilingCamera, false);
		KuPose FiducialPose = KuFiducialbasedLocalizerPr::getInstance()->getRobotPosbyFiducial(); // fiducial mark 인식 결과로 추정한 로봇의 위치
				
		if(FiducialPose.getID()!=-1)
		{
			if(FiducialPose.getID()!=pGGB->m_nIFDX)
			{
				pGGB->m_nIFDX=FiducialPose.getID();
				pGGB->m_dDistMarkfromRobot=DBL_MAX;
				pGGB->m_nFCount=0;
			}
			
//			pGGB->m_nFCount++;

			if(pGGB->m_nFCount == 0)
			{

				double dX = FiducialPose.getX() + pGGB->m_DelEncoderData.getX() * cos(FiducialPose.getThetaRad()) + 
				pGGB->m_DelEncoderData.getY() * sin(-FiducialPose.getThetaRad());

				double dY = FiducialPose.getY() + pGGB->m_DelEncoderData.getX() * sin(FiducialPose.getThetaRad()) + 
				pGGB->m_DelEncoderData.getY() * cos(FiducialPose.getThetaRad());
				double dThetaDeg = FiducialPose.getThetaDeg() + pGGB->m_DelEncoderData.getThetaDeg();

				//threshold for updating fiducial pose
				double dDistThres = pGGB->m_dRecognizingMarkDistTh;
				double dDx = dX - FiducialPose.getX();
				double dDy = dY - FiducialPose.getY();
				double dDist = sqrt(dDx*dDx + dDy*dDy);

				//if(dDist<dDistThres&&dDist<pGGB->m_dDistMarkfromRobot)
				{
					pGGB->m_dDistMarkfromRobot=dDist;
					//printf("\n%d발견\n",FiducialPose.getID());
					FiducialPose.setX(dX);
					FiducialPose.setY(dY);
					FiducialPose.setThetaDeg(dThetaDeg);

					double dDistToFiducialPose(0);
			
					dDistToFiducialPose = hypot(pGGB->m_RobotPos.getXm() - FiducialPose.getXm(), pGGB->m_RobotPos.getYm() - FiducialPose.getYm());
					//dDistToFiducialPose = hypot(pGGB->m_RobotPos.getXm() - KuFiducialbasedLocalizerPr::getInstance()->getFiducialMarkPose().getXm(),
					//							pGGB->m_RobotPos.getYm() - KuFiducialbasedLocalizerPr::getInstance()->getFiducialMarkPose().getYm());

					// if the new pose is closer than 4 m
					if((bRobotMoving && dDistToFiducialPose < 4.) || !bRobotMoving)
					{
						KuLBPFLocalizerPr::getInstance()->spreadParticleNearRobot(FiducialPose, 0.2);
						pGGB->m_nFCount++;
					}
					
					// Log
					if(bRobotMoving && dDistToFiducialPose >= 4.)
					{
						ANS_LOG_WRITE("Detected wrong fiducial mark recognition.", false);
					}
				}
				
				//KuLBPFLocalizerPr::getInstance()->initEncoderData();
			}
		}
	}
	else if(KuRobotParameter::getInstance()->getLocalization()== Localizer::SCANMATCHING)
	{
	
		pGGB->m_RobotPos =KuILBPFLocalizerPr::getInstance()->estimateRobotPos(pGGB->m_nLaserData181, pGGB->m_RobotPos,pGGB->m_DelEncoderData, pGGB->m_CeilingCamera,false);	
		KuScanMatchingLocalizerPr::getInstance()->estimateRobotPosP(pGGB->m_nLaserData181, pGGB->m_DelEncoderData,pGGB->m_RobotPos,pGGB->m_pMap,&pGGB->m_bMapping);
		KuPose DelPos= KuScanMatchingLocalizerPr::getInstance()->getDeltaPos();
		KuILBPFLocalizerPr::getInstance()->copyEncoderData(DelPos);	

		KuFiducialbasedLocalizerPr::getInstance()->estimateRobotPosbyFiducialmark(pGGB->m_RobotPos,pGGB->m_CeilingCamera, false);
		KuPose FiducialPose = KuFiducialbasedLocalizerPr::getInstance()->getRobotPosbyFiducial();
		
		if(FiducialPose.getID()!=-1){
			if(FiducialPose.getID()!=pGGB->m_nIFDX)
			{
				pGGB->m_nIFDX=FiducialPose.getID();
				pGGB->m_dDistMarkfromRobot=DBL_MAX;
				pGGB->m_nFCount=0;
			}
			
			pGGB->m_nFCount++;
			if(pGGB->m_nFCount>0)
			{				
				pGGB->m_nFCount=0;
				double dX = FiducialPose.getX() + pGGB->m_DelEncoderData.getX() * cos(FiducialPose.getThetaRad()) + 
				pGGB->m_DelEncoderData.getY() * sin(-FiducialPose.getThetaRad());

				double dY = FiducialPose.getY() + pGGB->m_DelEncoderData.getX() * sin(FiducialPose.getThetaRad()) + 
				pGGB->m_DelEncoderData.getY() * cos(FiducialPose.getThetaRad());
				double dThetaDeg = FiducialPose.getThetaDeg() + pGGB->m_DelEncoderData.getThetaDeg();

				//threshold for updating fiducial pose
				double dDistThres = pGGB->m_dRecognizingMarkDistTh;
				double dDx = dX - FiducialPose.getX();
				double dDy = dY - FiducialPose.getY();
				double dDist = sqrt(dDx*dDx + dDy*dDy);

				if(dDist<dDistThres&&dDist<pGGB->m_dDistMarkfromRobot)
				{
					pGGB->m_dDistMarkfromRobot=dDist;
					printf("\n\n발견!!!!!\n\n");
					FiducialPose.setX(dX);
					FiducialPose.setY(dY);
					FiducialPose.setThetaDeg(dThetaDeg);

					KuILBPFLocalizerPr::getInstance()->spreadParticleNearRobot(FiducialPose, 0.2);
				}

				
				//KuILBPFLocalizerPr::getInstance()->initEncoderData();
			}
		}
	}

	//AGV 주행--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

	generatedVel =  pGGB->m_KanayaMC.generateTRVelocity(pGGB->m_TargetPos, pGGB->m_RobotPos, pGGB->m_nDesiredVel );//경로의 target정보, 로봇의 위치를 기반으로 속도제어
	double dWeight =KuPathBlockPlannerPr::getInstance()->detectObstacles(pGGB->m_RobotPos,pGGB->m_nLaserData181,
					pGGB->m_nKinectLaserData,pGGB->m_nSonarObsState,generatedVel.m_nRotationDegVel );//센서값에 따른 장애물 감지
	//double dWeight = 1.;
	double dVelWeight = KuPathBlockPlannerPr::getInstance()->getVelocityWeight();//블록의 색상에 따른 속도값 제어..(6가지 단계로 속도제어)
	pGGB->m_nTransVel= generatedVel.m_nTranslationVel;	pGGB->m_nRotVel= generatedVel.m_nRotationDegVel;//Transvel, Rotvel에 입력
	dWeight=dWeight*dVelWeight;//두 weight를 곱해 최종 속도 제어에 쓰이는 값(dWeight)을 출력

// 	//AGV location 정보 send--------------------------------------------------------------------
// 	bool bSendflag = Clientpart::getInstance()->ReqLocation();//current block ID + next block ID
// 	//AGV location 정보 send--------------------------------------------------------------------

/**/
	// Zigbee를 위한 부분--------------------------------------------------------------------
	// 충돌 방지 영역의 모든 block들의 check 값을 1로 설정해야 동작함.
	if(pGGB->m_bOtherAGVPathFlag) //충돌지역 주행 여부
	{
		printf("Collision avoidance\n");
		//1.가도 되는지 확인
		if(!CAGVCommSupervisor::getInstance()->isSafeToPass()){//주행불가
			SSAGVWheelActuatorInterface::getInstance()->moveByTRVelocity(0, 0);//주행대기
			pGGB->drawNaviData();
			Sleep(2000);
			return;//반복문종료(thread 1주기 종료 --> doThread 맨위로 이동)
		}
		else{//주행가능
			CAGVCommSupervisor::getInstance()->setEntryState(true);
		}
		//2.배출지점 확인
	//	if(KuPathBlockPlannerPr::getInstance()->isOtherAGVPath())
		{//check point이고
			if(!KuPathBlockPlannerPr::getInstance()->isEntryOtherAGVPath()){//뒤에또다른 check point가 없으면
				printf("Collision avoidance end");
				pGGB->m_bOtherAGVPathFlag=false;//배출상태
				CAGVCommSupervisor::getInstance()->setEntryState(false);
			}
		}
	}
	else{//진입지점
		if(KuPathBlockPlannerPr::getInstance()->isOtherAGVPath()){//check point이고
			if(KuPathBlockPlannerPr::getInstance()->isEntryOtherAGVPath()){//다음 block에 check point가 있으면
				printf("Collision avoidance start\n");
				pGGB->m_bOtherAGVPathFlag=true;//진입상태
				SSAGVWheelActuatorInterface::getInstance()->moveByTRVelocity(0, 0);//주행대기
			//	CAGVCommSupervisor::getInstance()->setCheckState();
				Sleep(2000);
			}
		}
	}
/**/
	//zigbee를 위한 부분끝------------------------------------------------------------------

	if(pGGB->m_TargetPos.getID()==1) // 정지(회전은 가능)
	{
		int nTransVel=0,nRotateVel=0;
		KuPathBlockPlannerPr::getInstance()->getMotionData(&nTransVel,&nRotateVel);
		if(nTransVel==0&&nRotateVel==0){
/*			pGGB->m_KuVrWheelActuator.moveTRVelocity(0, 0);		*/	
			SSAGVWheelActuatorInterface::getInstance()->stop();

			// ISSAC
			//Clientpart::getInstance()->SetState(Clientpart::STATE_STOP);
		}
		else{
			//pGGB->m_KuVrWheelActuator.moveTRVelocity(nTransVel, nRotateVel);//vertual	
			SSAGVWheelActuatorInterface::getInstance()->moveByTRVelocity(nTransVel, nRotateVel);

			// ISSAC
			//Clientpart::getInstance()->SetState(Clientpart::STATE_RUN);
		}
	}
	else // 이동
	{
		SSAGVWheelActuatorInterface::getInstance()->moveByTRVelocity(generatedVel.m_nTranslationVel*dWeight, generatedVel.m_nRotationDegVel*dWeight);
		//pGGB->m_KuVrWheelActuator.moveTRVelocity(generatedVel.m_nTranslationVel*dWeight, generatedVel.m_nRotationDegVel*dWeight);
		if(dWeight==0.0)
		{
			if(KuPathBlockPlannerPr::getInstance()->isRobotOpeningDoor() == false &&
				KuPathBlockPlannerPr::getInstance()->getTerminateState() == false) // 로봇이 문을 열고 있는 중이 아니고 thread 종료 중이 아닌 경우
			{
				pGGB->playSoundObstacleDetection(true);
			}

			// ISSAC
			//Clientpart::getInstance()->SetState(Clientpart::STATE_STOP);
		}
		else
		{
			pGGB->playSoundObstacleDetection(false);

			// ISSAC
			//Clientpart::getInstance()->SetState(Clientpart::STATE_RUN);
		}
	}

	// Low battery
/*
	if(KuPRIMUSCommSupervisor::getInstance()->getExternalBatteryVoltage() < KuRobotParameter::getInstance()->getLowBatteryAlarmVoltage())
	{
		pGGB->playSoundLowBattery(true);
	}
*/


	//GPIO를위한 부분--------------------------
	int nCurrentBlockIdx = KuPathBlockPlannerPr::getInstance()->getCurrentBlockIdx();
	if((*KuPathBlockPlannerPr::getInstance()->getPathBlock())[nCurrentBlockIdx].waypoint == 0) // waypoint가 아닐 경우 AGV의 정지 여부 검사
	{
		pGGB->setAbnormalState(CAGVCommSupervisor::getInstance()->checkAbnormalState(nCurrentBlockIdx,dDistEc));
	}

	if((*KuPathBlockPlannerPr::getInstance()->getPathBlock())[nCurrentBlockIdx].waypoint == PathBlock::START) // starting point에서 relay 2번으로 신호 보냄
	{
		CAGVCommSupervisor::getInstance()->sendArrivalSignalAtStartingPoint(true);
	}
	else
	{
		CAGVCommSupervisor::getInstance()->sendArrivalSignalAtStartingPoint(false);
	}
	//printf("gotogoalbehavior::dothread running!\n");
	//GPIO를위한 부분끝------------------------


	pGGB->drawNaviData();

	if(KuPathBlockPlannerPr::getInstance()->getTerminateState() /*&& pGGB->m_KanayaMC.isArrived()*/)
	{
		pGGB->terminate();

		ANS_LOG_WRITE("Arrived to the destination.");

		if(SSAGVWheelActuatorInterface::getInstance()->getTVel() < 10) // mm, 로봇 모터 전원이 켜져 있을 경우에만 정상적인 값이 들어옴
		{
			Clientpart::getInstance()->SetState(Clientpart::STATE_STOP);
		}
	}

	//AGV 주행--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
	
//	double dTime = pGGB->finishTimeCheck(TotalTime);
//	printf("dTime=%f\n",dTime);

}

bool GotoGoalBehavior::getAbnormalState(void)
{
	return m_bAbnormalState;
}

void GotoGoalBehavior::setAbnormalState(bool bState)
{
	m_bAbnormalState = bState;
}

void GotoGoalBehavior::playSoundObstacleDetection(bool bPlay)
{
	m_bPlaySoundObstacleDetection = bPlay;
}

void GotoGoalBehavior::playSoundLowBattery(bool bPlay)
{
	m_bPlaySoundLowBattery = bPlay;
}

void GotoGoalBehavior::playSound(string sFileName)
{
	string strNameNPath;
	strNameNPath=KuRobotParameter::getInstance()->getMovieNameNPath() + "/" +  sFileName;

	char cFilePathName[100];	
	WCHAR wcharFilePathName[100];
	sprintf_s(cFilePathName, "%s", strNameNPath.c_str());

	MultiByteToWideChar(0,0,cFilePathName,100,wcharFilePathName,100);
	LPCWSTR lpcwstrFilePathName=wcharFilePathName;
	PlaySound(lpcwstrFilePathName,NULL, SND_SYNC);
}

int GotoGoalBehavior::getTransVel()
{
	return m_nTransVel;
}

int GotoGoalBehavior::getRotVel()
{
	return m_nRotVel;
}

/**
@brief Korean: 주행 관련 정보를 그려주는 함수.
@brief English: 
*/
void GotoGoalBehavior::drawNaviData()
{
	KuDrawingInfo::getInstance()->setRobotPos(m_RobotPos); //로봇 위치 화면에 표시		
	KuDrawingInfo::getInstance()->setAuxiliaryRobotPos(m_RobotPos); //로봇 위치 화면에 표시		
	KuDrawingInfo::getInstance()->setFrontLaserData181(m_nLaserData181);		
	KuDrawingInfo::getInstance()->setKinectRangeData(m_nKinectLaserData);
	KuDrawingInfo::getInstance()->setTargetPos(m_TargetPos); //타겟 위치 화면에 표시
 	KuDrawingInfo::getInstance()->setCeilingImageData(m_CeilingCamera);
	KuDrawingInfo::getInstance()->setKinectImageData(m_IplKinectImage);
	KuDrawingInfo::getInstance()->setLocalPath(m_DetourPathList);
	KuDrawingInfo::getInstance()->setLocalGoalPos(m_LocalGoalPos);	
	KuDrawingInfo::getInstance()->setMap(m_pMap);

	if(KuRobotParameter::getInstance()->getLocalization()== Localizer::PARTICLEFILTER)
	{
		KuDrawingInfo::getInstance()->setParticle( KuLBPFLocalizerPr::getInstance()->getParticle() );
	}
	else if(KuRobotParameter::getInstance()->getLocalization()== Localizer::SCANMATCHING)
	{
		KuDrawingInfo::getInstance()->setParticle( KuILBPFLocalizerPr::getInstance()->getParticle() );

	}	

	KuPose FiducialMarkImgCoord = KuFiducialbasedLocalizerPr::getInstance()->getFiducialMarkImgCoord();
	//list<KuPose> FiducialMarkList = KuFiducialbasedLocalizerPr::getInstance()->getFiducialMarkList();
	KuDrawingInfo::getInstance()->drawFiducialMarkInfoToImage(FiducialMarkImgCoord);
	//KuDrawingInfo::getInstance()->setFiducialMarkList(FiducialMarkList);

	//KuDrawingInfo::getInstance()->setParticle( KuLBPFLocalizerPr::getInstance()->getParticle() );
}
/**
 @brief Korean: 지도의 정보로부터 속도를 받아와서 Kanayama  프로세스의 속도 최대값을 변경한다.
 @brief English: 
*/
void GotoGoalBehavior::controlVelocityforMapInformation( KuPose PointPos, int** nMap)
{
	int nPointX = (int)PointPos.getX()/100.;
	int nPointY = (int)PointPos.getY()/100.;
	double dMaxVel=0.0;
	if(nPointX<1||nPointX>m_nGlobalMapX-1||nPointY<1||nPointY>m_nGlobalMapY-1)
		return;

	if(nMap[nPointX][nPointY] == KuMap::FIRDECEL)
	{
		dMaxVel=((double)m_nMaxTVel)*5/6;
	}
	else if(nMap[nPointX][nPointY] == KuMap::SECDECEL)
	{
		dMaxVel=((double)m_nMaxTVel)*4/6;
	}
	else if(nMap[nPointX][nPointY] == KuMap::THIDECEL)
	{
		dMaxVel=((double)m_nMaxTVel)*3/6;
	}
	else if(nMap[nPointX][nPointY] == KuMap::FOUDECEL)
	{
		dMaxVel=((double)m_nMaxTVel)*2/6;
	}
	else if(nMap[nPointX][nPointY] == KuMap::FIFDECEL)
	{
		dMaxVel=((double)m_nMaxTVel)*1/6;
	}
	else
	{
		dMaxVel=m_nMaxTVel;
	}
	m_KanayaMC.setMaxTRVel(dMaxVel,30);		
}
int GotoGoalBehavior::doObsDetection(double dRotVel)
{
	int_1DArray nObsState = SensorSupervisor::getInstance()->getObsData();

	int nState=0;

	if(nObsState[0]==Sensor::WARNING
		||nObsState[1]==Sensor::WARNING
		||nObsState[2]==Sensor::WARNING
		||nObsState[3]==Sensor::WARNING
		)
	{
		nState=Sensor::WARNING;
	}

	if(dRotVel<0)
	{
		if(nObsState[0]==Sensor::DANGER||nObsState[1]==Sensor::DANGER)
		{
			nState=Sensor::DANGER;
		}
	}
	else if(dRotVel>0)
	{
		if(nObsState[2]==Sensor::DANGER||nObsState[3]==Sensor::DANGER)
		{
			nState=Sensor::DANGER;
		}
	}

	return nState;
}
KuPose GotoGoalBehavior::getCurWayPoint( )
{
	return m_CurWayPoint;
}

void GotoGoalBehavior::setWaitTime(int nTime )
{
	m_nWaitingTime=nTime;
}
/**
 @brief Korean: 경로에서 타겟점을 선택하는 함수
 @brief English: 
*/
/*
KuPose GotoGoalBehavior::getStaticTargetPos(KuPose RobotPos,int nDistToTarget)
{
	int nPathInx=0;
	int nMinIdx=0;;
	bool bselectedTargetflag= false;
	KuPose TargetPos;
	int nPathSize = m_vecPath.size();
	double dPathX; 
	double dPathY;
	double dMinPathX;
	double dMinPathY;
	double dDist;
	double dMinDistfromPath=INFINITY_VALUE;
	double dDistMintoTarget=INFINITY_VALUE;
	int nselectedMinIndx=-1;

	//로봇에서 부터의 최소가 되는 지점 및 거리값
	while( true ) {

		nMinIdx++;
		if(nMinIdx >= nPathSize) {
			nMinIdx = nPathSize-1; //배열인덱스 이기 떄문에 1을 빼줘야 한다.
			break;
		}
		dMinPathX = m_vecPath[nMinIdx].getX();
		dMinPathY = m_vecPath[nMinIdx].getY();

		dDist=hypot(RobotPos.getX()-dMinPathX, RobotPos.getY()- dMinPathY);

		if(dDist< dMinDistfromPath)
		{
			dMinDistfromPath=dDist;
			nselectedMinIndx=nMinIdx;
		}		
	}

	m_nSelectedMinIndx = nselectedMinIndx;

	dDistMintoTarget=sqrt(pow((double)nDistToTarget,2)-pow(dMinDistfromPath,2));
	nPathInx=0;

	while(true) {

		nPathInx++;
		if(nPathInx >= nPathSize) {
			nPathInx = nPathSize-1; //배열인덱스 이기 떄문에 1을 빼줘야 한다.

			if(bselectedTargetflag==false)
			{
				dPathX = m_vecPath[nselectedMinIndx].getX();
				dPathY = m_vecPath[nselectedMinIndx].getY();
				TargetPos.setX( dPathX );
				TargetPos.setY( dPathY );
			}
			break;			
		}

		dPathX = m_vecPath[nPathInx].getX();
		dPathY = m_vecPath[nPathInx].getY();

		dDist = hypot(RobotPos.getX()-dPathX, RobotPos.getY()- dPathY);

		if(dDist< dDistMintoTarget&&nPathInx>=nselectedMinIndx)
		{
			TargetPos.setX( dPathX );
			TargetPos.setY( dPathY );
			bselectedTargetflag=true;
			m_nSelectTargetIdx = nPathInx;
		}	
	}

	return TargetPos;
}*/
void GotoGoalBehavior::initPath(list<KuPose> Pathlist )
{
	m_vecPath.clear();
	list<KuPose>::iterator iteratorPath;

	for(iteratorPath = Pathlist.begin();  iteratorPath != Pathlist.end(); iteratorPath++)
	{
		KuPose Pathpoint;
		Pathpoint=(*iteratorPath);
		m_vecPath.push_back(Pathpoint);
	}
}

KuPose GotoGoalBehavior::getStaticTargetPos(KuPose RobotPos,int nDistToTarget)
{
	int nPathInx=0;
	int nMinIdx=0;;
	bool bselectedTargetflag= false;
	KuPose TargetPos;
	int nPathSize = m_vecPath.size()-1;
	double dPathX; 
	double dPathY;
	double dMinPathX;
	double dMinPathY;
	double dDist;
	double dMinDistfromPath=INFINITY_VALUE;
	double dDistMintoTarget=INFINITY_VALUE;
	int nselectedMinIndx=-1;

	if(nPathSize<1) return m_GoalPos;

	//로봇에서 부터의 최소가 되는 지점 및 거리값
	while( true ) {

		nMinIdx++;
		if(nMinIdx >= nPathSize) {
			nMinIdx = nPathSize; //배열인덱스 이기 떄문에 1을 빼줘야 한다.	
			break;
		}
		dMinPathX = m_vecPath[nMinIdx].getX();
		dMinPathY = m_vecPath[nMinIdx].getY();

		dDist=hypot(RobotPos.getX()-dMinPathX, RobotPos.getY()- dMinPathY);

		if(dDist>nDistToTarget&&nMinIdx>m_nPrePathIndx-1)
		{
			dMinDistfromPath=dDist;
			nselectedMinIndx=nMinIdx;
			m_nPrePathIndx=nMinIdx;
			break;
		}	
	}

	if(nselectedMinIndx==-1){
		dPathX = m_vecPath[nPathSize].getX();
		dPathY = m_vecPath[nPathSize].getY();
		TargetPos.setX( dPathX );
		TargetPos.setY( dPathY );
	}
	else
	{
		dPathX = m_vecPath[nselectedMinIndx].getX();
		dPathY = m_vecPath[nselectedMinIndx].getY();
		TargetPos.setX( dPathX );
		TargetPos.setY( dPathY );
	}


	m_nSelectedMinIndx = nselectedMinIndx;


	return TargetPos;
}
/**
@brief Korean: 경로에서 타겟점을 선택하는 함수
@brief English: Chooses the target point from the path
*/
KuPose GotoGoalBehavior::getTargetPos(KuPose RobotPos,int nDistToTarget)
{
	int nPathInx=0;
	int nMinIdx=0;

	double dPathX; 
	double dPathY;
	double dMinPathX;
	double dMinPathY;

	double dMinDistfromPath=INFINITY_VALUE;
	double dDistMintoTarget=INFINITY_VALUE;
	double dDist=INFINITY_VALUE;
	double dnewDistToTarget=INFINITY_VALUE;


	double  dMinTangentthetaDeg=0;
	double  dTargetTangentthetaDeg=0;

	int nPathSize = m_vecPath.size();
	if(m_vecPath.size()<=1) return m_GoalPos;
	int nselectedTargetIndx=nPathSize-1;
	int nselectedMinIndx=nPathSize-1;

	bool bselectedTargetflag= false;

	KuPose TargetPos,MinPos;


	//로봇에서 부터의 최소가 되는 지점 및 거리값
	while( true ) {

		nMinIdx++;
		if(nMinIdx >= nPathSize) {
			nMinIdx = nPathSize-1; //배열인덱스 이기 떄문에 1을 빼줘야 한다.
			break;
		}
		dMinPathX = m_vecPath[nMinIdx].getX();
		dMinPathY = m_vecPath[nMinIdx].getY();

		dDist=hypot(RobotPos.getX()-dMinPathX, RobotPos.getY()- dMinPathY);

		if(dDist< dMinDistfromPath)
		{
			dMinDistfromPath=dDist;
			MinPos.setX( dMinPathX);
			MinPos.setY( dMinPathY );
			nselectedMinIndx=nMinIdx;
		}		
	}

	//m_nSelectedMinIndx = nselectedMinIndx;
	//로봇에서부터의 최소가 되는 지점에서의 법선 벡터
	dMinTangentthetaDeg=atan2(m_vecPath[nselectedMinIndx].getY()-m_vecPath[nselectedMinIndx-1].getY(),m_vecPath[nselectedMinIndx].getX()-m_vecPath[nselectedMinIndx-1].getX());

	//목표지점과 최소지점간의 거리
	dDistMintoTarget=sqrt(pow((double)nDistToTarget,2)-pow(dMinDistfromPath,2));
	nPathInx=0;

	//목표지점과 최소지점간의 거리에 따른  목표지점 선정
	while( true ) {

		nPathInx++;
		if(nPathInx >= nPathSize) {
			nPathInx = nPathSize-1; //배열인덱스 이기 떄문에 1을 빼줘야 한다.

			if(bselectedTargetflag==false)
			{
				dPathX = m_vecPath[nselectedMinIndx].getX();
				dPathY = m_vecPath[nselectedMinIndx].getY();
				TargetPos.setX( dPathX );
				TargetPos.setY( dPathY );
				nselectedTargetIndx=nselectedMinIndx;
			}
			break;			
		}			
		dPathX = m_vecPath[nPathInx].getX();
		dPathY = m_vecPath[nPathInx].getY();			

		dDist=hypot(RobotPos.getX()-dPathX, RobotPos.getY()- dPathY);

		if(dDist< dDistMintoTarget&&nPathInx>=nselectedMinIndx)
		{
			dDist=dDistMintoTarget;
			TargetPos.setX( dPathX );
			TargetPos.setY( dPathY );
			nselectedTargetIndx=nPathInx;
			bselectedTargetflag=true;
		}	

	}

	//목표 지점에서의 법선 벡터
	dTargetTangentthetaDeg=atan2(m_vecPath[nselectedTargetIndx].getY()-m_vecPath[nselectedTargetIndx-1].getY(),m_vecPath[nselectedTargetIndx].getX()-m_vecPath[nselectedTargetIndx-1].getX());


	//새로운 로봇과 목표지점간의 거리 계산
	dnewDistToTarget=nDistToTarget;
	double dTempDist= fabs(fabs(dMinTangentthetaDeg)-fabs(dTargetTangentthetaDeg));
	//	printf("dTempDist=%f\n",dTempDist);
	dTempDist=dTempDist/M_PI*nDistToTarget/80;

	if(dTempDist<1.0)	{dTempDist=1.0;}
	else if(dTempDist>3.0)	{ dTempDist=3.0;}

	dnewDistToTarget=dnewDistToTarget/(dTempDist);


	//로봇과 목표지점간의 거리에 따른 목표점 선택
	nPathInx=0;
	if(dMinDistfromPath>=dnewDistToTarget) dDistMintoTarget=300;
	else
	{
		dDistMintoTarget=sqrt(pow((double)dnewDistToTarget,2)-pow(dMinDistfromPath,2));
		if(dDistMintoTarget<300) dDistMintoTarget=300;
	}

	bselectedTargetflag= false;
	dTempDist=1000000000;

	while( true ) {

		nPathInx++;
		if(nPathInx >= nPathSize) {

			nPathInx = nPathSize-1; //배열인덱스 이기 떄문에 1을 빼줘야 한다.

			if(bselectedTargetflag==false)
			{
				//(*dTargetDist)=dDistMintoTarget;

				dPathX = m_vecPath[nselectedTargetIndx].getX();
				dPathY = m_vecPath[nselectedTargetIndx].getY();
				TargetPos.setX( dPathX );
				TargetPos.setY( dPathY );
				nselectedTargetIndx=nselectedMinIndx;
			}
			break;			
		}			
		dPathX = m_vecPath[nPathInx].getX();
		dPathY = m_vecPath[nPathInx].getY();			

		dDist=hypot(MinPos.getX()-dPathX, MinPos.getY()- dPathY);

		if(dDist<dTempDist&&dDist> dDistMintoTarget&&nPathInx>=nselectedMinIndx)
		{
			dTempDist=dDist;
			//	(*dTargetDist)=dTempDist;
			TargetPos.setX( dPathX );
			TargetPos.setY( dPathY );
			nselectedTargetIndx=nPathInx;
			bselectedTargetflag=true;
		}	

	}
	m_nSelectedMinIndx = nselectedTargetIndx;

	return TargetPos;
}

/**
@brief Korean: 경로에서 타겟점을 선택하는 함수
@brief English: Chooses the target point from the path
*/
KuPose GotoGoalBehavior::getTargetPosbyLocalPath(KuPose RobotPos,int nDistToTarget)
{
	int nPathInx=0;
	int nMinIdx=0;

	double dPathX; 
	double dPathY;
	double dMinPathX;
	double dMinPathY;

	double dMinDistfromPath=INFINITY_VALUE;
	double dDistMintoTarget=INFINITY_VALUE;
	double dDist=INFINITY_VALUE;
	double dnewDistToTarget=INFINITY_VALUE;


	double  dMinTangentthetaDeg=0;
	double  dTargetTangentthetaDeg=0;

	int nPathSize = m_vecLocalPath.size();
	if(m_vecLocalPath.size()<=1) return m_GoalPos;
	int nselectedTargetIndx=nPathSize-1;
	int nselectedMinIndx=nPathSize-1;

	bool bselectedTargetflag= false;

	KuPose TargetPos,MinPos;


	//로봇에서 부터의 최소가 되는 지점 및 거리값
	while( true ) {

		nMinIdx++;
		if(nMinIdx >= nPathSize) {
			nMinIdx = nPathSize-1; //배열인덱스 이기 떄문에 1을 빼줘야 한다.
			break;
		}
		dMinPathX = m_vecLocalPath[nMinIdx].getX();
		dMinPathY = m_vecLocalPath[nMinIdx].getY();

		dDist=hypot(RobotPos.getX()-dMinPathX, RobotPos.getY()- dMinPathY);

		if(dDist< dMinDistfromPath)
		{
			dMinDistfromPath=dDist;
			MinPos.setX( dMinPathX);
			MinPos.setY( dMinPathY );
			nselectedMinIndx=nMinIdx;
		}		
	}

	//m_nSelectedMinIndx = nselectedMinIndx;
	//로봇에서부터의 최소가 되는 지점에서의 법선 벡터
	dMinTangentthetaDeg=atan2(m_vecLocalPath[nselectedMinIndx].getY()-m_vecLocalPath[nselectedMinIndx-1].getY(),m_vecLocalPath[nselectedMinIndx].getX()-m_vecLocalPath[nselectedMinIndx-1].getX());

	//목표지점과 최소지점간의 거리
	dDistMintoTarget=sqrt(pow((double)nDistToTarget,2)-pow(dMinDistfromPath,2));
	nPathInx=0;

	//목표지점과 최소지점간의 거리에 따른  목표지점 선정
	while( true ) {

		nPathInx++;
		if(nPathInx >= nPathSize) {
			nPathInx = nPathSize-1; //배열인덱스 이기 떄문에 1을 빼줘야 한다.

			if(bselectedTargetflag==false)
			{
				dPathX = m_vecLocalPath[nselectedMinIndx].getX();
				dPathY = m_vecLocalPath[nselectedMinIndx].getY();
				TargetPos.setX( dPathX );
				TargetPos.setY( dPathY );
				nselectedTargetIndx=nselectedMinIndx;
			}
			break;			
		}			
		dPathX = m_vecLocalPath[nPathInx].getX();
		dPathY = m_vecLocalPath[nPathInx].getY();			

		dDist=hypot(RobotPos.getX()-dPathX, RobotPos.getY()- dPathY);

		if(dDist< dDistMintoTarget&&nPathInx>=nselectedMinIndx)
		{
			dDist=dDistMintoTarget;
			TargetPos.setX( dPathX );
			TargetPos.setY( dPathY );
			nselectedTargetIndx=nPathInx;
			bselectedTargetflag=true;
		}	

	}

	//목표 지점에서의 법선 벡터
	dTargetTangentthetaDeg=atan2(m_vecLocalPath[nselectedTargetIndx].getY()-m_vecLocalPath[nselectedTargetIndx-1].getY(),m_vecLocalPath[nselectedTargetIndx].getX()-m_vecLocalPath[nselectedTargetIndx-1].getX());


	//새로운 로봇과 목표지점간의 거리 계산
	dnewDistToTarget=nDistToTarget;
	double dTempDist= fabs(fabs(dMinTangentthetaDeg)-fabs(dTargetTangentthetaDeg));
	//	printf("dTempDist=%f\n",dTempDist);
	dTempDist=dTempDist/M_PI*nDistToTarget/80;

	if(dTempDist<1.0)	{dTempDist=1.0;}
	else if(dTempDist>3.0)	{ dTempDist=3.0;}

	dnewDistToTarget=dnewDistToTarget/(dTempDist);


	//로봇과 목표지점간의 거리에 따른 목표점 선택
	nPathInx=0;
	if(dMinDistfromPath>=dnewDistToTarget) dDistMintoTarget=300;
	else
	{
		dDistMintoTarget=sqrt(pow((double)dnewDistToTarget,2)-pow(dMinDistfromPath,2));
		if(dDistMintoTarget<300) dDistMintoTarget=300;
	}

	bselectedTargetflag= false;
	dTempDist=1000000000;

	while( true ) {

		nPathInx++;
		if(nPathInx >= nPathSize) {

			nPathInx = nPathSize-1; //배열인덱스 이기 떄문에 1을 빼줘야 한다.

			if(bselectedTargetflag==false)
			{
				//(*dTargetDist)=dDistMintoTarget;

				dPathX = m_vecLocalPath[nselectedTargetIndx].getX();
				dPathY = m_vecLocalPath[nselectedTargetIndx].getY();
				TargetPos.setX( dPathX );
				TargetPos.setY( dPathY );
				nselectedTargetIndx=nselectedMinIndx;
			}
			break;			
		}			
		dPathX = m_vecLocalPath[nPathInx].getX();
		dPathY = m_vecLocalPath[nPathInx].getY();			

		dDist=hypot(MinPos.getX()-dPathX, MinPos.getY()- dPathY);

		if(dDist<dTempDist&&dDist> dDistMintoTarget&&nPathInx>=nselectedMinIndx)
		{
			dTempDist=dDist;
			//	(*dTargetDist)=dTempDist;
			TargetPos.setX( dPathX );
			TargetPos.setY( dPathY );
			nselectedTargetIndx=nPathInx;
			bselectedTargetflag=true;
		}	

	}
	m_nSelectedMinIndx = nselectedTargetIndx;

	return TargetPos;
}
// /**
//  @brief Korean: 경로에서 타겟점을 선택하는 함수
//  @brief English: 
// */
// KuPose GotoGoalBehavior::getTargetPos(KuPose RobotPos,int nDistToTarget, double*dTargetDist)
// {
// 	int nPathInx=0;
// 	int nMinIdx=0;
// 
// 	double dPathX; 
// 	double dPathY;
// 	double dMinPathX;
// 	double dMinPathY;
// 
// 	double dMinDistfromPath=INFINITY_VALUE;
// 	double dDistMintoTarget=INFINITY_VALUE;
// 	double dDist=INFINITY_VALUE;
// 	double dnewDistToTarget=INFINITY_VALUE;
// 
// 
// 	int nselectedMinIndx=-1;
// 	int nselectedTargetIndx=-1;
// 	double  dMinTangentthetaDeg=0;
// 	double  dTargetTangentthetaDeg=0;
// 
// 	int nPathSize = m_vecPath.size();
// 
// 	bool bselectedTargetflag= false;
// 
// 	KuPose TargetPos,MinPos;
// 
// 
// 	//로봇에서 부터의 최소가 되는 지점 및 거리값
// 	while( true ) {
// 
// 		nMinIdx++;
// 		if(nMinIdx >= nPathSize) {
// 			nMinIdx = nPathSize-1; //배열인덱스 이기 떄문에 1을 빼줘야 한다.
// 			break;
// 		}
// 		dMinPathX = m_vecPath[nMinIdx].getX();
// 		dMinPathY = m_vecPath[nMinIdx].getY();
// 
// 		dDist=hypot(RobotPos.getX()-dMinPathX, RobotPos.getY()- dMinPathY);
// 
// 		if(dDist< dMinDistfromPath)
// 		{
// 			dMinDistfromPath=dDist;
// 			MinPos.setX( dMinPathX);
// 			MinPos.setY( dMinPathY );
// 			nselectedMinIndx=nMinIdx;
// 		}		
// 	}
// 	//로봇에서부터의 최소가 되는 지점에서의 법선 벡터
// 	dMinTangentthetaDeg=atan2(m_vecPath[nselectedMinIndx].getY()-m_vecPath[nselectedMinIndx-1].getY(),m_vecPath[nselectedMinIndx].getX()-m_vecPath[nselectedMinIndx-1].getX());
// 
// 	//목표지점과 최소지점간의 거리
// 	dDistMintoTarget=sqrt(pow((double)nDistToTarget,2)-pow(dMinDistfromPath,2));
// 	nPathInx=0;
// 
// 	//목표지점과 최소지점간의 거리에 따른  목표지점 선정
// 	while( true ) {
// 
// 		nPathInx++;
// 		if(nPathInx >= nPathSize) {
// 			nPathInx = nPathSize-1; //배열인덱스 이기 떄문에 1을 빼줘야 한다.
// 
// 			if(bselectedTargetflag==false)
// 			{
// 				dPathX = m_vecPath[nselectedMinIndx].getX();
// 				dPathY = m_vecPath[nselectedMinIndx].getY();
// 				TargetPos.setX( dPathX );
// 				TargetPos.setY( dPathY );
// 				nselectedTargetIndx=nselectedMinIndx;
// 			}
// 			break;			
// 		}			
// 		dPathX = m_vecPath[nPathInx].getX();
// 		dPathY = m_vecPath[nPathInx].getY();			
// 
// 		dDist=hypot(RobotPos.getX()-dPathX, RobotPos.getY()- dPathY);
// 
// 		if(dDist< dDistMintoTarget&&nPathInx>=nselectedMinIndx)
// 		{
// 			dDist=dDistMintoTarget;
// 			TargetPos.setX( dPathX );
// 			TargetPos.setY( dPathY );
// 			nselectedTargetIndx=nPathInx;
// 			bselectedTargetflag=true;
// 		}	
// 
// 	}
// 
// 	//목표 지점에서의 법선 벡터
// 	dTargetTangentthetaDeg=atan2(m_vecPath[nselectedTargetIndx].getY()-m_vecPath[nselectedTargetIndx-1].getY(),m_vecPath[nselectedTargetIndx].getX()-m_vecPath[nselectedTargetIndx-1].getX());
// 
// 
// 	//새로운 로봇과 목표지점간의 거리 계산
// 	dnewDistToTarget=nDistToTarget;
// 	double dTempDist= fabs(fabs(dMinTangentthetaDeg)-fabs(dTargetTangentthetaDeg));
// //	printf("dTempDist=%f\n",dTempDist);
// 	dTempDist=dTempDist/M_PI*nDistToTarget/80;
// 
// 	if(dTempDist<1.0)	{dTempDist=1.0;}
// 	else if(dTempDist>3.0)	{ dTempDist=3.0;}
// 
// 	dnewDistToTarget=dnewDistToTarget/(dTempDist);
// 
// 
// 	//로봇과 목표지점간의 거리에 따른 목표점 선택
// 	nPathInx=0;
// 	if(dMinDistfromPath>=dnewDistToTarget) dDistMintoTarget=300;
// 	else
// 	{
// 		dDistMintoTarget=sqrt(pow((double)dnewDistToTarget,2)-pow(dMinDistfromPath,2));
// 		if(dDistMintoTarget<300) dDistMintoTarget=300;
// 	}
// 
// 	bselectedTargetflag= false;
// 	dTempDist=1000000000;
// 
// 	while( true ) {
// 
// 		nPathInx++;
// 		if(nPathInx >= nPathSize) {
// 
// 			nPathInx = nPathSize-1; //배열인덱스 이기 떄문에 1을 빼줘야 한다.
// 
// 			if(bselectedTargetflag==false)
// 			{
// 				(*dTargetDist)=dDistMintoTarget;
// 
// 				dPathX = m_vecPath[nselectedTargetIndx].getX();
// 				dPathY = m_vecPath[nselectedTargetIndx].getY();
// 				TargetPos.setX( dPathX );
// 				TargetPos.setY( dPathY );
// 				nselectedTargetIndx=nselectedMinIndx;
// 			}
// 			break;			
// 		}			
// 		dPathX = m_vecPath[nPathInx].getX();
// 		dPathY = m_vecPath[nPathInx].getY();			
// 
// 		dDist=hypot(MinPos.getX()-dPathX, MinPos.getY()- dPathY);
// 
// 		if(dDist<dTempDist&&dDist> dDistMintoTarget&&nPathInx>=nselectedMinIndx)
// 		{
// 			dTempDist=dDist;
// 			(*dTargetDist)=dTempDist;
// 			TargetPos.setX( dPathX );
// 			TargetPos.setY( dPathY );
// 			nselectedTargetIndx=nPathInx;
// 			bselectedTargetflag=true;
// 		}	
// 
// 	}
// 	return TargetPos;
// }

/**
 @brief Korean: 센서데이터를 이용하여 타겟점과 로봇 사이의 장애물 유무를 검사하는 함수
 @brief English: 
*/
bool GotoGoalBehavior::checkObstacles(KuPose RobotPos,KuPose TargetPos,int_1DArray nLaserData181, int_1DArray nKinectRnageData ,double dTargetDist)
{

	double dGradientX = TargetPos.getXm() - RobotPos.getXm();
	double dGradientY = TargetPos.getYm() - RobotPos.getYm();
	double dDistofObst=hypot(TargetPos.getX()- RobotPos.getX(), TargetPos.getY()- RobotPos.getY());
	double dDistObstoRobot=dTargetDist*0.6;
	double dTempDistObstoRobot=dTargetDist;
	int nDistIndx=(int)dDistObstoRobot/100;
	double dOffset= (double)KuRobotParameter::getInstance()->getKinectXOffset();
	int nradiusofRobot=KuRobotParameter::getInstance()->getRobotRadius()*0.9;
	
	for(int i=0; i<Sensor::URG04LX_DATA_NUM181;i++)
	{
		if(nLaserData181[i]>50&&nLaserData181[i]<300)
		{
				return true;	 
		}
	}

	for (int ndistance = nDistIndx; ndistance <dDistObstoRobot; ndistance += nDistIndx) {

		double dRayOfX =  RobotPos.getX()+ ndistance*dGradientX;
		double dRayOfY = RobotPos.getY()+ ndistance*dGradientY;

		dTempDistObstoRobot=hypot(ndistance*dGradientX, ndistance*dGradientY);

		for(int i=0;i<Sensor::KINECT_SENSOR_FOV;i++){
			
			if(nLaserData181[i+66]<50)continue;
			if(nKinectRnageData[i] <50)continue;

			if((nKinectRnageData[i] != 1000000&&nKinectRnageData[i] >50)&&nKinectRnageData[i] <dDistofObst){
				double dAngleRad = (double)(i -  Sensor::KINECT_SENSOR_FOV/2) * D2R;
				double dX = RobotPos.getX() + ((double)nKinectRnageData[i] * cos(dAngleRad) + dOffset ) * cos(RobotPos.getThetaRad()) + 
					((double)nKinectRnageData[i] * sin(dAngleRad) ) * sin(-RobotPos.getThetaRad());
				double dY = RobotPos.getY() + ((double)nKinectRnageData[i] * cos(dAngleRad) + dOffset ) * sin(RobotPos.getThetaRad()) + 
					((double)nKinectRnageData[i] * sin(dAngleRad) ) * cos(RobotPos.getThetaRad());
				
				double tempD=hypot(dRayOfX-dX, dRayOfY- dY);

				if(tempD<nradiusofRobot&&tempD>0 ){	return true;	}// 거리에거리에 따른 장애물 감지
			}
			else if((nLaserData181[i+66] != 1000000&&nLaserData181[i+66]>50)&&nLaserData181[i+66]<dDistofObst){
				double dAngleRad = (double)(i -  Sensor::KINECT_SENSOR_FOV/2) * D2R;
				double dX = RobotPos.getX() + ((double)nLaserData181[i+66]* cos(dAngleRad) + dOffset ) * cos(RobotPos.getThetaRad()) + 
					((double)nLaserData181[i+66]* sin(dAngleRad) ) * sin(-RobotPos.getThetaRad());
				double dY = RobotPos.getY() + ((double)nLaserData181[i+66] * cos(dAngleRad) + dOffset ) * sin(RobotPos.getThetaRad()) + 
					((double)nLaserData181[i+66] * sin(dAngleRad) ) * cos(RobotPos.getThetaRad());

				double tempD=hypot(dRayOfX-dX, dRayOfY- dY);

				if(tempD<nradiusofRobot&&tempD>0 )
				{
					return true;	
				}// 거리에 따른 장애물 감지
			}
		}
	}	
	return false;
}
/**
 @brief Korean: 센서데이터를 이용하여 타겟점과 로봇 사이의 장애물 유무를 검사하는 함수
 @brief English: 
*/
bool GotoGoalBehavior::checkObstacles(int_1DArray nLaserData181, int_1DArray nKinectRnageData)
{

	int nLaserMinDis=KuRobotParameter::getInstance()->getURG04LXLaserMinDist();
	int nKinectMinDis=KuRobotParameter::getInstance()->getKinectMinDist();

	int nStart=(Sensor::URG04LX_DATA_NUM181-Sensor::KINECT_SENSOR_FOV)/2.0;
	int nEnd=(Sensor::URG04LX_DATA_NUM181+Sensor::KINECT_SENSOR_FOV)/2.0;

	for(int i=0;i<Sensor::URG04LX_DATA_NUM181;i++)
	{
		if(m_nLaserData181[i]<500)
		{
			return false;
		}

		if(i>nStart&&i<nEnd)
		{
			
			if(nKinectRnageData[i-nStart]<500)
			{
				return false;
			}
			
		}

	}
	return true;
}
/**
@brief Korean: 웨이포인트에 타겟포인트가 도달했는지 검사하는 함수
@brief English: 
*/
/*
KuPose GotoGoalBehavior::checkWayPoint(KuPose TargetPos,KuPose RobotPos, int *nselectIdx ,bool *bWaitflag)
{
	int nWayPointIdx=0;
	int nPathIdx=0;
	double dWayPointX;
	double dWayPointY;
	double dDist=INFINITY_VALUE;
	int nMarginGain=3;
	//웨이포인트에 정지했을 때 정지 반경때문에 못 미쳐정차할 시 
	//로봇 위치에서 타겟 점사이 범위에 웨이 포인트가 들어가는 것을 막기 위한 게인(1~3)

	int nWayPointSize = m_vecWayPoint.size();
	int nPathSize = m_vecPath.size();

	KuPose selectPos;

	while( true ) {

		if(nWayPointIdx >= nWayPointSize) {
			nWayPointIdx = nWayPointSize-1; //배열인덱스 이기 떄문에 1을 빼줘야 한다.
			break;
		}
		dWayPointX = m_vecWayPoint[nWayPointIdx].getX();
		dWayPointY = m_vecWayPoint[nWayPointIdx].getY();

		dDist=hypot(TargetPos.getX()-dWayPointX, TargetPos.getY()- dWayPointY);

		if(dDist<200&&nWayPointIdx!=nWayPointSize-1)
		{
			(*bWaitflag)=true;
			(*nselectIdx)=nWayPointIdx;		
		}	

		nPathIdx=0;
		while(true)//웨이포인트와 같은 패스위의 지점의 인덱스를 구함
		{
			if(	nPathIdx >= nPathSize) {
				nPathIdx = nPathSize-1; //배열인덱스 이기 떄문에 1을 빼줘야 한다.
				break;
			}
			dDist=hypot(m_vecPath[nPathIdx].getX()-dWayPointX, m_vecPath[nPathIdx].getY()- dWayPointY);

			if(dDist == 0)
			{
				break;
			}	
			nPathIdx++;
		}

		if((m_nSelectedMinIndx+nMarginGain < nPathIdx)&&(nPathIdx < m_nSelectTargetIdx))
			//패스위의 로봇에서 가까운 인덱스와 패스위의 타겟인덱스 사이에 웨이포인트인덱스(nPathIdx)가있다면
		{
			(*bWaitflag)=true;
			(*nselectIdx)=nWayPointIdx;	
		}
		nWayPointIdx++;
	}

	if((*bWaitflag)==true&&(*nselectIdx)!=-1){
		selectPos.setX( m_vecWayPoint[(*nselectIdx)].getX());
		selectPos.setY( m_vecWayPoint[(*nselectIdx)].getY() );
		selectPos.setPro( m_vecWayPoint[(*nselectIdx)].getPro() );
		return selectPos;
	}
	return TargetPos;
}*/
/**
@brief Korean: 웨이포인트에 타겟포인트가 도달했는지 검사하는 함수
@brief English: 
*/
KuPose GotoGoalBehavior::checkWayPoint(KuPose TargetPos,KuPose RobotPos, int *nselectIdx ,bool *bWaitflag)
{
	int nWayPointIdx=0;
	int nPathIdx=0;
	double dWayPointX;
	double dWayPointY;
	double dDist=INFINITY_VALUE;

	int nWayPointSize = m_vecWayPoint.size();
	int nPathSize = m_vecPath.size();
	m_vecWayPoint[0].setID(1);


	int nMinIdx=0;
	double dMinDistfromPath=10000000000;
	int nselectedMinIndx=0;
	if((*nselectIdx)>0)
	//로봇에서 부터의 최소가 되는 지점 및 거리값
	while( true ) {
		
		if(nMinIdx >= nPathSize) {
			nMinIdx = nPathSize-1; //배열인덱스 이기 떄문에 1을 빼줘야 한다.
			break;
		}
		double dMinPathX = m_vecWayPoint[(*nselectIdx)].getX();
		double dMinPathY = m_vecWayPoint[(*nselectIdx)].getY();

		dDist=hypot(m_vecPath[nMinIdx].getX()-dMinPathX, m_vecPath[nMinIdx].getY()- dMinPathY);

		if(dDist< dMinDistfromPath)
		{
			dMinDistfromPath=dDist;
			nselectedMinIndx=nMinIdx;
		}		
		nMinIdx++;
	}



	KuPose selectPos;
	if((*bWaitflag)==false){

		if(m_nSelectedMinIndx>nPathSize-1) m_nSelectedMinIndx=nPathSize-1;

		for(int nPathIdx=m_nSelectedMinIndx; nPathIdx>nselectedMinIndx;nPathIdx--)
		{
			for(int nWayPointIdx=1; nWayPointIdx<nWayPointSize;nWayPointIdx++)
			{

				dWayPointX = m_vecWayPoint[nWayPointIdx].getX();
				dWayPointY = m_vecWayPoint[nWayPointIdx].getY();
				
				if(nPathSize-1<nPathIdx) continue;

				dDist=hypot(m_vecPath[nPathIdx].getX()-dWayPointX, m_vecPath[nPathIdx].getY()- dWayPointY);

				if(dDist < 100&&m_vecWayPoint[nWayPointIdx].getID()!=1&&m_vecWayPoint[nWayPointIdx-1].getID()==1)
				{
					(*bWaitflag)=true;
					(*nselectIdx)=nWayPointIdx;		
				}	
			}
		}

		if((*bWaitflag)==true)
		{	
			if(nWayPointSize-1<(*nselectIdx)) return TargetPos;

			m_vecWayPoint[(*nselectIdx)].setID(1);
			selectPos.setX( m_vecWayPoint[(*nselectIdx)].getX());
			selectPos.setY( m_vecWayPoint[(*nselectIdx)].getY() );
			selectPos.setPro( m_vecWayPoint[(*nselectIdx)].getPro() );
			return selectPos;
		}

	}

	if((*bWaitflag)==true){
		if(nWayPointSize-1<(*nselectIdx)) return TargetPos;

		selectPos.setX( m_vecWayPoint[(*nselectIdx)].getX());
		selectPos.setY( m_vecWayPoint[(*nselectIdx)].getY() );
		selectPos.setPro( m_vecWayPoint[(*nselectIdx)].getPro() );
		return selectPos;
	}
	return TargetPos;
}
/**
@brief Korean: 웨이포인트에 타겟포인트가 도달했는지 검사하는 함수
@brief English: 
*/
KuPose GotoGoalBehavior::checkWayPointwithlocalpath(KuPose TargetPos,KuPose RobotPos, int *nselectIdx ,bool *bWaitflag)
{
	int nWayPointIdx=0;
	int nPathIdx=0;
	double dWayPointX;
	double dWayPointY;
	double dDist=INFINITY_VALUE;
	int nMarginGain=3;//웨이포인트에 정지했을 때 정지 반경때문에 못 미쳐정차할시 로봇 위치에서 타겟 점사이 범위에 웨이 포인트가 들어가는 것을 막기 위한 게인(1~3)

	int nWayPointSize = m_vecWayPoint.size();
	int nPathSize = m_vecLocalPath.size();

	KuPose selectPos;
	if((*bWaitflag)==false){

		if(m_nSelectedMinIndx>nPathSize-1) m_nSelectedMinIndx=nPathSize-1;//이곳을 추가해주세요.


		for(int nPathIdx=m_nSelectedMinIndx; nPathIdx>0;nPathIdx--)
		{
			for(int nWayPointIdx=1; nWayPointIdx<nWayPointSize;nWayPointIdx++)
			{

				dWayPointX = m_vecWayPoint[nWayPointIdx].getX();
				dWayPointY = m_vecWayPoint[nWayPointIdx].getY();

				dDist=hypot(m_vecLocalPath[nPathIdx].getX()-dWayPointX, m_vecLocalPath[nPathIdx].getY()- dWayPointY);

				if(dDist < 500&&m_vecWayPoint[nWayPointIdx].getID()!=1)
				{
					(*bWaitflag)=true;
					(*nselectIdx)=nWayPointIdx;		
					nPathIdx=0;
					m_vecWayPoint[nWayPointIdx].setID(1);

					selectPos.setX( m_vecWayPoint[(*nselectIdx)].getX());
					selectPos.setY( m_vecWayPoint[(*nselectIdx)].getY() );
					selectPos.setPro( m_vecWayPoint[(*nselectIdx)].getPro() );
					return selectPos;
				}	
			}
		}

	}

	if((*bWaitflag)==true){
		selectPos.setX( m_vecWayPoint[(*nselectIdx)].getX());
		selectPos.setY( m_vecWayPoint[(*nselectIdx)].getY() );
		selectPos.setPro( m_vecWayPoint[(*nselectIdx)].getPro() );
		return selectPos;
	}
	return TargetPos;
}

// KuPose GotoGoalBehavior::checkWayPoint(KuPose TargetPos,KuPose RobotPos, int *nselectIdx ,bool *bWaitflag)
// {
// 	int nWayPointIdx=0;
// 	int nPathIdx=0;
// 	double dWayPointX;
// 	double dWayPointY;
// 	double dDist=INFINITY_VALUE;
// 	int nMarginGain=3;//웨이포인트에 정지했을 때 정지 반경때문에 못 미쳐정차할시 로봇 위치에서 타겟 점사이 범위에 웨이 포인트가 들어가는 것을 막기 위한 게인(1~3)
// 
// 	int nWayPointSize = m_vecWayPoint.size();
// 	int nPathSize = m_vecPath.size();
// 
// 	KuPose selectPos;
// 	if((*bWaitflag)==false){
// 		while( true ) {
// 
// 			if(nWayPointIdx >= nWayPointSize) {
// 				nWayPointIdx = nWayPointSize-1; //배열인덱스 이기 떄문에 1을 빼줘야 한다.
// 				break;
// 			}
// 			dWayPointX = m_vecWayPoint[nWayPointIdx].getX();
// 			dWayPointY = m_vecWayPoint[nWayPointIdx].getY();
// 
// 			dDist=hypot(TargetPos.getX()-dWayPointX, TargetPos.getY()- dWayPointY);
// 
// 			if(dDist<200)
// 			{
// 				(*bWaitflag)=true;
// 				(*nselectIdx)=nWayPointIdx;		
// 			}	
// 
// 			nPathIdx=0;
// 			while(true)//웨이포인트와 같은 패스위의 지점의 인덱스를 구함
// 			{
// 				if(	nPathIdx >= nPathSize) {
// 					nPathIdx = nPathSize-1; //배열인덱스 이기 떄문에 1을 빼줘야 한다.
// 					break;
// 				}
// 				dDist=hypot(m_vecPath[nPathIdx].getX()-dWayPointX, m_vecPath[nPathIdx].getY()- dWayPointY);
// 
// 				if(dDist == 0)
// 				{
// 					break;
// 				}	
// 				nPathIdx++;
// 			}
// 
// 			if((m_nSelectedMinIndx+nMarginGain < nPathIdx)&&(nPathIdx < m_nSelectTargetIdx))//패스위의 로봇에서 가까운 인덱스와 패스위의 타겟인덱스 사이에 웨이포인트인덱스(nPathIdx)가있다면
// 			{
// 				(*bWaitflag)=true;
// 				(*nselectIdx)=nWayPointIdx;	
// 			}
// 			nWayPointIdx++;
// 		}
// 	}
// 
// 	if((*bWaitflag)==true){
// 		selectPos.setX( m_vecWayPoint[(*nselectIdx)].getX());
// 		selectPos.setY( m_vecWayPoint[(*nselectIdx)].getY() );
// 		selectPos.setPro( m_vecWayPoint[(*nselectIdx)].getPro() );
// 		return selectPos;
// 	}
// 	return TargetPos;
// }
bool GotoGoalBehavior::alignRobot(int_1DArray nData, KuPose GoalPos)
{
	char cDataLogFileName[300];
	memset(cDataLogFileName,0,sizeof(cDataLogFileName));
	string strDataPath = KuRobotParameter::getInstance()->getOutlineWayPointNameNPath();
	sprintf(cDataLogFileName,"%s/%d_%d_%0.2f.log",strDataPath.c_str(),(int)GoalPos.getX(),(int)GoalPos.getY(),GoalPos.getThetaDeg());

	cout<<"cDataLogFileName=="<<cDataLogFileName<<endl;
	int* nLaserData = NULL;

	KuPose tempDeltaPose, DeltaPose;//,GoalPos;

	int nPreRangeData_t[181];
	int_1DArray nPreRangeData = m_KuUtil.generateIntType1DArray(Sensor::URG04LX_DATA_NUM181,0);

	for (int i = 0 ; i < 181 ; i++){
		nPreRangeData_t[i] = 0;
	}

	FILE *fp;

	if((fp = fopen(cDataLogFileName, "r")) == NULL) {
		printf("File not found...\n");
		return false;
	}else{
		cout<<"DataLogFile loading success!!!"<<endl;

		int i = 0;
		while(!feof(fp)){
			fscanf(fp, "%d", &nPreRangeData_t[i]);
			i++;
		}
		for (int i = 0 ; i < 181 ; i++){
			nPreRangeData[i] = nPreRangeData_t[i];
		}
	}
	KuDrawingInfo::getInstance()->setAuxiliaryRobotPos(GoalPos);	
	KuDrawingInfo::getInstance()->setAlignLaserData181(nPreRangeData);
	tempDeltaPose = KuScanMatchingLocalizerPr::getInstance()->computeDeltaPose( nPreRangeData,  nData);

	if(tempDeltaPose.getPro()>0.001) return false;

	printf("DeltaPose.getX()=%f,DeltaPose.getY()=%f,DeltaPose.getThetaDeg()=%f\n",tempDeltaPose.getX(),tempDeltaPose.getY(),tempDeltaPose.getThetaDeg());

	double dDeltaX=tempDeltaPose.getX()*cos(GoalPos.getThetaRad() /*+90*D2R*/)-tempDeltaPose.getY()*sin(GoalPos.getThetaRad() /*+90*D2R*/);
	double dDeltaY=tempDeltaPose.getX()*sin(GoalPos.getThetaRad() /*+90*D2R*/)+tempDeltaPose.getY()*cos(GoalPos.getThetaRad() /*+90*D2R*/);

	DeltaPose.setX(dDeltaX);
	DeltaPose.setY(dDeltaY);
	DeltaPose.setThetaDeg(tempDeltaPose.getThetaDeg());

	m_RobotPos.setX(GoalPos.getX() + DeltaPose.getX());
	m_RobotPos.setY(GoalPos.getY() + DeltaPose.getY());
	// vision
	m_RobotPos.setThetaDeg(GoalPos.getThetaDeg() + DeltaPose.getThetaDeg());

	KuDrawingInfo::getInstance()->setRobotPos(m_RobotPos);

	return true;
}
/**
@brief Korean: 스레드로 돌아가는 함수
@brief English: 
*/
void GotoGoalBehavior::doThreadForMapping(void* arg)
{
	GotoGoalBehavior* pGGB = (GotoGoalBehavior*)arg;
	KuMapBuilderParameter InputParam;
	CCriticalSection Critsec;
	int_1DArray nLaserData181 = SensorSupervisor::getInstance()->getLaserDataFront();
	KuPose DelEncoderData =SensorSupervisor::getInstance()->getEncoderDelPos();
	double dGyro = SensorSupervisor::getInstance()->getGyroData();
	DelEncoderData.setThetaDeg(dGyro);
	bool bGoalBnd=false;

	if(pGGB->m_math.calcDistBetweenPoses(pGGB->m_GoalPos,pGGB->m_RobotPos) <1000){
		bGoalBnd=true;		
	}

	//if(pGGB->m_RobotPos.getID()==1&&fabs(dGyro)<3&&bGoalBnd==false)
	if(pGGB->m_bMapping&&fabs(dGyro)<2&&bGoalBnd==false&&KuILBPFLocalizerPr::getInstance()->checkdoMapping()==true)
	{
		printf("dGyro=%f\n",dGyro);
		InputParam.setDelRobotPos(DelEncoderData);
		InputParam.setRobotPos(pGGB->m_RobotPos);
		InputParam.setLaserData(nLaserData181);//맵빌딩에 필요한 각종 데이터를 맵빌더 파라미터에 삽입
		pGGB->m_LaserMapBuilder.buildMapFront(InputParam);//맵빌딩 시작

		Critsec.Lock();
		pGGB->copyBuildingMapToGlobalMap(pGGB->m_LaserMapBuilder.getMap(), pGGB->m_pMap->getMap(), pGGB->m_RobotPos);
		Critsec.Unlock();
		pGGB->m_LaserMapBuilder.initMap();
	}
	
	//이 부분 시작----------------------------------------------------------------------------
	double dCellSize= 100.0;
	double dRadiusofRobot_cell =KuRobotParameter::getInstance()->getRobotRadius()/dCellSize;
	
	int nRobotX=pGGB->m_RobotPos.getX()/dCellSize;
	int nRobotY=pGGB->m_RobotPos.getY()/dCellSize;

	for(int i=-dRadiusofRobot_cell; i<dRadiusofRobot_cell;i++)
	{
		for(int j=-dRadiusofRobot_cell; j<dRadiusofRobot_cell;j++)
		{
			if (nRobotX+i<0 || nRobotY+j<0 || nRobotX+i>=pGGB->m_pMap->getX() || nRobotY+j>=pGGB->m_pMap->getY()) {continue;}	// 맵사이즈를 벗어날 경우 예외 처리
			pGGB->m_pMap->getMap()[nRobotX+i][nRobotY+j] = KuMap::EMPTY_AREA;	
		}
	}
	//이 부분 끝===================================================================================
}
/**
@brief Korean: 스레드로 돌아가는 함수
@brief English: 
*/
// 황서연(140409): 소리 재생이 독립 thread에서 실행되지 않아 장애물 감지 시 doThread가 원활히 수행되지 않는 문제를 해결하고자 비활성화 함.
/*
void GotoGoalBehavior::doObstacleThread(void* arg)
{
	GotoGoalBehavior* pGGB = (GotoGoalBehavior*)arg;	

	if(pGGB->m_nObstacleDetectionTime<pGGB->m_dCheckObstacleDetectionTime)
	{
		pGGB->m_dCheckObstacleDetectionTime=0;
		pGGB->playSound();
	}

}
*/

void GotoGoalBehavior::thread_sound_obstacle_detection(void* arg)
{
	GotoGoalBehavior* pGGB = (GotoGoalBehavior*)arg;

	if(pGGB->m_bPlaySoundObstacleDetection)
	{
		pGGB->playSound("a.wav");
		Sleep(2000);
	}
}

void GotoGoalBehavior::thread_sound_low_battery(void* arg)
{
	GotoGoalBehavior* pGGB = (GotoGoalBehavior*)arg;

	if(pGGB->m_bPlaySoundLowBattery)
	{
		if(KuPRIMUSCommSupervisor::getInstance()->getExternalBatteryVoltage() > 0)
		{
			if(KuPRIMUSCommSupervisor::getInstance()->getExternalBatteryVoltage() < KuRobotParameter::getInstance()->getLowBatteryAlarmVoltage())
			{
				pGGB->playSound("low_battery.wav");
				Sleep(30000); // 30 sec.
			}
		}
	}
}

/**
 @brief Korean: 현재 사용중인  Localizer를 넘겨주는 함수
 @brief English: 
*/
Localizer* GotoGoalBehavior::getLocalizer()
{
	return KuLBPFLocalizerPr::getInstance();

}

/**
 @brief Korean: 현재 Behavior의 상태를 나타내는 함수
 @brief English: 
*/
bool GotoGoalBehavior::getBehaviorStates()
{
	return m_bThreadFlag;
}

/**
 @brief Korean: 클래스를 종료한다.
 @brief English: 
*/
void GotoGoalBehavior::terminate()
{
	m_Thread.terminate();
	m_thread_sound_obstacle_detection.terminate();
	m_thread_sound_low_battery.terminate();
	
	SSAGVWheelActuatorInterface::getInstance()->stop();
	if(KuDrawingInfo::getInstance()->getDirectionofPathflag()==true)
	{
		KuDrawingInfo::getInstance()->setDirectionofPathflag(false);
	}
	else
	{
		KuDrawingInfo::getInstance()->setDirectionofPathflag(true);
	}

	if(m_pMap!=NULL)kuMapRepository::getInstance()->saveMapNavi(m_pMap);
	
	saveRobotData();

	if(KuRobotParameter::getInstance()->getLocalization()== Localizer::PARTICLEFILTER)
	{
		 KuLBPFLocalizerPr::getInstance()->terminate();	
	}
	else if(KuRobotParameter::getInstance()->getLocalization()== Localizer::SCANMATCHING)
	{
		KuILBPFLocalizerPr::getInstance()->terminate();	
		KuScanMatchingLocalizerPr::getInstance()->terminate();
		KuFiducialbasedLocalizerPr::getInstance()->terminate();
	}	
	
	m_bThreadFlag = false;	
}

void GotoGoalBehavior::saveRobotData()
{

	if(m_dTimeB==0)	{m_dTimeB=finishTimeCheck(m_LITime);}

	m_DataLog<<m_nIDX<<" "<<m_StartPoint.getX()<<" "<<m_StartPoint.getY()<<" "<<m_StartPoint.getThetaDeg()<<" ";
	m_DataLog<<m_WayPoint.getX()<<" "<<m_WayPoint.getY()<<" "<<m_WayPoint.getThetaDeg()<<" ";
	m_DataLog<<m_RobotPos.getX()<<" "<<m_RobotPos.getY()<<" "<<m_RobotPos.getThetaDeg()<<" ";
	m_DataLog<<m_dTimeA<<" "<<m_dTimeB<<";"<<endl;
	m_nIDX++;
}

/**
 @brief Korean: Localizer를 시작한다.
 @brief English: 
*/
void GotoGoalBehavior::startLocalizer(KuPose RobotPos,bool blocalizer)
{

	if(blocalizer==Localizer::PARTICLEFILTER)
	{
		//INI 파일로부터 Particle filter을 수행하기 위해 필요한 정보를 얻어온다.------------------------------------
		int nMaxSampleNum = KuRobotParameter::getInstance()->getMaxParticleNum();
		int nMinSampleNum = KuRobotParameter::getInstance()->getMinParticleNUm();
		double dDevationforTrans = KuRobotParameter::getInstance()->getDeviationforTrans();
		double dDevationforRotae = KuRobotParameter::getInstance()->getDeviationforRotate();
		double dDevationforTransRotae = KuRobotParameter::getInstance()->getDeviationforTransRotate();
		//==================================================================================================================
		KuLBPFLocalizerPr::getInstance()->setDeviation(dDevationforTrans,dDevationforRotae,dDevationforTransRotae);
		KuLBPFLocalizerPr::getInstance()->setSampleNum(nMaxSampleNum,nMinSampleNum);

		int** nMap = kuMapRepository::getInstance()->getMap()->getMap();
		KuLBPFLocalizerPr::getInstance()->setMap(KuDrawingInfo::getInstance()->getMapSizeX(), KuDrawingInfo::getInstance()->getMapSizeY(),nMap);
		KuLBPFLocalizerPr::getInstance()->setRobotPos(RobotPos);	
		KuLBPFLocalizerPr::getInstance()->spreadParticleNearRobot(RobotPos,0.3);		
		KuLBPFLocalizerPr::getInstance()->start(50);	
	}
	else if(blocalizer==Localizer::SCANMATCHING)
	{
		initICPProcess();

		//INI 파일로부터 Particle filter을 수행하기 위해 필요한 정보를 얻어온다.------------------------------------
		int nMaxSampleNum = KuRobotParameter::getInstance()->getMaxParticleNum();
		int nMinSampleNum = KuRobotParameter::getInstance()->getMinParticleNUm();
		double dDevationforTrans = KuRobotParameter::getInstance()->getDeviationforTrans();
		double dDevationforRotae = KuRobotParameter::getInstance()->getDeviationforRotate();
		double dDevationforTransRotae = KuRobotParameter::getInstance()->getDeviationforTransRotate();
		//==================================================================================================================

		KuILBPFLocalizerPr::getInstance()->setDeviation(dDevationforTrans,dDevationforRotae,dDevationforTransRotae);
		KuILBPFLocalizerPr::getInstance()->setSampleNum(nMaxSampleNum,nMinSampleNum);

		m_pMap = kuMapRepository::getInstance()->getMap();
		int**  nMap = m_pMap->getMap();
		KuILBPFLocalizerPr::getInstance()->setMap(KuDrawingInfo::getInstance()->getMapSizeX(), KuDrawingInfo::getInstance()->getMapSizeY(),nMap);
		KuILBPFLocalizerPr::getInstance()->setRobotPos(m_RobotPos);
		KuILBPFLocalizerPr::getInstance()->spreadParticleNearRobot(m_RobotPos,0.3);	

		if(KuILBPFLocalizerPr::getInstance()->getThreadStates()==false)
			KuILBPFLocalizerPr::getInstance()->start(50);	
	}

	KuFiducialbasedLocalizerPr::getInstance()->initFiducialMark();
	if(KuFiducialbasedLocalizerPr::getInstance()->getThreadStates()==false)
		KuFiducialbasedLocalizerPr::getInstance()->start(100);
}

/**
@brief Korean: 소요 시간을 측정하기 위해서 초기화하는 함수
@brief English: Initializes to count the duration
*/
void GotoGoalBehavior::startTimeCheck(LARGE_INTEGER& nStart)
{
	QueryPerformanceCounter(&nStart);
}
/**
@brief Korean: 측정된 소요 시간을 받아오는 함수
@brief English: Gets the estimated duration
*/
float GotoGoalBehavior::finishTimeCheck(const LARGE_INTEGER nStart)
{
	LARGE_INTEGER nFinish, freq;

	QueryPerformanceFrequency(&freq);

	QueryPerformanceCounter(&nFinish);

	return (float)(1000.0 * (nFinish.QuadPart - nStart.QuadPart) / freq.QuadPart);
}


/**
 @brief Korean: GotoGoalBehavior를 실행시키는 함수
 @brief English: 
*/
bool GotoGoalBehavior::execute(KuCommandMessage CMessage )
{
	
	switch(CMessage.getCommandName()){

	case KuCommandMessage::START_THREAD:	
		if(initialize(CMessage)) 
			if(m_bAvoidModeflag){
				m_Thread.start(doThreadwithAvoid,this,100, "gotogoal_with_avoidance"); //메인 스레드 시작	
			}
			else{
				startTimeCheck(m_LITime);	
				m_Thread.start(doThread,this,100, "gotogoal_without_avoidance"); //메인 스레드 시작	
				m_KuMappingThread.start(doThreadForMapping, this, 100, "gotogoal_with_mapping"); //메인 스레드 시작
				//	m_ObstacleThread.start(doObstacleThread,this,100); 
				m_thread_sound_obstacle_detection.start(thread_sound_obstacle_detection, this, 200, "sound_obstacle_detection");
			}
		break;
	case KuCommandMessage::TERMINATE_THREAD:
		terminate();
		break;
	case KuCommandMessage::SUSPEND_THREAD:
		m_Thread.suspend();
		//m_ObstacleThread.suspend();
		m_thread_sound_obstacle_detection.suspend();
		break;
	case KuCommandMessage::RESUME_THREAD:
		m_Thread.resume();
		//m_ObstacleThread.resume();
		m_thread_sound_obstacle_detection.resume();
		break;
	default:break;
	}

	return true;
}
