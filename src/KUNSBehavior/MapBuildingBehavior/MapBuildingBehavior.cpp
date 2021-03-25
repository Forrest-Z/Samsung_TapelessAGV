#include "stdafx.h"
#include "MapBuildingBehavior.h"

MapBuildingBehavior::MapBuildingBehavior()
{
	initialize(); //초기화 작업
	cout<<"[MapBuildingBehavior]: Instance is created!!!"<<endl;
	m_pMap=NULL;
	m_IplCeilingCamera = cvCreateImage(cvSize( Sensor::CEILING_IMAGE_WIDTH, Sensor::CEILING_IMAGE_HEIGHT),8,1);
}

MapBuildingBehavior::~MapBuildingBehavior()
{
	cout<<"[MapBuildingBehavior]: Instance is destroyed!!!"<<endl;
	if(m_pMap!=NULL){
		delete m_pMap;

	}
}

/**
@brief Korean: 초기화 작업을 수행하는 함수.
@brief English: 
*/
void MapBuildingBehavior::initialize()
{
	m_bThreadFlag = false;

}
/**
@brief Korean: 초기화 작업을 수행하는 함수.
@brief English: 
*/
void MapBuildingBehavior::initialize(KuCommandMessage CMessage)
{
	m_nLaserDataFront = m_KuUtil.generateIntType1DArray(Sensor::URG04LX_DATA_NUM181,3900); //거리센서 정보를 저장하는 변수 초기화
	m_nLaserDataRear = m_KuUtil.generateIntType1DArray(Sensor::URG04LX_DATA_NUM181,3900); //거리센서 정보를 저장하는 변수 초기화

	m_bThreadFlag = true;
	m_RobotPos=CMessage.getRobotPos();
	m_nThreadFuncPeriod = CMessage.getBehaviorPeriod(); //스레드 함수 실행주기 입력.

	m_nMapSizeXm =0,m_nMapSizeYm =0;
	CMessage.getMapSizeXmYm(&m_nMapSizeXm, &m_nMapSizeYm);
	KuPose MBRobotPose = KuRobotParameter::getInstance()->getInitRobotPoseForMap();
	m_RobotPos.setXm(MBRobotPose.getXm());
	m_RobotPos.setYm(MBRobotPose.getYm());
	m_RobotPos.setThetaDeg(MBRobotPose.getThetaDeg());

	if(NULL==m_pMap)
	{
		//initialize map------------------------------------------------------------------
		m_pMap = new KuMap(m_nMapSizeXm*10, m_nMapSizeYm*10); 
		KuDrawingInfo::getInstance()->setMap(m_pMap); 
		//initialize map===================================================
	}

	KuMapBuilderParameter InputParamFront, InputParamRear;

	InputParamFront.setMapSizeXmYm(m_nMapSizeXm, m_nMapSizeYm);
	InputParamFront.setLaserScanIdx(Sensor::URG04LX_DATA_NUM181);
	InputParamFront.setMinDistofSensorData(KuRobotParameter::getInstance()->getURG04LXLaserMinDist()); // unit mm
	InputParamFront.setMaxDistofSensorData(KuRobotParameter::getInstance()->getURG04LXLaserMaxDist()); // unit mm
	InputParamFront.setLaserXOffset(KuRobotParameter::getInstance()->getFrontLaserXOffset());

	InputParamRear.setMapSizeXmYm(m_nMapSizeXm, m_nMapSizeYm);
	InputParamRear.setLaserScanIdx(Sensor::URG04LX_DATA_NUM181);
	InputParamRear.setMinDistofSensorData(KuRobotParameter::getInstance()->getURG04LXLaserMinDist()); // unit mm
	InputParamRear.setMaxDistofSensorData(KuRobotParameter::getInstance()->getURG04LXLaserMaxDist()); // unit mm
	InputParamRear.setLaserXOffset(KuRobotParameter::getInstance()->getRearLaserXOffset());

	m_LaserMapBuilder.initialize(InputParamFront, InputParamRear);

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
	// 	int** nCeilingMap = kuMapRepository::getInstance()->getCeilingMap();//
	KuILBPFLocalizerPr::getInstance()->setMap(KuDrawingInfo::getInstance()->getMapSizeX(), KuDrawingInfo::getInstance()->getMapSizeY(),nMap);
	//	KuILBPFLocalizerPr::getInstance()->setCeilingMap(KuDrawingInfo::getInstance()->getMapSizeX(), KuDrawingInfo::getInstance()->getMapSizeY(),nCeilingMap);
	KuILBPFLocalizerPr::getInstance()->setRobotPos(m_RobotPos);
	KuILBPFLocalizerPr::getInstance()->spreadParticleNearRobot(m_RobotPos,0.3);	
	KuILBPFLocalizerPr::getInstance()->start(50,true);	

	KuFiducialbasedLocalizerPr::getInstance()->initFiducialMark();
	KuFiducialbasedLocalizerPr::getInstance()->start(100);

}

/**
@brief Korean: ICP 프로세스를 수행하기 위해 필요한 초기화 작업을 수행하는 함수.
@brief English: 
*/
void MapBuildingBehavior::initICPProcess()
{
	m_ICPLocalizer.setInitRobotPos(m_RobotPos);
	SensorSupervisor::getInstance()->readSensorData(); //sensor reading
	int_1DArray nLaserData = m_KuUtil.generateIntType1DArray(Sensor::URG04LX_DATA_NUM181,3900);
	KuPose EncoderDelPos;

	nLaserData = SensorSupervisor::getInstance()->getLaserDataFront();
	EncoderDelPos =SensorSupervisor::getInstance()->getEncoderDelPos();

}

/**
@brief Korean: 스레드로 돌아가는 함수
@brief English: 
*/
void MapBuildingBehavior::doThread(void* arg)
{
	MapBuildingBehavior* pMBB = (MapBuildingBehavior*)arg;

	KuMapBuilderParameter InputParamFront, InputParamRear;
	if(SensorSupervisor::getInstance()->readSensorData()==false) pMBB->terminate();

	pMBB->m_nLaserDataFront = SensorSupervisor::getInstance()->getLaserDataFront();
	pMBB->m_nLaserDataRear = SensorSupervisor::getInstance()->getLaserDataRear();
	pMBB->m_IplCeilingCamera=SensorSupervisor::getInstance()->getCeilingImageData();
	pMBB->m_EncoderDelPos = SensorSupervisor::getInstance()->getEncoderDelPos();
	double dGyro=SensorSupervisor::getInstance()->getGyroData();
	double dDistEc=hypot(pMBB->m_EncoderDelPos.getX(),pMBB->m_EncoderDelPos.getY())*10.0;
	if(dDistEc>KuRobotParameter::getInstance()->getMaxRobotVelocity()*1.5)
	{
		pMBB->m_EncoderDelPos.init();
	}
	else pMBB->m_EncoderDelPos.setThetaDeg(dGyro);

	pMBB->m_RobotPos = pMBB->m_ICPLocalizer.estimateRobotPos(pMBB->m_nLaserDataFront, pMBB->m_EncoderDelPos);
	KuILBPFLocalizerPr::getInstance()->estimateRobotPos(pMBB->m_nLaserDataFront, pMBB->m_RobotPos,pMBB->m_EncoderDelPos, pMBB->m_IplCeilingCamera,true);	

	KuFiducialbasedLocalizerPr::getInstance()->estimateRobotPosbyFiducialmark(pMBB->m_RobotPos,pMBB->m_IplCeilingCamera, true);

	// Front
	InputParamFront.setDelRobotPos(pMBB->m_EncoderDelPos); 
	InputParamFront.setRobotPos(pMBB->m_RobotPos);
	InputParamFront.setLaserData(pMBB->m_nLaserDataFront);
	pMBB->m_LaserMapBuilder.buildMapFront(InputParamFront);		

	// Rear
 	if(SensorSupervisor::getInstance()->isRearLRFConnected())
	{
		InputParamRear.setDelRobotPos(pMBB->m_EncoderDelPos); 
		InputParamRear.setRobotPos(pMBB->m_RobotPos);
		InputParamRear.setLaserData(pMBB->m_nLaserDataRear);
		pMBB->m_LaserMapBuilder.buildMapRear(InputParamRear);
	}

	pMBB->drawNaviData();

}
/**
@brief Korean: 주행 관련 정보를 그려주는 함수.
@brief English: 
*/
void MapBuildingBehavior::drawNaviData()
{
	KuDrawingInfo::getInstance()->setFrontLaserData181(m_nLaserDataFront);
	KuDrawingInfo::getInstance()->setRearLaserData181(m_nLaserDataRear);
	KuDrawingInfo::getInstance()->setBuildingMap(m_LaserMapBuilder.getProMap());
	KuDrawingInfo::getInstance()->setRobotPos(m_RobotPos);		
	KuDrawingInfo::getInstance()->setCeilingImageData(m_IplCeilingCamera);
	KuDrawingInfo::getInstance()->setRegion(KuILBPFLocalizerPr::getInstance()->getRegion());
	KuDrawingInfo::getInstance()->setTemplateData(KuILBPFLocalizerPr::getInstance()->getTemplateData());
	//list<KuPose> FiducialMarkImgCoordList = KuFiducialbasedLocalizerPr::getInstance()->getFiducialMarkImgCoordList();
	list<KuPose> FiducialMarkList = KuFiducialbasedLocalizerPr::getInstance()->getFiducialMarkList();
	//KuDrawingInfo::getInstance()->drawFiducialMarkInfoToImage(FiducialMarkImgCoordList);
	KuDrawingInfo::getInstance()->setFiducialMarkList(FiducialMarkList);
	KuPose FiducialMarkImgCoord = KuFiducialbasedLocalizerPr::getInstance()->getFiducialMarkImgCoord();
	KuDrawingInfo::getInstance()->drawFiducialMarkInfoToImage(FiducialMarkImgCoord);
}

/**
@brief Korean: 격자 지도를 BMP형태로 생성하는 함수
@brief English: 
*/
void MapBuildingBehavior::convertMapData2BMPFile(bool bFiltering /*= true*/, bool bRenderMapBuilding /*= false*/, bool bDrawingInfoSetMap /*= true*/)
{
	//격자지도를 BMP형태로 생성----------------------------------------------------------------------------
	m_pMap->setMap(m_LaserMapBuilder.getMap());
	kuMapRepository::getInstance()->saveMap(m_pMap, bFiltering, bDrawingInfoSetMap);
	KuDrawingInfo::getInstance()->setRenderBuildingMapflag(bRenderMapBuilding);
	//-----------------------------------------------------------------------------------------------------
}

/**
@brief Korean: 확률 격자지도를 BMP형태로 생성하는 함수
@brief English: 
*/
void MapBuildingBehavior::convertProbMapData2BMPFile(void)
{
	//격자지도를 BMP형태로 생성----------------------------------------------------------------------------
	m_pMap->setProbMap(m_LaserMapBuilder.getProMap()); // 확률 지도를 set
 	kuMapRepository::getInstance()->saveProbMap(m_pMap, m_RobotPos);
 	KuDrawingInfo::getInstance()->setRenderBuildingMapflag(true);
	//-----------------------------------------------------------------------------------------------------
}

/**
 * @brief 임시로 저장한 확률 격자지도를 로드
 * @date 2014/05/07
 * @param 
 * @return void
 */
bool MapBuildingBehavior::loadTemporaryMap(void)
{
	bool bRes(false);
	string strTempMapPath = KuRobotParameter::getInstance()->getTempMapPath();

	bRes = kuMapRepository::getInstance()->loadProbMap(strTempMapPath, m_LaserMapBuilder.getProMap(), m_RobotPos); // 격자지도 로딩
	m_ICPLocalizer.setRobotPos(m_RobotPos);

	return bRes;
}

/**
@brief Korean: 클래스를 종료한다.
@brief English: 
*/
void MapBuildingBehavior::terminate()
{
	KuThread::terminate();
	m_bThreadFlag = false;	
	if(m_pMap!=NULL)
	{
		convertMapData2BMPFile();
		KuILBPFLocalizerPr::getInstance()->saveFeatureData();
		KuILBPFLocalizerPr::getInstance()->saveTemplateData();
		KuFiducialbasedLocalizerPr::getInstance()->saveFiducialFeatureMap();
	}
	KuILBPFLocalizerPr::getInstance()->terminate();

}
/**
@brief Korean: Behavior의 상태를 가져가는 함수
@brief English: 
*/
bool MapBuildingBehavior::getBehaviorStates()
{
	return m_bThreadFlag;
}
/**
@brief Korean: Behavior에서 사용되는 Localizer를 가져간다
@brief English: 
*/
Localizer* MapBuildingBehavior::getLocalizer()
{
	return &m_ICPLocalizer;
}

/**
@brief Korean: MapBuildingBehavior를 실행시키는 함수
@brief English: 
*/
bool MapBuildingBehavior::execute(KuCommandMessage CMessage)
{
	switch(CMessage.getCommandName()){
	case KuCommandMessage::START_THREAD:
		initialize(CMessage);		
		initICPProcess();
		KuThread::start(doThread,this,200, "MapBuildingBehavior::execute()"); //메인 스레드 시작			
		break;
	case KuCommandMessage::TERMINATE_THREAD:
		terminate();
		break;
	case KuCommandMessage::SUSPEND_THREAD:
		suspend();
		break;
	case KuCommandMessage::RESUME_THREAD:
		resume();
		break;
	default:break;
	}

	return true;

}