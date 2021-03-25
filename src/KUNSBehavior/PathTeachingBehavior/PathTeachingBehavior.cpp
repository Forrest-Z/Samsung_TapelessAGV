#include "stdafx.h"
#include "PathTeachingBehavior.h"

PathTeachingBehavior::PathTeachingBehavior()
{
	m_bThreadFlag = false; //������ �Լ��� ����ǵ��� �ϴ� �÷���
	m_IplCeilingCamera = cvCreateImage(cvSize( Sensor::CEILING_IMAGE_WIDTH, Sensor::CEILING_IMAGE_HEIGHT),8,1);
	//m_pathIdx=1;

	m_nIFDX=-1;
	m_nFCount=0;
	m_dDistMarkfromRobot=DBL_MAX;
	m_dRecognizingMarkDistTh = KuRobotParameter::getInstance()->getRecognizingDistTh();
}

PathTeachingBehavior::~PathTeachingBehavior()
{

}
/**
 @brief Korean: �ʱ�ȭ �۾��� �����ϴ� �Լ�.
 @brief English: 
*/
bool PathTeachingBehavior::initialize(KuCommandMessage CMessage)
{
	m_bThreadFlag = true;
	m_nIFDX=-1;
	m_nFCount=0;
	m_dDistMarkfromRobot=DBL_MAX;
	m_dRecognizingMarkDistTh = KuRobotParameter::getInstance()->getRecognizingDistTh();

	m_RobotPos=KuDrawingInfo::getInstance()->getRobotPos();
	m_saveRobotPos=m_RobotPos;

	startLocalizer(m_RobotPos,KuRobotParameter::getInstance()->getLocalization()); 

	//initICPProcess();
	initTeachingPathProcess();
	
	return true;
}

/**
 @brief Korean: Localizer�� �����Ѵ�.
 @brief English: 
*/
void PathTeachingBehavior::startLocalizer(KuPose RobotPos,bool blocalizer)
{
	if(blocalizer==Localizer::PARTICLEFILTER)
	{
		//INI ���Ϸκ��� Particle filter�� �����ϱ� ���� �ʿ��� ������ ���´�.------------------------------------
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

		//INI ���Ϸκ��� Particle filter�� �����ϱ� ���� �ʿ��� ������ ���´�.------------------------------------
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
 @brief Korean: ���� �������  Localizer�� �Ѱ��ִ� �Լ�
 @brief English: 
*/
Localizer* PathTeachingBehavior::getLocalizer()
{
	return KuLBPFLocalizerPr::getInstance();
}

/**
 @brief Korean: ���� Behavior�� ���¸� ��Ÿ���� �Լ�
 @brief English: 
*/
bool PathTeachingBehavior::getBehaviorStates()
{
	return m_bThreadFlag;
}
/**
@brief Korean: ICP ���μ����� �����ϱ� ���� �ʿ��� �ʱ�ȭ �۾��� �����ϴ� �Լ�.
@brief English: 
*/
void PathTeachingBehavior::initICPProcess()
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
@brief Korean: teachingPath ���μ����� �����ϱ� ���� �ʿ��� �ʱ�ȭ �۾��� �����ϴ� �Լ�.
@brief English: 
*/
void PathTeachingBehavior::initTeachingPathProcess()
{
// 	m_TPPlanner.clear();
// 	SensorSupervisor::getInstance()->readSensorData() ;
// 	m_nLaserData181 = SensorSupervisor::getInstance()->getURG04LXLaserData();	
// 	bool bsetWayPointflag=true;
// 	saveOutlineData( m_nLaserData181, m_RobotPos);
// 	m_PathList = m_TPPlanner.execute(m_RobotPos,&bsetWayPointflag);	
// 	KuDrawingInfo::getInstance()->setWayPointflag(bsetWayPointflag);
	m_PathList.clear();
}
/**
 @brief Korean: ������� ���ư��� �Լ�
 @brief English: 
*/
void PathTeachingBehavior::doThread(void* arg)
{
	PathTeachingBehavior *PTbh = (PathTeachingBehavior *) arg;	
	if(SensorSupervisor::getInstance()->readSensorData() ==false) return;
	PTbh->m_nLaserData181 = SensorSupervisor::getInstance()->getLaserDataFront();
	PTbh->m_IplCeilingCamera=SensorSupervisor::getInstance()->getCeilingImageData();
	PTbh->m_DelEncoderData =SensorSupervisor::getInstance()->getEncoderDelPos();
	double dGyro = SensorSupervisor::getInstance()->getGyroData();

	double dDistEc=hypot(PTbh->m_DelEncoderData.getX(),PTbh->m_DelEncoderData.getY())*10.0;
	if(dDistEc>1500){PTbh->m_DelEncoderData.init();	}
	else PTbh->m_DelEncoderData.setThetaDeg(dGyro);

	if(KuRobotParameter::getInstance()->getLocalization()== Localizer::PARTICLEFILTER)
	{
		PTbh->m_RobotPos = KuLBPFLocalizerPr::getInstance()->estimateRobotPos(PTbh->m_nLaserData181, PTbh->m_DelEncoderData,PTbh->m_IplCeilingCamera);	
		
		KuFiducialbasedLocalizerPr::getInstance()->estimateRobotPosbyFiducialmark(PTbh->m_RobotPos,PTbh->m_IplCeilingCamera, false);
		KuPose FiducialPose = KuFiducialbasedLocalizerPr::getInstance()->getRobotPosbyFiducial();
				
		if(FiducialPose.getID()!=-1){
			if(FiducialPose.getID()!=PTbh->m_nIFDX)
			{
				PTbh->m_nIFDX=FiducialPose.getID();
				PTbh->m_dDistMarkfromRobot=DBL_MAX;
				PTbh->m_nFCount=0;
			}
			
			
			PTbh->m_nFCount++;
			if(PTbh->m_nFCount>0)
			{
				PTbh->m_nFCount=0;

				double dX = FiducialPose.getX() + PTbh->m_DelEncoderData.getX() * cos(FiducialPose.getThetaRad()) + 
				PTbh->m_DelEncoderData.getY() * sin(-FiducialPose.getThetaRad());

				double dY = FiducialPose.getY() + PTbh->m_DelEncoderData.getX() * sin(FiducialPose.getThetaRad()) + 
				PTbh->m_DelEncoderData.getY() * cos(FiducialPose.getThetaRad());
				double dThetaDeg = FiducialPose.getThetaDeg() + PTbh->m_DelEncoderData.getThetaDeg();

				//threshold for updating fiducial pose
				double dDistThres = PTbh->m_dRecognizingMarkDistTh;
				double dDx = dX - FiducialPose.getX();
				double dDy = dY - FiducialPose.getY();
				double dDist = sqrt(dDx*dDx + dDy*dDy);

				if(dDist<dDistThres&&dDist<PTbh->m_dDistMarkfromRobot)
				{
					PTbh->m_dDistMarkfromRobot=dDist;
					printf("\n%d�߰�!!!!!\n",FiducialPose.getID());
					FiducialPose.setX(dX);
					FiducialPose.setY(dY);
					FiducialPose.setThetaDeg(dThetaDeg);
								
					KuLBPFLocalizerPr::getInstance()->spreadParticleNearRobot(FiducialPose, 0.2);
				}
				
				//KuLBPFLocalizerPr::getInstance()->initEncoderData();
			}
		}

	}
	else if(KuRobotParameter::getInstance()->getLocalization()== Localizer::SCANMATCHING)
	{
	
		PTbh->m_RobotPos =KuILBPFLocalizerPr::getInstance()->estimateRobotPos(PTbh->m_nLaserData181, PTbh->m_RobotPos,PTbh->m_DelEncoderData, PTbh->m_IplCeilingCamera,false);	
		bool bMapping = false;
		KuScanMatchingLocalizerPr::getInstance()->estimateRobotPosP(PTbh->m_nLaserData181, PTbh->m_DelEncoderData,PTbh->m_RobotPos,PTbh->m_pMap,&bMapping);
		KuPose DelPos= KuScanMatchingLocalizerPr::getInstance()->getDeltaPos();
		KuILBPFLocalizerPr::getInstance()->copyEncoderData(DelPos);	

		KuFiducialbasedLocalizerPr::getInstance()->estimateRobotPosbyFiducialmark(PTbh->m_RobotPos,PTbh->m_IplCeilingCamera, false);
		KuPose FiducialPose = KuFiducialbasedLocalizerPr::getInstance()->getRobotPosbyFiducial();
		
		if(FiducialPose.getID()!=-1){
			if(FiducialPose.getID()!=PTbh->m_nIFDX)
			{
				PTbh->m_nIFDX=FiducialPose.getID();
				PTbh->m_dDistMarkfromRobot=DBL_MAX;
				PTbh->m_nFCount=0;
			}
			
			PTbh->m_nFCount++;
			if(PTbh->m_nFCount>0)
			{				
				PTbh->m_nFCount=0;
				double dX = FiducialPose.getX() + PTbh->m_DelEncoderData.getX() * cos(FiducialPose.getThetaRad()) + 
				PTbh->m_DelEncoderData.getY() * sin(-FiducialPose.getThetaRad());

				double dY = FiducialPose.getY() + PTbh->m_DelEncoderData.getX() * sin(FiducialPose.getThetaRad()) + 
				PTbh->m_DelEncoderData.getY() * cos(FiducialPose.getThetaRad());
				double dThetaDeg = FiducialPose.getThetaDeg() + PTbh->m_DelEncoderData.getThetaDeg();

				//threshold for updating fiducial pose
				double dDistThres = PTbh->m_dRecognizingMarkDistTh;
				double dDx = dX - FiducialPose.getX();
				double dDy = dY - FiducialPose.getY();
				double dDist = sqrt(dDx*dDx + dDy*dDy);

				if(dDist<dDistThres&&dDist<PTbh->m_dDistMarkfromRobot)
				{
					PTbh->m_dDistMarkfromRobot=dDist;
					printf("\n\n�߰�!!!!!\n\n");
					FiducialPose.setX(dX);
					FiducialPose.setY(dY);
					FiducialPose.setThetaDeg(dThetaDeg);

					KuILBPFLocalizerPr::getInstance()->spreadParticleNearRobot(FiducialPose, 0.2);
				}

				
				//KuILBPFLocalizerPr::getInstance()->initEncoderData();
			}
		}
	}

	double dDist = sqrt(pow((PTbh->m_RobotPos.getX() - PTbh->m_saveRobotPos.getX() ),2)+pow((PTbh->m_RobotPos.getY() - PTbh->m_saveRobotPos.getY()),2));

	if(dDist > 100){
		PTbh->m_PathList.push_back(PTbh->m_RobotPos);
		PTbh->m_saveRobotPos = PTbh->m_RobotPos;
	}

	PTbh->drawNaviData();
}
void PathTeachingBehavior::saveOutlineData(int_1DArray nData, KuPose RobotPos)
{

	char cData[300]; 
	memset(cData,0,sizeof(cData));
	string strDataPath = KuRobotParameter::getInstance()->getOutlineWayPointNameNPath();
	sprintf(cData,"%s/%d_%d_%0.2f.log",strDataPath.c_str(),(int)RobotPos.getX(),(int)RobotPos.getY(),RobotPos.getThetaDeg());
	ofstream DataLog;
	DataLog.open(cData);
	int i=0;
	for(i=0; i<Sensor::URG04LX_DATA_NUM181 -1 ; i++){
		DataLog<<nData[i]<<" ";
	}
	DataLog<<nData[i]<<endl;
	DataLog.close();
}

/**
@brief Korean: ���� ���� ������ �׷��ִ� �Լ�.
@brief English: 
*/
void PathTeachingBehavior::drawNaviData()
{
	KuDrawingInfo::getInstance()->setRobotPos(m_RobotPos); //�κ� ��ġ ȭ�鿡 ǥ��		
	KuDrawingInfo::getInstance()->setFrontLaserData181(m_nLaserData181);
	KuDrawingInfo::getInstance()->setCeilingImageData(m_IplCeilingCamera);
	KuDrawingInfo::getInstance()->setAuxiliaryRobotPos(m_RobotPos); //�κ� ��ġ ȭ�鿡 ǥ��
	if(KuRobotParameter::getInstance()->getLocalization()== Localizer::PARTICLEFILTER)
	{
		KuDrawingInfo::getInstance()->setParticle( KuLBPFLocalizerPr::getInstance()->getParticle() );
	}
	else if(KuRobotParameter::getInstance()->getLocalization()== Localizer::SCANMATCHING)
	{
		KuDrawingInfo::getInstance()->setParticle( KuILBPFLocalizerPr::getInstance()->getParticle() );

	}	

	KuPose FiducialMarkImgCoord = KuFiducialbasedLocalizerPr::getInstance()->getFiducialMarkImgCoord();
	KuDrawingInfo::getInstance()->drawFiducialMarkInfoToImage(FiducialMarkImgCoord);

	KuDrawingInfo::getInstance()->setPath(m_PathList); //��� ȭ�鿡 ǥ��
}

/**
@brief Korean: �н��� ��θ� �����ϴ� �Լ�.
@brief English: 
*/
void  PathTeachingBehavior::savePath()
{
	string strDataPath =KuRobotParameter::getInstance()->getTeachingPathNameNPath();
	string strNewPath;
	char cFilePathName[300];
	memset(cFilePathName,0,sizeof(cFilePathName));
	sprintf(cFilePathName,"%s/path.txt",strDataPath.c_str());
	strNewPath=cFilePathName;//char �����Ͱ��� string�� ����

	ofstream DataLog;
	DataLog.open(strNewPath);

	list<KuPose>::iterator it;

	//�������
	for(it=m_PathList.begin(); it!=m_PathList.end(); it++){
		KuPose Path;
		Path.setX(it->getX());
		Path.setY(it->getY());
		Path.setThetaDeg(it->getThetaDeg());
		DataLog<<Path.getX()<<" "<<Path.getY()<<" "<<Path.getThetaDeg()<<endl;
	}

	DataLog.close();
}

/**
 @brief Korean: Ŭ������ �����Ѵ�.
 @brief English: 
*/
void PathTeachingBehavior::terminate()
{
	if(m_bThreadFlag)
	{
		KuThread::terminate();
		savePath();

		if(KuRobotParameter::getInstance()->getPathteaching()!="yes")
		{
			KuLBPFLocalizerPr::getInstance()->terminate();
		}
		//KuZoneControlPr::getInstance()->saveZoneMap(m_pathIdx);
		KuDrawingInfo::getInstance()->setDirectionofPathflag(true);
	}
	m_bThreadFlag = false;
	//m_pathIdx++;//����� ������ ī��Ʈ

}
/**
 @brief Korean: PathTeachingBehavior�� �����Ű�� �Լ�
 @brief English: 
*/
bool PathTeachingBehavior::execute(KuCommandMessage CMessage )
{

	switch(CMessage.getCommandName()){

	case KuCommandMessage::START_THREAD:
		initialize(CMessage);
		KuThread::start(doThread,this,100, "PathTeachingBehavior::execute()"); //���� ������ ����			
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