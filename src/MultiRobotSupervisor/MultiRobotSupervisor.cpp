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
@brief Korean: Thread ������ �Լ�
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
		m_KuThread.start(doThread,this,m_nThreadTime, "MultiRobotSupervisor::Start()"); //���� ������ ����			

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

	//���� AGV ��ġ ���� ����------------------------------------------------
	pMRS->updateRobotPose();
	//���� AGV ��ġ ���� ����=================================================

	KuMap* pMap=kuMapRepository::getInstance()->getMap();//KuZoneControlPr::getInstance()->getZoneMap();//zone map �������°� 

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

		RobotPos1= KuDrawingInfo::getInstance()->getRobotPos();//�ڱ� ��ġ
		RobotPos2 = pMRS->m_AGVRobotPos[nID];//�����ġ

		int nTVel1 = pMRS->m_GotoGoalBeh.getTransVel();//�ڽ��� �ӵ�
		int nRVel1 = pMRS->m_GotoGoalBeh.getRotVel();//�ڽ��� �ӵ�

		int nTVel2 = MultiRobotActuatorInterface::getInstance()->getTransVel(nID);
		int nRVel2 = MultiRobotActuatorInterface::getInstance()->getRotVel(nID);

		if(pMRS->checkBetweenAGVPose(RobotPos1,RobotPos2, nTVel1,nRVel1,nTVel2,nRVel2)&&pMRS->m_AGVTime[nID]<DEADLOCKTIME)
		{
			if (nAGVID >nID) //m_AGV2�� ���� if�� ����(����)
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

				list<KuPose>  DetourPathList = pMRS->generateDetourPath(pMap, RobotPos1, RobotPos2, CurPathlist,&bcheck);//��ȸ��� ����� �Լ�
	
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

	//�����ǥ�� ������ǥ�� ��ȯ�ϴ°�.
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
@brief Korean: [��������] '�ϳ��� zone���� �ϳ��� AGV�� �����Ѵ�'�� ���� �����ϴ� �Լ�
*/
int  MultiRobotSupervisor::Precon_AGVQuantity(KuPose Robot1TargetPos, KuPose RobotPos2,int nAGVID, int nID)
{
	double dRobotDist = _hypot((double)(Robot1TargetPos.getX() - RobotPos2.getX()), (double)(Robot1TargetPos.getY() - RobotPos2.getY()));

	// ���� Zone�� AGV�� ���

	if (dRobotDist <= ROBOTBOUNDARYRADIUSLENGTH)//�ڱ�κ��� Ÿ������ ��� �κ����� �Ÿ��� 1800 �����̸� �浹 ���ɼ� COLLIOSION ����
	{
		if (nAGVID >nID) //m_AGV2�� ���� if�� ����(����)
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
@brief Korean: �����Ǻ��ϴ� �Լ�
*/
int  MultiRobotSupervisor::DetourforAGVQuantity(KuPose RobotPos2)
{
	double dRobotMovementDist = _hypot((double)(RobotPos2.getX() - m_PreRobotPos2.getX()), (double)(RobotPos2.getY() - m_PreRobotPos2.getY()));
	double dRobotMovementRad = fabs(RobotPos2.getThetaDeg() - m_PreRobotPos2.getThetaDeg());

	if ((dRobotMovementDist <= ROBOTMOVEMENTDIST)&&(dRobotMovementRad <= 3))//��� �κ��� ���������� �κ���ġ�� ���� �κ���ġ�� ���ؼ� �̵����� 10�����̰� ������ 3�����ϸ� ����BREAKDOWNAGV�÷��׸���
	{
		return BREAKDOWNAGV;
	}

	return DEFAULTST;
}

/**
@brief Korean:��ȸ��� ����� �Լ�
*/
list<KuPose> MultiRobotSupervisor::generateDetourPath(KuMap * pMap, KuPose RobotPos,KuPose Breakdownrobotpos,list<KuPose> CurPathlist,bool* bcheck  ) 
{
	int nSizeX = pMap->getX();
	int nSizeY = pMap->getY();
	list<KuPose> Pathlist;
	list<KuPose> tempPathlist;//�ӽ�path

	if(m_pPbstacleMap==NULL)
		m_pPbstacleMap = new KuMap(nSizeX, nSizeY);

	//============================================
	// �޸� ���� �Ҵ�
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

	double dRadiusofRobot_cell = (int)(KuRobotParameter::getInstance()->getRobotRadius()/((double)100.0))*4.0;//�κ��� ������ ���� �����//�������� ������ �����ϰ� ������, ���϶�� -2�� -5�̷������� ����ũ�Ի�

	for(int i=-dRadiusofRobot_cell; i<dRadiusofRobot_cell;i++)//���峭 �κ� �ֺ� �ݰ� ������������ ���� 
	{
		for(int j=-dRadiusofRobot_cell; j<dRadiusofRobot_cell;j++)
		{
			if (nRobotX+i<0 || nRobotY+j<0 || nRobotX+i>=pMap->getX() || nRobotY+j>=pMap->getY()) {continue;}	// �ʻ���� ��� ��� ���� ó��
			nMap[nRobotX+i][nRobotY+j] = KuMap::OCCUPIED_AREA;	
		}
	}

	KuDrawingInfo::getInstance()->setMap(m_pPbstacleMap);//�׳� �����̿� �����Ȱ� �����ٶ�� ��������

	list<KuPose>::iterator iteratorPath;
	int nCnt=0;
	KuPose selPathpoint;
	int nselCnt=-1;
	double dMinDist=100000000000;
	int nMinpointNum=0;
	KuPose LastPos;

	for(iteratorPath = CurPathlist.begin();  iteratorPath != CurPathlist.end(); iteratorPath++)//���� �������� �κ� ��ġ���� ��Ŭ����� �Ÿ����ؼ� ���� ����� ������
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

		if(nCnt < nMinpointNum)//���Լ����� ���� ������������� ������
			continue;

		KuPose Pathpoint;

		Pathpoint=(*iteratorPath);

		double dRobotPathDist = _hypot((double)(RobotPos.getX() - Pathpoint.getX()), (double)(RobotPos.getY() - Pathpoint.getY()));
		double dRobottoBreakdownRobotDist = _hypot((double)(Breakdownrobotpos.getX() - Pathpoint.getX()), (double)(Breakdownrobotpos.getY() - Pathpoint.getY()));

		if(dRobottoBreakdownRobotDist>ROBOTBOUNDARYRADIUSLENGTH&&dRobotPathDist>ROBOTBOUNDARYRADIUSLENGTH)//�ڱ�κ���ġ�� �н�����Ʈ����� �Ÿ��� ���ؼ� 1800���� ����
		{																								//�׸��� ���峭�κ���ġ�� �н�����Ʈ����� �Ÿ��� ���ؼ� 1800���� ���� 
			                         //�������� �����Ҷ�� �κ��ٿ������ 1800���� ū �ڱ�κ��ݴ��� �н���������Ʈ�� ��������
			//���� ����Ʈ�� Ÿ�� ��� �ȴ�//(����)
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
	for(iteratorPath = CurPathlist.begin();  iteratorPath != CurPathlist.end(); iteratorPath++)//���� �н����� ȸ�ǰ�� ���Ŀ� ������θ� �ٿ��ֱ� ���ؼ�
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
	m_KuPathPlanner.generatePath(selPathpoint, RobotPos); //��� ����
	Pathlist = m_KuPathPlanner.getPath();

	if(Pathlist.size()==0){
		(*bcheck)=false;
	}//��λ��� 	
	else 
	{
		(*bcheck)=true;
	}

	for(iteratorPath = tempPathlist.begin();  iteratorPath != tempPathlist.end(); iteratorPath++)//������ ȸ�ǰ�ο� �Ʊ� ���� ���� �н��� ���̴� �Լ�
	{
		KuPose Pathpoint;
		Pathpoint=(*iteratorPath);
		Pathlist.push_back(Pathpoint);
	}


	return Pathlist;
}

/**
@brief Korean: ���� �������  Behavior�� state�� ��Ÿ���� �Լ�
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
	int nBehaviorName = CMessage.getBehaviorName(); //������ ���� �̸��� �ε����� ������ �´�.
	int nBehaviorPeriod = CMessage.getBehaviorPeriod();
	KuPose GoalPos = CMessage.getGoalPos();
	KuPose RobotPos = CMessage.getRobotPos();

	Start(100);

	m_GotoGoalBeh.execute(CMessage);
}

/**
@brief Korean: ���� �������  Behavior�� Localizer�� ��Ÿ���� �Լ�
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
	TotalTcpipCommunication::getInstance()->startClient(100); // Live ��ȣ: 5 sec, Location & state: 100 msec
	//TotalTcpipCommunication::getInstance()->clientseq();

	return true;
}
/**
 @brief Korean: ROBOT pose �޾ƿ�
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

