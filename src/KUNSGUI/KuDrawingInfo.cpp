#include "stdafx.h"
#include "KuDrawingInfo.h"


KuDrawingInfo::KuDrawingInfo()
{
	initVariable(); //변수 초기화 함수.
	cout<<"[KuDrawingInfo]: Singletone type instance is created!!!"<<endl;
	m_nMap=NULL;
	m_dProMap=NULL;
	m_nCeilingMap=NULL;
	m_nCADMap=NULL;
	m_nZoneMap=NULL;
	m_nMapX=0;
	m_nMapY=0;
	m_bDirectionofPathflag=true;
	m_bRenderCADMapflag=false;
	m_bWaitDiffAGVflag=false;
	m_bRenderZoneMapflag=false;
	memset(m_Global3DPose,0,sizeof(m_Global3DPose));
	m_bDataAccess = false;
}

KuDrawingInfo::~KuDrawingInfo()
{

	if(m_nMap!=NULL)
	{
		for(int i = 0 ; i < m_nMapX ; i++){
			delete[] m_nMap[i];
			delete[] m_dProMap[i];

			m_nMap[i] = NULL;
			m_dProMap[i] = NULL;
		}
		delete[] m_nMap;
		delete[] m_dProMap;

	}

	if(m_nCeilingMap!=NULL)
	{
		for(int i = 0 ; i < m_nMapX ; i++){
			delete[] m_nCeilingMap[i];
			m_nMap[i] = NULL;
		}
		delete[] m_nCeilingMap;
	}
	
	cout<<"[KuDrawingInfo]: Singletone type instance is destroyed!!!"<<endl;
}

/**
@brief Korean: 변수들을 초기화 한다. 
@brief English: write in English
*/
void KuDrawingInfo::initVariable()
{
	m_nLaserDataFront181 = m_KuUtil.generateIntType1DArray(Sensor::URG04LX_DATA_NUM181,0);
	m_nLaserDataRear181 = m_KuUtil.generateIntType1DArray(Sensor::URG04LX_DATA_NUM181,0);

	m_nAlignLaserData181 = m_KuUtil.generateIntType1DArray(Sensor::URG04LX_DATA_NUM181,0);

	m_nTData = m_KuUtil.generateIntType1DArray(Sensor::URG04LX_DATA_NUM181*2,0);

	m_nKinectRangeData = m_KuUtil.generateIntType1DArray(Sensor::KINECT_SENSOR_FOV,0);

	m_IplKinectImg = cvCreateImage(cvSize( Sensor::IMAGE_WIDTH, Sensor::IMAGE_HEIGHT),8,3);

	m_IplCeilingImage = cvCreateImage(cvSize( Sensor::CEILING_IMAGE_WIDTH, Sensor::CEILING_IMAGE_HEIGHT),8,1);

	m_nTVByKeyEvt=m_nRVByKeyEvt=0; //키보드 이벤트를 통해 받은 로봇 속도를 저장하는 변수.
	
	m_bRenderMapflag=true;//지도 그리기로 결정

	m_bRenderBuildingMapflag=false;
	
	m_bRenderCeilingImagflag=false;
}

/**
@brief Korean: 지도정보를 저장하는 함수.
@brief English: write in English
*/
void KuDrawingInfo::setMap(KuMap *pMap)
{
	m_CriticalSection.Lock();

	if(m_nMap==NULL){
		//처음 m_nMap을 생성할때
		m_nMapX = pMap->getX();
		m_nMapY = pMap->getY();

		m_nMap = new int*[m_nMapX];
		m_dProMap = new double*[m_nMapX];

		if(m_nMap){
			for(int i = 0 ; i < m_nMapX ; i++){
				m_nMap[i] = new int[m_nMapY];
				m_dProMap[i] = new double[m_nMapY];

			}
		}
	}
	else if(m_nMap!=NULL&&(m_nMapX != pMap->getX()||m_nMapY != pMap->getY()))
	{
		for(int i = 0 ; i < m_nMapX ; i++){
			delete[] m_nMap[i];
			delete[] m_dProMap[i];

			m_nMap[i] = NULL;
			m_dProMap[i] = NULL;
		}
		delete[] m_nMap;
		delete[] m_dProMap;

		m_nMapX = pMap->getX();
		m_nMapY = pMap->getY();

		m_nMap = new int*[m_nMapX];
		m_dProMap = new double*[m_nMapX];

		if(m_nMap){
			for(int i = 0 ; i < m_nMapX ; i++){
				m_nMap[i] = new int[m_nMapY];
				m_dProMap[i] = new double[m_nMapY];

			}
		}
	}
	//지도 데이터 복사
	int** nMap = pMap->getMap();
	double** dProMap = pMap->getProbMap();


	for(int i=0; i<m_nMapX; i++){
		for(int j=0; j<m_nMapY; j++){

			m_nMap[i][j] = nMap[i][j];

			m_dProMap[i][j] = dProMap[i][j]; 
		}
	}

	m_CriticalSection.Unlock();
}
/**
 @brief Korean: 저장된 지도정보를 얻어가는 함수. 
 @brief English: write in English
*/
int** KuDrawingInfo::getMap()
{
	return m_nMap;
}


/**
@brief Korean: 지도정보를 저장하는 함수.
@brief English: write in English
*/
void KuDrawingInfo::setCeilingMap(KuMap *pMap)
{
	m_CriticalSection.Lock();

	if(m_nCeilingMap==NULL){
		//처음 m_nMap을 생성할때
		m_nMapX = pMap->getX();
		m_nMapY = pMap->getY();

		m_nCeilingMap = new int*[m_nMapX];

		if(m_nMap){
			for(int i = 0 ; i < m_nMapX ; i++){
				m_nCeilingMap[i] = new int[m_nMapY];
			}
		}
	}
	else if(m_nCeilingMap!=NULL&&(m_nMapX != pMap->getX()||m_nMapY != pMap->getY()))
	{
		for(int i = 0 ; i < m_nMapX ; i++){
			delete[] m_nCeilingMap[i];
			m_nCeilingMap[i] = NULL;
		}
		delete[] m_nCeilingMap;

		m_nMapX = pMap->getX();
		m_nMapY = pMap->getY();

		m_nCeilingMap = new int*[m_nMapX];


		if(m_nCeilingMap){
			for(int i = 0 ; i < m_nMapX ; i++){
				m_nCeilingMap[i] = new int[m_nMapY];
			}
		}
	}
	//지도 데이터 복사
	int** nMap = pMap->getMap();


	for(int i=0; i<m_nMapX; i++){
		for(int j=0; j<m_nMapY; j++){

			m_nCeilingMap[i][j] = nMap[i][j];
		}
	}

	m_CriticalSection.Unlock();
}
/**
 @brief Korean: 저장된 지도정보를 얻어가는 함수. 
 @brief English: write in English
*/
int** KuDrawingInfo::getCeilingMap()
{
	return m_nCeilingMap;
}


/**
@brief Korean: 지도정보를 저장하는 함수.
@brief English: write in English
*/
void KuDrawingInfo::setCADMap(int** nCADMap)
{
	if(m_nMapX==0||m_nMapY==0) return;
	
	m_CriticalSection.Lock();

	if(m_nCADMap==NULL){
	
		m_nCADMap = new int*[m_nMapX];

		if(m_nMap){
			for(int i = 0 ; i < m_nMapX ; i++){
				m_nCADMap[i] = new int[m_nMapY];
			}
		}
	}
	
	for(int i=0; i<m_nMapX; i++){
		for(int j=0; j<m_nMapY; j++){

			m_nCADMap[i][j] = nCADMap[i][j];
		}
	}

	m_CriticalSection.Unlock();
}
/**
 @brief Korean: 저장된 지도정보를 얻어가는 함수. 
 @brief English: write in English
*/
int** KuDrawingInfo::getCADMap()
{
	return m_nCADMap;
}


/**
@brief Korean: 지도정보를 저장하는 함수.
@brief English: write in English
*/
void KuDrawingInfo::setZoneMap(KuMap *pMap)
{
	m_CriticalSection.Lock();

	if(m_nZoneMap==NULL){
		//처음 m_nMap을 생성할때
		m_nMapX = pMap->getX();
		m_nMapY = pMap->getY();

		m_nZoneMap = new int*[m_nMapX];

		if(m_nZoneMap){
			for(int i = 0 ; i < m_nMapX ; i++){
				m_nZoneMap[i] = new int[m_nMapY];
			}
		}
	}
	else if(m_nZoneMap!=NULL&&(m_nMapX != pMap->getX()||m_nMapY != pMap->getY()))
	{
		for(int i = 0 ; i < m_nMapX ; i++){
			delete[] m_nZoneMap[i];

			m_nZoneMap[i] = NULL;
		}
		delete[] m_nMap;

		m_nMapX = pMap->getX();
		m_nMapY = pMap->getY();

		m_nZoneMap = new int*[m_nMapX];

		if(m_nZoneMap){
			for(int i = 0 ; i < m_nMapX ; i++){
				m_nZoneMap[i] = new int[m_nMapY];
			}
		}
	}
	int **nMap=pMap->getMap();

	for(int i=0; i<m_nMapX; i++){
		for(int j=0; j<m_nMapY; j++){

			m_nZoneMap[i][j] = nMap[i][j];

		}
	}

	m_CriticalSection.Unlock();
}
/**
 @brief Korean: 저장된 지도정보를 얻어가는 함수. 
 @brief English: write in English
*/
int** KuDrawingInfo::getZoneMap()
{
	return m_nZoneMap;
}


/**
 @brief Korean: 저장된 지도의 X 방향의 크기를 가져가는 함수 (단위: 10cm)
 @brief English: write in English
*/
int KuDrawingInfo::getMapSizeX()
{
	return m_nMapX;
}
/**
 @brief Korean: 저장된 지도의 Y 방향의 크기를 가져가는 함수 (단위: 10cm)
 @brief English: write in English
*/
int  KuDrawingInfo::getMapSizeY()
{
	return m_nMapY;
}


/**
 @brief Korean: 전방 레이저 데이터를 저장하는 함수.
 @brief English: write in English
*/
void KuDrawingInfo::setFrontLaserData181(int_1DArray nLaserData181)
{
	m_CriticalSection.Lock();
	//레이저 데이터 복사과정-------------------------------------------------------
	for(int i=0; i<Sensor::URG04LX_DATA_NUM181; i++)
	{
		m_nLaserDataFront181[i] =  nLaserData181[i];
	}
	//*******************************************************************
	m_CriticalSection.Unlock();

}

/**
 @brief Korean: 후방 레이저 데이터를 저장하는 함수.
 @brief English: write in English
*/
void KuDrawingInfo::setRearLaserData181(int_1DArray nLaserData181)
{
	m_CriticalSection.Lock();
	//레이저 데이터 복사과정-------------------------------------------------------
	for(int i=0; i<Sensor::URG04LX_DATA_NUM181; i++)
	{
		m_nLaserDataRear181[i] =  nLaserData181[i];
	}
	//*******************************************************************
	m_CriticalSection.Unlock();

}

/**
 @brief Korean: 저장된 레이저 데이터를 넘겨주는 함수.
 @brief English: write in English
*/
int_1DArray KuDrawingInfo::getLaserDataFront181()
{
	return m_nLaserDataFront181;
}

/**
 @brief Korean: 저장된 레이저 데이터를 넘겨주는 함수.
 @brief English: write in English
*/
int_1DArray KuDrawingInfo::getLaserDataRear181()
{
	return m_nLaserDataRear181;
}

/**
 @brief Korean: 레이저 데이터를 저장하는 함수.
 @brief English: write in English
*/
void KuDrawingInfo::setAlignLaserData181(int_1DArray nLaserData181)
{
	m_CriticalSection.Lock();
	//레이저 데이터 복사과정-------------------------------------------------------
	for(int i=0; i<Sensor::URG04LX_DATA_NUM181; i++){
			m_nAlignLaserData181[i] =  nLaserData181[i]; 
	}
	//*******************************************************************
	m_CriticalSection.Unlock();

}

/**
 @brief Korean: 저장된 레이저 데이터를 넘겨주는 함수.
 @brief English: write in English
*/
int_1DArray KuDrawingInfo::getAlignLaserData181()
{
	return m_nAlignLaserData181;
}
/**
 @brief Korean: 키넥트 데이터를 저장하는 함수.
 @brief English: write in English
*/
void KuDrawingInfo::setKinectRangeData(int_1DArray nKinectRangeData)
{
	m_CriticalSection.Lock();
	//레이저 데이터 복사과정-------------------------------------------------------
	for(int i=0; i<Sensor::KINECT_SENSOR_FOV; i++){
			m_nKinectRangeData[i] =  nKinectRangeData[i]; 
	}
	//*******************************************************************
	m_CriticalSection.Unlock();

}

/**
 @brief Korean: 저장된 키게트 데이터를 넘겨주는 함수.
 @brief English: write in English
*/
int_1DArray KuDrawingInfo::getKinectRangeData()
{
	return m_nKinectRangeData;
}
/**
 @brief Korean: 저장된 로봇 위치를 넘겨주는 함수.
 @brief English: write in English
*/
KuPose  KuDrawingInfo::getRobotPos()
{
	return m_RobotPos;
}

/**
 @brief Korean: 로봇 위치를 저장하는 함수
 @brief English: write in English
*/
void  KuDrawingInfo::setPathBlockPos(vector<PBlock> vecPathBlockPos)
{
	m_CriticalSection.Lock();
	m_vecPathBlockPos.clear();
	for(int i=0; i<vecPathBlockPos.size();i++)
	{
		m_vecPathBlockPos.push_back(vecPathBlockPos[i]);
	}
	m_CriticalSection.Unlock();
}
/**
 @brief Korean: 저장된 로봇 위치를 넘겨주는 함수.
 @brief English: write in English
*/
 void  KuDrawingInfo::getPathBlockPos(vector<PBlock>*vecPathBlockPos)
{
	m_CriticalSection.Lock();

	for(int i=0; i<m_vecPathBlockPos.size();i++)
	{
		(*vecPathBlockPos).push_back(m_vecPathBlockPos[i]);
	}
	m_CriticalSection.Unlock();

}

 void KuDrawingInfo::clearGlobalPathBlockPos()
 {
	 m_vecGlobalPathBlock.clear();
 }


/**
 @brief Korean: 로봇 위치를 저장하는 함수
 @brief English: write in English
*/
void  KuDrawingInfo::setRobotPos(KuPose RobotPos)
{
	m_RobotPos=RobotPos;
}
/**
 @brief Korean: 로봇 위치를 저장하는 함수
 @brief English: write in English
*/
void  KuDrawingInfo::setRobotPos2(KuPose RobotPos)
{
	m_RobotPos2=RobotPos;
}
/**
 @brief Korean: 저장된 로봇 위치를 넘겨주는 함수.
 @brief English: write in English
*/
KuPose  KuDrawingInfo::getRobotPos2()
{
	return m_RobotPos2;
}

/**
 @brief Korean: 실험 목적의 로봇 위치를 넘겨주는 함수.
 @brief English: write in English
*/
KuPose KuDrawingInfo::getAuxiliaryRobotPos()
{
	
	KuPose AuxiliaryRobotPos;
	m_CriticalSection.Lock();
	AuxiliaryRobotPos = m_AuxiliaryRobotPos;
	m_CriticalSection.Unlock();

	return AuxiliaryRobotPos;
}

/**
 @brief Korean: 실험 목적의 로봇 위치를 저장하는 함수
 @brief English: write in English
*/
void KuDrawingInfo::setAuxiliaryRobotPos(KuPose AuxiliaryRobotPos)
{
	m_CriticalSection.Lock();
	m_AuxiliaryRobotPos = AuxiliaryRobotPos;
	m_CriticalSection.Unlock();

}

/**
 @brief Korean: 실험 목적의 이상적인 로봇 위치를 넘겨주는 함수.
 @brief English: write in English
*/
KuPose KuDrawingInfo::getIdealRobotPos()
{
	KuPose IdealRobotPos;
	m_CriticalSection.Lock();
	IdealRobotPos = m_IdealRobotPos;
	m_CriticalSection.Unlock();

	return IdealRobotPos;

}
/**
 @brief Korean: //실험 목적의 이상적인 로봇 위치를 저장하는 함수
 @brief English: write in English
*/
void KuDrawingInfo::setIdealRobotPos(KuPose IdealRobotPos)
{

	m_CriticalSection.Lock();
	m_IdealRobotPos = IdealRobotPos;
	m_CriticalSection.Unlock();

}

/**
 @brief Korean: 저장된 골 위치를 넘겨주는 함수.
 @brief English: write in English
*/
KuPose KuDrawingInfo::getGoalPos()
{
	KuPose GoalPos;
	m_CriticalSection.Lock();
	GoalPos = m_GoalPos;
	m_CriticalSection.Unlock();

	return GoalPos;
}

/**
 @brief Korean: 골 위치를 저장하는 함수
 @brief English: write in English
*/
void KuDrawingInfo::setGoalPos(KuPose GoalPos)
{
	m_CriticalSection.Lock();
	m_GoalPos=GoalPos;
	m_CriticalSection.Unlock();

} 

/**
 @brief Korean: 로봇 속도를 저장하는 함수
 @brief English: write in English
*/
void KuDrawingInfo::setRobotTRVel(int nTVel, int nRVel)
{
	m_CriticalSection.Lock();
	if(0==nTVel && 0 == nRVel){
		m_nTVByKeyEvt = 0;
		m_nRVByKeyEvt = 0;

	}else{
		m_nTVByKeyEvt += nTVel;
		m_nRVByKeyEvt += nRVel;
	}
	m_CriticalSection.Unlock();
	
}

/**
 @brief Korean: 로봇 속도를 넘겨주는 함수
 @brief English: write in English
*/
void KuDrawingInfo::getRobotTRVel(int* nTVel, int* nRVel)
{

	m_CriticalSection.Lock();
	*nTVel = m_nTVByKeyEvt;
	*nRVel = m_nRVByKeyEvt;
	m_CriticalSection.Unlock();

}

/**
 @brief Korean: 파티클에 대한 정보를 저장하는 함수
 @brief English: write in English
*/
void KuDrawingInfo::setParticle(vector<Sample> vecParticle)
{
	m_CriticalSection.Lock();
	m_vecParticle = vecParticle;
	m_CriticalSection.Unlock();

}

/**
@brief Korean: 파티클에 대한 정보를 넘겨주는 함수
@brief English: write in English
*/
vector<Sample> KuDrawingInfo::getParticle()
{
	vector<Sample> vecParticle;
	m_CriticalSection.Lock();
	vecParticle = m_vecParticle;
	m_CriticalSection.Unlock();

	return vecParticle;
}

/**
@brief Korean: ///경로를 저장하는 함수.
@brief English: write in English
*/
void KuDrawingInfo::setPath(list<KuPose> listPath)
{

	m_CriticalSection.Lock();
	m_listPath = listPath;
	m_CriticalSection.Unlock();

}

/**
@brief Korean: 경로를 넘겨주는 함수.
@brief English: write in English
*/
list<KuPose> KuDrawingInfo::getPath()
{
	m_CriticalSection.Lock();
	list<KuPose> listPath;
	list<KuPose>::iterator it;

	for (it = m_listPath.begin(); it != m_listPath.end(); it++) {
		listPath.push_back(*it);
	}
	m_CriticalSection.Unlock();

	return listPath;
}


/**
@brief Korean: ///경로를 저장하는 함수.
@brief English: write in English
*/
void KuDrawingInfo::setvecPathlist(vector<list<KuPose>> veclistPathlist)
{

	m_CriticalSection.Lock();
	m_veclistPathlist = veclistPathlist;
	m_CriticalSection.Unlock();

}

/**
@brief Korean: 경로를 넘겨주는 함수.
@brief English: write in English
*/
vector<list<KuPose>>  KuDrawingInfo::getvecPathlist()
{
	vector<list<KuPose>> veclistPath;

	m_CriticalSection.Lock();
	for(int i=0; i<m_veclistPathlist.size();i++)
	{
		list<KuPose>::iterator it;
		list<KuPose> pathlist;
		for (it = m_veclistPathlist[i].begin(); it != m_veclistPathlist[i].end(); it++) {
			pathlist.push_back(*it);
		}
		veclistPath.push_back(pathlist);
	}
	m_CriticalSection.Unlock();

	return veclistPath;
}
/**
@brief Korean: ///경로를 저장하는 함수.
@brief English: write in English
*/
void KuDrawingInfo::setLocalPath(list<KuPose> listPath)
{

	m_CriticalSection.Lock();
	m_listLocalPath = listPath;
	m_CriticalSection.Unlock();

}

/**
@brief Korean: 경로를 넘겨주는 함수.
@brief English: write in English
*/
list<KuPose> KuDrawingInfo::getLocalPath()
{
	list<KuPose> listPath;
	m_CriticalSection.Lock();
	listPath = m_listLocalPath;
	m_CriticalSection.Unlock();

	return listPath;
}
/**
@brief Korean: //target pos를 저장하는 함수
@brief English: write in English
*/
void KuDrawingInfo::setTargetPos(KuPose TargetPos)
{
	m_CriticalSection.Lock();
	m_TargetPos =TargetPos;
	m_CriticalSection.Unlock();

}

/**
@brief Korean: //target pos를 넘겨주는 함수.
@brief English: write in English
*/
KuPose KuDrawingInfo::getTargetPos()
{
	KuPose TargetPos;
	m_CriticalSection.Lock();
	TargetPos = m_TargetPos;
	m_CriticalSection.Unlock();
	return TargetPos;
}
/**
@brief Korean: //target pos를 저장하는 함수
@brief English: write in English
*/
void KuDrawingInfo::setLocalGoalPos(KuPose TargetPos)
{
	m_CriticalSection.Lock();
	m_LocalGoalPos =TargetPos;
	m_CriticalSection.Unlock();

}

/**
@brief Korean: //target pos를 넘겨주는 함수.
@brief English: write in English
*/
KuPose KuDrawingInfo::getLocalGoalPos()
{
	KuPose LocalGoalPos;
	m_CriticalSection.Lock();
	LocalGoalPos = m_LocalGoalPos;
	m_CriticalSection.Unlock();
	return LocalGoalPos;
}
/**
@brief Korean: 작성되는 지도의 확률적인 값을 얻어가는 부분
@brief English: write in English
*/
double ** KuDrawingInfo::getBuildingMap( int* nX, int* nY)
{	
	m_CriticalSection.Lock();
	*nX = m_nMapX;
	*nY = m_nMapY;
	double ** dMap;

	if(NULL != m_dProMap){ //2차 배열이 할당된 적이 없는 상태이다.
		dMap = m_dProMap;
	}
	m_CriticalSection.Unlock();

	return dMap;

}
/**
@brief Korean: 작성되는 지도의 확률적인 값을 얻어가는 부분
@brief English: write in English
*/
void KuDrawingInfo::setBuildingMap( double**  dMap )
{
	m_CriticalSection.Lock();
	m_dProMap = dMap  ;
	m_CriticalSection.Unlock();
}
/**
@brief Korean: 다수의 골지점을 얻어가는 부분
@brief English: write in English
*/
list<KuPose> KuDrawingInfo::getGoalPosList()
{
	list<KuPose> GoalPosList;
	m_CriticalSection.Lock();
	GoalPosList = m_GoalPosList;
	m_CriticalSection.Unlock();
	return GoalPosList;
}
/**
@brief Korean: 다수의 골지점을 저장하는 부분
@brief English: write in English
*/
void KuDrawingInfo::setGoalPosList(list<KuPose> GoalPosList)
{
	m_CriticalSection.Lock();
	m_GoalPosList = GoalPosList;
	m_CriticalSection.Unlock();
}
/**
@brief Korean: 지도의 그리기 여부를 가져오는 함수
@brief English: write in English
*/
void KuDrawingInfo::setRenderMapflag(bool bRenderMapflag)
{
	m_bRenderMapflag=bRenderMapflag;
}
/**
@brief Korean:  지도의 그리기 여부를  정하는 함수
@brief English: write in English
*/
bool KuDrawingInfo::getRenderMapflag()
{
	return m_bRenderMapflag;
}

/**
@brief Korean: 지도의 그리기 여부를 가져오는 함수
@brief English: write in English
*/
void KuDrawingInfo::setRenderZoneMapflag(bool bRenderZoneMapflag)
{
	m_bRenderZoneMapflag=bRenderZoneMapflag;
}
/**
@brief Korean:  지도의 그리기 여부를  정하는 함수
@brief English: write in English
*/
bool KuDrawingInfo::getRenderZoneMapflag()
{
	return m_bRenderZoneMapflag;
}
/**
	@brief Korean:  레이저의 그리기 여부를 정하는 함수
	@brief English: write in English
*/
void KuDrawingInfo::setRenderLaserflag(bool bRendeLaserflag)
{
	m_bRendeLaserflag=bRendeLaserflag;
}
/**
	@brief Korean:  레이저의 그리기 여부를 가져오는 함수
	@brief English: write in English
*/
bool KuDrawingInfo::getRenderLaserflag()
{
	return m_bRendeLaserflag;
}
/**
	@brief Korean: 키넥트의 그리기 여부를 정하는 함수
	@brief English: write in English
*/
void KuDrawingInfo::setRenderKinectflag(bool bRenderKinectflag)
{
	m_bRenderKinectflag=bRenderKinectflag;
}
/**
	@brief Korean:  키넥트의 그리기 여부를 가져오는 함수
	@brief English: write in English
*/
bool KuDrawingInfo::getRenderKinectflag()
{
	return m_bRenderKinectflag;
}
/**
	@brief Korean: 경로의 그리기 여부를 정하는 함수
	@brief English: write in English
*/
void KuDrawingInfo::setRenderPathflag(bool bRenderPathflag)
{
	m_bRenderPathflag=bRenderPathflag;
}
/**
	@brief Korean:  경로의 그리기 여부를 가져오는 함수
	@brief English: write in English
*/
bool KuDrawingInfo::getRenderPathflag()
{
	return m_bRenderPathflag;
}

/**
	@brief Korean: 키넥트의 이미지 데이터를 저장하는 함수
	@brief English: write in English
*/
void KuDrawingInfo::setKinectImageData(IplImage* IplKinectImg)
{
	if(IplKinectImg!=NULL){
		m_CriticalSection.Lock();
		//cvCopy(IplKinectImg,m_IplKinectImg);
		for(int i=0; i<320; i++){
			for(int j=0; j<240; j++){
				m_IplKinectImg->imageData[(j*320+i)*3] = IplKinectImg->imageData[(j*320+i)*3];
				m_IplKinectImg->imageData[(j*320+i)*3+1] = IplKinectImg->imageData[(j*320+i)*3+1];
				m_IplKinectImg->imageData[(j*320+i)*3+2] = IplKinectImg->imageData[(j*320+i)*3+2];
			}
		}
		m_CriticalSection.Unlock();
	}
}

/**
	@brief Korean: 키넥트의 이미지 데이터를 가져가는 함수
	@brief English: write in English
*/
void KuDrawingInfo::getKinectImageData(IplImage* IplKinectImg)
{
	m_CriticalSection.Lock();
	for(int i=0; i<320; i++){
		for(int j=0; j<240; j++){
			IplKinectImg->imageData[(j*320+i)*3] = m_IplKinectImg->imageData[(j*320+i)*3];
			IplKinectImg->imageData[(j*320+i)*3+1] = m_IplKinectImg->imageData[(j*320+i)*3+1];
			IplKinectImg->imageData[(j*320+i)*3+2] = m_IplKinectImg->imageData[(j*320+i)*3+2];
		}
	}
	m_CriticalSection.Unlock();

}
/**
	@brief Korean: 천장카메라의 이미지 데이터를 저장하는 함수
	@brief English: write in English
*/
void KuDrawingInfo::setCeilingImageData(IplImage* imageData)
{
	if(imageData!=NULL){
		m_CriticalSection.Lock();
		cvCopy(imageData,m_IplCeilingImage);
		m_CriticalSection.Unlock();
	}
}
/**
	@brief Korean: 천장카메라의 이미지 데이터를 가져가는 함수
	@brief English: write in English
*/
void KuDrawingInfo::getCeilingImageData(IplImage* ImageData)
{
	if(m_IplCeilingImage!=NULL)
	{
		m_CriticalSection.Lock();
		cvCopy(m_IplCeilingImage, ImageData);
		m_CriticalSection.Unlock();
	}

}
/**
	@brief Korean: 작성중인 지도의 그리기 여부를 결정하는 함수
	@brief English: write in English
*/
void KuDrawingInfo::setRenderBuildingMapflag(bool bRenderBuildingMapflag)
{
	m_bRenderBuildingMapflag=bRenderBuildingMapflag;
}
/**
	@brief Korean: 작성중인 지도의 그리기 여부를 가져가는 함순
	@brief English: write in English
*/
bool KuDrawingInfo::getRenderBuildingMapflag()
{
	return m_bRenderBuildingMapflag;
}
/**
	@brief Korean: 경로의 방향을 정하는 함수
	@brief English: write in English
*/
void KuDrawingInfo::setDirectionofPathflag(bool bDirectionofPathflag)
{
	m_bDirectionofPathflag=bDirectionofPathflag;
}

/**
	@brief Korean: 경로의 방향을  가져오는 함수
	@brief English: write in English
*/
bool KuDrawingInfo::getDirectionofPathflag()
{
	return m_bDirectionofPathflag;
}

void KuDrawingInfo::setvecLandmarkPos(vector<KuPose> vecLandmark)
{
	m_CriticalSection.Lock();
	m_vecLandmark = vecLandmark;
	m_CriticalSection.Unlock();
}
vector<KuPose> KuDrawingInfo::getvecLandmarkPos()
{
	vector<KuPose> vecLandmark;
	m_CriticalSection.Lock();
	vecLandmark = m_vecLandmark;
	m_CriticalSection.Unlock();
	return vecLandmark;

}
/**
	@brief Korean: 
	@brief English: write in English
*/
void KuDrawingInfo::setRenderCeilingImageflag(bool bRenderCeilingImagflag)
{
	m_bRenderCeilingImagflag=bRenderCeilingImagflag;
}
/**
	@brief Korean:  
	@brief English: write in English
*/
bool KuDrawingInfo::getRenderCeilingImageflag()
{
	return m_bRenderCeilingImagflag;
}
/**
@brief Korean: 경유점 list를  저장하는 함수.
@brief English: write in English
*/
void KuDrawingInfo::setWayPointList(list<KuPose> listWayPoint)
{

	m_CriticalSection.Lock();
	m_listWayPoint = listWayPoint;
	m_CriticalSection.Unlock();

}

/**
@brief Korean: 경유점 list 를 넘겨주는 함수.
@brief English: write in English
*/
list<KuPose> KuDrawingInfo::getWayPointList()
{
	list<KuPose> listWayPoint;
	m_CriticalSection.Lock();
	listWayPoint = m_listWayPoint;
	m_CriticalSection.Unlock();

	return listWayPoint;
}
/**
	@brief Korean: 
	@brief English: write in English
*/
void KuDrawingInfo::setWayPointflag(bool bWayPointflag)
{
	m_bWayPointflag=bWayPointflag;
}
/**
	@brief Korean:  
	@brief English: write in English
*/
bool KuDrawingInfo::getWayPointflag()
{
	return m_bWayPointflag;
}

void KuDrawingInfo::setKinectGlobal3DPos(KuPose* pGlobal3DPose)
{
	m_CriticalSection.Lock();
	int nSize = Sensor::IMAGE_WIDTH*Sensor::IMAGE_HEIGHT;
	for(int i=0; i<nSize; i++){
		m_Global3DPose[i] = pGlobal3DPose[i];
	}
	m_CriticalSection.Unlock();
}

KuPose*  KuDrawingInfo::getKinectGlobal3DPos()
{
	KuPose* pGlobal3DPose; 
	m_CriticalSection.Lock();
	pGlobal3DPose = m_Global3DPose;
	m_CriticalSection.Unlock();
	return pGlobal3DPose;
}

void KuDrawingInfo::setRenderCADMapflag(bool bRenderCADMapflag)
{
	m_bRenderCADMapflag=bRenderCADMapflag;
}
/**
	@brief Korean:  
	@brief English: write in English
*/
bool KuDrawingInfo::getRenderCADMapflag()
{
	return m_bRenderCADMapflag;
}
/**
	@brief Korean: 키넥트의 그리기 여부를 정하는 함수
	@brief English: write in English
*/
void KuDrawingInfo::setRenderKinectDepthflag(bool bRenderKinectDepthflag)
{
	m_bRenderKinectDepthflag=bRenderKinectDepthflag;
}
/**
	@brief Korean:  키넥트의 그리기 여부를 가져오는 함수
	@brief English: write in English
*/
bool KuDrawingInfo::getRenderKinectDepthflag()
{
	return m_bRenderKinectDepthflag;
}
/**
	@brief Korean: 
	@brief English: write in English
*/
void KuDrawingInfo::setWaitforDiffAGV(bool bWaitDiffAGVflag)
{
	m_bWaitDiffAGVflag=bWaitDiffAGVflag;
}
/**
	@brief Korean:  
	@brief English: write in English
*/
bool KuDrawingInfo::getWaitforDiffAGV()
{
	return m_bWaitDiffAGVflag;
}

/**
 @brief Korean: 레이저 데이터를 저장하는 함수.
 @brief English: write in English
*/
void KuDrawingInfo::setTData(int_1DArray nLaserData181)
{
	m_CriticalSection.Lock();
	//레이저 데이터 복사과정-------------------------------------------------------
	for(int i=0; i<Sensor::URG04LX_DATA_NUM181*2; i++){
			m_nTData[i] =  nLaserData181[i]; 
	}
	//*******************************************************************
	m_CriticalSection.Unlock();

}

/**
 @brief Korean: 저장된 레이저 데이터를 넘겨주는 함수.
 @brief English: write in English
*/
int_1DArray KuDrawingInfo::getTData()
{
	return m_nTData;
}

void KuDrawingInfo::getRegion(vector<CLAMPData>* fRegionresult)
{
	m_CriticalSection.Lock();
	fRegionresult->clear();
	for(int i=0; i<m_FRD.size();i++)
	{
		(*fRegionresult).push_back(m_FRD[i]);
	}
	m_CriticalSection.Unlock();

}
void KuDrawingInfo::setRegion(vector<CLAMPData>  fRegionresult)
{
	m_CriticalSection.Lock();
	m_FRD.clear();
	for(int i=0; i<fRegionresult.size();i++)
	{
		m_FRD.push_back(fRegionresult[i]);
	}
	int aa = fRegionresult.size();
	m_CriticalSection.Unlock();

}

void KuDrawingInfo::getTemplateData(vector<CFREAKData>* FTemplate)
{
	m_CriticalSection.Lock();
	FTemplate->clear();
	for(int i=0; i<m_FTD.size();i++)
	{
		(*FTemplate).push_back(m_FTD[i]);
	}
	m_CriticalSection.Unlock();

}

void KuDrawingInfo::setTemplateData(vector<CFREAKData> fTemplateresult)
{
	m_CriticalSection.Lock();

	m_FTD.clear();
	for(int i=0; i<fTemplateresult.size();i++)
	{
		m_FTD.push_back(fTemplateresult[i]);
	}
	m_CriticalSection.Unlock();

}
/**
@brief Korean: Fidutial Mark의 방향 정보와 ID 정보를 화면에 그러주기 위한 정보를 넘겨주는 함수
@brief English: write in English
*/
void KuDrawingInfo::drawFiducialMarkInfoToImage(list <KuPose> lFiducialLandMarkImgPos)
{
	char cData[1024]={0};
	CvFont* font = new CvFont;          // 폰트
	cvInitFont(font, CV_FONT_VECTOR0, 1.2f, 1.2f, 0, 2); 

	//drawing 하는 부분//
	//cout<<"랜드 마크 갯수: "<<lFiducialLandMarkImgPos.size()<<endl;
	for(list <KuPose>::iterator iter_FiducialLandMarkPos = lFiducialLandMarkImgPos.begin();
		iter_FiducialLandMarkPos != lFiducialLandMarkImgPos.end(); 
		iter_FiducialLandMarkPos++ ){
			int nID = iter_FiducialLandMarkPos->getID();
			int nX = (int)iter_FiducialLandMarkPos->getX();//좌우가 바꿔어 있는 영상이므로 다시 변환
			int nY = m_IplCeilingImage->height-(int)iter_FiducialLandMarkPos->getY();//좌우가 바꿔어 있는 영상이므로 다시 변환
			double dThetaRad = m_RobotPos.getThetaRad(); 
			int nEndX = (int)(20*cos(M_PI/2-dThetaRad));
			int nEnxY = (int)(20*sin(M_PI/2-dThetaRad));
			cvCircle(m_IplCeilingImage,cvPoint(nX,nY),3,CV_RGB(255,0,0),-1,8,0);
			cvLine(m_IplCeilingImage,cvPoint(nX,nY),cvPoint(nX+nEndX,nY-nEnxY),CV_RGB(0,255,0),3,8,0);
			sprintf_s(cData,"%d",nID);
			cvPutText(m_IplCeilingImage,cData,cvPoint(nX-(nEndX+40),nY+(nEnxY+40)),font,CV_RGB(0,255,255));
	} 

}

/**
@brief Korean: Fidutial Mark의 방향 정보와 ID 정보를 화면에 그러주기 위한 정보를 넘겨주는 함수
@brief English: write in English
*/
void KuDrawingInfo::drawFiducialMarkInfoToImage(KuPose lFiducialLandMarkImgPos)
{
	char cData[1024]={0};
	CvFont* font = new CvFont;          // 폰트
	cvInitFont(font, CV_FONT_VECTOR0, 1.2f, 1.2f, 0, 2); 

	//drawing 하는 부분//
	//cout<<"랜드 마크 갯수: "<<lFiducialLandMarkImgPos.size()<<endl;

	int nID = lFiducialLandMarkImgPos.getID();
	int nX = (int)lFiducialLandMarkImgPos.getX();//좌우가 바꿔어 있는 영상이므로 다시 변환
	int nY = m_IplCeilingImage->height-(int)lFiducialLandMarkImgPos.getY();//좌우가 바꿔어 있는 영상이므로 다시 변환
	double dThetaRad = m_RobotPos.getThetaRad(); 
	int nEndX = (int)(20*cos(M_PI/2-dThetaRad));
	int nEnxY = (int)(20*sin(M_PI/2-dThetaRad));
	cvCircle(m_IplCeilingImage,cvPoint(nX,nY),3,CV_RGB(255,0,0),-1,8,0);
	cvLine(m_IplCeilingImage,cvPoint(nX,nY),cvPoint(nX+nEndX,nY-nEnxY),CV_RGB(0,255,0),3,8,0);
	sprintf_s(cData,"%d",nID);
	cvPutText(m_IplCeilingImage,cData,cvPoint(nX-(nEndX+40),nY+(nEnxY+40)),font,CV_RGB(0,255,255));

}
/**
@brief Korean: Fiducial Mark List를 받아오는 함수
@brief English: write in English
*/
void KuDrawingInfo::setFiducialMarkList(list<KuPose> FiducialLandMarkList)
{
	m_CriticalSection.Lock();
	m_FiducialLandMarkList = FiducialLandMarkList;
	m_CriticalSection.Unlock();

}

/**
@brief Korean: Fiducial Mark List를  넘겨주는 함수
@brief English: write in English
*/
list<KuPose> KuDrawingInfo::getFiducialMarkList()
{
	m_CriticalSection.Lock();
	list<KuPose> FiducialLandMarkList;
	FiducialLandMarkList = m_FiducialLandMarkList;
	m_CriticalSection.Unlock();		
	return FiducialLandMarkList;
}