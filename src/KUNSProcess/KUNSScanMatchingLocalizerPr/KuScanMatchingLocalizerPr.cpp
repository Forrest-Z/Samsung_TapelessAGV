#include "stdafx.h"
#include "KuScanMatchingLocalizerPr.h"

KuScanMatchingLocalizerPr::KuScanMatchingLocalizerPr()
{

	initialize(); //각종 변수 초기화 함수 실행.
	cout<<"[KUNSScanMatchingLocalizerPr]: Instance is created!!!"<<endl;

}
KuScanMatchingLocalizerPr::~KuScanMatchingLocalizerPr()
{
	cout<<"[KuScanMatchingLocalizerPr]: Instance is destroyed!!!"<<endl;
}


/**
@brief Korean: 각종 변수등을 초기화 하는 함수
@brief English : 
*/
void KuScanMatchingLocalizerPr::initialize()
{
	m_RobotPos.init();	
	m_DeltaPosForICP.init();
	m_LastRobotPosForICP.init();
	m_bMapping=false;
	m_pMap=NULL;
	m_bThreadFlag= false;
	m_bIsThreadFuncGenerated=false;
	for(int i=0; i< 181; i++) {
		m_CartesianCoordiNewRangeData[i].setXm(0.);
		m_CartesianCoordiNewRangeData[i].setYm(0.);
		m_CartesianCoordiLastRangeData[i].setXm(0.);
		m_CartesianCoordiLastRangeData[i].setYm(0.);
	}

	for(int i=0; i< 181*2; i++) {
		m_CartesianCoordiLastRangeDataT[i].setXm(0.);
		m_CartesianCoordiLastRangeDataT[i].setYm(0.);

	}

	m_nVirtualLaserData= m_kuUtil.generateIntType1DArray(Sensor::URG04LX_DATA_NUM181,KuRobotParameter::getInstance()->getURG04LXLaserMaxDist());
	m_nTVirtualLaserData= m_kuUtil.generateIntType1DArray(Sensor::URG04LX_DATA_NUM181*2,KuRobotParameter::getInstance()->getURG04LXLaserMaxDist());
	m_nLaserData= m_kuUtil.generateIntType1DArray(Sensor::URG04LX_DATA_NUM181,KuRobotParameter::getInstance()->getURG04LXLaserMaxDist());
	m_ncopyLaserData=m_kuUtil.generateIntType1DArray(Sensor::URG04LX_DATA_NUM181,KuRobotParameter::getInstance()->getURG04LXLaserMaxDist());

	m_dAccumulatedDeltaMovementForICP = 0.0;	
	m_dLaserSensorOffsetmm = KuRobotParameter::getInstance()->getFrontLaserXOffset(); //레이저 센서와 로봇 중심간의 거리, unit mm
	m_dMaxSensorRangeMM = KuRobotParameter::getInstance()->getURG04LXLaserMaxDist(); //사용할 센서데이터의 최대 거리, unit mm
	m_dMinSensorRangeMM= KuRobotParameter::getInstance()->getURG04LXLaserMinDist(); //사용할 센서데이터의 최대 거리, unit mm
	// ICP가 수행될 간격. 이 거리마다 ICP가 수행. 
	// 너무 작으면 오차 발생. 최소 0.3~0.5m를 권장
	// 너무 크면 오차 발생. 특히 좁은 구역에서 작으면 매칭되는 포인트가 적어서 각도가 확~ 틀어짐.
	m_dDistanceForICPExecution = 0.1;
	m_dKXGain= 0.4;   //로봇의 전진방향 오차를 극복하는 게인. 실제 로봇의 경우 너무 크면 출렁이고, 너무 작으면 원하는 속도보다 느리게 움직임.
	m_dKYGain = 1.2;    //로봇의 측면방향 오차를 극복하는 게인. 영향이 크지는 않지만, 크면 불안정하고, 작으면 측면방향 오차를 보정 못함.
	m_dKThetaGain = 0.3;  //목적지를 향한 로봇의 방향(heading)을 극복하는 게인. 크면 매우 출렁이고(특히 초기에 제자리에서 목적지를 향해 휙 돔), 작으면 딴 방향을 향함.
	m_dDesiredVel = 500;
	m_dRatioofRadius = 560*3.14/180;

	srand((unsigned)time(NULL)); //랜덤 씨드를 항상 바꿔주기 위해서 설정해야한다.
}

/**
@brief Korean: 절대좌표상의 로봇의 X(mm)좌표를 내보내는 함수
@brief English : 
*/
double KuScanMatchingLocalizerPr::getRobotPosX()
{
	return m_RobotPos.getX();	
}

/**
@brief Korean: 절대좌표상의 로봇의 Y(mm)좌표를 내보내는 함수
@brief English : 
*/
double KuScanMatchingLocalizerPr::getRobotPosY()
{
	return m_RobotPos.getY();	
}

/**
@brief Korean: 절대좌표상의 로봇의 각도(deg)를 내보내는 함수
@brief English : 
*/
double KuScanMatchingLocalizerPr::getRobotPosDeg()
{
	return m_RobotPos.getThetaDeg();	

}

/**
@brief Korean: 절대 좌표상의 로봇의 X(mm)좌표를 설정하는 함수 
@brief English : 
*/
void KuScanMatchingLocalizerPr::setRobotPosX(double dRobotPosX)
{
	m_RobotPos.setX(dRobotPosX);	
	m_LastRobotPosForICP.setX(dRobotPosX); 
}

/**
@brief Korean: 절대 좌표상의 로봇의 Y(mm)좌표를 설정하는 함수 
@brief English : 
*/
void KuScanMatchingLocalizerPr::setRobotPosY(double dRobotPosY)
{
	m_RobotPos.setY( dRobotPosY ); 	
	m_LastRobotPosForICP.setY( dRobotPosY );	
}

/**
@brief Korean: 절대 좌표상의 로봇의 각도를 설정하는 함수 
@brief English : 
*/
void KuScanMatchingLocalizerPr::setRobotPosDeg(double dRobotPosThetaDeg)
{
	m_RobotPos.setThetaDeg( dRobotPosThetaDeg );
	m_LastRobotPosForICP.setThetaDeg(dRobotPosThetaDeg);	
}

/**
@brief Korean: 절대 좌표상의 로봇 위치를 설정하는 함수 
@brief English : 
*/
void KuScanMatchingLocalizerPr::setRobotPos(KuPose RobotPos)
{
	m_RobotPos = RobotPos;
	m_LastRobotPosForICP = RobotPos;	
	m_dAccumulatedDeltaMovementForICP = 0.0;
}

/**
@brief Korean: 절대 좌표상의 로봇 위치를 내보내는 함수
@brief English : 
*/
KuPose KuScanMatchingLocalizerPr::getRobotPos()
{
	return m_RobotPos;
}


/**
@brief Korean: 절대 좌표상의 초기 로봇 위치를 설정하는 함수 
@brief English : 
*/
void KuScanMatchingLocalizerPr::setInitRobotPos(KuPose RobotPos)
{
	m_RobotPos = RobotPos;
	m_LastRobotPosForICP = RobotPos;
	m_nMapSizeX=KuRobotParameter::getInstance()->getMapSizeXm()*10;
	m_nMapSizeY=KuRobotParameter::getInstance()->getMapSizeYm()*10;

	if(NULL==m_pMap)
	{
		m_pMap = new KuMap(m_nMapSizeX, m_nMapSizeY); 
	}
	m_bcheck=true;

	if(m_bIsThreadFuncGenerated==false)
	{
		m_bIsThreadFuncGenerated= true;
		m_Thread.start(doThread, this, 200, "KuScanMatchingLocalizerPr::setInitRobotPos()"); //메인 스레드 시작	

	}

}
void KuScanMatchingLocalizerPr::terminate()
{
	m_bIsThreadFuncGenerated= false;
	m_Thread.terminate();
}
bool KuScanMatchingLocalizerPr::getThreadStates()
{
	return m_bIsThreadFuncGenerated;
}
/**
@brief Korean:  Encoder로 추정된 로봇의 위치 값을 받아오는 함수
@brief English: 
of range sensor
*/
KuPose KuScanMatchingLocalizerPr::estimateRobotPosByDeadReckoning(KuPose EncoderDelPos)
{

	double dX = m_RobotPos.getX() + EncoderDelPos.getX() * cos(m_RobotPos.getThetaRad()) + 
		EncoderDelPos.getY() * sin(-m_RobotPos.getThetaRad());

	double dY = m_RobotPos.getY() + EncoderDelPos.getX() * sin(m_RobotPos.getThetaRad()) + 
		EncoderDelPos.getY() * cos(m_RobotPos.getThetaRad());
	double dThetaDeg = m_RobotPos.getThetaDeg() + EncoderDelPos.getThetaDeg();

	// pose update	
	m_RobotPos.setX(dX);
	m_RobotPos.setY(dY);
	m_RobotPos.setThetaDeg(dThetaDeg);

	return m_RobotPos;

}
/**
@brief Korean:   Encoder로 추정된 로봇의 위치 값을 받아 오는 함수
@brief English: 
of range sensor
*/
KuPose KuScanMatchingLocalizerPr::estimateRobotPosByDeadReckoning(KuPose RobotPos ,KuPose EncoderDelPos)
{

	double dX = RobotPos.getX() + EncoderDelPos.getX() * cos(RobotPos.getThetaRad()) + 
		EncoderDelPos.getY() * sin(-RobotPos.getThetaRad());

	double dY = RobotPos.getY() + EncoderDelPos.getX() * sin(RobotPos.getThetaRad()) + 
		EncoderDelPos.getY() * cos(RobotPos.getThetaRad());
	double dThetaDeg = RobotPos.getThetaDeg() + EncoderDelPos.getThetaDeg();

	// pose update	
	RobotPos.setX(dX);
	RobotPos.setY(dY);
	RobotPos.setThetaDeg(dThetaDeg);

	return RobotPos;
}
/**
@brief Korean: 거리센서 값을 저장함. 
poloar coordinate기준의 거리 센서 값을, 
cartesian coordinate기준의 값을 변경한다.
@brief English : 
*/
void KuScanMatchingLocalizerPr::copyRangeData(int_1DArray  nData)
{
	KuCartesianCoordinate2D CartesianCoordiData;
	KuCartesianCoordinate2D PreCartesianCoordiData;


	for(int i=0; i<LASER_SCAN_IDX; i++){
		double ld = (0 < i) ? fabs((double)(nData[i] - nData[i-1])/1000.0) : 1.;
		double rd = (i < LASER_SCAN_IDX - 1) ? fabs((double)(nData[i+1] - nData[i])/1000.0) : 1.;

		if (//ld < 0.05&& 
			//rd < 0.05 &&
			nData[i]< m_dMaxSensorRangeMM &&
			nData[i]>m_dMinSensorRangeMM)
		{	// 레이저스캐너의 유효거리를 벗어나면 처리 안함.

			CartesianCoordiData=m_math.transfromPolar2CartesianMM( nData[i], (i-90), m_dLaserSensorOffsetmm );

			if(0.05<hypot(PreCartesianCoordiData.getXm()-CartesianCoordiData.getXm(),PreCartesianCoordiData.getYm()-CartesianCoordiData.getYm()))
			{			
				m_CartesianCoordiNewRangeData[i] = CartesianCoordiData;
				PreCartesianCoordiData = CartesianCoordiData;
			}

		}	
		else
		{
			m_CartesianCoordiNewRangeData[i].setXm(0.);
			m_CartesianCoordiNewRangeData[i].setYm(0.);
		}

	}


}
/**
@brief Korean: 거리센서 값을 저장함. 
poloar coordinate기준의 거리 센서 값을, 
cartesian coordinate기준의 값을 변경한다.
@brief English : 
*/
void KuScanMatchingLocalizerPr::copyRangeDatafromMap(int_1DArray  nData)
{

	KuCartesianCoordinate2D CartesianCoordiData;
	KuCartesianCoordinate2D PreCartesianCoordiData;


	for(int i=0; i<LASER_SCAN_IDX; i++){
		double ld = (0 < i) ? fabs((double)(nData[i] - nData[i-1])/1000.0) : 1.;
		double rd = (i < LASER_SCAN_IDX - 1) ? fabs((double)(nData[i+1] - nData[i])/1000.0) : 1.;

		if (//ld < 0.05&& 
			//rd < 0.05 &&
			nData[i]< m_dMaxSensorRangeMM &&
			nData[i]>m_dMinSensorRangeMM)
		{	// 레이저스캐너의 유효거리를 벗어나면 처리 안함.

			CartesianCoordiData=m_math.transfromPolar2CartesianMM( nData[i], (i-90), m_dLaserSensorOffsetmm );

			if(0.05<hypot(PreCartesianCoordiData.getXm()-CartesianCoordiData.getXm(),PreCartesianCoordiData.getYm()-CartesianCoordiData.getYm()))
			{			
				m_CartesianCoordiLastRangeData[i] = CartesianCoordiData;
				PreCartesianCoordiData = CartesianCoordiData;
			}

		}	
		else
		{
			m_CartesianCoordiLastRangeData[i].setXm(0.);
			m_CartesianCoordiLastRangeData[i].setYm(0.);
		}

	}

}
/**
@brief Korean: 거리센서 값을 저장함. 
poloar coordinate기준의 거리 센서 값을, 
cartesian coordinate기준의 값을 변경한다.
@brief English : 
*/
void KuScanMatchingLocalizerPr::copyRangeDatafromMapT(int_1DArray  nData)
{

	KuCartesianCoordinate2D CartesianCoordiData;
	KuCartesianCoordinate2D PreCartesianCoordiData;


	for(int i=0; i<LASER_SCAN_IDX; i++){

		if (nData[i]< m_dMaxSensorRangeMM &&
			nData[i]>m_dMinSensorRangeMM)
		{	// 레이저스캐너의 유효거리를 벗어나면 처리 안함.

			CartesianCoordiData=m_math.transfromPolar2CartesianMM( nData[i], (i-90), m_dLaserSensorOffsetmm );

			if(0.05<hypot(PreCartesianCoordiData.getXm()-CartesianCoordiData.getXm(),PreCartesianCoordiData.getYm()-CartesianCoordiData.getYm()))
			{			
				m_CartesianCoordiLastRangeDataT[i] = CartesianCoordiData;
				PreCartesianCoordiData = CartesianCoordiData;
			}

		}	
		else
		{
			m_CartesianCoordiLastRangeDataT[i].setXm(0.);
			m_CartesianCoordiLastRangeDataT[i].setYm(0.);
		}

	}

	for(int i=LASER_SCAN_IDX; i<2*LASER_SCAN_IDX; i++){

		if (nData[i]< m_dMaxSensorRangeMM &&
			nData[i]>m_dMinSensorRangeMM)
		{	// 레이저스캐너의 유효거리를 벗어나면 처리 안함.

			CartesianCoordiData=m_math.transfromPolar2CartesianMM( nData[i], (i-90-LASER_SCAN_IDX), m_dLaserSensorOffsetmm );

			if(0.05<hypot(PreCartesianCoordiData.getXm()-CartesianCoordiData.getXm(),PreCartesianCoordiData.getYm()-CartesianCoordiData.getYm()))
			{			
				m_CartesianCoordiLastRangeDataT[i] = CartesianCoordiData;
				PreCartesianCoordiData = CartesianCoordiData;
			}

		}	
		else
		{
			m_CartesianCoordiLastRangeDataT[i].setXm(0.);
			m_CartesianCoordiLastRangeDataT[i].setYm(0.);
		}

	}

}
/**
@brief Korean: 추정된 로봇의 지도 정보를 바탕으로 가상의 레이저 데이터를 받아온다
@brief English: 
*/
inline void  KuScanMatchingLocalizerPr::calculateLaserDatafromMap( KuPose RobotPos,KuMap* pMap)
{
	double dCellSize=Sensor::CELLSIZE;
	const int LASER_MAX_DIST = m_dMaxSensorRangeMM;
	const int LASER_MIN_DIST =m_dMinSensorRangeMM;//unit mm
	int nScanIdx = Sensor::URG04LX_DATA_NUM181;
	double dX_LASER_OFFSET =m_dLaserSensorOffsetmm;
	int nMapSizeX=pMap->getX();
	int nMapSizeY=pMap->getY();

	int nRobotX = (int)RobotPos.getX()/100;
	int nRobotY = (int)RobotPos.getY()/100;
	int nX=0., nY=0;
	double dTempX;
	double dTempY;

	double dX,dY,dZ;
	double dCR=cos(RobotPos.getThetaRad());
	double dSR=sin(RobotPos.getThetaRad());

	int ** nMap=pMap->getMap();

	for (int i=0; i<nScanIdx; i++){
		m_nVirtualLaserData[i]=LASER_MAX_DIST;
		for(int j = 0; j < LASER_MAX_DIST; j += 50) // 3 cm step
		{
			dTempX = j * cos((i - 90) * D2R) + dX_LASER_OFFSET;
			dTempY = j * sin((i - 90) * D2R);

			nX = nRobotX + (int)((dTempX * dCR - dTempY * dSR)/100.0);
			nY = nRobotY + (int)((dTempX * dSR + dTempY * dCR)/100.0);

			if(nX < 1 || nX >=nMapSizeX-1 || nY < 1 || nY >= nMapSizeY-1) continue;
			//if( nMap[nX][nY]== KuMap::OCCUPIED_AREA||nMap[nX][nY] == KuMap::UNKNOWN_AREA)

			if( nMap[nX][nY]== KuMap::OCCUPIED_AREA)
			{
				if(m_nVirtualLaserData[i] > j  )
				{
					m_nVirtualLaserData[i] = j;
				}
				break;
			}

		}

	}

}
/**
@brief Korean: 추정된 로봇의 지도 정보를 바탕으로 가상의 레이저 데이터를 받아온다
@brief English: 
*/
void  KuScanMatchingLocalizerPr::penetrateLaserDatafromMap( KuPose RobotPos,KuMap* pMap)
{
	double dCellSize=Sensor::CELLSIZE;
	const int LASER_MAX_DIST = m_dMaxSensorRangeMM;
	const int LASER_MIN_DIST =m_dMinSensorRangeMM;//unit mm
	int nScanIdx = Sensor::URG04LX_DATA_NUM181;
	double dX_LASER_OFFSET =m_dLaserSensorOffsetmm;
	int nMapSizeX=pMap->getX();
	int nMapSizeY=pMap->getY();

	int nRobotX = (int)RobotPos.getX()/100;
	int nRobotY = (int)RobotPos.getY()/100;
	int nX=0., nY=0;
	double dTempX;
	double dTempY;

	double dX,dY,dZ;
	double dCR=cos(RobotPos.getThetaRad());
	double dSR=sin(RobotPos.getThetaRad());

	int ** nMap=pMap->getMap();

	for (int i=0; i<nScanIdx; i++){

		m_nTVirtualLaserData[i] = m_nVirtualLaserData[i] ;
	}

	for (int i=nScanIdx; i<nScanIdx*2; i++){
		m_nTVirtualLaserData[i]=LASER_MAX_DIST;
		bool bOccflag=true;
		for(int j = m_nTVirtualLaserData[i-nScanIdx]; j < LASER_MAX_DIST; j += 50) // 3 cm step
		{
			dTempX = j * cos((i-nScanIdx - 90) * D2R) + dX_LASER_OFFSET;
			dTempY = j * sin((i-nScanIdx - 90) * D2R);

			nX = nRobotX + (int)((dTempX * dCR - dTempY * dSR) /100.0);
			nY = nRobotY + (int)((dTempX * dSR + dTempY * dCR) /100.0);

			if(nX < 1 || nX >=nMapSizeX-1 || nY < 1 || nY >= nMapSizeY-1) continue;

			if( nMap[nX][nY]== KuMap::OCCUPIED_AREA&&bOccflag)
			{
				continue;
			}
			else if(nMap[nX][nY]== KuMap::EMPTY_AREA)
			{
				bOccflag=false;
			}

			if( nMap[nX][nY]== KuMap::OCCUPIED_AREA)
			{
				if(m_nTVirtualLaserData[i] > j  )
				{
					m_nTVirtualLaserData[i] = j;
					break;
				}
			}
		}
	}
	KuDrawingInfo::getInstance()->setTData(m_nTVirtualLaserData);
}
/**
@brief Korean:ICP 알고리듬을 이용하여 로봇의 위치를 추정하는 함수
@brief English: 
*/
double  KuScanMatchingLocalizerPr::estimateRobotPosP(int_1DArray nData, KuPose DeltaEncoderData,KuPose RobotPos,KuMap* pMap,bool* bMapping)
{
	m_RobotPos.setX(RobotPos.getX());
	m_RobotPos.setY(RobotPos.getY());
	m_RobotPos.setThetaDeg(RobotPos.getThetaDeg());

	(*bMapping)=m_bMapping;
	computeAccumulatedDeltaMovementForICP(DeltaEncoderData); //ICP수행을 위해 엔코더에서 얻은 delta x,y,theta. 변화량을 누적한다.

	m_bICPExecution = isAccDeltaMovementForICPOver(m_dDistanceForICPExecution); //ICP를 수행할지를 정하는 함수.

	m_dError=1.0;
	m_CriticalSection.Lock();
	for(int i=0; i<Sensor::URG04LX_DATA_NUM181;i++)
		m_nLaserData[i]=nData[i];

	int** nMap = m_pMap->getMap();
	int** nRefMap = pMap->getMap();

	for(int i=0; i<m_nMapSizeX;i++)
	{
		for(int j=0; j<m_nMapSizeY;j++)
		{
			nMap[i][j] = nRefMap[i][j];
		}
	}
	m_CriticalSection.Unlock();

	return m_dError;
}	
/**
@brief Korean:ICP 알고리듬을 이용하여 로봇의 위치를 추정하는 함수
@brief English: 
*/
double  KuScanMatchingLocalizerPr::estimateRobotPos(int_1DArray nData, KuPose DeltaEncoderData,KuPose RobotPos,KuMap* pMap)
{
	//	m_RobotPos=RobotPos;
	estimateRobotPosByDeadReckoning(DeltaEncoderData);

	computeAccumulatedDeltaMovementForICP(DeltaEncoderData); //ICP수행을 위해 엔코더에서 얻은 delta x,y,theta. 변화량을 누적한다.

	m_bICPExecution = isAccDeltaMovementForICPOver(m_dDistanceForICPExecution); //ICP를 수행할지를 정하는 함수.

	m_dError=1.0;
	m_CriticalSection.Lock();
	for(int i=0; i<Sensor::URG04LX_DATA_NUM181;i++)
		m_nLaserData[i]=nData[i];

	int** nMap = m_pMap->getMap();
	int** nRefMap = pMap->getMap();

	for(int i=0; i<m_nMapSizeX;i++)
	{
		for(int j=0; j<m_nMapSizeY;j++)
		{
			nMap[i][j] = nRefMap[i][j];
		}
	}
	m_CriticalSection.Unlock();

	//if(bICPExecution)
	{
		// 		m_CriticalSection.Lock();
		// 		calculateLaserDatafromMap( m_RobotPos, pMap);
		// 		penetrateLaserDatafromMap( m_RobotPos, pMap);
		// 		m_CriticalSection.Unlock();
		// 
		// 		//KuDrawingInfo::getInstance()->setAlignLaserData181(m_nVirtualLaserData);
		// 
		// 		KuPose DeltaPos=computeDeltaPoseforNavi(m_nTVirtualLaserData,nData,m_DeltaPosForICP);
		// 
		// 		estimateRobotPosByDeadReckoning(DeltaPos);
	}

	return m_dError;
}	
void KuScanMatchingLocalizerPr::doThread(void* arg)
{

	KuScanMatchingLocalizerPr* pSML = (KuScanMatchingLocalizerPr*)arg;

	if(/*pSML->m_bICPExecution&&*/hypot(pSML->m_DeltaPos.getX(),pSML->m_DeltaPos.getY())<10){
		pSML->m_DeltaPos.init();
		pSML->m_dAccumulatedDeltaMovementForICP = 0.0;
		pSML->m_CriticalSection.Lock();
		for(int i=0; i<Sensor::URG04LX_DATA_NUM181;i++)	pSML->m_ncopyLaserData[i]=pSML->m_nLaserData[i];
		pSML->calculateLaserDatafromMap( pSML->m_RobotPos, pSML->m_pMap);
		//	pSML->penetrateLaserDatafromMap( pSML->m_RobotPos, pSML->m_pMap);
		pSML->m_CriticalSection.Unlock();

		KuPose DeltaPos = pSML->computeDeltaPoseforNavi(pSML->m_nVirtualLaserData,pSML->m_ncopyLaserData,pSML->m_DeltaPosForICP);

		double dX = pSML->m_DeltaPos.getX() + DeltaPos.getX() * cos(pSML->m_DeltaPos.getThetaRad()) + 
			DeltaPos.getY() * sin(-pSML->m_DeltaPos.getThetaRad());
		double dY = pSML->m_DeltaPos.getY() + DeltaPos.getX() * sin(pSML->m_DeltaPos.getThetaRad()) + 
			DeltaPos.getY() * cos(pSML->m_DeltaPos.getThetaRad());
		double dThetaDeg = pSML->m_DeltaPos.getThetaDeg() + DeltaPos.getThetaDeg();

		pSML->m_CriticalSection.Lock();
		// pose update	
		pSML->m_DeltaPos.setX(dX);
		pSML->m_DeltaPos.setY(dY);
		pSML->m_DeltaPos.setThetaDeg(dThetaDeg);
		pSML->m_CriticalSection.Unlock();
		
		//pSML->estimateRobotPosByDeadReckoning(DeltaPos);
		KuDrawingInfo::getInstance()->setAlignLaserData181(pSML->m_nVirtualLaserData);
	}

}
KuPose  KuScanMatchingLocalizerPr::getDeltaPos()
{
	KuPose DeltaPos;
	// pose update	
	m_CriticalSection.Lock();
	DeltaPos.setX(m_DeltaPos.getX()/10.0);
	DeltaPos.setY(m_DeltaPos.getY()/10.0);
	DeltaPos.setThetaDeg(m_DeltaPos.getThetaDeg()/10.0);
	m_CriticalSection.Unlock();

	m_CriticalSection.Lock();
	m_DeltaPos.setX(m_DeltaPos.getX()-DeltaPos.getX());
	m_DeltaPos.setY(m_DeltaPos.getY()-DeltaPos.getY());
	m_DeltaPos.setThetaDeg(m_DeltaPos.getThetaDeg()-DeltaPos.getThetaDeg());
	m_CriticalSection.Unlock();

	return DeltaPos;
}


/**
@brief Korean: 이전 거리정보(t-1)와 현재 거리정보(t)를 비교하여 위치의 변화량을 구하는 함수
@brief English: 
*/
KuPose KuScanMatchingLocalizerPr::computeDeltaPoseforNavi( int_1DArray nPreLaserData, int_1DArray nNewLaserData,KuPose DeltaEncoderData)
{
	double PreData[362], NewData[362]; //--> 181*2 181은 레이저 데이터 개수, 2는 x,y
	int NoNewData, NoPreData;

	copyRangeData(nNewLaserData);
	copyRangeDatafromMap(nPreLaserData);
	m_DeltaPosForICP.init();
	// 형식에 맞춰서 센서값 입력
	NoPreData = 0;
	for (int i=0; i<LASER_SCAN_IDX; i++) {
		double dDIst= sqrt(m_CartesianCoordiLastRangeData[i].getXm() * m_CartesianCoordiLastRangeData[i].getXm() + 
			m_CartesianCoordiLastRangeData[i].getYm() * m_CartesianCoordiLastRangeData[i].getYm() );
		if (  dDIst<= m_dMaxSensorRangeMM*MM2M&&dDIst> m_dMinSensorRangeMM*MM2M) {
			PreData[NoPreData*2+0] = m_CartesianCoordiLastRangeData[i].getXm();
			PreData[NoPreData*2+1] = m_CartesianCoordiLastRangeData[i].getYm();
			NoPreData++;
		}
	}
	NoNewData = 0;
	for (int i=0; i<LASER_SCAN_IDX; i++) {
		double dDIst= sqrt(m_CartesianCoordiNewRangeData[i].getXm() * m_CartesianCoordiNewRangeData[i].getXm() + 
			m_CartesianCoordiNewRangeData[i].getYm() * m_CartesianCoordiNewRangeData[i].getYm() );
		if (  dDIst<= m_dMaxSensorRangeMM*MM2M&&dDIst> m_dMinSensorRangeMM*MM2M)  {
			NewData[NoNewData*2+0] = m_CartesianCoordiNewRangeData[i].getXm();
			NewData[NoNewData*2+1] = m_CartesianCoordiNewRangeData[i].getYm() ;
			NoNewData++;
		}
	}


	// calculate delta pose start -------------------------------------------------------------------------------------------------------------------------
	KuPose DeltaPose;
	double RMatrix[3][3];
	double TMatrix[3];
	m_dError=10000.0;
	if(m_bcheck==true)
	{
		m_bcheck=false;
		m_dError=m_pICP.icp_Mathch(RMatrix, TMatrix, PreData, NoPreData, NewData, NoNewData, 20, 50, 0.5, 0.1, 10.0, DeltaEncoderData.getXm(),DeltaEncoderData.getYm(),DeltaEncoderData.getThetaRad());
		m_bcheck=true;
	}
	else
	{
		return DeltaPose;
	}


	m_RobotPos.setID(-1);

	if(	m_dError<0.09
		&&m_dError!=-1
		&&1.0>fabs(TMatrix[0])
		&&1.0>fabs(TMatrix[1])
		&&0.3<hypot(TMatrix[0],TMatrix[1])
		&&5.0*D2R>fabs(RMatrix[0][0]))
		//TMatrix[2]>(NoNewData)/1.5)
	{		
		printf("-----------------\t");
		printf("derror=%f\t",m_dError);
		printf("Match=%f\n",TMatrix[2]);

		DeltaPose.setXm(TMatrix[0]);
		DeltaPose.setYm(TMatrix[1]);
		DeltaPose.setThetaRad(RMatrix[0][0]);
		m_bMapping=false;
	}
	else if(m_dError<0.1/*&&TMatrix[2]>65*/
		&&0.3>hypot(TMatrix[0],TMatrix[1])
		&&3.0*D2R>fabs(RMatrix[0][0]))
	{
		m_bMapping=true;
	}
	else
	{
		m_bMapping=false;
	}

	// calculate delta pose end -------------------------------------------------------------------------------------------------------------------------

	return DeltaPose;
}
/**
@brief Korean: 평균 0, sigma인 가우시안 분포를 따르는 값을 출력하는 함수
@brief English: Function returning values according to Gaussian distribution with sigma and x
*/
double KuScanMatchingLocalizerPr::GetGaussianValue(double sigma, double x)
{
	if (sigma==0) return 0.1;

	return 1/sqrt(2*M_PI*sigma*sigma)*exp(-x*x/2.0/sigma/sigma);
}

/**
@brief Korean: 이전 거리정보(t-1)와 현재 거리정보(t)를 비교하여 위치의 변화량을 구하는 함수
@brief English: 
*/
KuPose KuScanMatchingLocalizerPr::computeDeltaPose( int_1DArray nPreLaserData, int_1DArray nNewLaserData)
{
	double PreData[362], NewData[362]; //--> 181*2 181은 레이저 데이터 개수, 2는 x,y
	int NoNewData, NoPreData;

	copyRangeData(nNewLaserData);
	copyRangeDatafromMap(nPreLaserData);

	// 형식에 맞춰서 센서값 입력
	NoPreData = 0;
	for (int i=0; i<LASER_SCAN_IDX; i++) {
		double dDIst= sqrt(m_CartesianCoordiLastRangeData[i].getXm() * m_CartesianCoordiLastRangeData[i].getXm() + 
			m_CartesianCoordiLastRangeData[i].getYm() * m_CartesianCoordiLastRangeData[i].getYm() );
		if (  dDIst<= m_dMaxSensorRangeMM*MM2M&&dDIst> m_dMinSensorRangeMM*MM2M) {
			PreData[NoPreData*2+0] = m_CartesianCoordiLastRangeData[i].getXm();
			PreData[NoPreData*2+1] = m_CartesianCoordiLastRangeData[i].getYm();
			NoPreData++;
		}
	}
	NoNewData = 0;

	for (int i=0; i<LASER_SCAN_IDX; i++) {
		double dDIst= sqrt(m_CartesianCoordiNewRangeData[i].getXm() * m_CartesianCoordiNewRangeData[i].getXm() + 
			m_CartesianCoordiNewRangeData[i].getYm() * m_CartesianCoordiNewRangeData[i].getYm() );
		if (  dDIst<= m_dMaxSensorRangeMM*MM2M&&dDIst> m_dMinSensorRangeMM*MM2M)  {
			NewData[NoNewData*2+0] = m_CartesianCoordiNewRangeData[i].getXm();
			NewData[NoNewData*2+1] = m_CartesianCoordiNewRangeData[i].getYm() ;
			NoNewData++;
		}
	}


	// calculate delta pose start -------------------------------------------------------------------------------------------------------------------------
	KuPose DeltaPose;

	double RMatrix[3][3];
	double TMatrix[3];
	m_dError=m_pICP.icp(RMatrix, TMatrix, PreData, NoPreData, NewData, NoNewData, 50, 100, 0.9, 0.2, 3.0);

	DeltaPose.setXm(TMatrix[0]);
	DeltaPose.setYm(TMatrix[1]);
	DeltaPose.setThetaRad(RMatrix[0][0]);
	DeltaPose.setPro(m_dError);
	// calculate delta pose end -------------------------------------------------------------------------------------------------------------------------

	return DeltaPose;
}

/**
@brief Korean: 거리정보를 이용하여 로봇의 위치를 추정하는 함수
@brief English: 
*/
double KuScanMatchingLocalizerPr::setDeltaPose(KuPose DeltaPos)
{
	bool bReturn = false;
	int NoNewData, NoPreData;
	double PreData[362], NewData[362]; //--> 181*2 181은 레이저 데이터 개수, 2는 x,y
	double Tmatrix[3], Rmatrix[3][3];
	double RMSError=-1.0;

	m_DeltaPos = DeltaPos; // 엔코더에서 얻은 delta x,y,theta. 변화량. 절대값 아님.

	// 	bool bICPExecution = isAccDeltaMovementForICPOver(m_dDistanceForICPExecution); //ICP를 수행할지를 정하는 함수.
	// 	
	// 	if (bICPExecution) {
	// -------------------------------------------------------- ICP ---------------------------------------------------------------- //			


	// 형식에 맞춰서 센서값 입력
	NoPreData = 0;
	for (int i=0; i<LASER_SCAN_IDX; i++) {
		if ( sqrt(m_CartesianCoordiLastRangeData[i].getXm() * m_CartesianCoordiLastRangeData[i].getXm() + 
			m_CartesianCoordiLastRangeData[i].getYm() * m_CartesianCoordiLastRangeData[i].getYm() ) <= m_dMaxSensorRangeMM*MM2M
			&&m_CartesianCoordiLastRangeData[i].getXm() !=0&&m_CartesianCoordiLastRangeData[i].getYm()!=0) {
				PreData[NoPreData*2+0] = m_CartesianCoordiLastRangeData[i].getXm();
				PreData[NoPreData*2+1] = m_CartesianCoordiLastRangeData[i].getYm();
				NoPreData++;
		}
	}

	NoNewData = 0;
	for (int i=0; i<LASER_SCAN_IDX; i++) {
		if ( sqrt( m_CartesianCoordiNewRangeData[i].getXm() * m_CartesianCoordiNewRangeData[i].getXm() + 
			m_CartesianCoordiNewRangeData[i].getYm() * m_CartesianCoordiNewRangeData[i].getYm() ) <= m_dMaxSensorRangeMM*MM2M
			&&m_CartesianCoordiNewRangeData[i].getXm() !=0&&m_CartesianCoordiNewRangeData[i].getYm()!=0) {
				NewData[NoNewData*2+0] = m_CartesianCoordiNewRangeData[i].getXm();
				NewData[NoNewData*2+1] = m_CartesianCoordiNewRangeData[i].getYm();
				NoNewData++;
		}
	}

	if(NoPreData<10||NoNewData<10) return -1.0;


	// ICP 수행
	RMSError = m_pICP.icp(Rmatrix, Tmatrix, PreData, NoPreData, NewData, NoNewData, 30, 500, 0.1, 0.01, 2.0);
	//printf("(%.3f, %.3f, %.3f) - (%.3f, %.3f, %.3f)\n", Rmatrix[0][0], Rmatrix[1][1], Rmatrix[2][2],Tmatrix[0],Tmatrix[1],Tmatrix[2]);		
	// ----------------------------------------------------------------------------------------------------------------------------- //


	if (RMSError!=-1 && RMSError<1.0){
		computeRobotPoseByICPFor2D(Tmatrix[0], Tmatrix[1], Rmatrix[0][0]);

	}
	// 		else{
	// 			computeRobotPoseByICPFor2D(0,0,0);
	// 		}
	//			bReturn = true;
	//	} 
	// 	else{ //ICP를 수행하지 않는 상황에서는 엔코더 데이터 만으로 로봇의 위치를 계산한다.
	// 		calculateRobotPosByEncoderData();
	// 	}
	// 		
	//	computeAccumulatedDeltaMovementForICP(DeltaPos); //ICP수행을 위해 엔코더에서 얻은 delta x,y,theta. 변화량을 누적한다.


	return RMSError;
}
/**
@brief Korean: ICP로 추출한 행렬(matrix)을 이용하여 로봇의 위치를 추정하는 함수
@brief English: 
*/
void KuScanMatchingLocalizerPr::computeRobotPoseByICPFor2D(double dX, double dY, double dTheta)
{
	double s1=0.,c1=0.;
	//	m_RobotPos = m_LastRobotPosForICP;

	// ------------------------- movement by encoder data ------------------------- //
	// yaw 계산에 반복되는 싸인,코싸인값 미리 계산
	// 	s1 = sin(m_RobotPos.getThetaRad() );	// yaw
	// 	c1 = cos(m_RobotPos.getThetaRad());
	// 
	// 
	// 	// localizer를 수행하여 보정하는 위치
	// 	double dXm = (c1 * m_DeltaPosForICP.getXm() + -s1 * m_DeltaPosForICP.getYm());
	// 	double dYm = (s1 * m_DeltaPosForICP.getXm() + c1 * m_DeltaPosForICP.getYm());
	// 
	// 	m_RobotPos.setXm( m_RobotPos.getXm() + dXm);
	// 	m_RobotPos.setYm( m_RobotPos.getYm() + dYm);
	// 	m_RobotPos.setThetaRad( m_RobotPos.getThetaRad() + m_DeltaPosForICP.getThetaRad() );
	// 
	// 	m_DeltaPosForICP.init();
	// 	

	// ------------------------- motion correction by ICP result ------------------------- //
	// 반복되는 싸인,코싸인값 미리 계산
	s1 = sin(m_RobotPos.getThetaRad());	// yaw
	c1 = cos(m_RobotPos.getThetaRad());


	// localizer를 수행하여 보정하는 위치
	double dXm = (c1*dX + -s1*dY);
	double dYm = (s1*dX + c1*dY);

	m_RobotPos.setXm( m_RobotPos.getXm() + dXm);
	m_RobotPos.setYm( m_RobotPos.getYm() + dYm);
	m_RobotPos.setThetaRad( m_RobotPos.getThetaRad() + dTheta);

	m_estimatedRobotPos = m_RobotPos;
}


/**
@brief Korean: Encoder 정보만을 이용한 로봇위치 계산 함수
@brief English: 
*/
void KuScanMatchingLocalizerPr::calculateRobotPosByEncoderData()
{
	double s1=0.,c1=0.;


	// roll-pitch-yaw 계산에 반복되는 싸인,코싸인값 미리 계산
	s1 = sin(m_RobotPos.getThetaRad());	// yaw
	c1 = cos(m_RobotPos.getThetaRad() );


	// localizer를 수행하여 보정하는 위치
	double dXm = (cos(m_RobotPos.getThetaRad() ) * m_DeltaPos.getXm() + -s1 * m_DeltaPos.getYm() );
	double dYm = (s1 * m_DeltaPos.getXm() + c1*m_DeltaPos.getYm() );
	m_RobotPos.setXm( m_RobotPos.getXm() + dXm );
	m_RobotPos.setYm( m_RobotPos.getYm() + dYm );
	m_RobotPos.setThetaRad( m_RobotPos.getThetaRad() + m_DeltaPos.getThetaRad() );
}

/**
@brief Korean: ICP수행을 위해 엔코더에서 얻은 delta x,y,theta. 변화량을 누적하는 함수.
@brief English: 
*/
void KuScanMatchingLocalizerPr::computeAccumulatedDeltaMovementForICP(KuPose DeltaPos)
{
	m_dAccumulatedDeltaMovementForICP += fabs(DeltaPos.getXm()) + fabs(DeltaPos.getYm()) + fabs(DeltaPos.getThetaRad())*20.0/m_math.getPI();	// 각도는 180도 이동을 20m 이동한 것으로 취급

	double dXm = DeltaPos.getXm() * cos(m_DeltaPosForICP.getThetaRad() ) - DeltaPos.getYm() * sin(m_DeltaPosForICP.getThetaRad());
	double dYm = DeltaPos.getXm() *sin(m_DeltaPosForICP.getThetaRad()) + DeltaPos.getYm() * cos(m_DeltaPosForICP.getThetaRad());

	m_DeltaPosForICP.setXm( m_DeltaPosForICP.getXm() + dXm );
	m_DeltaPosForICP.setYm( m_DeltaPosForICP.getYm() + dYm );
	m_DeltaPosForICP.setThetaRad( m_DeltaPosForICP.getThetaRad() + DeltaPos.getThetaRad() );


}

/**
@brief Korean: 로봇이 일정거리이상 움직였는지 여부를 검사하는 함수			
@brief English: 
*/
bool KuScanMatchingLocalizerPr::isAccDeltaMovementForICPOver(double dValue)
{
	if (m_dAccumulatedDeltaMovementForICP > dValue) {
		//m_dAccumulatedDeltaMovementForICP = 0.0;
		return true;
	}
	else return false;
}

KuPose KuScanMatchingLocalizerPr::getEstimatedRobotPos()
{

	return m_estimatedRobotPos;
}

inline double KuScanMatchingLocalizerPr::GetRandValue(double sigma)
{
	return sigma*0.5*( (double)rand()/(double)RAND_MAX - 0.5);
	//return sigma*2.0*( (double)rand()/(double)RAND_MAX - 0.5);
}

void KuScanMatchingLocalizerPr::setParameter(double dKXGain, double dKYGain,double dKThetaGain,double dDesiredVel, double dWheelbase)
{

	m_dKXGain = dKXGain;   //로봇의 전진방향 오차를 극복하는 게인. 실제 로봇의 경우 너무 크면 출렁이고, 너무 작으면 원하는 속도보다 느리게 움직임.
	m_dKYGain = dKYGain*2.0;   //로봇의 측면방향 오차를 극복하는 게인. 영향이 크지는 않지만, 크면 불안정하고, 작으면 측면방향 오차를 보정 못함.
	m_dKThetaGain = dKThetaGain/2.0;  //목적지를 향한 로봇의 방향(heading)을 극복하는 게인. 크면 매우 출렁이고(특히 초기에 제자리에서 목적지를 향해 휙 돔), 작으면 딴 방향을 향함.
	m_dDesiredVel= dDesiredVel;
	m_dRatioofRadius =dWheelbase*M_PI/180;

}
/**
@brief Korean: 모션에 대한 불확실성 정보를 받아온다
@brief English: set uncertainty of wheel motion
of range sensor
*/
void KuScanMatchingLocalizerPr::setDeviation(double dDeviationforTrans, double dDeviationforRotate,double dDeviationforTransRotate )
{
	m_pICP.setDeviation( dDeviationforTrans,  dDeviationforRotate, dDeviationforTransRotate );
}
