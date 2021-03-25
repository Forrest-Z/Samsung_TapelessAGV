#include "stdafx.h"
#include "KuScanMatchingLocalizerPr.h"

KuScanMatchingLocalizerPr::KuScanMatchingLocalizerPr()
{

	initialize(); //���� ���� �ʱ�ȭ �Լ� ����.
	cout<<"[KUNSScanMatchingLocalizerPr]: Instance is created!!!"<<endl;

}
KuScanMatchingLocalizerPr::~KuScanMatchingLocalizerPr()
{
	cout<<"[KuScanMatchingLocalizerPr]: Instance is destroyed!!!"<<endl;
}


/**
@brief Korean: ���� �������� �ʱ�ȭ �ϴ� �Լ�
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
	m_dLaserSensorOffsetmm = KuRobotParameter::getInstance()->getFrontLaserXOffset(); //������ ������ �κ� �߽ɰ��� �Ÿ�, unit mm
	m_dMaxSensorRangeMM = KuRobotParameter::getInstance()->getURG04LXLaserMaxDist(); //����� ������������ �ִ� �Ÿ�, unit mm
	m_dMinSensorRangeMM= KuRobotParameter::getInstance()->getURG04LXLaserMinDist(); //����� ������������ �ִ� �Ÿ�, unit mm
	// ICP�� ����� ����. �� �Ÿ����� ICP�� ����. 
	// �ʹ� ������ ���� �߻�. �ּ� 0.3~0.5m�� ����
	// �ʹ� ũ�� ���� �߻�. Ư�� ���� �������� ������ ��Ī�Ǵ� ����Ʈ�� ��� ������ Ȯ~ Ʋ����.
	m_dDistanceForICPExecution = 0.1;
	m_dKXGain= 0.4;   //�κ��� �������� ������ �غ��ϴ� ����. ���� �κ��� ��� �ʹ� ũ�� �ⷷ�̰�, �ʹ� ������ ���ϴ� �ӵ����� ������ ������.
	m_dKYGain = 1.2;    //�κ��� ������� ������ �غ��ϴ� ����. ������ ũ���� ������, ũ�� �Ҿ����ϰ�, ������ ������� ������ ���� ����.
	m_dKThetaGain = 0.3;  //�������� ���� �κ��� ����(heading)�� �غ��ϴ� ����. ũ�� �ſ� �ⷷ�̰�(Ư�� �ʱ⿡ ���ڸ����� �������� ���� �� ��), ������ �� ������ ����.
	m_dDesiredVel = 500;
	m_dRatioofRadius = 560*3.14/180;

	srand((unsigned)time(NULL)); //���� ���带 �׻� �ٲ��ֱ� ���ؼ� �����ؾ��Ѵ�.
}

/**
@brief Korean: ������ǥ���� �κ��� X(mm)��ǥ�� �������� �Լ�
@brief English : 
*/
double KuScanMatchingLocalizerPr::getRobotPosX()
{
	return m_RobotPos.getX();	
}

/**
@brief Korean: ������ǥ���� �κ��� Y(mm)��ǥ�� �������� �Լ�
@brief English : 
*/
double KuScanMatchingLocalizerPr::getRobotPosY()
{
	return m_RobotPos.getY();	
}

/**
@brief Korean: ������ǥ���� �κ��� ����(deg)�� �������� �Լ�
@brief English : 
*/
double KuScanMatchingLocalizerPr::getRobotPosDeg()
{
	return m_RobotPos.getThetaDeg();	

}

/**
@brief Korean: ���� ��ǥ���� �κ��� X(mm)��ǥ�� �����ϴ� �Լ� 
@brief English : 
*/
void KuScanMatchingLocalizerPr::setRobotPosX(double dRobotPosX)
{
	m_RobotPos.setX(dRobotPosX);	
	m_LastRobotPosForICP.setX(dRobotPosX); 
}

/**
@brief Korean: ���� ��ǥ���� �κ��� Y(mm)��ǥ�� �����ϴ� �Լ� 
@brief English : 
*/
void KuScanMatchingLocalizerPr::setRobotPosY(double dRobotPosY)
{
	m_RobotPos.setY( dRobotPosY ); 	
	m_LastRobotPosForICP.setY( dRobotPosY );	
}

/**
@brief Korean: ���� ��ǥ���� �κ��� ������ �����ϴ� �Լ� 
@brief English : 
*/
void KuScanMatchingLocalizerPr::setRobotPosDeg(double dRobotPosThetaDeg)
{
	m_RobotPos.setThetaDeg( dRobotPosThetaDeg );
	m_LastRobotPosForICP.setThetaDeg(dRobotPosThetaDeg);	
}

/**
@brief Korean: ���� ��ǥ���� �κ� ��ġ�� �����ϴ� �Լ� 
@brief English : 
*/
void KuScanMatchingLocalizerPr::setRobotPos(KuPose RobotPos)
{
	m_RobotPos = RobotPos;
	m_LastRobotPosForICP = RobotPos;	
	m_dAccumulatedDeltaMovementForICP = 0.0;
}

/**
@brief Korean: ���� ��ǥ���� �κ� ��ġ�� �������� �Լ�
@brief English : 
*/
KuPose KuScanMatchingLocalizerPr::getRobotPos()
{
	return m_RobotPos;
}


/**
@brief Korean: ���� ��ǥ���� �ʱ� �κ� ��ġ�� �����ϴ� �Լ� 
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
		m_Thread.start(doThread, this, 200, "KuScanMatchingLocalizerPr::setInitRobotPos()"); //���� ������ ����	

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
@brief Korean:  Encoder�� ������ �κ��� ��ġ ���� �޾ƿ��� �Լ�
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
@brief Korean:   Encoder�� ������ �κ��� ��ġ ���� �޾� ���� �Լ�
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
@brief Korean: �Ÿ����� ���� ������. 
poloar coordinate������ �Ÿ� ���� ����, 
cartesian coordinate������ ���� �����Ѵ�.
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
		{	// ��������ĳ���� ��ȿ�Ÿ��� ����� ó�� ����.

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
@brief Korean: �Ÿ����� ���� ������. 
poloar coordinate������ �Ÿ� ���� ����, 
cartesian coordinate������ ���� �����Ѵ�.
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
		{	// ��������ĳ���� ��ȿ�Ÿ��� ����� ó�� ����.

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
@brief Korean: �Ÿ����� ���� ������. 
poloar coordinate������ �Ÿ� ���� ����, 
cartesian coordinate������ ���� �����Ѵ�.
@brief English : 
*/
void KuScanMatchingLocalizerPr::copyRangeDatafromMapT(int_1DArray  nData)
{

	KuCartesianCoordinate2D CartesianCoordiData;
	KuCartesianCoordinate2D PreCartesianCoordiData;


	for(int i=0; i<LASER_SCAN_IDX; i++){

		if (nData[i]< m_dMaxSensorRangeMM &&
			nData[i]>m_dMinSensorRangeMM)
		{	// ��������ĳ���� ��ȿ�Ÿ��� ����� ó�� ����.

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
		{	// ��������ĳ���� ��ȿ�Ÿ��� ����� ó�� ����.

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
@brief Korean: ������ �κ��� ���� ������ �������� ������ ������ �����͸� �޾ƿ´�
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
@brief Korean: ������ �κ��� ���� ������ �������� ������ ������ �����͸� �޾ƿ´�
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
@brief Korean:ICP �˰����� �̿��Ͽ� �κ��� ��ġ�� �����ϴ� �Լ�
@brief English: 
*/
double  KuScanMatchingLocalizerPr::estimateRobotPosP(int_1DArray nData, KuPose DeltaEncoderData,KuPose RobotPos,KuMap* pMap,bool* bMapping)
{
	m_RobotPos.setX(RobotPos.getX());
	m_RobotPos.setY(RobotPos.getY());
	m_RobotPos.setThetaDeg(RobotPos.getThetaDeg());

	(*bMapping)=m_bMapping;
	computeAccumulatedDeltaMovementForICP(DeltaEncoderData); //ICP������ ���� ���ڴ����� ���� delta x,y,theta. ��ȭ���� �����Ѵ�.

	m_bICPExecution = isAccDeltaMovementForICPOver(m_dDistanceForICPExecution); //ICP�� ���������� ���ϴ� �Լ�.

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
@brief Korean:ICP �˰����� �̿��Ͽ� �κ��� ��ġ�� �����ϴ� �Լ�
@brief English: 
*/
double  KuScanMatchingLocalizerPr::estimateRobotPos(int_1DArray nData, KuPose DeltaEncoderData,KuPose RobotPos,KuMap* pMap)
{
	//	m_RobotPos=RobotPos;
	estimateRobotPosByDeadReckoning(DeltaEncoderData);

	computeAccumulatedDeltaMovementForICP(DeltaEncoderData); //ICP������ ���� ���ڴ����� ���� delta x,y,theta. ��ȭ���� �����Ѵ�.

	m_bICPExecution = isAccDeltaMovementForICPOver(m_dDistanceForICPExecution); //ICP�� ���������� ���ϴ� �Լ�.

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
@brief Korean: ���� �Ÿ�����(t-1)�� ���� �Ÿ�����(t)�� ���Ͽ� ��ġ�� ��ȭ���� ���ϴ� �Լ�
@brief English: 
*/
KuPose KuScanMatchingLocalizerPr::computeDeltaPoseforNavi( int_1DArray nPreLaserData, int_1DArray nNewLaserData,KuPose DeltaEncoderData)
{
	double PreData[362], NewData[362]; //--> 181*2 181�� ������ ������ ����, 2�� x,y
	int NoNewData, NoPreData;

	copyRangeData(nNewLaserData);
	copyRangeDatafromMap(nPreLaserData);
	m_DeltaPosForICP.init();
	// ���Ŀ� ���缭 ������ �Է�
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
@brief Korean: ��� 0, sigma�� ����þ� ������ ������ ���� ����ϴ� �Լ�
@brief English: Function returning values according to Gaussian distribution with sigma and x
*/
double KuScanMatchingLocalizerPr::GetGaussianValue(double sigma, double x)
{
	if (sigma==0) return 0.1;

	return 1/sqrt(2*M_PI*sigma*sigma)*exp(-x*x/2.0/sigma/sigma);
}

/**
@brief Korean: ���� �Ÿ�����(t-1)�� ���� �Ÿ�����(t)�� ���Ͽ� ��ġ�� ��ȭ���� ���ϴ� �Լ�
@brief English: 
*/
KuPose KuScanMatchingLocalizerPr::computeDeltaPose( int_1DArray nPreLaserData, int_1DArray nNewLaserData)
{
	double PreData[362], NewData[362]; //--> 181*2 181�� ������ ������ ����, 2�� x,y
	int NoNewData, NoPreData;

	copyRangeData(nNewLaserData);
	copyRangeDatafromMap(nPreLaserData);

	// ���Ŀ� ���缭 ������ �Է�
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
@brief Korean: �Ÿ������� �̿��Ͽ� �κ��� ��ġ�� �����ϴ� �Լ�
@brief English: 
*/
double KuScanMatchingLocalizerPr::setDeltaPose(KuPose DeltaPos)
{
	bool bReturn = false;
	int NoNewData, NoPreData;
	double PreData[362], NewData[362]; //--> 181*2 181�� ������ ������ ����, 2�� x,y
	double Tmatrix[3], Rmatrix[3][3];
	double RMSError=-1.0;

	m_DeltaPos = DeltaPos; // ���ڴ����� ���� delta x,y,theta. ��ȭ��. ���밪 �ƴ�.

	// 	bool bICPExecution = isAccDeltaMovementForICPOver(m_dDistanceForICPExecution); //ICP�� ���������� ���ϴ� �Լ�.
	// 	
	// 	if (bICPExecution) {
	// -------------------------------------------------------- ICP ---------------------------------------------------------------- //			


	// ���Ŀ� ���缭 ������ �Է�
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


	// ICP ����
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
	// 	else{ //ICP�� �������� �ʴ� ��Ȳ������ ���ڴ� ������ ������ �κ��� ��ġ�� ����Ѵ�.
	// 		calculateRobotPosByEncoderData();
	// 	}
	// 		
	//	computeAccumulatedDeltaMovementForICP(DeltaPos); //ICP������ ���� ���ڴ����� ���� delta x,y,theta. ��ȭ���� �����Ѵ�.


	return RMSError;
}
/**
@brief Korean: ICP�� ������ ���(matrix)�� �̿��Ͽ� �κ��� ��ġ�� �����ϴ� �Լ�
@brief English: 
*/
void KuScanMatchingLocalizerPr::computeRobotPoseByICPFor2D(double dX, double dY, double dTheta)
{
	double s1=0.,c1=0.;
	//	m_RobotPos = m_LastRobotPosForICP;

	// ------------------------- movement by encoder data ------------------------- //
	// yaw ��꿡 �ݺ��Ǵ� ����,�ڽ��ΰ� �̸� ���
	// 	s1 = sin(m_RobotPos.getThetaRad() );	// yaw
	// 	c1 = cos(m_RobotPos.getThetaRad());
	// 
	// 
	// 	// localizer�� �����Ͽ� �����ϴ� ��ġ
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
	// �ݺ��Ǵ� ����,�ڽ��ΰ� �̸� ���
	s1 = sin(m_RobotPos.getThetaRad());	// yaw
	c1 = cos(m_RobotPos.getThetaRad());


	// localizer�� �����Ͽ� �����ϴ� ��ġ
	double dXm = (c1*dX + -s1*dY);
	double dYm = (s1*dX + c1*dY);

	m_RobotPos.setXm( m_RobotPos.getXm() + dXm);
	m_RobotPos.setYm( m_RobotPos.getYm() + dYm);
	m_RobotPos.setThetaRad( m_RobotPos.getThetaRad() + dTheta);

	m_estimatedRobotPos = m_RobotPos;
}


/**
@brief Korean: Encoder �������� �̿��� �κ���ġ ��� �Լ�
@brief English: 
*/
void KuScanMatchingLocalizerPr::calculateRobotPosByEncoderData()
{
	double s1=0.,c1=0.;


	// roll-pitch-yaw ��꿡 �ݺ��Ǵ� ����,�ڽ��ΰ� �̸� ���
	s1 = sin(m_RobotPos.getThetaRad());	// yaw
	c1 = cos(m_RobotPos.getThetaRad() );


	// localizer�� �����Ͽ� �����ϴ� ��ġ
	double dXm = (cos(m_RobotPos.getThetaRad() ) * m_DeltaPos.getXm() + -s1 * m_DeltaPos.getYm() );
	double dYm = (s1 * m_DeltaPos.getXm() + c1*m_DeltaPos.getYm() );
	m_RobotPos.setXm( m_RobotPos.getXm() + dXm );
	m_RobotPos.setYm( m_RobotPos.getYm() + dYm );
	m_RobotPos.setThetaRad( m_RobotPos.getThetaRad() + m_DeltaPos.getThetaRad() );
}

/**
@brief Korean: ICP������ ���� ���ڴ����� ���� delta x,y,theta. ��ȭ���� �����ϴ� �Լ�.
@brief English: 
*/
void KuScanMatchingLocalizerPr::computeAccumulatedDeltaMovementForICP(KuPose DeltaPos)
{
	m_dAccumulatedDeltaMovementForICP += fabs(DeltaPos.getXm()) + fabs(DeltaPos.getYm()) + fabs(DeltaPos.getThetaRad())*20.0/m_math.getPI();	// ������ 180�� �̵��� 20m �̵��� ������ ���

	double dXm = DeltaPos.getXm() * cos(m_DeltaPosForICP.getThetaRad() ) - DeltaPos.getYm() * sin(m_DeltaPosForICP.getThetaRad());
	double dYm = DeltaPos.getXm() *sin(m_DeltaPosForICP.getThetaRad()) + DeltaPos.getYm() * cos(m_DeltaPosForICP.getThetaRad());

	m_DeltaPosForICP.setXm( m_DeltaPosForICP.getXm() + dXm );
	m_DeltaPosForICP.setYm( m_DeltaPosForICP.getYm() + dYm );
	m_DeltaPosForICP.setThetaRad( m_DeltaPosForICP.getThetaRad() + DeltaPos.getThetaRad() );


}

/**
@brief Korean: �κ��� �����Ÿ��̻� ���������� ���θ� �˻��ϴ� �Լ�			
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

	m_dKXGain = dKXGain;   //�κ��� �������� ������ �غ��ϴ� ����. ���� �κ��� ��� �ʹ� ũ�� �ⷷ�̰�, �ʹ� ������ ���ϴ� �ӵ����� ������ ������.
	m_dKYGain = dKYGain*2.0;   //�κ��� ������� ������ �غ��ϴ� ����. ������ ũ���� ������, ũ�� �Ҿ����ϰ�, ������ ������� ������ ���� ����.
	m_dKThetaGain = dKThetaGain/2.0;  //�������� ���� �κ��� ����(heading)�� �غ��ϴ� ����. ũ�� �ſ� �ⷷ�̰�(Ư�� �ʱ⿡ ���ڸ����� �������� ���� �� ��), ������ �� ������ ����.
	m_dDesiredVel= dDesiredVel;
	m_dRatioofRadius =dWheelbase*M_PI/180;

}
/**
@brief Korean: ��ǿ� ���� ��Ȯ�Ǽ� ������ �޾ƿ´�
@brief English: set uncertainty of wheel motion
of range sensor
*/
void KuScanMatchingLocalizerPr::setDeviation(double dDeviationforTrans, double dDeviationforRotate,double dDeviationforTransRotate )
{
	m_pICP.setDeviation( dDeviationforTrans,  dDeviationforRotate, dDeviationforTransRotate );
}
