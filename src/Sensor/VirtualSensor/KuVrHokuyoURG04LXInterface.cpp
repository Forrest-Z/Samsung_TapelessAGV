#include "stdafx.h"
#include "KuVrHokuyoURG04LXInterface.h"

KuVrHokuyoUTM30LXInterface::KuVrHokuyoUTM30LXInterface()
{
	m_nLaserData181 = m_KuUtil.generateIntType1DArray(Sensor::URG04LX_DATA_NUM181,0);

	m_nMap=NULL;

}
KuVrHokuyoUTM30LXInterface::~KuVrHokuyoUTM30LXInterface()
{
	if(m_nMap!=NULL)
{
		for(int i = 0 ; i < m_nMapX ; i++){
			delete[] m_nMap[i];
			m_nMap[i] = NULL;

		}
		delete[] m_nMap;

	}
}

/**
 @brief Korean: 가상의 센서와 연결하는 함수
 @brief English: 
*/
bool KuVrHokuyoUTM30LXInterface::connect(int** nMap)
{
	m_CriticalSection.Lock();

	m_nMapX= KuDrawingInfo::getInstance()->getMapSizeX();
	m_nMapY=  KuDrawingInfo::getInstance()->getMapSizeY();

		if(m_nMap==NULL){
		//처음 m_nMap을 생성할때
		m_nMap = new int*[m_nMapX];
		if(m_nMap){
			for(int i = 0 ; i < m_nMapX ; i++){
				m_nMap[i] = new int[m_nMapY];
			}
		}
	}


	for(int i=0; i<m_nMapX; i++){
		for(int j=0; j<m_nMapY; j++){
			if(nMap[i][j]==KuMap::FIXED_CAD_AREA)
				m_nMap[i][j] = KuMap::OCCUPIED_AREA;
			else
				m_nMap[i][j] = nMap[i][j];
		}
	}

		m_CriticalSection.Unlock();
	return true;
}

/**
 @brief Korean: 가상의 181개 레이저 데이터를 넘겨주는 함수
 @brief English: 
*/
int_1DArray KuVrHokuyoUTM30LXInterface::getData181(KuPose RobotPos)
{
	if(m_nMap==NULL){
		//가상 레이저와 연결이 되어 있지 않은 상태
		return m_nLaserData181;
	}


	double dLaserOffSet =530;

	int_1DArray  nLaserData181= m_KuUtil.generateIntType1DArray(Sensor::URG04LX_DATA_NUM181,0);
	double dGradientX,dGradientY;
	double dComDist,f;
	double RayOfX,RayOfY;
	int nPointX=0, nPointY=0;
	int nX=0,nY=0;
	int nAngle=0;
	double dAngle=0;



	nPointX = (RobotPos.getX()+dLaserOffSet*cos(RobotPos.getThetaRad()))/100.; 
	nPointY = (RobotPos.getY()+dLaserOffSet*sin(RobotPos.getThetaRad()))/100.; 

	for (int i=0; i<Sensor::URG04LX_DATA_NUM181; i++){

		dAngle = (i+RobotPos.getThetaDeg()-90);
		
		if(dAngle > 180) dAngle -=360;
		else if(dAngle < -180) dAngle += 360;

		nX=nY=0;   	
		dGradientX = cos( dAngle*D2R );	// 현재 처리하는 레이저 점의 기울기를 구한다.
		dGradientY = sin( dAngle*D2R );
		dComDist=0;
		f=0.;

		m_nLaserData181[i] = MAX_LASER_DISTANCE;

		while(dComDist  <= MAX_LASER_DISTANCE / 100) { 

			RayOfX = f * dGradientX;
			RayOfY = f * dGradientY;
			dComDist = sqrt(RayOfX*RayOfX + RayOfY*RayOfY);
			f=f+0.5;

			nX = (int)(nPointX + RayOfX);
			nY = (int)(nPointY + RayOfY);

			if(nX<1||nY<1||nX>m_nMapX||nY>m_nMapY) continue;
			
		if(m_nMap[nX][nY] ==  KuMap::UNKNOWN_AREA||m_nMap[nX][nY] == KuMap::OCCUPIED_AREA ){
		//if(m_nMap[nX][nY] !=  KuMap::EMPTY_AREA ){
			nLaserData181[i] = (int)(dComDist * 100.);
				break;    
			}


		}
	
	}

	m_CriticalSection.Lock();
	for(int i=0; i<Sensor::URG04LX_DATA_NUM181; i++)
		m_nLaserData181[i] = nLaserData181[i];
	m_CriticalSection.Unlock();

	return m_nLaserData181;
}