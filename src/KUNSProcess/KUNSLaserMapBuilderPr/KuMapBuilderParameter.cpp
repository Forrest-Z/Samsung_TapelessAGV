#include "stdafx.h"
#include "KuMapBuilderParameter.h"
/**
@brief Korean: 생성자 함수
@brief English:
*/
KuMapBuilderParameter::KuMapBuilderParameter()
{
	m_nLaserScanIdx = 181;
	m_nMinDistofSensorData = 30;//mm
	m_nMaxDistofSensorData = 30000;//mm
	m_nMapSizeXm = 100;//mm
	m_nMapSizeYm = 100;//mm
	m_dLaserOffset = 230;//mm
	m_nCellSize = 100;//mm
	m_dThicknessofWall = 30;//mm
	m_dRadiusofRobot = 400;//mm
	m_bMapBuildingspeedflag=false;
	m_dSigma=50;
	m_dXOffset=230;
}
/**
@brief Korean: 소멸자 함수
@brief English:
*/
KuMapBuilderParameter::~KuMapBuilderParameter()
{
	
}
/**
@brief Korean: 로봇변위를 설정하는 함수
@brief English:
*/
void KuMapBuilderParameter::setDelRobotPos(KuPose DelRobotPos)
{
	m_DelRobotPos = DelRobotPos;
}
/**
@brief Korean: 로봇변위를 넘겨주는 함수
@brief English:
*/
KuPose KuMapBuilderParameter::getDelRobotPos()
{
	return m_DelRobotPos;
}
/**
@brief Korean: 로봇위치를 설정하는 함수
@brief English:
*/
void KuMapBuilderParameter::setRobotPos(KuPose RobotPos)
{
	m_RobotPos = RobotPos;
}
/**
@brief Korean: 로봇위치를 넘겨주는 함수
@brief English:
*/
KuPose KuMapBuilderParameter::getRobotPos()
{
	return m_RobotPos;
}
/**
@brief Korean: 거리 정보를 설정하는 함수
@brief English:
*/
void KuMapBuilderParameter::setLaserData(int_1DArray nLaserData)
{
	m_nLaserData = nLaserData;	
}
/**
@brief Korean: 거리정보를 내보내는 함수
@brief English:
*/
int_1DArray KuMapBuilderParameter::getLaserData()
{
	return m_nLaserData;
}
/**
@brief Korean: Index를 설정하는 함수
@brief English:
*/
void KuMapBuilderParameter::setLaserScanIdx(int nLaserScanIdx)
{
	m_nLaserScanIdx = nLaserScanIdx;
}
/**
@brief Korean: Index를 내보내는 함수
@brief English:
*/
int KuMapBuilderParameter::getLaserScanIdx()
{
	return m_nLaserScanIdx;
}
/**
@brief Korean: Sensor 장착 X Offset을 설정하는 함수
@brief English:
*/
void KuMapBuilderParameter::setLaserXOffset(double dOffset)
{
	m_dLaserOffset = dOffset;
}
/**
@brief Korean: Sensor 장착 X Offset을 내보내는 함수
@brief English:
*/
double KuMapBuilderParameter::getLaserXOffset()
{
	return m_dLaserOffset;
}
/**
@brief Korean: Sensor 측정가능 최소거리를 설정하는 함수
@brief English:
*/
void KuMapBuilderParameter::setMinDistofSensorData(int nDist)
{
	m_nMinDistofSensorData = nDist;	
}
/**
@brief Korean: Sensor 측정가능 최대거리를 설정하는 함수
@brief English:
*/
void KuMapBuilderParameter::setMaxDistofSensorData(int nDist)
{
	m_nMaxDistofSensorData = nDist;
}
/**
@brief Korean: Sensor 측정가능 최소거리를 내보내는 함수
@brief English:
*/
int KuMapBuilderParameter::getMinDistofSensorData()
{
	return m_nMinDistofSensorData;
}
/**
@brief Korean: Sensor 측정가능 최대거리를 내보내는 함수
@brief English:
*/
int KuMapBuilderParameter::getMaxDistofSensorData()
{
	return m_nMaxDistofSensorData;	
}
/**
@brief Korean: 지도의 크기를 설정하는 함수
@brief English:
*/
void KuMapBuilderParameter::setMapSizeXmYm(int nMapSizeXm, int nMapSizeYm)
{
	m_nMapSizeXm = nMapSizeXm;
	m_nMapSizeYm = nMapSizeYm;
}
/**
@brief Korean: 지도의 크기를 내보내는 함수
@brief English:
*/
void KuMapBuilderParameter::getMapSizeXmYm(int* nMapSizeXm, int* nMapSizeYm)
{
	*nMapSizeXm = m_nMapSizeXm;
	*nMapSizeYm = m_nMapSizeYm;
}
/**
@brief Korean: 지도작성의 속도를 향상시키는 flag를 설정하는 함수
@brief English:
*/
void KuMapBuilderParameter::setLaserUpdateSpeedflag(bool bMapBuildingspeedflag)
{
	m_bMapBuildingspeedflag = bMapBuildingspeedflag;	
}
/**
@brief Korean: 지도작성의 속도를 향상시키는 flag를 내보내는 함수
@brief English:
*/
bool KuMapBuilderParameter::getLaserUpdateSpeedflag()
{
	 return m_bMapBuildingspeedflag;	
}
/**
@brief Korean: 격자의 크기를 설정해주는 함수
@brief English:
*/
void KuMapBuilderParameter::setCellSize(int nCellSize)
{
	m_nCellSize = nCellSize;	
}
/**
@brief Korean: 격자의 크기를 내보내는 함수
@brief English:
*/
int KuMapBuilderParameter::getCellSize()
{
	return m_nCellSize;	
}
/**
@brief Korean: 벽의 두깨를 설정해주는 함수
@brief English:
*/
void KuMapBuilderParameter::setThicknessofWall(double dThicknessofWall)
{
	m_dThicknessofWall = dThicknessofWall;	
}
/**
@brief Korean: 벽의 두깨를 내보내는 함수
@brief English:
*/
double KuMapBuilderParameter::getThicknessofWall()
{
	return m_dThicknessofWall;	
}
/**
@brief Korean: 로봇의 반지름을 설정해주는 함수
@brief English:
*/
void KuMapBuilderParameter::setRadiusofRobot(double dRadiusofRobot)
{
	m_dRadiusofRobot = dRadiusofRobot;	
}
/**
@brief Korean: 로봇의 반지름을 내보내는 함수
@brief English:
*/
double KuMapBuilderParameter::getRadiusofRobot()
{
	return m_dRadiusofRobot;	
}
/**
@brief Korean: 시그마를 설정해주는 함수
@brief English:
*/
void KuMapBuilderParameter::setSigma(double dSigma)
{
	m_dSigma = dSigma;	
}
/**
@brief Korean: 시그마를 내보내는 함수
@brief English:
*/
double KuMapBuilderParameter::getSigma()
{
	return m_dSigma;	
}

/**
@brief Korean: 시그마를 설정해주는 함수
@brief English:
*/
void KuMapBuilderParameter::setXOffset(double dXOffset)
{
	m_dXOffset = dXOffset;	
}
/**
@brief Korean: 시그마를 내보내는 함수
@brief English:
*/
double KuMapBuilderParameter::getXOffset()
{
	return m_dXOffset;	
}