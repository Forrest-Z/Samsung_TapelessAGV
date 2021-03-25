#include "stdafx.h"
#include "KuPose.h"

KuPose::KuPose()
{
	init();// 모든 변수 초기화	
}

KuPose::~KuPose()
{
	
}

/**
@brief Korean:  로봇 위치 및 관련 변수 초기화
@brief English: 
*/
void KuPose::init()
{ 
	m_dPixX = m_dPixY = 0; //영상 픽셀값
	m_dX = 0.; //unit mm
	m_dY = 0.; //unit mm
	m_dZ = 0.; //unit mm
	m_dThetaDeg = 0.; //unit degree
	m_dThetaRad = 0.; //unit radian
	m_dDist=0.0;
	m_nID=-1;
	m_dPro=0.0;

}

/**
@brief Korean: 영상 픽셀의 X좌표를 다른 클래스에서 가져감
@brief English: 
*/
double KuPose::getPixX()
{
	return m_dPixX;
}

/**
@brief Korean: 영상 픽셀의 Y좌표를 다른 클래스에서 가져감
@brief English: 
*/
double KuPose::getPixY()
{
	return m_dPixY;
}

/**
@brief Korean: 다른 클래스에서 영상 픽셀의 X좌표를 가져온 후 저장함
@brief English : 
*/
void KuPose::setPixX(double dPixX)
{
	m_dPixX = dPixX;
}

/**
@brief Korean:  다른 클래스에서 영상 픽셀의 Y좌표를 가져온 후 저장함
@brief English : 
*/
void KuPose::setPixY(double dPixY)
{
	m_dPixY = dPixY;
}

/**
@brief Korean: 절대좌표상의 X(mm)좌표를 다른 클래스에서 가져감
@brief English : 
*/
double KuPose::getX()
{
	return m_dX;
}

/**
@brief Korean: 절대좌표상의 Y(mm)좌표를 다른 클래스에서 가져감
@brief English : 
*/
double KuPose::getY()
{
	return m_dY;
}		

/**
@brief Korean: 절대좌표상의 Z(mm)좌표를 다른 클래스에서 가져감
@brief English : 
*/
double KuPose::getZ()
{
	return m_dZ;
}

/**
@brief Korean: 절대좌표상의 (degree)각도를 다른 클래스에서 가져감
@brief English : 
*/
double KuPose::getThetaDeg() 
{ 
	return m_dThetaDeg; 
}

/**
@brief Korean: 절대좌표상의 (radian)각도를 다른 클래스에서 가져감
@brief English : 
*/
double KuPose::getThetaRad() 
{ 
	return m_dThetaRad;
}

/**
@brief Korean: 다른 클래스에서 절대좌표상의 X(mm)좌표를 가져온 후 저장함 
@brief English : 
*/
void KuPose::setX(double dX)
{ 
	m_dX = dX; 
}

/**
@brief Korean: 다른 클래스에서 절대좌표상의 Y(mm)좌표를 가져온 후 저장함 
@brief English : 
*/
void KuPose::setY(double dY)
{ 
	m_dY = dY; 
}

/**
@brief Korean: 다른 클래스에서 절대좌표상의 Z(mm)좌표를 가져온 후 저장함 
@brief English : 
*/
void KuPose::setZ(double dZ)
{ 
	m_dZ = dZ; 
}

/**
@brief Korean: 다른 클래스에서 절대좌표상의 X(m)좌표를 가져온 후 저장함 
@brief English : 
*/
void KuPose::setXm(double dXm)
{
	m_dX = dXm*1000;
}

/**
@brief Korean: 다른 클래스에서 절대좌표상의 Y(m)좌표를 가져온 후 저장함 
@brief English : 
*/
void KuPose::setYm(double dYm)
{
	m_dY = dYm*1000;
}

/**
@brief Korean: 다른 클래스에서 절대좌표상의 Z(m)좌표를 가져온 후 저장함 
@brief English : 
*/
void KuPose::setZm(double dZm)
{
	m_dZ = dZm*1000;
}

/**
@brief Korean: 절대좌표상의 X(m)좌표를 다른 클래스에서 가져감
@brief English : 
*/
double KuPose::getXm()
{
	return m_dX/1000.;
}

/**
@brief Korean: 절대좌표상의 Y(m)좌표를 다른 클래스에서 가져감
@brief English : 
*/
double KuPose::getYm()
{
	return m_dY/1000.;
}

/**
@brief Korean: 절대좌표상의 Z(m)좌표를 다른 클래스에서 가져감
@brief English : 
*/
double KuPose::getZm()
{
	return m_dZ/1000.;
}

/**
@brief Korean: 다른 클래스에서 절대좌표상의 (degree)각도를 가져온 후 저장함 
@brief English : 
*/
void KuPose::setThetaDeg(double dThetaDeg)
{ 
	m_dThetaDeg = dThetaDeg;  
	if(m_dThetaDeg > 180) m_dThetaDeg -=360;
	if(m_dThetaDeg < -180) m_dThetaDeg +=360;
	m_dThetaRad = m_dThetaDeg*3.141592/180;
} 

/**
@brief Korean: 다른 클래스에서 절대좌표상의 (radian)각도를 가져온 후 저장함 
@brief English : 
*/
void KuPose::setThetaRad(double dThetaRad)
{ 
	m_dThetaRad = dThetaRad;  
	if(m_dThetaRad > 3.141592) m_dThetaRad -= 2*3.141592;
	if(m_dThetaRad < -3.141592) m_dThetaRad += 2*3.141592;

	m_dThetaDeg = m_dThetaRad*180/3.141592;
}

/**
@brief Korean: 절대좌표상의 거리를 다른 클래스에서 가져감
@brief English : 
*/
double KuPose::getDist()
{ 
	return m_dDist;
}

/**
@brief Korean: 지도에서 저장되어 있는 랜드마크들의 ID값을 다른 클래스에서 가져감
@brief English : 
*/
int KuPose::getID()
{
	return m_nID;
}

/**
@brief Korean: 다른 클래스에서 절대좌표상의 거리를 가져온 후 저장함 
@brief English : 
*/
void KuPose::setDist(double dDist)
{
	m_dDist = dDist;	
}

/**
@brief Korean: 다른 클래스에서 지도에서 저장되어 있는 랜드마크들의 ID값을 가져온 후 저장함 
@brief English : 
*/
void KuPose::setID(int nID)
{
	m_nID = nID;
}

/**
@brief Korean: 다른 클래스에서 확률 값을 가져온 후 저장함 
@brief English : 
*/
void KuPose::setPro(double dPro)
{
	m_dPro = dPro;
}

/**
@brief Korean:  저장된 확률 값을  다른 클래스에서 가져감
@brief English : 
*/
double KuPose::getPro()
{
	return m_dPro;
}
