#include "stdafx.h"
#include "ANSPose.h"

#define R2D 57.295791
#define D2R 0.0174532

CANSPose::CANSPose(void)
	: m_fX(0)
	, m_fY(0)
	, m_fZ(0)
	, m_fTh(0)
{
}


CANSPose::~CANSPose(void)
{
}


/**
 * @brief Return X value in meters.
 * @date 2014/05/12
 * @return float
 */
float CANSPose::getXm(void)
{
	return m_fX;
}


/**
 * @brief Return Y value in meters.
 * @date 2014/05/12
 * @return float
 */
float CANSPose::getYm(void)
{
	return m_fY;
}


/**
 * @brief Return Z value in meters.
 * @date 2014/05/12
 * @return float
 */
float CANSPose::getZm(void)
{
	return m_fZ;
}


/**
 * @brief Return X value in milimeters.
 * @date 2014/05/12
 * @return float
 */
float CANSPose::getXmm(void)
{
	return m_fX * 1000;
}


/**
 * @brief Return Y value in milimeters.
 * @date 2014/05/12
 * @return float
 */
float CANSPose::getYmm(void)
{
	return m_fY * 1000;
}


/**
 * @brief Return Z value in milimeters.
 * @date 2014/05/12
 * @return float
 */
float CANSPose::getZmm(void)
{
	return m_fZ * 1000;
}


/**
 * @brief Return theta value in radian.
 * @date 2014/05/12
 * @return float
 */
float CANSPose::getThRad(void)
{
	return m_fTh;
}


/**
 * @brief Return theta value in degrees.
 * @date 2014/05/12
 * @return float
 */
float CANSPose::getThDeg(void)
{
	return m_fTh * (float)R2D;
}


/**
 * @brief Set X value in meters.
 * @date 2014/05/12
 * @param fX
 * @return void
 */
void CANSPose::setXm(float fX)
{
	m_fX = fX;
}


/**
 * @brief Set Y value in meters.
 * @date 2014/05/12
 * @param fY
 * @return void
 */
void CANSPose::setYm(float fY)
{
	m_fY = fY;
}


/**
 * @brief Set Z value in meters.
 * @date 2014/05/12
 * @param fZ
 * @return void
 */
void CANSPose::setZm(float fZ)
{
	m_fZ = fZ;
}


/**
 * @brief Set X value in milimeters.
 * @date 2014/05/12
 * @param fX
 * @return void
 */
void CANSPose::setXmm(float fX)
{
	m_fX = fX / 1000;
}


/**
 * @brief Set Y value in milimeters.
 * @date 2014/05/12
 * @param fY
 * @return void
 */
void CANSPose::setYmm(float fY)
{
	m_fY = fY / 1000;
}


/**
 * @brief Set Z value in milimeters.
 * @date 2014/05/12
 * @param fZ
 * @return void
 */
void CANSPose::setZmm(float fZ)
{
	m_fZ = fZ / 1000;
}


/**
 * @brief Set theta value in radian.
 * @date 2014/05/12
 * @param fTh
 * @return void
 */
void CANSPose::setThRad(float fTh)
{
	m_fTh = fTh;
}


/**
 * @brief Set theta value in degrees.
 * @date 2014/05/12
 * @param fTh
 * @return void
 */
void CANSPose::setThDeg(float fTh)
{
	m_fTh = fTh * (float)D2R;
}

/**
 * @brief "=" operator
 * @date 2014/05/12
 * @param pose
 * @return CANSPose&
 */
CANSPose& CANSPose::operator =(CANSPose& pose)
{
	if(this != &pose)
	{
		m_fX = pose.getXm();
		m_fY = pose.getYm();
		m_fZ = pose.getZm();
		m_fTh = pose.getThRad();
	}

	return *this;
}


/**
 * @brief Set x, y, th values in mm.
 * @date 2014/11/16
 * @param fX
 * @param fY
 * @param fTh
 * @return void
 */
void CANSPose::set(float fX, float fY, float fTh)
{
	m_fX = fX;
	m_fY = fY;
	m_fTh = fTh;
}