#pragma once

class CANSPose
{
public:
	// Constructor and destructor
	CANSPose(void);
	~CANSPose(void);

	// Variables

	// Functions
	float getXm(void);
	float getYm(void);
	float getZm(void);
	float getXmm(void);
	float getYmm(void);
	float getZmm(void);
	float getThRad(void);
	float getThDeg(void);
	void setXm(float fX);
	void setYm(float fY);
	void setZm(float fZ);
	void setXmm(float fX);
	void setYmm(float fY);
	void setZmm(float fZ);
	void setThRad(float fTh);
	void setThDeg(float fTh);
	void set(float fX, float fY, float fTh);

	CANSPose& operator =(CANSPose& pose);

private:
	// Variables
	float m_fX; // m
	float m_fY; // m
	float m_fZ; // m
	float m_fTh; // rad

	// Functions

};

