#ifndef CKUNS_MATH_H
#define CKUNS_MATH_H



#include <iostream>
#include <cmath>
#include <time.h>
#include "../KUNSPose/KuPose.h"



#define M_PI 3.141592
#define D2R 0.017453292
#define R2D 57.29579143

#define M2CM 100 //���ϱ� ����
#define M2MM 1000 //���ϱ� ����
#define MM2M 0.001 //���ϱ� ����

#define CM2GRID 0.1 // 10cm 1���� ���ϱ� ����
#define MM2GRID 0.01 
#define GRID2MM 100

#define SEC2MIN 60

struct RangeData
{
	double x;
	double y;
	double z;
};
using namespace std;

class KuCartesianCoordinate2D
{
	
private:
	double m_dXmm; //unit mm
	double m_dYmm; //unit mm
	double m_dXm; //unit mm
	double m_dYm; //unit mm

public:
	void setXmm(double dXmm){
		m_dXmm = dXmm; 
		m_dXm = m_dXmm * MM2M;
	}
	void setYmm(double dYmm){
		m_dYmm = dYmm; 
		m_dYm = m_dYmm * MM2M;
	}

	void setXm(double dXm){
		m_dXm = dXm; 
		m_dXmm = m_dXm * M2MM;
	}
	void setYm(double dYm){
		m_dYm = dYm; 
		m_dYmm = m_dYm * M2MM;
	}

	double getXmm(){ return m_dXmm;}
	double getXm(){ return m_dXm;}
	double getYmm(){ return m_dYmm;}
	double getYm(){ return m_dYm;}

};

class CartesianCoordinate3D :public KuCartesianCoordinate2D
{

public:
	double m_dZmm; //unit mm	
	double m_dZm; //unit m

};

class KuMath
{


	static const int GIRD_RESOLUTION = 10; // �Ѱ��ڰ� 10cm �� ǥ���ȴ�.

public:
	double AI2MM(double dVal);
	double MM2AI(double dVal);
	double CM2MM(double dVal);
	double MM2CM(double dVal);
	//double M2CM(double dVal);

	double getPI();
	double getRad(double dDeg );
	double getDeg(double dRad );
	KuPose getTransformedPose(double dDist, KuPose SensorConfiguration, KuPose RobotPose);
	double changeAngle(double ThetaRad);	
	void transformRangeDataToLocalCoordinate(int* nInputLaserData, double dTiltDeg,RangeData* LocalCoordinateData);


	
	double calcRandomValue(double dSig); //2*sigma ~ 2*sigma ���� ������ ������ ���� �����ϴ� �Լ�.
	double calcNormalProbabilityDensity(double dX, double dSig); //���� Ȯ���е� �Լ��� ���ϴ� �Լ�.
	double calcSimpleNormalProbabilityDensity(double dX, double dSig); //���ð� ������ ���� ������ ���� Ȯ���е� �Լ��� ���ϴ� �Լ�.
	double calcDistBetweenPoses(KuPose Pos1, KuPose Pos2); //�Է¹��� �� �������� �Ÿ��� ����ϴ� �Լ�. ���ϰ��� mm
	double calcDistBetweenPoses(double dPosX1mm, double dPosX2mm, double dPosY1mm, double dPosY2mm); //�Է¹��� �� �������� �Ÿ��� ����ϴ� �Լ�. �Է°� mm, ���ϰ��� mm
	double calcDistBetweenPosesM(KuPose Pos1, KuPose Pos2); //�Է¹��� �� �������� �Ÿ��� ����ϴ� �Լ�. ���ϰ��� M

	KuCartesianCoordinate2D transfromPolar2CartesianM(double dDistM, int nThetaDeg, double dSensorOffsetM);
	KuCartesianCoordinate2D transfromPolar2CartesianMM(double dDistMM, int nThetaDeg, double dSensorOffsetMM);


	KuMath();
	~KuMath();
};

#endif 