#include "stdafx.h"
#include "KuMath.h"


KuMath::KuMath()
{
	srand((unsigned)time(NULL)); //���� ���带 �׻� �ٲ��ֱ� ���ؼ� �����ؾ��Ѵ�.
}

KuMath::~KuMath()
{

}
double KuMath::AI2MM(double dVal)
{ 
	return dVal*GIRD_RESOLUTION*CM2MM(1); 
}

double KuMath::MM2AI(double dVal)
{ 
	return dVal/100; 
}

double KuMath::MM2CM(double dVal)
{ 
	return dVal/10.; 
}
// double KuMath::M2CM(double dVal)
// { 
// 	return dVal*100; 
// }

double KuMath::CM2MM(double dVal)
{ 
	return dVal*10; 
}

double KuMath::getPI()
{
	return 3.141592;
}

double KuMath::getRad(double dDeg )
{
	return dDeg*3.141592/180.;
}
double KuMath::getDeg(double dRad )
{
	return dRad*180./3.141592;
}

KuPose KuMath::getTransformedPose(double dDist, KuPose SensorConfiguration, KuPose RobotPose)
{
	KuPose TransformedPos;

	double dGlobalX = cos(RobotPose.getThetaRad())*(dDist*cos(SensorConfiguration.getThetaRad())+SensorConfiguration.getX())
		- sin(RobotPose.getThetaRad())*(dDist*sin(SensorConfiguration.getThetaRad())+SensorConfiguration.getY())
		+ RobotPose.getX();
	double dGrobalY = sin(RobotPose.getThetaRad())*(dDist*cos(SensorConfiguration.getThetaRad())+SensorConfiguration.getX())
		+ cos(RobotPose.getThetaRad())*(dDist*sin(SensorConfiguration.getThetaRad())+SensorConfiguration.getY())
		+ RobotPose.getY();

	TransformedPos.setX(dGlobalX);
	TransformedPos.setY(dGrobalY);

	return TransformedPos;
}
double KuMath::changeAngle(double ThetaRad)
{
	if (ThetaRad>M_PI)			ThetaRad += -2*M_PI;
	else if (ThetaRad<-M_PI)		ThetaRad +=  2*M_PI;

	return ThetaRad;
}


void KuMath::transformRangeDataToLocalCoordinate(int* nInputLaserData, double dTiltDeg,RangeData* LocalCoordinateData)
{
	// ��������ĳ�ʿ��� ���� �Ÿ������Ϳ� ���� offset�� ����Ͽ� 
	// ���������δ� �κ� �߽����κ��� ���� x,y,z�� �����ش�.
	// �� �κп��� ���� offset�� ��Ȯ�� ���Ǿ�� �Ѵ�.
	// visio ȭ�� ����. LaserOffset.vsd

// 	const int LASER_MAX_DIST =CSystemParameter::getInstance()->getUpperLaserMaxDist();// CSensorConfiguration::URG_04LX_LASER_MAX_DIST;
// 	const int LASER_MIN_DIST = CSystemParameter::getInstance()->getUpperLaserMinDist(); //unit mm
// 	int nScanIdx = 181;//CSensorConfiguration::URG_04LX_LASER_SCAN_IDX;
// 	double dX_LASER_OFFSET = CSystemParameter::getInstance()->getUpperLaserXOffset();//CSensorConfiguration::URG_04LX_LASER_X_LASER_OFFSET;
// 	double dZ_LASER_OFFSET = CSystemParameter::getInstance()->getUpperLaserHeight();//CSensorConfiguration::URG_04LX_LASER_Z_LASER_OFFSET;
// 	double dX_TILT_OFFSET = CSystemParameter::getInstance()->getTiltMotorXOffset();//CSensorConfiguration::X_TiltOffset;
// 	double dZ_TILT_OFFSET =CSystemParameter::getInstance()->getTiltMotorHeight();//CSensorConfiguration::Z_TiltOffset;
// 	double dTiltRad = dTiltDeg*-1*D2R;
// 
// 
// 	double dX,dY,dZ;
// 
// 	for (int i=0; i<nScanIdx; i++){   
// 		if (nInputLaserData[i] > (double) LASER_MAX_DIST || 
// 			nInputLaserData[i] < LASER_MIN_DIST
// 			){ // ��������ĳ���� ��ȿ�Ÿ��� �����.... ���� �ľ� ��..
// 
// 				LocalCoordinateData[i].x = 0;
// 				LocalCoordinateData[i].y = 0;
// 				LocalCoordinateData[i].z = 0;
// 				continue;
// 		}
// 		dX = nInputLaserData[i]*cos( (i-90)*D2R );
// 		dY = nInputLaserData[i]*sin( (i-90)*D2R );
// 		dZ = 0;
// 
// 		// �� �濡 ����ϸ�....
// 		LocalCoordinateData[i].x = (dX + dX_LASER_OFFSET)*cos(dTiltRad) + (dZ + dZ_LASER_OFFSET)*sin(dTiltRad) + dX_TILT_OFFSET;
// 		LocalCoordinateData[i].y = dY;
// 		LocalCoordinateData[i].z = (dX + dX_LASER_OFFSET)*sin(-dTiltRad) + (dZ + dZ_LASER_OFFSET)*cos(dTiltRad) + dZ_TILT_OFFSET;   
// 
// 	}
}

/**
 @brief Korean: 2*sigma ~ 2*sigma ���� ������ ������ ���� �����ϴ� �Լ�.
 @brief English: return random value from -2*sigma to 2*sigma
 */
double KuMath::calcRandomValue(double dSig)
{
	return dSig*2.0*( (double)rand()/(double)RAND_MAX - 0.5);
}


/**
@brief Korean: ���� Ȯ���е� �Լ��� ���ϴ� �Լ�. 
			   ��� 0, sigma�� ����þ� ������ ������ �� ���.
@brief English: 
*/
double KuMath::calcNormalProbabilityDensity(double dX, double dSig)
{
	if (dSig==0) return 0;

	return 1./sqrt(2*M_PI*dSig*dSig) * exp(-dX*dX/2.0/dSig/dSig);
}
/**
@brief Korean:���ð� ������ ���� ������ ���� Ȯ���е� �Լ��� ���ϴ� �Լ�. 
			  ��� 0, sigma�� ����þ� ������ ������ �� ���.
@brief English: 
*/
double KuMath::calcSimpleNormalProbabilityDensity(double dX, double dSig)
{
	if (dSig==0) return 0;

	return exp(-dX*dX/2.0/dSig/dSig);
}

/**
@brief Korean: �Է¹��� �� �������� �Ÿ��� ����ϴ� �Լ�. ���ϰ��� mm
@brief English: 
*/
double KuMath::calcDistBetweenPoses(KuPose Pos1, KuPose Pos2)
{
	return sqrt( pow(( Pos1.getX() - Pos2.getX()),2) +  pow(( Pos1.getY() - Pos2.getY()),2) );
}

/**
@brief Korean: �Է¹��� �� �������� �Ÿ��� ����ϴ� �Լ�. ���ϰ��� M
@brief English: 
*/
double KuMath::calcDistBetweenPosesM(KuPose Pos1, KuPose Pos2)
{
	return sqrt( pow(( Pos1.getXm() - Pos2.getXm()),2) +  pow(( Pos1.getYm() - Pos2.getYm()),2) );
}


/**
@brief Korean: �Է¹��� �� �������� �Ÿ��� ����ϴ� �Լ�. �Է°� mm, ���ϰ��� mm
@brief English: 
*/
double KuMath::calcDistBetweenPoses(double dPosX1mm, double dPosX2mm, double dPosY1mm, double dPosY2mm)
{
	return sqrt( pow(( dPosX1mm - dPosX2mm), 2) +  pow(( dPosY1mm - dPosY2mm ), 2) );
}


/**
@brief Korean: �Ÿ������� ���� �Է¹��� ������(polar coodinate ���� �Ÿ�(mm), ����)�� 
			   Cartesian coordiate����(x,y)�� �ٲ��ִ� �Լ�. input--> �Ÿ�, ����, ���� offset
			   input MM�����Ÿ�, ����(deg)
			   output: KuCartesianCoordinate2D type (data class)
@brief English: 
*/
KuCartesianCoordinate2D KuMath::transfromPolar2CartesianMM(double dDistMM, int nThetaDeg, double dSensorOffsetMM)
{
	KuCartesianCoordinate2D CartesianCoordinate;

	double dXmm = dDistMM * cos( (double)nThetaDeg * D2R ) + dSensorOffsetMM;
	double dYmm = dDistMM * sin( (double)nThetaDeg * D2R );

	CartesianCoordinate.setXmm( dXmm );
	CartesianCoordinate.setYmm( dYmm );
	
	return CartesianCoordinate;
}

/**
@brief Korean: �Ÿ������� ���� �Է¹��� ������(polar coodinate ���� �Ÿ�(m), ����)�� 
			   Cartesian coordiate����(x,y)�� �ٲ��ִ� �Լ�. input--> �Ÿ�, ����, ���� offset
			   input M�����Ÿ�, ����(deg)
			   output: KuCartesianCoordinate2D type (data class)
@brief English: 
*/
KuCartesianCoordinate2D KuMath::transfromPolar2CartesianM(double dDistM, int nThetaDeg, double dSensorOffsetM)
{
	KuCartesianCoordinate2D CartesianCoordinate;

	double dXm = dDistM * cos( (double)nThetaDeg * D2R ) + dSensorOffsetM;
	double dYm = dDistM * sin( (double)nThetaDeg * D2R );

	CartesianCoordinate.setXm( dXm );
	CartesianCoordinate.setYm( dYm );
	
	return CartesianCoordinate;
}
