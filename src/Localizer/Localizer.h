#ifndef LOCALIZER_H
#define LOCALIZER_H

#include <iostream>
#include <fstream>
#include <cmath>
#include <float.h>
#include <list>
//#include "Sample.h"
//#include <semaphore.h> //세마포어를 위해서
#include <errno.h>//세마포어를 위해서
#include "../KUNSPose/KuPose.h"



using namespace std;
class Localizer
{
public:
	static const int PARTICLEFILTER =0;
	static const int SCANMATCHING =1;

public:
		 
protected:
    
public:
	//	virtual void start(int nPeriod) = 0;
	//	virtual void terminate() = 0;
	virtual KuPose getRobotPos()=0;
 	virtual void setRobotPos(KuPose RobotPos)=0;
	virtual double getRobotPosX() = 0;
	virtual double getRobotPosY() = 0;
	virtual double getRobotPosDeg()= 0;	
	virtual void setRobotPosX(double dRobotPosX) = 0;
	virtual void setRobotPosY(double dRobotPosY) = 0;
	virtual void setRobotPosDeg(double dRobotPosThetaDeg) =0;
	//virtual void setMap(int nMapSizeX, int nMapSizeY, int** nMap)=0;



// 	virtual KuPose estimateRobotPosByDeadReckoning(KuPose EncoderDelPos)=0;
// 	virtual KuPose estimateRobotPos(int* nRangeData, KuPose EncoderDelPos)=0; //kinect sonar data 가지고 위치추정 할때..사용하는 함수
// 	virtual void setMap(int nMapSizeX, int nMapSizeY, int** nMap)=0; 
// 	virtual list<Sample> getParticleList()=0;
// 	virtual void spreadParticleNearRobot(KuPose RobotPos, double dRegionSize)=0;
// 	virtual void setSampleRegion(int minX, int maxX, int minY, int maxY)=0;
// 	virtual void resetSamples()=0;
//  virtual bool isAccDeltaMovementOver(double dMovement, double dAngle)=0;

};

#endif
