/*______________________________________________________________________________________________
PROJECT : Intelligent Robotics Lab. Department of Mecanical Engineering Korea University                                    
(c) Copyright 2012 KUME Intelligent Robotics Lab.                                             
All rights reserved                                                                         
$Directed by Jae-Bok Song                                                                     
$Designed by Joong-Tae Park                                                                  
$Description : 가상으로 Wheelacutuatorinterface 기능을 수행하는 클래스
$Created on: 2012. 6. 12.                                                                          
$Author: Joong-Tae Park     
$e-mail: jtpark1114@gmail.com
______________________________________________________________________________________________*/


#ifndef KUNS_VIRTUAL_WHEEL_ACTUATOR_INTERFACE_H
#define KUNS_VIRTUAL_WHEEL_ACTUATOR_INTERFACE_H


#include <iostream>
#include "../../KUNSMath/KuMath.h"
#include "../../KUNSPose/KuPose.h"
#include "../../KUNSUtil/KUNSSingletone/KuSingletone.h"

using namespace std;
class KuVrWheelActuatorInterface : public KuSingletone <KuVrWheelActuatorInterface>
{

private:
	static const int RADIUS = 75;//mm //바퀴의 반경. 
	static const int ENCODER_RESOLUTION = 2048; 
	static const int GEAR_RATIO = 150;
	static const int BETWEEN_WHEEL = 500;
	static const int MAX_TRANSLATION_VELOCITY = 1000;//1000mm/s
	static const int MAX_ROTATION_VELOCITY = 30;//deg/s

	
private:
	
	//for encoder
	double m_dEncData[2];
	double m_dReferenceLeftWheelEncoderCount;
	double m_dReferenceRightWheelEncoderCount;
	double m_dLeftWheelEncoderCount;
	double m_dRightWheelEncoderCount;
	double m_dLeftWheelDistance;
	double m_dRightWheelDistance;
	double m_dAverageWheelDistance;
	double m_dDistance2RobotCenter;
	double m_dDeltaX;
	double m_dDeltaY;
	double m_dDeltaT; 
	double m_dReferenceX;
    double m_dReferenceY;
    double m_dReferenceT;
	//------------------------------------	
	
	//vel;
	double m_dRightWheelVel;
	double m_dLeftWheelVel;
	
	//for robotPos
	KuPose m_RobotPos, m_DelEncoderPos;

	double m_dOldOdometryPos[3];
	double m_dOldRefPos[3];
	double m_dDelOdoPos[3];

	// 엔코더에 기반하여 상대적인 이동량의 누적치를 계산하는 기능
	double m_dAccumulatedDeltaMovement;
	double m_dAccumulatedDeltaAngle;

	//노이즈를 저장하는 변수
	double m_dNoiseFactor; //노이즈 펙터

private:
	void InitVariable();
	void calcEncoderData();
	void computeAccumulatedDeltaMovement(KuPose EncoderDelPos);

public:
	KuPose getRobotPos();
	void setRobotPos(KuPose RobotPos);
	
	KuPose getDelEncoderData();
	bool isAccDeltaMovementOver(double dMovement, double dAngle);
	void moveTRVelocity(double dTVel, double dRotDegVel);
	void setNoiseFactor(int nNoiseFactor); //노이즈 값을 설정하는 함수. 1~10사이의 값
	void moveByVelocityXYT(int deltaX, int deltaY, int deltaTheta );
	
	
	KuVrWheelActuatorInterface();
	~KuVrWheelActuatorInterface();



};

#endif /*KUNS_VIRTUAL_WHEEL_ACTUATOR_INTERFACE_H*/
