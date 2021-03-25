/*______________________________________________________________________________________________
PROJECT : Intelligent Robotics Lab. Department of Mecanical Engineering Korea University                                    
(c) Copyright 2007 KUME Intelligent Robotics Lab.                                             
All rights reserved                                                                         
$Directed by Jae-Bok Song                                                                     
$Designed by Joong-Tae Park                                                                  
$Description : samsung agv의 속도 및 엔코도등을 제어하는 기능을 제공하는 클래스
$Data: 2007/09                                                                           
$Author: Joong-Tae Park                                                                      
______________________________________________________________________________________________*/

#ifndef SAMSUNG_AGV__WHEEL_ACTUATOR_INTERFACE_H
#define SAMSUNG_AGV__WHEEL_ACTUATOR_INTERFACE_H
#include <vector>
#include <limits>
#include "../../KUNSUtil/KUNSSerialComm/KuSerialComm.h"
#include "../../KUNSMath/KuMath.h"
#include "../../KUNSPose/KuPose.h"
#include "../../KUNSUtil/KUNSSingletone/KuSingletone.h"
#include "../../KUNSUtil/KUNSCriticalSection/KuCriticalSection.h"
#include "../../KUNSUtil/KUNSSmartPointer/KuSmartPointer.h"




using namespace std;
class SSAGVWheelActuatorInterface : public KuSingletone <SSAGVWheelActuatorInterface>
{
	static const int GET_ENCODER_PROTOCOL_SIZE = 3;
	static const int RECV_ENCODER_PROTOCOL_SIZE = 4;
	static const int LEFT_VEL_SIGN = 2;
	static const int RIGHT_VEL_SING = 7;
	static const int VEL_PROTOCOL_SIZE = 13;

private:
	KuSerialComm m_KuSerialComm;
	bool m_bISConnected;
	char m_getEncoderProtocol[GET_ENCODER_PROTOCOL_SIZE];
	unsigned char m_RecvEncoderProtocol[MAXBLOCK];

	int m_nLeftEncoderData;
	int m_nRightEncoderData;
	int m_nDistBetweenWheel; //로봇 양바퀴사이의 거리 unit--> mm
	int m_nEncoderResolution; //로봇의 엔코도 resolution.
	int m_nGearRatio; // 로봇의 기어비. 
	double m_dWheelRadius; // 로봇의 바퀴 반지름. unit mm

	double m_dWheelEncoderCount[2]; //0-->Left , 1-->Right
	double m_dReferenceWheelEncoderCount[2]; //0-->Left , 1-->Right
	double m_dWheelDistance[2]; //0-->Left , 1-->Right
	double m_dAverageWheelDistance;
	double m_dDistance2RobotCenter;
	double m_dReferenceXYT[3]; //[0]-->x, [1]-->y, [2]-->theta

	int m_nRefLeftVelmm;
	int m_nRefRightVelmm; 

	//pulse & encoder variables
	bool m_bLeftMortorRotateSign, m_bRightMortorRotateSign;
	int m_nRefLeftPulseCnt, m_nRefRightPulseCnt;

private:
	static const int TVEL_INCREMENT = 20; //20mm/sec 병진속도의 증가량.
	static const int RVEL_INCREMENT = 1; // 1deg/sec 회전속도의 증가량.
	int MAX_TRANSLATION_VELOCITY;
	static const int MAX_ROTATION_VELOCITY = 60; //30deg/sec

	double m_dTranslationVelocity;
	double m_dRotationVelocity;

	KuPose m_RobotPos;

	double m_dReferenceX;
	double m_dReferenceY;
	double m_dReferenceT;
	int m_nPort;
public:
	bool connect(string strSerialPort);
	bool disConnect();
	void stop();
	bool Ready();

	double getX();
	double getY();
	double getThetaDeg();
	double getTVelIncrement();
	double getRVelIncrement();
	double getTVel();
	double getRVel();
	double getVoltage();
	KuPose getDelEncoderData();
	void setMaxTransVel(int nVel);


	void convertPulseCnt2EncoderCnt(int nCurLeftPulseCnt, int nCurRightPulseCnt,int* nLeftEncoderCnt, int* nRightEncoderCnt);
	void moveByTRVelocity(int nTranslationVelocity, int nRotationalVelocity);
	KuPose calcDelEncoderPos(int nLeftWheelEncCnt, int nRightWheelEncCnt);
	void setRobotPos(KuPose RobotPos);
	KuPose getRobotPos();

	SSAGVWheelActuatorInterface();
	~SSAGVWheelActuatorInterface();
};
#endif





