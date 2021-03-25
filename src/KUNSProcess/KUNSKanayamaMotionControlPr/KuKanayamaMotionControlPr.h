/*______________________________________________________________________________________________
PROJECT : Intelligent Robotics Lab. Department of Mecanical Engineering Korea University                                    
(c) Copyright 2007 KUME Intelligent Robotics Lab.                                             
All rights reserved                                                                         
$Directed by Jae-Bok Song                                                                     
$Designed by Joong-Tae Park                                                                  
$Description : kanauama의 제어이론을 사용한 모션컨트롤 프로세스
$Created on: 2012. 6. 12.                                                                        
$Author: Joong-Tae Park      
$e-mail: jtpark1114@gmail.com                                                                 
______________________________________________________________________________________________*/

#ifndef C_KUNS_KANAYAMA_MOTION_CONTROL_PROCESS_H
#define C_KUNS_KANAYAMA_MOTION_CONTROL_PROCESS_H

#include <list>
#include <iostream>
#include "../../KUNSPose/KuPose.h"
#include "../../KUNSMath/KuMath.h"

using namespace std;

class KuVelocity
{
public:
	int m_nTranslationVel;
	int m_nXVel;
	int m_nYVel;
	int m_nRotationDegVel;

};

class KuKanayamaMotionControlPr 
{
public:
	static const int OMNI_WHEEL_MODEL = 1;
	static const int TWO_WHEEL_MODEL = 2;

private:
	int m_nMaxTVel;
	int m_nMinTVel;	
	int m_nMaxRotDegVel;
	int m_nMinRotDegVel;
	int m_nTurnVel;
	double m_dRotationtempDegVel;	
	double m_dTranstempVel;	

	bool m_bAlignRobotAngleFlag;
	bool m_bArrived;

	double m_dKXGain;   //로봇의 전진방향 오차를 극복하는 게인. 실제 로봇의 경우 너무 크면 출렁이고, 너무 작으면 원하는 속도보다 느리게 움직임.
	double m_dKYGain;   //로봇의 측면방향 오차를 극복하는 게인. 영향이 크지는 않지만, 크면 불안정하고, 작으면 측면방향 오차를 보정 못함.
	double m_dKThetaGain;  //목적지를 향한 로봇의 방향(heading)을 극복하는 게인. 크면 매우 출렁이고(특히 초기에 제자리에서 목적지를 향해 휙 돔), 작으면 딴 방향을 향함.

	int m_nMotionModelflag;
	bool m_bDirectionofRotationflag;
public:
	void setMaxTRVel(int nMaxTVel, int nMaxRotDegVel);
	void setMinTRVel(int nMinTVel, int nMinRotDegVel);	
	void setMinTRVel(int nTurnVel);
	void setGain(double dKXGain, double dKYGain, double dKThetaGain);
	KuVelocity generateTRVelocity(KuPose TargetPos, KuPose RobotPos, double dDesiredVel);
	void init();
	void setMotionmodel(int nMotionModelflag);
	bool getAlignRobotAngle();
	void setDirectionofRotation(bool bDirectionofRotationflag );
	bool isArrived(void);

	KuKanayamaMotionControlPr();
	~KuKanayamaMotionControlPr();


};
#endif /*C_KUNS_KANAYAMA_MOTION_CONTROL_PROCESS_H*/
