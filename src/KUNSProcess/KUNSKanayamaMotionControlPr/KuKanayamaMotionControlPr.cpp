#include "stdafx.h"
#include "KuKanayamaMotionControlPr.h"
#include "../../ANSCommon/ANSCommon.h"

KuKanayamaMotionControlPr::KuKanayamaMotionControlPr()
{
	m_nMaxTVel = 1000; //1m/sec.
	m_nMinTVel = 200; //0.2m/sec.
	m_nMaxRotDegVel = 50; //50deg/sec.
	m_nMinRotDegVel = 21; 
	//m_dDesiredVel = 300;
	m_dTranstempVel=100;
	m_dRotationtempDegVel=0;

	m_bAlignRobotAngleFlag = true;

	m_nTurnVel = 22; //25deg/sec.
	m_dKXGain = 0.4;   //로봇의 전진방향 오차를 극복하는 게인. 실제 로봇의 경우 너무 크면 출렁이고, 너무 작으면 원하는 속도보다 느리게 움직임.
	m_dKYGain = 1.0;   //로봇의 측면방향 오차를 극복하는 게인. 영향이 크지는 않지만, 크면 불안정하고, 작으면 측면방향 오차를 보정 못함.
	m_dKThetaGain = 0.8;  //목적지를 향한 로봇의 방향(heading)을 극복하는 게인. 크면 매우 출렁이고(특히 초기에 제자리에서 목적지를 향해 휙 돔), 작으면 딴 방향을 향함.

	m_nMotionModelflag=TWO_WHEEL_MODEL;

	m_bArrived = false;
}

KuKanayamaMotionControlPr::~KuKanayamaMotionControlPr()
{

}

/**
@brief Korean: 최대 V,W값을 설정하는 함수.
@brief English: 
*/
void KuKanayamaMotionControlPr::setMaxTRVel(int nMaxTVel, int nMaxRotDegVel)
{
	m_nMaxTVel = nMaxTVel; 
	m_nMaxRotDegVel = nMaxRotDegVel; 
}

/**
@brief Korean: 최소 V,W값을 설정하는 함수.
@brief English: 
*/
void KuKanayamaMotionControlPr::setMinTRVel(int nMinTVel, int nMinRotDegVel)
{
	m_nMinTVel = nMinTVel; 	
	m_nMinRotDegVel = nMinRotDegVel;
}

/**
@brief Korean: 목표점이 로봇 뒤에 있는 경우 제자리에서 회전해야 하는 경우가 발생한다.
@brief         이때 제자리 에서 회전할 속도를 정해주는 함수.
@brief English: 
*/
void KuKanayamaMotionControlPr::setMinTRVel(int nTurnVel)
{
	m_nTurnVel = nTurnVel; 	
}

/**
@brief Korean: kanayama 제어식의 게인값을 설정할 수 있는 함수.
@brief English: 
*/
void KuKanayamaMotionControlPr::setGain(double dKXGain, double dKYGain, double dKThetaGain)
{
	m_dKXGain = dKXGain;
	m_dKYGain = dKYGain;
	m_dKThetaGain = dKThetaGain;
}
/**
@brief Korean: 
@brief English: 
*/
void KuKanayamaMotionControlPr::setDirectionofRotation(bool bDirectionofRotationflag )
{
	m_bDirectionofRotationflag = bDirectionofRotationflag;
}

void KuKanayamaMotionControlPr::init()
{

	m_bAlignRobotAngleFlag = true;

	m_nTurnVel = 22; //25deg/sec.

}
/**
@brief Korean:모션 모델에 따라 설정해준다.
m_bMotionModelflag=OMNI_WHEEL_MODEL=1
m_bMotionModelflag=TWO_WHEEL_MODEL=2
@brief English: 
*/
void KuKanayamaMotionControlPr::setMotionmodel(int nMotionModelflag)
{
	m_nMotionModelflag=nMotionModelflag;
}

bool KuKanayamaMotionControlPr::getAlignRobotAngle()
{
	return m_bAlignRobotAngleFlag;
}

/**
@brief Korean: kanayama 제어식을 구현한 함수. 리턴값으로 타겟지점까지의 계산된 v,w를 제공해준다.
@brief English: 
*/
KuVelocity KuKanayamaMotionControlPr::generateTRVelocity(KuPose TargetPos, KuPose RobotPos, double dDesiredVel)
{
	KuVelocity generatedVel;

	//예외처리 구간--------------------------------------------------------------------------------------------------------------
	//목표점이 로봇 뒤에 있는 경우이다.
	if(m_bAlignRobotAngleFlag==true){			
		double dAngleDiff = atan2(TargetPos.getY()-RobotPos.getY(),TargetPos.getX()-RobotPos.getX()) - RobotPos.getThetaRad();
		if (dAngleDiff >= M_PI){ dAngleDiff -= 2.0*M_PI; }			
		else if (dAngleDiff <= -M_PI){ dAngleDiff += 2.0*M_PI; }

//		if(m_bDirectionofRotationflag==true)
//		{
//			if (fabs(dAngleDiff) > M_PI*1/36) { 
//				generatedVel.m_nTranslationVel = 0;
//				generatedVel.m_nXVel = 0;
//				generatedVel.m_nYVel = 0;
//				generatedVel.m_nRotationDegVel = (int)(m_nTurnVel);
//				return generatedVel;
//			}	
//		}
//		else
//		{
//			if (fabs(dAngleDiff) > M_PI*1/36) { 
//				generatedVel.m_nTranslationVel = 0;
//				generatedVel.m_nXVel = 0;
//				generatedVel.m_nYVel = 0;
//				generatedVel.m_nRotationDegVel = (int)(-m_nTurnVel);
//				return generatedVel;
//			}	
//		}
 		if (dAngleDiff > M_PI*1/36) { 
 			generatedVel.m_nTranslationVel = 0;
 			generatedVel.m_nXVel = 0;
 			generatedVel.m_nYVel = 0;
 			generatedVel.m_nRotationDegVel = (int)(m_nTurnVel);
 			return generatedVel;
 		}	
 		else if (dAngleDiff < -M_PI*1/36) {
 			generatedVel.m_nTranslationVel = 0;
 			generatedVel.m_nXVel = 0;
 			generatedVel.m_nYVel = 0;
 			generatedVel.m_nRotationDegVel = (int)(-m_nTurnVel);
 			return generatedVel;
 		}
		m_bAlignRobotAngleFlag = false;
	
	}
	//==============================================================================================================================


	double dAngleDiff = atan2(TargetPos.getY()-RobotPos.getY(),TargetPos.getX()-RobotPos.getX()) - RobotPos.getThetaRad();
	if (dAngleDiff >= M_PI){ dAngleDiff -= 2.0*M_PI; }			
	else if (dAngleDiff <= -M_PI){ dAngleDiff += 2.0*M_PI; }

	if (fabs(dAngleDiff) > M_PI*1/3) { 
		m_bAlignRobotAngleFlag=true;

		generatedVel.m_nTranslationVel = 0;
		generatedVel.m_nRotationDegVel = 0;
		generatedVel.m_nXVel = 0;
		generatedVel.m_nYVel = 0;

		return generatedVel;

	}

	double dMaxV = (double)m_nMaxTVel/1000.;     /// 로봇이 낼 수 있는 최대 속도
	double dMaxW = m_nMaxRotDegVel*D2R; ///로봇이 낼 수 있는 최대 각속도
	double dMinW = m_nMinRotDegVel*D2R; ///로봇이 낼 수 있는 최소 각속도


	// 목표위치와 로봇위치 사이의 에라를 계산.
	// 목표위치와 로봇위치는 절대좌표계 기준. 로봇의 모션은(v,w) 로봇좌표계 기준.
	// 따라서 절대좌표계 상에서의 에라를 상대좌표계로 변환해야 함.

	double derrorX = (TargetPos.getXm() - RobotPos.getXm()) * cos(-RobotPos.getThetaRad()) + (TargetPos.getYm() - RobotPos.getYm()) * sin(RobotPos.getThetaRad());
	double derrorY = (TargetPos.getXm() - RobotPos.getXm()) * sin(-RobotPos.getThetaRad()) + (TargetPos.getYm() - RobotPos.getYm()) * cos(-RobotPos.getThetaRad());
	double derrorThRad = atan2(TargetPos.getYm() - RobotPos.getYm(), TargetPos.getXm() - RobotPos.getXm()) - RobotPos.getThetaRad();
	if(derrorThRad > M_PI ) derrorThRad = derrorThRad - 2*M_PI;
	if(derrorThRad <-M_PI ) derrorThRad = derrorThRad + 2*M_PI;

	if(m_nMotionModelflag==TWO_WHEEL_MODEL)
	{
		//Kanayama의 식을 이용해서 error를 줄이기 위한, 즉 목표점을 향해 가기 위한 로봇의 속도 계산.
		double dTVel = dDesiredVel/1000.*cos(derrorThRad) + m_dKXGain*derrorX;
		double dRotVel = 0 + dDesiredVel/1000.*(m_dKYGain*derrorY + m_dKThetaGain*sin(derrorThRad));
		if (dTVel>dMaxV) dTVel=dMaxV; 
		if (dRotVel>dMaxW) dRotVel=dMaxW;
		if (dRotVel<-dMaxW) dRotVel=-dMaxW;

		if(fabs(derrorX*1000)<200) // mm
		{
			generatedVel.m_nTranslationVel = 0;
			generatedVel.m_nRotationDegVel = 0;
			generatedVel.m_nXVel = 0;
			generatedVel.m_nYVel = 0;

			m_bArrived = true;
		}
		else
		{	
			if(dTVel<0)dTVel=0;
			generatedVel.m_nTranslationVel = (int)(dTVel*1000);
			generatedVel.m_nRotationDegVel = (int)(dRotVel*R2D);
			generatedVel.m_nXVel = 0;
			generatedVel.m_nYVel = 0;

			m_bArrived = false;
		}
	}
	else if(m_nMotionModelflag==OMNI_WHEEL_MODEL)
	{

		double derror=hypot(derrorX,derrorY);
		double dXVel=dDesiredVel/1000.*derrorX/derror;
		double dYVel=dDesiredVel/1000.*derrorY/derror;

		if (dXVel>dMaxV) dXVel=dMaxV; 
		else if (dXVel<-dMaxV) dXVel=-dMaxV; 
		if (dYVel>dMaxV) dYVel=dMaxV; 
		else if (dYVel<-dMaxV) dYVel=-dMaxV; 


		if(derror*1000<150){
			generatedVel.m_nTranslationVel = 0;
			generatedVel.m_nRotationDegVel = 0;
			generatedVel.m_nXVel = 0;
			generatedVel.m_nYVel = 0;

			m_bArrived = true;
		}
		else{		
			generatedVel.m_nXVel = (int)(dXVel*1000);
			generatedVel.m_nYVel = (int)(dYVel*1000);
			generatedVel.m_nTranslationVel = 0;
			generatedVel.m_nRotationDegVel = 0;

			m_bArrived = false;
		}

	}

	return generatedVel;
}

bool KuKanayamaMotionControlPr::isArrived(void)
{
	return m_bArrived;
}