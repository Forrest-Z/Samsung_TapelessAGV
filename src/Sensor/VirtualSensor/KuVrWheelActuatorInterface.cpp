#include "stdafx.h"
#include "KuVrWheelActuatorInterface.h"

KuVrWheelActuatorInterface::KuVrWheelActuatorInterface()
{

	m_dAccumulatedDeltaMovement = 0.0;
	m_dAccumulatedDeltaAngle = 0.0;

	InitVariable();
		
}

KuVrWheelActuatorInterface::~KuVrWheelActuatorInterface()
{
	
}


void KuVrWheelActuatorInterface::InitVariable()
{
	//for encoder
	m_dEncData[0] =0.;
	m_dEncData[1] =0.;
	m_dReferenceLeftWheelEncoderCount =0.;
	m_dReferenceRightWheelEncoderCount=0.;
	m_dLeftWheelEncoderCount=0.;
	m_dRightWheelEncoderCount=0.;
	m_dLeftWheelDistance=0.;
	m_dRightWheelDistance=0.;
	m_dAverageWheelDistance=0.;
	m_dDistance2RobotCenter=0.;
	m_dDeltaX =0.;
	m_dDeltaY =0.;
	m_dDeltaT =0.; 
	m_dReferenceX = 0;
    m_dReferenceY = 0;
    m_dReferenceT = 0;
	
	m_dRightWheelVel = m_dLeftWheelVel =0.;

	m_dNoiseFactor =0;
}

/**
 @brief Korean: 노이즈 값을 설정하는 함수. 1~10사이의 값
 @brief English: 
*/
void KuVrWheelActuatorInterface::setNoiseFactor(int nNoiseFactor)
{
	m_dNoiseFactor = (double)nNoiseFactor/1000.;
}

void KuVrWheelActuatorInterface::moveTRVelocity(double dTVel, double dRotDegVel)
{

	if(dTVel>MAX_TRANSLATION_VELOCITY)dTVel=MAX_TRANSLATION_VELOCITY;
	else if(dTVel<-MAX_TRANSLATION_VELOCITY)dTVel=-MAX_TRANSLATION_VELOCITY;
	if(dRotDegVel>MAX_ROTATION_VELOCITY)dRotDegVel=MAX_ROTATION_VELOCITY;
	else if(dRotDegVel<-MAX_ROTATION_VELOCITY)dRotDegVel=-MAX_ROTATION_VELOCITY;	
		
	double dW = dRotDegVel*D2R;
	m_dRightWheelVel = dTVel + (BETWEEN_WHEEL * dW)/2.;
	m_dLeftWheelVel = dTVel - (BETWEEN_WHEEL * dW)/2.;
	
	int nRandomVal = rand()%10;
	double dNoise = nRandomVal;
	dNoise = dNoise * m_dNoiseFactor;
	m_dRightWheelVel = m_dRightWheelVel + m_dRightWheelVel*dNoise; //10오차

	nRandomVal = rand()%10;
	dNoise = nRandomVal;
	dNoise = dNoise * m_dNoiseFactor;
	m_dLeftWheelVel = m_dLeftWheelVel + m_dLeftWheelVel*dNoise; //10오차

    //10으로 나눈 이유는 0.1초마다 엔코더를 세기 때문에 
	m_dEncData[0] += m_dLeftWheelVel / 10 * (GEAR_RATIO * ENCODER_RESOLUTION) /
					 (2 * M_PI * RADIUS);

	m_dEncData[1] += m_dRightWheelVel / 10 * (GEAR_RATIO * ENCODER_RESOLUTION) /
					 (2 * M_PI * RADIUS);

	calcEncoderData(); 
}
void KuVrWheelActuatorInterface::moveByVelocityXYT(int deltaX, int deltaY, int deltaTheta )
{
	m_dDeltaX=deltaX/10.0;
	m_dDeltaY=deltaY/10.0;
	m_dDeltaT=deltaTheta/10.0;

	//상대좌표를 절대좌표로 변환하는것.
	m_dReferenceX +=  m_dDeltaX * cos( m_dReferenceT ) - m_dDeltaY * sin( m_dReferenceT );
	m_dReferenceY +=  m_dDeltaX * sin( m_dReferenceT ) + m_dDeltaY * cos( m_dReferenceT);
	m_dReferenceT =  m_dReferenceT+m_dDeltaT;


	if(m_dReferenceT > M_PI){
		m_dReferenceT -= 2*M_PI;
	}
	else if(m_dReferenceT < -M_PI){
		m_dReferenceT += 2*M_PI;
	}

	m_DelEncoderPos.setX(m_dDeltaX); 
	m_DelEncoderPos.setY(m_dDeltaY);
	m_DelEncoderPos.setThetaRad(m_dDeltaT); 

}
void KuVrWheelActuatorInterface::setRobotPos(KuPose RobotPos)
{
	m_RobotPos = RobotPos;
	m_dReferenceX = RobotPos.getX();
	m_dReferenceY = RobotPos.getY();
	m_dReferenceT = RobotPos.getThetaRad();
}

KuPose KuVrWheelActuatorInterface::getRobotPos()
{
	m_RobotPos.setX( m_dReferenceX );
	m_RobotPos.setY( m_dReferenceY );
	m_RobotPos.setThetaRad( m_dReferenceT );

	return m_RobotPos;
}

KuPose KuVrWheelActuatorInterface::getDelEncoderData()
{
	return m_DelEncoderPos;
}


void KuVrWheelActuatorInterface::computeAccumulatedDeltaMovement(KuPose EncoderDelPos)
{
	m_dAccumulatedDeltaMovement += sqrt(EncoderDelPos.getX()*EncoderDelPos.getX() + EncoderDelPos.getY()*EncoderDelPos.getY());
	m_dAccumulatedDeltaAngle += fabs(EncoderDelPos.getThetaDeg());
}

bool KuVrWheelActuatorInterface::isAccDeltaMovementOver(double dMovement, double dAngle)
{
	if (m_dAccumulatedDeltaMovement > dMovement || m_dAccumulatedDeltaAngle > dAngle) {
		m_dAccumulatedDeltaMovement = 0.0;
		m_dAccumulatedDeltaAngle = 0.0;
		return true;
	}
	else return false;
}
void KuVrWheelActuatorInterface::calcEncoderData()
{
	m_dLeftWheelEncoderCount = m_dEncData[0];
	m_dRightWheelEncoderCount = m_dEncData[1];

	m_dLeftWheelDistance = ( (m_dLeftWheelEncoderCount - m_dReferenceLeftWheelEncoderCount)
		/ (GEAR_RATIO * ENCODER_RESOLUTION)
		) * 2 * M_PI * RADIUS;


	m_dRightWheelDistance = ( (m_dRightWheelEncoderCount - m_dReferenceRightWheelEncoderCount )
		/ (GEAR_RATIO * ENCODER_RESOLUTION)
		) * 2 * M_PI * RADIUS;

	m_dReferenceLeftWheelEncoderCount = m_dLeftWheelEncoderCount;
	m_dReferenceRightWheelEncoderCount = m_dRightWheelEncoderCount;

	m_dAverageWheelDistance = (m_dLeftWheelDistance + m_dRightWheelDistance)/2;

	m_dDeltaT = (m_dRightWheelDistance - m_dLeftWheelDistance) / BETWEEN_WHEEL;

	if(fabs(m_dDeltaT) >= 0.0017f ) {
		m_dDistance2RobotCenter = m_dAverageWheelDistance / m_dDeltaT;
		m_dDeltaY = m_dDistance2RobotCenter - ( m_dDistance2RobotCenter * cos(m_dDeltaT) );
		m_dDeltaX = m_dDistance2RobotCenter * sin(m_dDeltaT);
	}
	else {
		m_dDistance2RobotCenter = 0.0f;
		m_dDeltaY = 0;
		m_dDeltaX = m_dAverageWheelDistance;
	}

	//상대좌표를 절대좌표로 변환하는것.
	m_dReferenceX +=  m_dDeltaX * cos( m_dReferenceT ) + m_dDeltaY * sin( -m_dReferenceT );
	m_dReferenceY +=  m_dDeltaX * sin( m_dReferenceT ) + m_dDeltaY * cos( m_dReferenceT  );
	m_dReferenceT =  m_dReferenceT+m_dDeltaT;


	if(m_dReferenceT > M_PI){
		m_dReferenceT -= 2*M_PI;
	}
	else if(m_dReferenceT < -M_PI){
		m_dReferenceT += 2*M_PI;
	}

	m_DelEncoderPos.setX(m_dDeltaX); 
	m_DelEncoderPos.setY(m_dDeltaY);
	m_DelEncoderPos.setThetaRad(m_dDeltaT); 
	computeAccumulatedDeltaMovement(m_DelEncoderPos);
	
}