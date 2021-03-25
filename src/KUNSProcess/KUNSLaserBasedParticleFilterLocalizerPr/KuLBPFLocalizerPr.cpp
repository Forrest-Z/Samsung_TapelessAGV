#include "stdafx.h"
#include "KuLBPFLocalizerPr.h"


KuLBPFLocalizerPr::KuLBPFLocalizerPr()
{
	m_bIsParticlConverged = false;
	m_dAccumulatedDeltaMovement = 0.0;
	m_dAccumulatedDeltaAngle = 0.0;
	m_RobotPos.init(); //로봇 위치 초기화

	m_bIsThreadFuncGenerated = false; //스레드가 생성되지 않았다는 플래그
	m_doThreadFunc = false; //스레드 함수가 실행되도록 하는 플래그
	m_nThreadFuncPeriod = 0; //스레드 함수 실행 주기..단위는ms
	m_bIsNewSensorData = false;
	m_bfirst = true;
	m_dDelEncoderData.init();
	m_cvMatImage.create(Sensor::CEILING_IMAGE_HEIGHT,Sensor::CEILING_IMAGE_WIDTH,CV_8UC1);//Result image initialization
	m_nLaserData = m_KuUtil.generateIntType1DArray(Sensor::URG04LX_DATA_NUM181,0);
	m_nSpreadIdx=-1;

}

KuLBPFLocalizerPr::~KuLBPFLocalizerPr()
{

}

/**
 @brief Korean:  로봇 위치 초기화
 @brief English: 
*/
void KuLBPFLocalizerPr::init()
{
	m_RobotPos.init(); //로봇 위치 초기화
}

/**
 @brief Korean:  로봇 위치를 설정한다
 @brief English: 
*/
void KuLBPFLocalizerPr::setRobotPos(KuPose RobotPos)
{
	m_RobotPos = RobotPos;

}

/**
 @brief Korean:  로봇의 X좌표를 설정한다
 @brief English: 
*/
void KuLBPFLocalizerPr::setRobotPosX(double dPoseX)
{
	m_RobotPos.setX(dPoseX);
}
/**
 @brief Korean:  로봇의 Y좌표를 설정한다
 @brief English: 
*/
void KuLBPFLocalizerPr::setRobotPosY(double dPoseY)
{
	m_RobotPos.setY(dPoseY);
}
/**
 @brief Korean:  로봇의 각도 Degree를 설정한다
 @brief English: 
*/
void KuLBPFLocalizerPr::setRobotPosDeg(double dPoseDeg)
{
	m_RobotPos.setThetaDeg(dPoseDeg);
}
/**
 @brief Korean:  로봇의 각도 Radian를 설정한다
 @brief English: 
*/
void KuLBPFLocalizerPr::setRobotPosRad(double dPoseRad)
{
	m_RobotPos.setThetaRad(dPoseRad);
}
/**
 @brief Korean: 지도정보를 설정한다
 @brief English: 
*/
void KuLBPFLocalizerPr::setMap(int nMapSizeX, int nMapSizeY, int** nMap)
{
	m_ParticleFilter.setMap(nMapSizeX, nMapSizeY, nMap);
	setRangeSensorParameter(); 
}
/**
 @brief Korean: 거리센서의 파라미터 정보를 받아온다
 @brief English: set parameters of range sensor
*/
void KuLBPFLocalizerPr::setRangeSensorParameter()
{
	int nNumOfSensor = Sensor::URG04LX_DATA_NUM181;
	int nSensorMaxDist = KuRobotParameter::getInstance()->getURG04LXLaserMaxDist();
	int nMinAngle = -90;
	int nNoOfSensingPoint = nNumOfSensor;
	double dInterval = 1;
	double dRangeSensorOffset = KuRobotParameter::getInstance()->getFrontLaserXOffset();
	double dMaxScannerDist = (double)nSensorMaxDist;
	m_ParticleFilter.setRangeSensorParameter(nNoOfSensingPoint, nMinAngle, dInterval, dMaxScannerDist, dRangeSensorOffset); 
}

/**
 @brief Korean:  로봇의 X좌표를 가져 간다
 @brief English:
*/
double KuLBPFLocalizerPr::getRobotPosX()
{
	return m_RobotPos.getX();
}
/**
 @brief Korean:  로봇의 Y좌표를 가져 간다
 @brief English:
*/
double KuLBPFLocalizerPr::getRobotPosY()
{
	return m_RobotPos.getY();
}
/**
 @brief Korean:  로봇의  각도(Degree)를 가져 간다
 @brief English:
*/
double KuLBPFLocalizerPr::getRobotPosDeg()
{
	return m_RobotPos.getThetaDeg();
}
/**
 @brief Korean:  로봇의  각도(Radian)를 가져 간다
 @brief English:
*/
double KuLBPFLocalizerPr::getRobotPosRad()
{
	return m_RobotPos.getThetaRad();
}
/**
 @brief Korean:  로봇의  위치 값을 가져 간다
 @brief English:
*/
KuPose KuLBPFLocalizerPr::getRobotPos()
{
	return m_RobotPos;
}



/**
 @brief Korean:  샘플들의 위치 값들을 받아온다
 @brief English:
*/
vector<Sample> KuLBPFLocalizerPr::getParticle()
{
	vector<Sample> vecParticle;
	m_CriticalSection.Lock();
	vecParticle = m_vecParticle;
	m_CriticalSection.Unlock();

	return vecParticle;
}

/**
 @brief Korean:  거리센서의 거리 값을  설정한다
 @brief English:
*/
void KuLBPFLocalizerPr::copyRangeData(int_1DArray nLaserData)
{

	m_CriticalSection.Lock();
	for(int i=0; i<Sensor::URG04LX_DATA_NUM181; i++){
		m_nLaserData[i] = nLaserData[i];
	}	
	m_CriticalSection.Unlock();


}

/**
 @brief Korean: 엔코더의 정보를 설정한다
 @brief English:
*/
void KuLBPFLocalizerPr::copyEncoderData(KuPose delEncoderData)
{

	m_CriticalSection.Lock();
	m_dDelEncoderData.setX(m_dDelEncoderData.getX()+delEncoderData.getX());
	m_dDelEncoderData.setY(m_dDelEncoderData.getY()+delEncoderData.getY());
	m_dDelEncoderData.setThetaDeg(m_dDelEncoderData.getThetaDeg()+delEncoderData.getThetaDeg());
	m_CriticalSection.Unlock();

}

/**
 @brief Korean: 로봇의 병진 운동에 대한 델타 값을 계산한다
 @brief English:
*/
void KuLBPFLocalizerPr::computeAccumulatedDeltaMovement(KuPose EncoderDelPos)
{
	m_dAccumulatedDeltaMovement += sqrt(EncoderDelPos.getX()*EncoderDelPos.getX() + EncoderDelPos.getY()*EncoderDelPos.getY());
	m_dAccumulatedDeltaAngle += fabs(EncoderDelPos.getThetaDeg());
}

/**
 @brief Korean: 로봇의 회전 운동에 대한 델타 값을 계산한다
 @brief English:
*/
bool KuLBPFLocalizerPr::isAccDeltaMovementOver(double dMovement, double dAngle)
{
	if (m_dAccumulatedDeltaMovement > dMovement || m_dAccumulatedDeltaAngle > dAngle) {
		m_dAccumulatedDeltaMovement = 0.0;
		m_dAccumulatedDeltaAngle = 0.0;
		return true;
	}
	else return false;
}

/**
 @brief Korean: 모션에 대한 불확실성 정보를 받아온다
 @brief English: set uncertainty of wheel motion
 of range sensor
*/
void KuLBPFLocalizerPr::setDeviation(double dDeviationforTrans, double dDeviationforRotate,double dDeviationforTransRotate )
{
	m_ParticleFilter.setDeviation( dDeviationforTrans,  dDeviationforRotate, dDeviationforTransRotate );
}

/**
 @brief Korean: 샘플의 최대 최소 개수를 받아온다
 @brief English: set  max and min sample nubers
 of range sensor
*/
void KuLBPFLocalizerPr::setSampleNum(int nMaxSample,int nMinSample)
{
	m_ParticleFilter.setSampleNum( nMaxSample,  nMinSample );
}

/**
 @brief Korean:  엔코더로만 추정된 로봇의 위치 값을 받아온다
 @brief English: 
 of range sensor
*/
KuPose KuLBPFLocalizerPr::estimateRobotPosByDeadReckoning(KuPose EncoderDelPos)
{

	copyEncoderData(EncoderDelPos);

	double dX = m_RobotPos.getX() + EncoderDelPos.getX() * cos(m_RobotPos.getThetaRad()) + 
		EncoderDelPos.getY() * sin(-m_RobotPos.getThetaRad());

	double dY = m_RobotPos.getY() + EncoderDelPos.getX() * sin(m_RobotPos.getThetaRad()) + 
		EncoderDelPos.getY() * cos(m_RobotPos.getThetaRad());
	double dThetaDeg = m_RobotPos.getThetaDeg() + EncoderDelPos.getThetaDeg();

	// pose update	
	m_RobotPos.setX(dX);
	m_RobotPos.setY(dY);
	m_RobotPos.setThetaDeg(dThetaDeg);

	computeAccumulatedDeltaMovement(EncoderDelPos);

	return m_RobotPos;

}


void KuLBPFLocalizerPr::doThread(void* arg)
{
	KuLBPFLocalizerPr::getInstance()->estimateRobotPosUsingParticleFilter();	
}

void KuLBPFLocalizerPr::terminate()
{
	m_bIsThreadFuncGenerated = false; //스레드가 생성되지 않았다는 플래그
	m_doThreadFunc = false; //스레드 함수가 실행되도록 하는 플래그
	m_KuThread.terminate();
}

void KuLBPFLocalizerPr::start(int nPeriod)
{
if(false==m_bIsThreadFuncGenerated){
	m_bIsThreadFuncGenerated=true;
	m_KuThread.start(doThread, this, nPeriod, "KuLBPFLocalizerPr");		
	}
}

/**
 @brief Korean:  로봇 주변에 샘플들을 부려준다
 @brief English: 
 of range sensor
*/
void KuLBPFLocalizerPr::spreadParticleNearRobot(KuPose RobotPos, double dRegionSize)
{
	m_ParticleFilter.setSamplesNearRobot(RobotPos.getX(), RobotPos.getY(), RobotPos.getThetaRad(), dRegionSize);
	m_ParticleFilter.ResetReservation();
	m_ParticleFilter.m_bCalculationStop = true;

}
/**
 @brief Korean:  
 @brief English: 
 of range sensor
*/
bool KuLBPFLocalizerPr::getParticleConvergeStatus()
{
	return m_bIsParticlConverged;
}

/**
 @brief Korean:  파티클 필터를 이용하여 로봇의 위치를 추정한다
 @brief English: 
 of range sensor
*/
void KuLBPFLocalizerPr::estimateRobotPosUsingParticleFilter()
{
	if(m_bfirst == false && m_bIsNewSensorData == false){
 		return ;
 	} 

	
	KuPose dDelEncoderPos;
	int nLaserData[181];
	double dDelEncoderData[3];
	double dSamplePos[3];

	m_CriticalSection.Lock();
	dDelEncoderPos = m_dDelEncoderData;
	dDelEncoderData[0] = m_dDelEncoderData.getX();
	dDelEncoderData[1] = m_dDelEncoderData.getY();
	dDelEncoderData[2] = m_dDelEncoderData.getThetaRad();
	m_dDelEncoderData.init();


	for(int i=0;i<181;i++){
		nLaserData[i] = m_nLaserData[i];
	}

	m_CriticalSection.Unlock();
	bool bMatchingflag=false;
	KuPose LandMarkPos;


	if( bMatchingflag == false){ //랜드마크를 탐지못하고 있을때..
		//파티클만으로 로봇의 위치를 추정한다.
		m_ParticleFilter.setEncoderData(dDelEncoderData);
		m_ParticleFilter.setRangeData(nLaserData);
		m_nPaticleFilterState = m_ParticleFilter.getSamplePos(dSamplePos , m_dEstimatedRangeData);    
	}
  	else{ //랜드마크 탐지. 추정된 위치에 파티클 다시 뿌린다.
  		//TRACE("랜드 마크 인식\n"); 		
  		m_ParticleFilter.setEncoderData(dDelEncoderData);
  		m_ParticleFilter.setRangeData(nLaserData);
  		m_nPaticleFilterState = m_ParticleFilter.getSamplePos(dSamplePos , m_dEstimatedRangeData);    
		spreadParticleNearRobot(LandMarkPos,0.3);		 		
  	}


	if(dSamplePos[0]!=0.0 && dSamplePos[1]!=0.0 ){
		m_CriticalSection.Lock();
		m_RobotPos.setX(dSamplePos[0]);
		m_RobotPos.setY(dSamplePos[1]);
		m_RobotPos.setThetaRad(dSamplePos[2]);
		m_CriticalSection.Unlock();

	}

	
	//업데이트 할때 마다 샘플정보를 변수에 저장한다.
	m_CriticalSection.Lock();
	m_nParticleNum = m_ParticleFilter.m_nSampleNum;
	m_vecParticle = m_ParticleFilter.getParticle();	
	m_bIsNewSensorData = false;	
	m_CriticalSection.Unlock();

	//----------------------------------------------

	m_bfirst = false;
}

/**
 @brief Korean: 지정된 영역에 샘플을 골고루 뿌리고 확률값을 초기화시킴.
 @brief English: distribute samples uniformly on predefined region with normalized probabilities.
*/
void KuLBPFLocalizerPr::resetSamples()
{
	m_ParticleFilter.setSamples();
	m_bIsParticlConverged = false;
}

/**
 @brief Korean: 파티클 필터를 이용하여 로봇의 위치를 추정한다 .
 @brief English: 
*/
KuPose KuLBPFLocalizerPr::estimateRobotPos(int_1DArray nRangeData, KuPose EncoderDelPos, IplImage * IplCeilingImage )
{
	copyRangeData(nRangeData); //거리센서 데이터를 복사해준다. 
	copyEncoderData(EncoderDelPos); //엔코더 데이터를 복사해준다. 
//	copyCeilingImage( IplCeilingImage);


		m_CriticalSection.Lock();
	double dX = m_RobotPos.getX() + EncoderDelPos.getX() * cos(m_RobotPos.getThetaRad()) + 
		EncoderDelPos.getY() * sin(-m_RobotPos.getThetaRad());
	double dY = m_RobotPos.getY() + EncoderDelPos.getX() * sin(m_RobotPos.getThetaRad()) + 
		EncoderDelPos.getY() * cos(m_RobotPos.getThetaRad());
	double dThetaDeg = m_RobotPos.getThetaDeg() + EncoderDelPos.getThetaDeg();

	// pose update 
 	m_RobotPos.setX(dX);
 	m_RobotPos.setY(dY);
 	m_RobotPos.setThetaDeg(dThetaDeg);
	m_bIsNewSensorData = true;

	computeAccumulatedDeltaMovement(EncoderDelPos);
	KuPose RobotPose =  m_RobotPos;	
	m_CriticalSection.Unlock();

	return RobotPose;

}
/**
 @brief Korean: 천장 이미지를 저장한다.
 @brief English: 
*/
void KuLBPFLocalizerPr::copyCeilingImage(IplImage * IplCeilingImage)
{
	for(int i = 0; i < Sensor::CEILING_IMAGE_WIDTH * Sensor::CEILING_IMAGE_HEIGHT; i++)
	{
		m_cvMatImage.data[i]=IplCeilingImage->imageData[i];
	}
	
}
