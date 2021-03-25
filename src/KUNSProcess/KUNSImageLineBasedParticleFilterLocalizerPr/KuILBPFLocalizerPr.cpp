#include "stdafx.h"
#include "KuILBPFLocalizerPr.h"


KuILBPFLocalizerPr::KuILBPFLocalizerPr()
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
	m_cvMatImageT.create(Sensor::CEILING_IMAGE_HEIGHT,Sensor::CEILING_IMAGE_WIDTH,CV_8UC1);//Result image initialization
	m_cvMatImageBright.create(Sensor::CEILING_IMAGE_HEIGHT,Sensor::CEILING_IMAGE_WIDTH,CV_8UC1);//Result image initialization

	m_nLaserData = m_KuUtil.generateIntType1DArray(Sensor::URG04LX_DATA_NUM181,0);
	m_bsaveFR=false;
	m_bsaveFT=false;

	m_dHeight =KuRobotParameter::getInstance()->getHeightCamera2Mark()/1000.0 ;3.1;//1.74;//1.45;//1.74;
	m_bMapping=false;
}

KuILBPFLocalizerPr::~KuILBPFLocalizerPr()
{

}

/**
@brief Korean:  로봇 위치 초기화
@brief English: 
*/
void KuILBPFLocalizerPr::init()
{
	m_RobotPos.init(); //로봇 위치 초기화
	m_bsaveFR=false;
	m_bsaveFT=false;
}

/**
@brief Korean:  로봇 위치를 설정한다
@brief English: 
*/
void KuILBPFLocalizerPr::setRobotPos(KuPose RobotPos)
{
	m_RobotPos = RobotPos;

}

/**
@brief Korean:  로봇의 X좌표를 설정한다
@brief English: 
*/
void KuILBPFLocalizerPr::setRobotPosX(double dPoseX)
{
	m_RobotPos.setX(dPoseX);
}
/**
@brief Korean:  로봇의 Y좌표를 설정한다
@brief English: 
*/
void KuILBPFLocalizerPr::setRobotPosY(double dPoseY)
{
	m_RobotPos.setY(dPoseY);
}
/**
@brief Korean:  로봇의 각도 Degree를 설정한다
@brief English: 
*/
void KuILBPFLocalizerPr::setRobotPosDeg(double dPoseDeg)
{
	m_RobotPos.setThetaDeg(dPoseDeg);
}
/**
@brief Korean:  로봇의 각도 Radian를 설정한다
@brief English: 
*/
void KuILBPFLocalizerPr::setRobotPosRad(double dPoseRad)
{
	m_RobotPos.setThetaRad(dPoseRad);
}
/**
@brief Korean: 지도정보를 설정한다
@brief English: 
*/
void KuILBPFLocalizerPr::setMap(int nMapSizeX, int nMapSizeY, int** nMap)
{
	m_ParticleFilter.setMap(nMapSizeX, nMapSizeY, nMap);
	setRangeSensorParameter(); 
}

/**
@brief Korean: 거리센서의 파라미터 정보를 받아온다
@brief English: set parameters of range sensor
*/
void KuILBPFLocalizerPr::setRangeSensorParameter()
{
	int nNumOfSensor = Sensor::URG04LX_DATA_NUM181;
	int nSensorMaxDist = KuRobotParameter::getInstance()->getURG04LXLaserMaxDist();
	int nMinAngle = -90;
	int nNoOfSensingPoint = nNumOfSensor;
	double dInterval = 1;
	double dRangeSensorOffset = KuRobotParameter::getInstance()->getFrontLaserXOffset();
	double dMaxScannerDist = (double)nSensorMaxDist;
	m_ParticleFilter.setRangeSensorParameter(nNoOfSensingPoint, nMinAngle, dInterval, dMaxScannerDist, dRangeSensorOffset); 
	
	double dCam_px2m = 5.0400e-006;
	double dCam_fx = KuRobotParameter::getInstance()->getCeilingCameraPrameterFx();
	double dCam_fy =  KuRobotParameter::getInstance()->getCeilingCameraPrameterFy();
	m_dCam_f = (double)((dCam_fx + dCam_fy) / 2. * dCam_px2m);

	double dHeight=KuRobotParameter::getInstance()->getEllipseHeight();
	double dWidth=KuRobotParameter::getInstance()->getEllipseWidth();

	m_ParticleFilter.setCameraParameter( m_dCam_f,dHeight,dWidth);
}

/**
@brief Korean:  로봇의 X좌표를 가져 간다
@brief English:
*/
double KuILBPFLocalizerPr::getRobotPosX()
{
	return m_RobotPos.getX();
}
/**
@brief Korean:  로봇의 Y좌표를 가져 간다
@brief English:
*/
double KuILBPFLocalizerPr::getRobotPosY()
{
	return m_RobotPos.getY();
}
/**
@brief Korean:  로봇의  각도(Degree)를 가져 간다
@brief English:
*/
double KuILBPFLocalizerPr::getRobotPosDeg()
{
	return m_RobotPos.getThetaDeg();
}
/**
@brief Korean:  로봇의  각도(Radian)를 가져 간다
@brief English:
*/
double KuILBPFLocalizerPr::getRobotPosRad()
{
	return m_RobotPos.getThetaRad();
}
/**
@brief Korean:  로봇의  위치 값을 가져 간다
@brief English:
*/
KuPose KuILBPFLocalizerPr::getRobotPos()
{
	return m_RobotPos;
}

/**
@brief Korean: 지도정보를 재설정한다
@brief English: 
*/
void KuILBPFLocalizerPr::updateMapData(int nMapSizeX, int nMapSizeY, int** nMap)
{
	m_CriticalSection.Lock();
	m_ParticleFilter.updateMapData(nMapSizeX, nMapSizeY, nMap);
	m_CriticalSection.Unlock();
}

/**
@brief Korean:  샘플들의 위치 값들을 받아온다
@brief English:
*/
vector<Sample> KuILBPFLocalizerPr::getParticle()
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
void KuILBPFLocalizerPr::copyRangeData(int_1DArray nLaserData)
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
void KuILBPFLocalizerPr::copyEncoderData(KuPose delEncoderData)
{

	m_CriticalSection.Lock();
	m_dDelEncoderData.setX(m_dDelEncoderData.getX()+delEncoderData.getX());
	m_dDelEncoderData.setY(m_dDelEncoderData.getY()+delEncoderData.getY());
	m_dDelEncoderData.setThetaDeg(m_dDelEncoderData.getThetaDeg()+delEncoderData.getThetaDeg());
	m_CriticalSection.Unlock();

}
/**
@brief Korean: 엔코더의 정보를 설정한다
@brief English:
*/
void KuILBPFLocalizerPr::initEncoderData()
{
	m_dDelEncoderData.init();
}
/**
@brief Korean: 로봇의 병진 운동에 대한 델타 값을 계산한다
@brief English:
*/
void KuILBPFLocalizerPr::computeAccumulatedDeltaMovement(KuPose EncoderDelPos)
{
	m_dAccumulatedDeltaMovement += sqrt(EncoderDelPos.getX()*EncoderDelPos.getX() + EncoderDelPos.getY()*EncoderDelPos.getY());
	m_dAccumulatedDeltaAngle += fabs(EncoderDelPos.getThetaDeg());
}

/**
@brief Korean: 로봇의 회전 운동에 대한 델타 값을 계산한다
@brief English:
*/
bool KuILBPFLocalizerPr::isAccDeltaMovementOver(double dMovement, double dAngle)
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
void KuILBPFLocalizerPr::setDeviation(double dDeviationforTrans, double dDeviationforRotate,double dDeviationforTransRotate )
{
	m_ParticleFilter.setDeviation( dDeviationforTrans,  dDeviationforRotate, dDeviationforTransRotate );
}

/**
@brief Korean: 샘플의 최대 최소 개수를 받아온다
@brief English: set  max and min sample nubers
of range sensor
*/
void KuILBPFLocalizerPr::setSampleNum(int nMaxSample,int nMinSample)
{
	m_ParticleFilter.setSampleNum( nMaxSample,  nMinSample );
}

/**
@brief Korean:  엔코더로만 추정된 로봇의 위치 값을 받아온다
@brief English: 
of range sensor
*/
KuPose KuILBPFLocalizerPr::estimateRobotPosByDeadReckoning(KuPose EncoderDelPos)
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

void KuILBPFLocalizerPr::doThread(void* arg)
{
	KuILBPFLocalizerPr::getInstance()->estimateRobotPosUsingParticleFilter();	
}



void KuILBPFLocalizerPr::terminate()
{
	m_bIsThreadFuncGenerated = false; //스레드가 생성되지 않았다는 플래그
	m_doThreadFunc = false; //스레드 함수가 실행되도록 하는 플래그
	m_KuThread.terminate();
}

void KuILBPFLocalizerPr::saveParameter()
{
	double dWidth,dHeight;
	m_FDetectorAlg.getEllipseData(&dWidth,&dHeight);
	KuRobotParameter::getInstance()->setEllipseHeight(dHeight);
	KuRobotParameter::getInstance()->setEllipseWidth(dWidth);
	KuRobotParameter::getInstance()->setHeight(m_dHeight);
	KuRobotParameter::getInstance()->saveParameter();
}

void KuILBPFLocalizerPr::start(int nPeriod,bool bMapping)
{
	if(false==m_bIsThreadFuncGenerated){
		m_bIsThreadFuncGenerated=true;
		if(bMapping==true)
		{
			m_FRD.clear();
			m_FTD.clear();
		}
		m_KuThread.start(doThread,this, 100, "KuILBPFLocalizerPr::start()");	
	}
}
bool KuILBPFLocalizerPr::getThreadStates()
{
	return m_bIsThreadFuncGenerated;
}

/**
@brief Korean:  로봇 주변에 샘플들을 부려준다
@brief English: 
of range sensor
*/
void KuILBPFLocalizerPr::spreadParticleNearRobot(KuPose RobotPos, double dRegionSize)
{
	m_ParticleFilter.ResetReservation();
	m_ParticleFilter.m_bCalculationStop = true;
	m_ParticleFilter.setSamplesNearRobot(RobotPos.getX(), RobotPos.getY(), RobotPos.getThetaRad(), dRegionSize);
}
/**
@brief Korean:  
@brief English: 
of range sensor
*/
bool KuILBPFLocalizerPr::getParticleConvergeStatus()
{
	return m_bIsParticlConverged;
}

/**
@brief Korean:  
@brief English: 
of range sensor
*/
void KuILBPFLocalizerPr::calLampHeight(KuPose RobotPos, vector<CLAMPData>* FRD)
{
	double dselheightalpha=0.0;
	double dminerrorheight=10000000.0;
	double derrorheight=0.0;
	double derrorheightDist=0.0;
	double dminerrorheightDist=10000000.0;
	double dcnt=0;
	double dmaxcnt=0;
	double dy=0.0 ,dx=0.0;
	vector<CLAMPData> tempFRD ;

	for(int i=0; i<(*FRD).size();i++)
	{
		dminerrorheightDist=10000000.0;
		derrorheightDist=0.0;

		tempFRD.push_back((*FRD)[i]);

		if(tempFRD[i].add==false)
		{
			for(double alpha=-2.4; alpha<2.4;alpha+=0.1)
			{
				if(m_dHeight+alpha<1.5) continue;

				tempFRD[i].nmatchnum=1;
				tempFRD[i].Rx=RobotPos.getXm();
				tempFRD[i].Ry=RobotPos.getYm();
				tempFRD[i].Rt=RobotPos.getThetaRad();	

				float fFx, fFy, fFz, fFpi; // feature pose in global coordinate
				trans_img2global((float)tempFRD[i].Rx, (float)tempFRD[i].Ry, (float)tempFRD[i].Rt,
					tempFRD[i].u, tempFRD[i].v,	fFx, fFy, fFz,
					KuSiriusCameraInterface::getInstance()->getCamContext(),m_dHeight+alpha);

				tempFRD[i].x=fFx;
				tempFRD[i].y=fFy;

				for(int j=0; j<i;j++)
				{
					if((fabs(tempFRD[i].Rx-tempFRD[j].Rx)>0.1||fabs(tempFRD[i].Ry-tempFRD[j].Ry)>0.1)
						&&tempFRD[i].ntype==tempFRD[j].ntype)
					{
						float fFx, fFy, fFz, fFpi; // feature pose in global coordinate

						trans_img2global((float)tempFRD[j].Rx, (float)tempFRD[j].Ry, (float)tempFRD[j].Rt, 
							tempFRD[j].u, tempFRD[j].v, fFx, fFy, fFz,
							KuSiriusCameraInterface::getInstance()->getCamContext(),m_dHeight+alpha);

						derrorheightDist= hypot(fFx-tempFRD[i].x,fFy-tempFRD[i].y);	

						if(derrorheightDist<0.3&&tempFRD[i].add!=tempFRD[j].add
							&&derrorheightDist<dminerrorheightDist)
						{						
							dminerrorheightDist=derrorheightDist;	
							dselheightalpha=alpha;
						}
					}

				}		
			}	
			(*FRD)[i].ceilingheight=m_dHeight+dselheightalpha;
		}
	}

}

/**
@brief Korean:  
@brief English: 
of range sensor
*/
void KuILBPFLocalizerPr::registerLamp( KuPose RobotPos, vector<CLAMPData> *FRD)
{
	double dy=0.0 ,dx=0.0;
	int ndetectLampNum=0;

	for(int i=0; i<(*FRD).size();i++)
	{		
		(*FRD)[i].detect=false;

		if((*FRD)[i].add==false)
		{
			float fFx, fFy, fFz, fFpi; // feature pose in global coordinate

			(*FRD)[i].Rx=RobotPos.getXm();
			(*FRD)[i].Ry=RobotPos.getYm();
			(*FRD)[i].Rt=RobotPos.getThetaRad();

			trans_img2global((float)(*FRD)[i].Rx, (float)(*FRD)[i].Ry, (float)(*FRD)[i].Rt, 
				(*FRD)[i].u, (*FRD)[i].v, fFx, fFy, fFz,
				KuSiriusCameraInterface::getInstance()->getCamContext(),(*FRD)[i].ceilingheight);

			(*FRD)[i].nmatchnum=1;
			(*FRD)[i].x=fFx;
			(*FRD)[i].y=fFy;
			(*FRD)[i].th=(180-(*FRD)[i].angle)+RobotPos.getThetaDeg();

			printf("Angle=%f,R=%f\n",(*FRD)[i].angle,RobotPos.getThetaDeg());
			ndetectLampNum++;
			if ((*FRD)[i].th>180) (*FRD)[i].th -= 2.0*180;
			else if ((*FRD)[i].th<-180) (*FRD)[i].th += 2.0*180;
			if ((*FRD)[i].th<0) (*FRD)[i].th += 180;
		}
		
		(*FRD)[i].add=true;
	
		for(int j=0; j<i;j++)
		{
			if(fabs((*FRD)[i].x-(*FRD)[j].x)<1.0&&fabs((*FRD)[i].y-(*FRD)[j].y)<1.0
				&&(fabs((*FRD)[i].Rx-(*FRD)[j].Rx)>0.1||fabs((*FRD)[i].Ry-(*FRD)[j].Ry)>0.1)
				&&(*FRD)[i].ntype==(*FRD)[j].ntype
				)
			{			
				(*FRD)[j].ceilingheight=((*FRD)[j].ceilingheight*(*FRD)[j].nmatchnum+(*FRD)[i].ceilingheight)/(double)((*FRD)[j].nmatchnum+1);					
			
				float fFx, fFy, fFz, fFpi; // feature pose in global coordinate

				(*FRD)[j].x=((*FRD)[j].x*(*FRD)[j].nmatchnum+(*FRD)[i].x)/(double)((*FRD)[j].nmatchnum+1);
				(*FRD)[j].y=((*FRD)[j].y*(*FRD)[j].nmatchnum+(*FRD)[i].y)/(double)((*FRD)[j].nmatchnum+1);

				if((*FRD)[j].th>150&&(*FRD)[i].th<30)
				{
					(*FRD)[i].th+=180;
				}
				else if((*FRD)[j].th<30&&(*FRD)[i].th>150)
				{
					(*FRD)[i].th-=180;
				}

				(*FRD)[j].th=((*FRD)[j].th*(*FRD)[j].nmatchnum+(*FRD)[i].th)/(double)((*FRD)[j].nmatchnum+1);	
				
				if ((*FRD)[j].th<0) (*FRD)[j].th += 180;
				else if ((*FRD)[j].th>180) (*FRD)[j].th -= 180;
				
				(*FRD)[j].area=((*FRD)[j].area*(*FRD)[j].nmatchnum+(*FRD)[i].area)/(double)((*FRD)[j].nmatchnum+1);	

				int mv, mu;

				trans_global2img((float)(*FRD)[j].Rx, (float)(*FRD)[j].Ry, (float)(*FRD)[j].Rt,
					(*FRD)[j].x,(*FRD)[j].y, (*FRD)[j].ceilingheight, 
					mu,mv, KuSiriusCameraInterface::getInstance()->getCamContext());

				(*FRD)[j].mv=mv;
				(*FRD)[j].mu=mu;

				(*FRD)[j].nmatchnum=(*FRD)[j].nmatchnum+1;

				(*FRD)[j].detect=true;
				(*FRD)[i].add=false;
				break;
			}
			else if(fabs((*FRD)[i].x-(*FRD)[j].x)<1.0&&fabs((*FRD)[i].y-(*FRD)[j].y)<1.0
				&&(*FRD)[i].ntype==(*FRD)[j].ntype)				
			{
				(*FRD)[i].add=false;
			}
		}
	}	

	for(int i=0; i<(*FRD).size();i++)
	{
		if(hypot(RobotPos.getXm()-(*FRD)[i].Rx,RobotPos.getYm()-(*FRD)[i].Ry)>5.0&&(*FRD)[i].nmatchnum<40){
			(*FRD)[i].add=false;
		}
	}
}

/**
@brief Korean: 이중 matching된 결과 제거
@brief English: 
*/
void KuILBPFLocalizerPr::symmetryTest(vector<DMatch>& matches1, vector<DMatch>& matches2, vector<DMatch>& symMatches)
{
	// for all matches image#1 -> image#2
	for(int i=0; i<matches1.size(); i++)
	{
		matches1[i].imgIdx=0;
		// ignore deleted matches
		if(matches1.size()<2)
			continue;

		// for all matches image#2 -> image#1
		for(int j=0; j<matches2.size(); j++)
		{
			// ignore deleted matches
			if(matches2.size() < 2)
				continue;

			// Match symmetry test
			if( ( matches1[i].queryIdx == matches2[j].trainIdx) && ( matches2[j].queryIdx == matches1[i].trainIdx)&&(matches1[i].distance<100&&matches2[j].distance<100))
			{	
				matches1[i].imgIdx=1;
				// add symmetrical match
				symMatches.push_back(DMatch( matches1[i].queryIdx, matches1[i].trainIdx, matches1[i].distance));
				break;		// next match in image#1 -> image#2
			}

		}

	}

}


void KuILBPFLocalizerPr::trans_global2img(const float& fRx, const float& fRy, const float& fRth, const float& fFx, const float& fFy, const float& fFz, int& nu, int& nv, const CamContext* pContext)
{
	
	double fCRth=cos(fRth);
	double fSRth=sin(fRth);
	double a11=fCRth;
	double a12=-fSRth;
	double a14=fRx + pContext->cam_offset_x * fCRth;
	double a21=fSRth;
	double a22=fCRth;
	double a24=fRy+ pContext->cam_offset_x * fSRth;
 	float a33=1.0;
 	float a44=1.0;

	double b11=a11;
	double b12=-a12;
	double b13=0.0;
	double b14=a12*a24-a14*a22;

	double b21=-a21;
	double b22=a11;
	double b23=0.0;
	double b24=a14*a21-a11*a24;

	double b31=0.0;
	double b32=0.0;
	double b33=a11*a22-a12*a21;
	double b34=0.0;
		
	double b41=0.0;
	double b42=0.0;
	double b43=0.0;
	double b44=a11*a22-a12*a21;	
	
	double d=b31*fFx+b32*fFy+b33*fFz;
	double Det=1;//a11*a22-a12*a21;

	nv = 1.0/(fFz*Det)*(pContext->cam_fy*(b11*fFx+b12*fFy+b13*fFz+b14)+pContext->img_center_v*d);
	nu =1.0/(fFz*Det)*(pContext->cam_fx *(b21*fFx+b22*fFy+b23*fFz+b24)+pContext->img_center_u*d);
	
}

void KuILBPFLocalizerPr::trans_img2global(const float& fRx, const float& fRy, const float& fRth, const int& nu, const int& nv, float& fFx, float& fFy, float& fFz, const CamContext* pContext, double dHeight)
{

	double fCRth=cos(fRth);
	double fSRth=sin(fRth);
	double a11=fCRth;
	double a12=-fSRth;
	double a14=fRx + pContext->cam_offset_x * fCRth;
	double a21=fSRth;
	double a22=fCRth;
	double a24=fRy+ pContext->cam_offset_x * fSRth;
	double a33=1.0;
	double a44=1.0;

	double b11=pContext->cam_fx;
	double b13=-pContext->img_center_v*pContext->cam_fx ;
	double b22=pContext->cam_fy;
	double b23= -pContext->cam_fy*pContext->img_center_u;
	double b33=pContext->cam_fx*pContext->cam_fy;

	double Det=pContext->cam_fx*pContext->cam_fy;

	double A1=dHeight/Det*(b11*nv+b13);
	double A2=dHeight/Det*(b22*nu+b23);
	double A3=dHeight/Det*b33;

	fFx = a11*A1+a12*A2+a14;
	fFy = a21*A1+a22*A2+a24;
	fFz = a33*A3;

}
void KuILBPFLocalizerPr::initFREAKData(vector<CFREAKData> *CFData,KuPose RobotPos)
{
	for(int i=0; i<(*CFData).size();i++)
	{
		(*CFData)[i].Rx=RobotPos.getXm();
		(*CFData)[i].Ry=RobotPos.getYm();
		(*CFData)[i].Rt=RobotPos.getThetaRad();
		(*CFData)[i].th=RobotPos.getThetaRad();
		(*CFData)[i].nmatchnum=1;
		(*CFData)[i].similarity=0;
		(*CFData)[i].ceilingheight=m_dHeight;
		(*CFData)[i].add=true;
	}
}
void KuILBPFLocalizerPr::matchFREAKData(vector<DMatch> *matches, vector<CFREAKData> *mergeFTD,
	vector<CFREAKData> *tempFTD,vector<DMatch> *matchesA,vector<CFREAKData> CFData ,KuPose RobotPos )
{
	vector<KeyPoint> keypointsA, keypointsB;
	KeyPoint keypoints;

	Mat descriptorsA, descriptorsB;
	vector<DMatch> tempmatchesA, matchesB;

	BruteForceMatcher<Hamming> matcher;


	for(int i=0; i<m_FTD.size();i++)
	{
		m_FTD[i].detect =false;
		m_FTD[i].add=true;

		if(_hypot(RobotPos.getXm()-m_FTD[i].Rx,RobotPos.getYm()-m_FTD[i].Ry)<3.0
			||_hypot(RobotPos.getXm()-m_FTD[i].x,RobotPos.getYm()-m_FTD[i].y)<5.0) 
		{
			keypoints.angle = m_FTD[i].angle;
			keypoints.response = m_FTD[i].response;
			keypoints.pt.x = m_FTD[i].u;
			keypoints.pt.y = m_FTD[i].v;

			keypointsA.push_back(keypoints);
			descriptorsA.push_back(m_FTD[i].sourceImage);
			(*tempFTD).push_back(m_FTD[i]);		
		}		
		else
		{
			(*mergeFTD).push_back(m_FTD[i]);
		}		
	}

	if(CFData.size()>0&&(*tempFTD).size()>0)
	{
		for(int i=0; i<CFData.size();i++)
		{
			keypoints.angle = CFData[i].angle;
			keypoints.response = CFData[i].response;
			keypoints.pt.x = CFData[i].u;
			keypoints.pt.y = CFData[i].v;

			keypointsB.push_back(keypoints);
			descriptorsB.push_back(CFData[i].sourceImage);
		}

		//	matcher.match(descriptorsA, descriptorsB, matches);

		matcher.match(descriptorsA, descriptorsB, (*matchesA));
		matcher.match(descriptorsB, descriptorsA,  matchesB);
		symmetryTest((*matchesA), matchesB, (*matches));	
	}
}
void KuILBPFLocalizerPr::checkAssociateFREAKData(vector<DMatch> *matches, vector<CFREAKData> *mergeFTD,
	vector<CFREAKData> *tempFTD,vector<DMatch> *matchesA,vector<CFREAKData> CFData ,KuPose RobotPos )
{
	for(int h=0; h<(*tempFTD).size();h++)
	{
		(*tempFTD)[h].associate=false;
	}

	for(int m=0;(*matchesA).size()>m;m++)
	{
		if((*matchesA)[m].imgIdx==1)
			(*tempFTD)[(*matchesA)[m].queryIdx].associate=true;
	}

	for(int h=0; h<(*tempFTD).size();h++)
	{
		if((*tempFTD)[h].associate==false)
			(*mergeFTD).push_back((*tempFTD)[h]);
	}	

	for(int h=0; h<CFData.size();h++)
	{
		CFData[h].associate=false;
	}

	for(int m=0;(*matchesA).size()>m;m++)
	{
		if((*matchesA)[m].imgIdx==1)
			CFData[(*matchesA)[m].trainIdx].associate=true;
	}	

	for(int h=0; h<CFData.size();h++)
	{
		if(CFData[h].associate==false)
			(*mergeFTD).push_back(CFData[h]);
	}

}

void KuILBPFLocalizerPr::calFREAKHeight(int nIDXA,int nIDXB,KuPose RobotPos,double dLampH, vector<CFREAKData> *tempFTD,vector<CFREAKData> *CFData)
{
	double dminDist=10000000;

	for(double alpha=-2.4; alpha<10.0;alpha+=0.1)
	{
		if(m_dHeight+alpha<dLampH*0.9) continue;

		float fFx, fFy, fFz, fFpi; // feature pose in global coordinate
		float fDFx, fDFy, fDFz, fDFpi; // feature pose in global coordinate


		trans_img2global((float)(*tempFTD)[nIDXA].Rx, (float)(*tempFTD)[nIDXA].Ry, (float)(*tempFTD)[nIDXA].Rt, 
			(*tempFTD)[nIDXA].u, (*tempFTD)[nIDXA].v, fFx, fFy, fFz,KuSiriusCameraInterface::getInstance()->getCamContext(),m_dHeight+alpha);

		trans_img2global((float)(*CFData)[nIDXB].Rx, (float)(*CFData)[nIDXB].Ry, (float)(*CFData)[nIDXB].Rt, 
			(*CFData)[nIDXB].u, (*CFData)[nIDXB].v, fDFx, fDFy, fDFz,KuSiriusCameraInterface::getInstance()->getCamContext(),m_dHeight+alpha);


		double dDist=_hypot(fFx-fDFx,fFy-fDFy);

		if(dDist<dminDist&&dDist<0.3)
		{
			dminDist=dDist;
			(*CFData)[nIDXB].ceilingheight=alpha+m_dHeight;
			(*tempFTD)[nIDXA].detect=true;
		}
	}

}
bool KuILBPFLocalizerPr::exceptionmatchhandling(int nIDXA, int nIDXB, vector<CFREAKData>tempFTD,vector<CFREAKData>CFData,vector<CFREAKData> *mergeFTD)
{
	float fFx, fFy, fFz, fFpi; // feature pose in global coordinate
	float fDFx, fDFy, fDFz, fDFpi; // feature pose in global coordinate

	trans_img2global((float)tempFTD[nIDXA].Rx, (float)tempFTD[nIDXA].Ry, (float)tempFTD[nIDXA].Rt, 
		tempFTD[nIDXA].u, tempFTD[nIDXA].v, fFx, fFy, fFz,
		KuSiriusCameraInterface::getInstance()->getCamContext(),m_dHeight);

	trans_img2global((float)CFData[nIDXB].Rx, (float)CFData[nIDXB].Ry, (float)CFData[nIDXB].Rt,
		CFData[nIDXB].u, CFData[nIDXB].v, fDFx, fDFy, fDFz,
		KuSiriusCameraInterface::getInstance()->getCamContext(),m_dHeight);

	if(_hypot(fFx-fDFx,fFy-fDFy)>0.5){
		 (*mergeFTD).push_back(tempFTD[nIDXA]);
		return false;
	}

	int nu,nv;

	if(_hypot((double)(tempFTD[nIDXA].Rx-CFData[nIDXB].Rx),(double)(tempFTD[nIDXA].Ry-CFData[nIDXB].Ry))>0.1)
	{
		trans_global2img((float)CFData[nIDXB].Rx, (float)CFData[nIDXB].Ry, (float)CFData[nIDXB].Rt,
			tempFTD[nIDXA].x,  tempFTD[nIDXA].y, tempFTD[nIDXA].ceilingheight,nu,nv,
			KuSiriusCameraInterface::getInstance()->getCamContext());

		double dImageD=_hypot(nu-CFData[nIDXB].u,nv-CFData[nIDXB].v);

		if(dImageD>50){
			(*mergeFTD).push_back(tempFTD[nIDXA]);
			return false;;
		}
	}
	return true;

}
void KuILBPFLocalizerPr::eliminateFREAKData(KuPose RobotPos,vector<CFREAKData>*mergeFTD)
{
	for(int i=0; i<(*mergeFTD).size();i++)
	{
		(*mergeFTD)[i].add=true;
		if(hypot(RobotPos.getXm()-(*mergeFTD)[i].Rx,RobotPos.getYm()-(*mergeFTD)[i].Ry)>3.0
			&&(*mergeFTD)[i].nmatchnum<30){
				(*mergeFTD)[i].add=false;
		}
	}

	for(int i=0; i<(*mergeFTD).size();i++)
	{
		for(int j=0; j<i;j++)
		{	
			double dDist= _hypot((*mergeFTD)[i].x-(*mergeFTD)[j].x,(*mergeFTD)[i].y-(*mergeFTD)[j].y);
			if(dDist<0.3)
			{			
				if((*mergeFTD)[i].nmatchnum>(*mergeFTD)[j].nmatchnum&&(*mergeFTD)[i].add!=false)
				{
					(*mergeFTD)[j].add=false;
				}
				else if((*mergeFTD)[i].nmatchnum<(*mergeFTD)[j].nmatchnum&&(*mergeFTD)[j].add!=false)
				{
					(*mergeFTD)[i].add=false;
				}
			}
		}
	}
}
void KuILBPFLocalizerPr::updateFREAKData(int nIDXA, int nIDXB, vector<CFREAKData>tempFTD,vector<CFREAKData> CFData,vector<CFREAKData>*mergeFTD)
{
	float fFx(0.0), fFy(0.0), fFz(0.0); // feature pose in global coordinate

	tempFTD[nIDXA].ceilingheight=(tempFTD[nIDXA].ceilingheight*tempFTD[nIDXA].nmatchnum+CFData[nIDXB].ceilingheight)/(double)(tempFTD[nIDXA].nmatchnum+1);
	trans_img2global((float)tempFTD[nIDXA].Rx, (float)tempFTD[nIDXA].Ry, (float)tempFTD[nIDXA].Rt, tempFTD[nIDXA].u, tempFTD[nIDXA].v, fFx, fFy, fFz,KuSiriusCameraInterface::getInstance()->getCamContext(),tempFTD[nIDXA].ceilingheight);
	tempFTD[nIDXA].x=fFx;
	tempFTD[nIDXA].y=fFy;
	tempFTD[nIDXA].th=(tempFTD[nIDXA].th*tempFTD[nIDXA].nmatchnum+CFData[nIDXB].th)/(double)(tempFTD[nIDXA].nmatchnum+1);	
	tempFTD[nIDXA].nmatchnum=tempFTD[nIDXA].nmatchnum+1;
	(*mergeFTD).push_back(tempFTD[nIDXA]);
}


/**
@brief Korean:  파티클 필터를 이용하여 로봇의 위치를 추정한다
@brief English: 
of range sensor
*/
void KuILBPFLocalizerPr::estimateRobotPosUsingParticleFilter()
{
	if(m_bfirst == false && m_bIsNewSensorData == false){
		return ;
	} 

	KuPose dDelEncoderPos;
	int nLaserData[181];
	double dDelEncoderData[3];
	double dSamplePos[3];
	double dLampH=0.0;
	double dLampNM=0;
	int nLampNUM=0;

	if(m_bMapping==true)
	{
		LARGE_INTEGER t;
		startTimeCheck(t);

		computeAccumulatedDeltaMovement(m_dDelEncoderData);
		
		if(m_dAccumulatedDeltaMovement<100)
		{
			m_bsaveFR=false;
			m_bsaveFT=false;
			m_dAccumulatedDeltaMovement=0;
			m_FDetectorAlg.trainingRegion(m_cvMatImage);
			return;
		}
			
		m_CriticalSection.Lock();
		for(int i = 0; i < Sensor::CEILING_IMAGE_WIDTH * Sensor::CEILING_IMAGE_HEIGHT; i++)
		{
			m_cvMatImageT.data[i]=m_cvMatImage.data[i];
			m_cvMatImageBright.data[i]=255;
			if(m_cvMatImage.data[i]<225){
				m_cvMatImageBright.data[i]=0;
			}	
		}	
		KuPose RobotPos=m_RobotPos;
		m_CriticalSection.Unlock();

		vector<CLAMPData> FRD = m_FDetectorAlg.detectRegion(m_cvMatImageBright,true);	
		nLampNUM=FRD.size();
		calLampHeight( RobotPos, &FRD);
		registerLamp(RobotPos, &FRD);

		m_FDetectorAlg.setRegion(FRD);	

		if(!m_bsaveFR)
		{
			m_CriticalSection.Lock();
			m_FRD.clear();
			for(int i=0; i<FRD.size();i++)
			{
				if(FRD[i].add==true)
				{
					m_FRD.push_back(FRD[i]);
					dLampH+=FRD[i].ceilingheight*FRD[i].nmatchnum;
					dLampNM+=FRD[i].nmatchnum;
				}
			}
			m_CriticalSection.Unlock();
		}
		
		
		dLampH=dLampH/dLampNM;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		
// 		Mat PImage;	 
// 		cvtColor(m_cvMatImageT, PImage, CV_GRAY2BGR);	

		vector<CFREAKData> tempFTD ;
		vector<CFREAKData> mergeFTD ;

		KeyPoint keypoints;
		vector<KeyPoint> veckeypoints;

		vector<DMatch> matches;
		vector<DMatch> matchesA;


		if(_hypot(RobotPos.getX()-m_PreRobotpos.getX(),RobotPos.getY()-m_PreRobotpos.getX())>100
			||fabs(RobotPos.getThetaDeg()-m_PreRobotpos.getThetaDeg())>10)
		{
			m_PreRobotpos=RobotPos;		

			vector<CFREAKData> CFData = m_FDetectorAlg.checktempletRegion(m_cvMatImageT,true);
		
			initFREAKData(&CFData,RobotPos);

			matchFREAKData(&matches, &mergeFTD,&tempFTD,&matchesA,CFData,RobotPos );
		
			if(matches.size()>0)
			{
				checkAssociateFREAKData(&matches, &mergeFTD,&tempFTD,&matchesA,CFData,RobotPos );				

				for(int m=0;matches.size()>m;m++)
				{
					int nIDXB=matches[m].trainIdx;
					int nIDXA=matches[m].queryIdx;

					if(!exceptionmatchhandling( nIDXA,  nIDXB, tempFTD,CFData,&mergeFTD))
					{
						continue;
					}
					
					if(matches[m].distance<75)
					{
						calFREAKHeight(nIDXA,nIDXB, RobotPos, dLampH, &tempFTD, &CFData);
						
						if(tempFTD[nIDXA].detect==true)
						{
							updateFREAKData(nIDXA,nIDXB,tempFTD,CFData, &mergeFTD);
							
							//그리기------
// 							keypoints.angle = CFData[nIDXB].angle;
// 							keypoints.response = CFData[nIDXB].response;
// 							keypoints.pt.x = CFData[nIDXB].u;
// 							keypoints.pt.y = CFData[nIDXB].v;
// 							keypoints.size = 7.0;
// 							veckeypoints.push_back(keypoints);
// 
// 							int nColor=tempFTD[nIDXA].nmatchnum;
// 							if(nColor>255)nColor=255;
// 	
// 							line(PImage, Point(CFData[nIDXB].u, CFData[nIDXB].v), Point(tempFTD[nIDXA].u, tempFTD[nIDXA].v), Scalar(0, 0, nColor), 2, 8);
							//그리기====
						}
						else
						{
							mergeFTD.push_back(tempFTD[nIDXA]);
						}
					}
					else
					{
						mergeFTD.push_back(tempFTD[nIDXA]);
					}
				}
			}
			else
			{
				for(int k=0;k<tempFTD.size();k++)
				{
					mergeFTD.push_back(tempFTD[k]);
				}
				if(CFData.size()>0)
				{
					for(int k=0;k<CFData.size();k++)
					{
						mergeFTD.push_back(CFData[k]);
					}
				}
			}	
	
// 			drawKeypoints(PImage,veckeypoints,PImage,Scalar(0,255,0),DrawMatchesFlags::DRAW_RICH_KEYPOINTS);	 
// 			imshow("FREAK FeaturesA",PImage);
// 			cvWaitKey(10);

			eliminateFREAKData( RobotPos,&mergeFTD);			


			if(!m_bsaveFT)
			{
				m_CriticalSection.Lock();
				m_FTD.clear();
				for(int i=0; i<mergeFTD.size();i++)
				{
					if(mergeFTD[i].add==true)
						m_FTD.push_back(mergeFTD[i]);
				}
				m_CriticalSection.Unlock();
			}

		}

		double dtimePoint = finishTimeCheck(t);
		printf("T:%0.2f \n",dtimePoint);

	}
	else
	{
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

		m_CriticalSection.Lock();
		m_ParticleFilter.copyCeilingImage(m_cvMatImage);		
		m_CriticalSection.Unlock();

		//TRACE("랜드 마크 인식\n"); 		
		m_ParticleFilter.setEncoderData(dDelEncoderData);
		m_ParticleFilter.setRangeData(nLaserData);

		m_nPaticleFilterState = m_ParticleFilter.getSamplePos(dSamplePos , m_dEstimatedRangeData);    
		vector<CLAMPData> FRD;vector<CFREAKData> FTD;
		m_ParticleFilter.getFeature(&FRD,&FTD);
		KuDrawingInfo::getInstance()->setRegion(FRD);
		KuDrawingInfo::getInstance()->setTemplateData(FTD);

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
	}

}

/**
@brief Korean: 소요 시간을 측정하기 위해서 초기화하는 함수
@brief English: Initializes to count the duration
*/
void KuILBPFLocalizerPr::startTimeCheck(LARGE_INTEGER& nStart)
{
	QueryPerformanceCounter(&nStart);
}
/**
@brief Korean: 측정된 소요 시간을 받아오는 함수
@brief English: Gets the estimated duration
*/
float KuILBPFLocalizerPr::finishTimeCheck(const LARGE_INTEGER nStart)
{
	LARGE_INTEGER nFinish, freq;

	QueryPerformanceFrequency(&freq);

	QueryPerformanceCounter(&nFinish);

	return (float)(1000.0 * (nFinish.QuadPart - nStart.QuadPart) / freq.QuadPart);
}



void KuILBPFLocalizerPr::saveFeatureData()
{
	m_bsaveFR=true;
	char cFilePathName[150];
	memset(cFilePathName,0,sizeof(cFilePathName));
	sprintf_s(cFilePathName,"./Data/FeatureData/FeatureData.log");
	ofstream DataLog;
	DataLog.open(cFilePathName);
	int i=0;
	int nCnt(0);

	for(i=0; i<m_FRD.size() ; i++)
	{
		if(m_FRD[i].add==true&&m_FRD[i].nmatchnum>20)
		{
			nCnt++;
		}
	}

	DataLog << nCnt << endl;

	for(i=0; i<m_FRD.size() ; i++){
		if(m_FRD[i].add==true&&m_FRD[i].nmatchnum>20)
		{
			//m_FRD[i].angle=0.0;
			//m_FRD[i].th=m_FRD[i].th-M_PI;
			DataLog<<m_FRD[i].ntype<<" ";
			DataLog<<m_FRD[i].u<<" ";
			DataLog<<m_FRD[i].v<<" ";
			DataLog<<m_FRD[i].mu<<" ";
			DataLog<<m_FRD[i].mv<<" ";
			DataLog<<m_FRD[i].x<<" ";
			DataLog<<m_FRD[i].y<<" ";
			DataLog<<m_FRD[i].Rx<<" ";
			DataLog<<m_FRD[i].Ry<<" ";
			DataLog<<m_FRD[i].Rt<<" ";
			DataLog<<m_FRD[i].area<<" ";
			DataLog<<m_FRD[i].th<<" ";
			DataLog<<m_FRD[i].angle<<" ";
			DataLog<<m_FRD[i].width<<" ";
			DataLog<<m_FRD[i].height<<" ";
			DataLog<<m_FRD[i].nmatchnum<<" ";
			DataLog<<m_FRD[i].ceilingheight<<" ";
			DataLog<<i<<" ";
			DataLog<<m_FRD[i].add<<" ";
			DataLog<<" "<<endl;
		}
	}

	DataLog.close();	

	m_bsaveFR=false;

}

/**
 @brief Korean: 
 @brief English: 
*/
vector<CLAMPData> KuILBPFLocalizerPr::loadFeatureData( )
{
	char cFilePathName[150];
	memset(cFilePathName,0,sizeof(cFilePathName));
	sprintf_s(cFilePathName,"./Data/FeatureData/FeatureData.log");	
	
	int u, v; // center points
	int mu, mv; // center points
	double x, y; // center points
	int area; // area of blob
	double th; // polar coordinate
	float angle; // polar coordinate
	double width, height; // center points
	int idx;
	bool add;
	double ceilingheight;
	int nmatchnum;
	double dRth,dRx,dRy;
	CLAMPData FRData;
	int ntype;
	
	ifstream DataLog;
	DataLog.open(cFilePathName);
	m_FRD.clear();

	if( !DataLog.is_open() )
	{
		return m_FRD;
	}

	int nCnt;
	DataLog >> nCnt;

	if(nCnt != 0)
	{
		while(!DataLog.eof()){
			DataLog >> ntype >> u >> v >> mu >> mv;		
			DataLog>> x >> y>>dRx>>dRy>>dRth;
			if(u<0||v<0){break;/*return m_FRD;*/}
			DataLog >> area >> th >> angle;
			DataLog >> width >> height >>nmatchnum>>ceilingheight>> idx >> add;

			FRData.ntype=ntype;
			FRData.u=u;
			FRData.v=v;
			FRData.mu=mu;
			FRData.mv=mv;
			FRData.x=x;
			FRData.y=y;
			FRData.Rx=dRx;
			FRData.Ry=dRy;
			FRData.Rt=dRth;
			FRData.area=area;
			FRData.th=th;
			FRData.angle=angle;
			FRData.width=width;
			FRData.height=height;
			FRData.nmatchnum=nmatchnum;
			FRData.ceilingheight=ceilingheight;
			FRData.idx=idx;
			FRData.add=add;
			FRData.detect=false;
			m_FRD.push_back(FRData);
		}
	}

	if(m_FRD.size()>1) m_FRD.pop_back();

	DataLog.close();
	m_ParticleFilter.setRegion(m_FRD);
	return m_FRD;
}

void KuILBPFLocalizerPr::saveTemplateData()
{
	m_bsaveFT=true;

	char cFilePathName[150];
	memset(cFilePathName,0,sizeof(cFilePathName));
	sprintf_s(cFilePathName,"./Data/FeatureData/TemplateData.log");

	ofstream DataLog;
	DataLog.open(cFilePathName, ios::out);
	int i=0;
	int nCnt(0);

	for(i = 0; i < m_FTD.size(); i++)
	{
		if(m_FTD[i].add == true && m_FTD[i].nmatchnum > 30)
		{
			nCnt++;
		}
	}

	DataLog << nCnt << endl; // size

	for(i = 0; i < m_FTD.size(); i++){
		if(m_FTD[i].add == true && m_FTD[i].nmatchnum > 30)
		{
			DataLog<<m_FTD[i].u<<" ";
			DataLog<<m_FTD[i].v<<" ";
			DataLog<<m_FTD[i].x<<" ";
			DataLog<<m_FTD[i].y<<" ";
			DataLog<<m_FTD[i].th<<" ";
			DataLog<<m_FTD[i].Rx<<" ";
			DataLog<<m_FTD[i].Ry<<" ";
			DataLog<<m_FTD[i].Rt<<" ";
			DataLog<<m_FTD[i].angle<<" ";
			DataLog<<i<<" ";
			DataLog<<m_FTD[i].ceilingheight<<" ";
			DataLog<<m_FTD[i].similarity<<" ";
			DataLog<<m_FTD[i].nmatchnum<<" ";
			DataLog<<m_FTD[i].imageSizeX<<" ";
			DataLog<<m_FTD[i].imageSizeY<<" ";
			DataLog<<m_FTD[i].sourceImage<<" ";
			DataLog<<" "<<endl;
		}
	}

	DataLog.close();

	m_bsaveFT=false;

}

/**
 @brief Korean: 
 @brief English: 
*/
vector<CFREAKData> KuILBPFLocalizerPr::loadTemplateData()
{
	char cFilePathName[150];
	memset(cFilePathName,0,sizeof(cFilePathName));
	sprintf_s(cFilePathName,"./Data/FeatureData/TemplateData.log");	
	
	int u, v; // center points
	double x, y; // center points
	double th; // polar coordinate
	double Rx, Ry, Rt; // center points

	float angle; // polar coordinate
	int idx;
	bool add;
	int MatchNum;
	int imageSizeX;
	int imageSizeY;
	Mat sourceImage;
	CFREAKData FTData;
	int nData;
	char chT;
	double dceilingheight;
	double dsimilarity;	
	int nmatchnum;

	ifstream DataLog;
	DataLog.open(cFilePathName);
	m_FTD.clear();

	if( !DataLog.is_open() )
	{
		return m_FTD;
	}

	int nCnt;
	DataLog >> nCnt;

	if(nCnt != 0)
	{
		while(!DataLog.eof()){
			DataLog >> u >> v >> x >> y;

			if(u<0||v<0)
			{
				break;
			}
			DataLog >> th >> Rx >> Ry >> Rt >>angle >> idx;
			DataLog >> dceilingheight >>dsimilarity>> nmatchnum >> imageSizeX>>imageSizeY; 
			sourceImage.create(imageSizeX,imageSizeY,CV_8UC1);//X,Y 방향 주의!
			DataLog >> chT;	
	

			for(int i = 0; i < imageSizeX*imageSizeY; i++)
			{
				DataLog >> nData;
				sourceImage.data[i]=nData;
				DataLog >> chT;
			}
			//flip(sourceImage,sourceImage,1);
			//flip(sourceImage,sourceImage,0);

			FTData.u=u;
			FTData.v=v;
			FTData.x=x;
			FTData.y=y;
			FTData.th=th;
			FTData.Rx=Rx;
			FTData.Ry=Ry;
			FTData.Rt=Rt;
			FTData.angle=angle;
			FTData.idx=idx;
			FTData.ceilingheight=dceilingheight;
			FTData.similarity=dsimilarity;
			FTData.nmatchnum=nmatchnum;
			//FTData.MatchNum=MatchNum;
			FTData.imageSizeX=imageSizeX;
			FTData.imageSizeY=imageSizeY;
			FTData.sourceImage=sourceImage;
			FTData.detect=false;

			sourceImage.release();

			m_FTD.push_back(FTData);
		}
	}

	if(m_FTD.size()>1) m_FTD.pop_back();

	DataLog.close();
	m_ParticleFilter.setTemplate(m_FTD);
	return m_FTD;
}



inline int KuILBPFLocalizerPr::max(int a, int b)
{
	return a > b ? a : b; 
}

inline int KuILBPFLocalizerPr::min(int a, int b)
{
	return a < b ? a : b; 
}

/**
@brief Korean: 지정된 영역에 샘플을 골고루 뿌리고 확률값을 초기화시킴.
@brief English: distribute samples uniformly on predefined region with normalized probabilities.
*/
void KuILBPFLocalizerPr::setSamples()
{
	m_ParticleFilter.setSamples();
	m_ParticleFilter.ResetReservation();
	m_ParticleFilter.m_bCalculationStop = true;
}

vector<CLAMPData>  KuILBPFLocalizerPr::getRegion()
{
	vector<CLAMPData> FRD;

	m_CriticalSection.Lock();
	for(int i=0; i<m_FRD.size();i++)
	{
		FRD.push_back(m_FRD[i]);
	}
	m_CriticalSection.Unlock();

	return FRD;
}

vector<CFREAKData> KuILBPFLocalizerPr::getTemplateData()
{

	vector<CFREAKData> FTD;
	m_CriticalSection.Lock();
	for(int i=0; i<m_FTD.size();i++)
	{
		FTD.push_back(m_FTD[i]);
	}
	m_CriticalSection.Unlock();


	return FTD;
}

bool  KuILBPFLocalizerPr::checkdoMapping()
{
	double dSampleSD=1000000;
	double dSampleSDForRotation=1000000;
	double dVisionWeight=1000000;

	bool bcheckflag=false;
	m_ParticleFilter.getSampleSD(&dSampleSD,&dSampleSDForRotation,&dVisionWeight);
	printf("----------%f---------\n",dVisionWeight);
	if(dSampleSD<50&&dSampleSDForRotation<1.0&&dVisionWeight>0.7)
	{
		bcheckflag=true;
	}

	return  bcheckflag;
}
/**
@brief Korean: 파티클 필터를 이용하여 로봇의 위치를 추정한다 .
@brief English: 
*/
KuPose KuILBPFLocalizerPr::estimateRobotPos(int_1DArray nRangeData, KuPose RobotPos,KuPose EncoderDelPos, IplImage * IplCeilingImage,bool bMapping )
{
	m_bMapping=bMapping;
	copyRangeData(nRangeData); //거리센서 데이터를 복사해준다. 
	copyEncoderData(EncoderDelPos); //엔코더 데이터를 복사해준다. 
	copyCeilingImage( IplCeilingImage);
		
	if(m_bMapping)
	{
		// pose update 
		m_RobotPos.setX(RobotPos.getX() );
		m_RobotPos.setY(RobotPos.getY() );
		m_RobotPos.setThetaDeg( RobotPos.getThetaDeg());
	}
	else
	{
		double dX = m_RobotPos.getX() + EncoderDelPos.getX() * cos(m_RobotPos.getThetaRad()) + 
			EncoderDelPos.getY() * sin(-m_RobotPos.getThetaRad());

		double dY = m_RobotPos.getY() + EncoderDelPos.getX() * sin(m_RobotPos.getThetaRad()) + 
			EncoderDelPos.getY() * cos(m_RobotPos.getThetaRad());
		double dThetaDeg = m_RobotPos.getThetaDeg() + EncoderDelPos.getThetaDeg();

		m_RobotPos.setX(dX);
		m_RobotPos.setY(dY);
		m_RobotPos.setThetaDeg(dThetaDeg);

	}
	m_bIsNewSensorData = true;


	return m_RobotPos;

}
/**
@brief Korean: 천장 이미지를 저장한다.
@brief English: 
*/
void KuILBPFLocalizerPr::copyCeilingImage(IplImage * IplCeilingImage)
{
	if(IplCeilingImage==NULL)
		return;

	IplImage* IplEqualizeCeilingImage;
	IplEqualizeCeilingImage = cvCreateImage(cvSize(Sensor::CEILING_IMAGE_WIDTH,Sensor::CEILING_IMAGE_HEIGHT),8,1);

	m_CriticalSection.Lock();
	cvFlip(IplCeilingImage,IplEqualizeCeilingImage,-1);
	m_CriticalSection.Unlock();

	m_CriticalSection.Lock();
	for(int i = 0; i < Sensor::CEILING_IMAGE_WIDTH * Sensor::CEILING_IMAGE_HEIGHT; i++)
	{
		m_cvMatImage.data[i]=IplEqualizeCeilingImage->imageData[i];
	}
	m_CriticalSection.Unlock();

	cvReleaseImage(&IplEqualizeCeilingImage);

}
