#include <stdafx.h>
#include "KuFiducialbasedLocalizerPr.h"


KuFiducialbasedLocalizerPr::KuFiducialbasedLocalizerPr()
{
	m_dAccumulatedDeltaMovement = 0.0;
	m_dAccumulatedDeltaAngle = 0.0;

	m_RobotPos.init(); //로봇 위치 초기화
	m_dDelEncoderData.init();

	m_doThreadFunc = false; //스레드가 생성되지 않았다는 플래그
	m_bIsThreadFuncGenerated= false;
	m_nThreadFuncPeriod = 0; //스레드 함수 실행 주기..단위는ms
	m_bMapping=false;
	m_pIplCeilingGrayImage = cvCreateImage(cvSize(Sensor::CEILING_IMAGE_WIDTH,Sensor::CEILING_IMAGE_HEIGHT),8,1);
	m_binitflag=false;
	m_vecFiducialMark.clear();
	
}

KuFiducialbasedLocalizerPr::~KuFiducialbasedLocalizerPr()
{
	cvReleaseImage(&m_pIplCeilingGrayImage);

}
/**
@brief Korean: 로봇 위치를 초기화해주는 함수
@brief English:
*/
void KuFiducialbasedLocalizerPr::init()
{
	m_RobotPos.init(); //로봇 위치 초기화
	m_vecFiducialMark.clear();

}

/**
@brief Korean: 로봇 위치를 받아오는 함수
@brief English:
*/
void KuFiducialbasedLocalizerPr::setRobotPos(KuPose RobotPos,bool bReliableflag)
{
	m_RobotPos = RobotPos;
}

/**
@brief Korean:  로봇의 X좌표를 설정한다
@brief English: 
*/
void KuFiducialbasedLocalizerPr::setRobotPosX(double dPoseX)
{
	m_RobotPos.setX(dPoseX);
}
/**
@brief Korean:  로봇의 Y좌표를 설정한다
@brief English: 
*/
void KuFiducialbasedLocalizerPr::setRobotPosY(double dPoseY)
{
	m_RobotPos.setY(dPoseY);
}
/**
@brief Korean:  로봇의 각도 Degree를 설정한다
@brief English: 
*/
void KuFiducialbasedLocalizerPr::setRobotPosDeg(double dPoseDeg)
{
	m_RobotPos.setThetaDeg(dPoseDeg);
}
/**
@brief Korean:  로봇의 각도 Radian를 설정한다
@brief English: 
*/
void KuFiducialbasedLocalizerPr::setRobotPosRad(double dPoseRad)
{
	m_RobotPos.setThetaRad(dPoseRad);
}
/**
@brief Korean:  로봇의 X좌표를 가져 간다
@brief English:
*/
double KuFiducialbasedLocalizerPr::getRobotPosX()
{
	return m_RobotPos.getX();
}
/**
@brief Korean:  로봇의 Y좌표를 가져 간다
@brief English:
*/
double KuFiducialbasedLocalizerPr::getRobotPosY()
{
	return m_RobotPos.getY();
}
/**
@brief Korean:  로봇의 각도(Degree)를 가져 간다
@brief English:
*/
double KuFiducialbasedLocalizerPr::getRobotPosDeg()
{
	return m_RobotPos.getThetaDeg();
}
/**
@brief Korean:  로봇의 각도(Radian)를 가져 간다
@brief English:
*/
double KuFiducialbasedLocalizerPr::getRobotPosRad()
{
	return m_RobotPos.getThetaRad();
}
/**
@brief Korean:  로봇의 위치 값을 가져 간다
@brief English:
*/
KuPose KuFiducialbasedLocalizerPr::getRobotPos()
{
	return m_RobotPos;//m_RobotPos;
}
/**
@brief Korean: 엔코더 정보를 전역 변수로 복사하는 함수
@brief English:
*/
void KuFiducialbasedLocalizerPr::copyEncoderData(KuPose delEncoderData)
{
	m_CriticalSection.Lock();
	m_dDelEncoderData.setX(m_dDelEncoderData.getX()+delEncoderData.getX());
	m_dDelEncoderData.setY(m_dDelEncoderData.getY()+delEncoderData.getY());
	m_dDelEncoderData.setThetaDeg(m_dDelEncoderData.getThetaDeg()+delEncoderData.getThetaDeg());
	m_CriticalSection.Unlock();
}

/**
@brief Korean: 로봇의 병진 운동에 대한 변위 값을 계산한다
@brief English:
*/
void KuFiducialbasedLocalizerPr::computeAccumulatedDeltaMovement(KuPose EncoderDelPos)
{
	m_dAccumulatedDeltaMovement += sqrt(EncoderDelPos.getX()*EncoderDelPos.getX() + EncoderDelPos.getY()*EncoderDelPos.getY());
	m_dAccumulatedDeltaAngle += fabs(EncoderDelPos.getThetaDeg());
}
/**
@brief Korean: 로봇의 회전 운동에 대한 변위 값을 계산한다
@brief English:
*/
bool KuFiducialbasedLocalizerPr::isAccDeltaMovementOver(double dMovement, double dAngle)
{
	if (m_dAccumulatedDeltaMovement > dMovement || m_dAccumulatedDeltaAngle > dAngle) {
		m_dAccumulatedDeltaMovement = 0.0;
		m_dAccumulatedDeltaAngle = 0.0;
		return true;
	}
	else return false;
}

/**
@brief Korean: 쓰레드 함수
@brief English:
*/
void KuFiducialbasedLocalizerPr::doThread(void* arg)
{
	KuFiducialbasedLocalizerPr* pSLP = (KuFiducialbasedLocalizerPr*)arg;
	
	if(pSLP->m_binitflag==true)	pSLP->estimateRobotPosbyFiducialmark();	
}

/**
@brief Korean: 쓰레드 함수를 종료시키는 함수
@brief English:
*/
void KuFiducialbasedLocalizerPr::terminate()
{
	m_KuThread.terminate();
	m_bIsThreadFuncGenerated = false; //스레드가 생성되지 않았다는 플래그

}

/**
@brief Korean: 쓰레드 함수를 실행시키는 함수
@brief English:
*/
void KuFiducialbasedLocalizerPr::start(int nPeriod )
{
	if(false==m_bIsThreadFuncGenerated){
		m_bIsThreadFuncGenerated=true;
		m_KuThread.start(doThread,this, 100, "KuFiducialbasedLocalizerPr::start");		
	}
}
bool KuFiducialbasedLocalizerPr::getThreadStates()
{
	return m_bIsThreadFuncGenerated;
}

/**
@brief Korean:  현재 로봇의 위치와 천장영상을 이용하여 fidutial mark 검출 및 등록 또는 로봇의 위치를 추정하는 함수 
bMapping: fiducial mark를 검출 하여 등록할 것인가? 로봇의 위치를 추정할 것인가 결정하는 변수
@brief English:
*/
KuPose KuFiducialbasedLocalizerPr::estimateRobotPosbyFiducialmark(KuPose RobotPos,IplImage* IplCeilingImage, bool bMapping)
{
	m_binitflag=true;
	m_CriticalSection.Lock();	
	cvCopy(IplCeilingImage, m_pIplCeilingGrayImage);
	m_RobotPos=RobotPos;
	m_bMapping=bMapping;
	m_CriticalSection.Unlock();

	return m_RobotPosbyFidicial;
}

/**
@brief Korean: 지역 좌표계를 전역 좌표계로 바꾸어주는 함수
@brief English:
*/
KuPose KuFiducialbasedLocalizerPr::transferLocalPos2GlobalPos(KuPose FiducialLandMarkPos)
{
	KuPose estimateGlobalRobotPos;

	//------------------------------------------------------------------------
	bool bNewFeature = true;
	list<KuPose>::iterator it;
	for(it = m_FiducialMarkDBList.begin(); it!= m_FiducialMarkDBList.end(); it++){
		if(it->getID() == FiducialLandMarkPos.getID()){
			double dThetaRad = atan2(FiducialLandMarkPos.getY(),FiducialLandMarkPos.getX());
			double dDeg = dThetaRad*R2D;
			if(dThetaRad > M_PI) dThetaRad -= 2*M_PI;
			if(dThetaRad < -M_PI) dThetaRad += 2*M_PI;

			estimateGlobalRobotPos.setID(it->getID() );
			//double aa = FiducialLandMarkPos.getX();
			//double bb = FiducialLandMarkPos.getY();
			//double cc = it->getX();
			//double dd = it->getY();
			double dX = FiducialLandMarkPos.getX()*cos(it->getThetaRad()-FiducialLandMarkPos.getThetaRad()) - FiducialLandMarkPos.getY()*sin(it->getThetaRad()-FiducialLandMarkPos.getThetaRad());
			double dY = FiducialLandMarkPos.getX()*sin(it->getThetaRad()-FiducialLandMarkPos.getThetaRad()) + FiducialLandMarkPos.getY()*cos(it->getThetaRad()-FiducialLandMarkPos.getThetaRad());

			estimateGlobalRobotPos.setX(it->getX() - dX);
			estimateGlobalRobotPos.setY(it->getY() - dY);
			estimateGlobalRobotPos.setThetaDeg(it->getThetaDeg()-FiducialLandMarkPos.getThetaDeg());

			return estimateGlobalRobotPos;
		}
	}

	estimateGlobalRobotPos.setID(-1);
	return estimateGlobalRobotPos;

}
/**
@brief Korean: fidutial mark를 검출 하여 등록하거나 로봇의 위치를 추정하는 함수
@brief English:
*/
void KuFiducialbasedLocalizerPr::estimateRobotPosbyFiducialmark()
{	

	KuPose FiducialLandMarkGlobalPos;	
	//KuPose RobotPosbyFiducialmark;
	KuPose NULLPos;
	NULLPos.setID(-1);
	
	if(m_bMapping==true)
	{	

		FiducialLandMarkGlobalPos = m_ALRecognizer.start(m_pIplCeilingGrayImage,m_RobotPos);
		FiducialLandMarkGlobalPos.setThetaDeg(FiducialLandMarkGlobalPos.getThetaDeg()+m_RobotPos.getThetaDeg());	

		if(FiducialLandMarkGlobalPos.getID()==-1)return;

		list<KuPose> FiducialMarkList;
		FiducialMarkList = registerFiducialMark(FiducialLandMarkGlobalPos);
		m_FiducialMarkImgCoordList = m_ALRecognizer.getRecogFiducialLandMarkList();


		list<KuPose>::iterator it;
		m_CriticalSection.Lock();	
		m_FiducialMarkList.clear();
		for (it = FiducialMarkList.begin(); it != FiducialMarkList.end(); it++) {
			m_FiducialMarkList.push_back(*it );
		}
		m_CriticalSection.Unlock();

		m_CriticalSection.Lock();	
		if(m_FiducialMarkImgCoordList.size()>0)
		{
			it = m_FiducialMarkImgCoordList.begin();
			m_FiducialMarkImgCoord=(*it);
		}
		m_CriticalSection.Unlock();

	}
	else
	{
		FiducialLandMarkGlobalPos = m_ALRecognizer.start(m_pIplCeilingGrayImage,NULLPos); // 인식된 mark의 global 3D pose
		m_poseFiducialMark = FiducialLandMarkGlobalPos;
		m_RobotPosbyFidicial = transferLocalPos2GlobalPos(FiducialLandMarkGlobalPos); // fiducial mark 관측으로 추정한 로봇의 위치

		if(m_nFiducialID==FiducialLandMarkGlobalPos.getID())
		{
			m_nFiducialCnt+=1;
		}
		else if(FiducialLandMarkGlobalPos.getID() != -1)
		{
			m_nFiducialID=FiducialLandMarkGlobalPos.getID();
			m_nFiducialCnt=0;
		}

		if(FiducialLandMarkGlobalPos.getID() != -1&&m_nFiducialCnt>3)
		{
			m_FiducialMarkImgCoordList = m_ALRecognizer.getRecogFiducialLandMarkList();

			list<KuPose>::iterator it;
			if(m_FiducialMarkImgCoordList.size()>0)
			{
				it = m_FiducialMarkImgCoordList.begin();
				m_FiducialMarkImgCoord=(*it);
			}
			else
			{
				m_FiducialMarkImgCoordList.clear();
				m_FiducialMarkImgCoord.init();
			}
		}
		else 
		{
			m_FiducialMarkImgCoord.init();
		}
	
	
	}
}

KuPose KuFiducialbasedLocalizerPr::getFiducialMarkPose(void)
{
	return m_poseFiducialMark;
}

KuPose KuFiducialbasedLocalizerPr::getRobotPosbyFiducial()
{
	return m_RobotPosbyFidicial;
}
/**
@brief Korean: 천장영상에서 관측된 fidutial mark를 확인하고 list 형태로 저장하는 함수
@brief English: 
*/
list<KuPose> KuFiducialbasedLocalizerPr::registerFiducialMark(KuPose RecogFiducialMark)
{
	list<KuPose> FiducialMarkList;

	if(m_vecFiducialMark.size() == 0 && RecogFiducialMark.getID() !=-1){
		int nMatchNum=1;
		RecogFiducialMark.setDist(nMatchNum);
		m_vecFiducialMark.push_back(RecogFiducialMark);
	}	
	else{
		bool bDectflag=false;
		for(int i=0; i<m_vecFiducialMark.size();i++){
			if(RecogFiducialMark.getID()==m_vecFiducialMark[i].getID())
			{
				printf("\n\nID=%d,Deg=%f,Deg=%f\n\n",RecogFiducialMark.getID(),m_vecFiducialMark[i].getThetaDeg(),RecogFiducialMark.getThetaDeg());
				bDectflag=true;
				double dMatchNum=m_vecFiducialMark[i].getDist();
				double dX=(m_vecFiducialMark[i].getX()*dMatchNum+RecogFiducialMark.getX())/(dMatchNum+1);
				double dY=(m_vecFiducialMark[i].getY()*dMatchNum+RecogFiducialMark.getY())/(dMatchNum+1);
				//double dT=(m_vecFiducialMark[i].getThetaDeg()*dMatchNum+RecogFiducialMark.getThetaDeg())/(dMatchNum+1);
				double dTheta1=m_vecFiducialMark[i].getThetaDeg();
				double dTheta2=RecogFiducialMark.getThetaDeg();
				double dTheta=0;
				if(dTheta1>0&&dTheta2<0)
				{
					dTheta=(dTheta1*dMatchNum+dTheta2+360)/(dMatchNum+1);
					if(dTheta>180) dTheta=dTheta-360;
				}
				else if(dTheta1<0&&dTheta2>0)
				{
					dTheta=(dTheta1*dMatchNum+dTheta2-360)/(dMatchNum+1);
					if(dTheta<-180) dTheta=dTheta+360;
				}
				else
				{
					dTheta=(m_vecFiducialMark[i].getThetaDeg()*dMatchNum+RecogFiducialMark.getThetaDeg())/(dMatchNum+1);
				}
// 				double dT=(m_vecFiducialMark[i].getThetaDeg()*dMatchNum+RecogFiducialMark.getThetaDeg())/(dMatchNum+1);
// 				double dT=(m_vecFiducialMark[i].getThetaDeg()*dMatchNum+RecogFiducialMark.getThetaDeg())/(dMatchNum+1);

				m_vecFiducialMark[i].setX(dX);
				m_vecFiducialMark[i].setY(dY);
				m_vecFiducialMark[i].setThetaDeg(dTheta);
				m_vecFiducialMark[i].setDist(dMatchNum+1);
			}
		}

		if(bDectflag==false)
		{
			m_vecFiducialMark.push_back(RecogFiducialMark);
		}
	}	
	for(int i=0; i<m_vecFiducialMark.size();i++){
		FiducialMarkList.push_back(m_vecFiducialMark[i]);
	}

	return FiducialMarkList;
}
/**
@brief Korean: fidutail mark의 정보를 불러오는 함수
@brief English: 
*/
void KuFiducialbasedLocalizerPr::loadFiducialFeatureMap()
{

	int ID=0;
	double x=0.0;
	double y=0.0;
	double z=0.0;
	double ThetaDeg=0.0;
	list<KuPose> FiducialMarkList;
	KuPose FiducialMarkPos;
	ifstream FeatureData;

	m_FiducialMarkDBList.clear();

	FeatureData.open(KuRobotParameter::getInstance()->getAlFeatureMapNameNPath().c_str());
	
	while(!FeatureData.eof())
	{
		FeatureData >> ID >> x >> y >> z >> ThetaDeg;
		FiducialMarkPos.setID(ID);
		FiducialMarkPos.setX(x);
		FiducialMarkPos.setY(y);
		FiducialMarkPos.setZ(z);
		FiducialMarkPos.setThetaDeg(ThetaDeg);
		m_FiducialMarkDBList.push_back(FiducialMarkPos);
	
	}
	FeatureData.close();
	
	m_FiducialMarkList=m_FiducialMarkDBList;
}


/**
@brief Korean: fidutail mark의 정보를 불러오는 함수
@brief English: 
*/
void KuFiducialbasedLocalizerPr::saveFiducialFeatureMap()
{
	//Fiducial mark ---------------------------------------
	ofstream FiducialMarkListdata(KuRobotParameter::getInstance()->getAlFeatureMapNameNPath().c_str());
	list<KuPose>::iterator it;
	for(it = m_FiducialMarkList.begin();it!=m_FiducialMarkList.end();it++ ){
		FiducialMarkListdata<<it->getID()<<" "<<it->getX()<<" "<<it->getY()<<" "<<it->getZ()<<" "<<it->getThetaDeg()<<endl;
	}

	FiducialMarkListdata.close();

}

/**
@brief Korean: fidutail mark의 카메라 영상상의 정보를 가져가는 함수
@brief English: 
*/
list<KuPose> KuFiducialbasedLocalizerPr::getFiducialMarkImgCoordList()
{
	list<KuPose> FiducialMarkList;
	list<KuPose>::iterator it;

	m_CriticalSection.Lock();	

	for (it = m_FiducialMarkImgCoordList.begin(); it != m_FiducialMarkImgCoordList.end(); it++) {
		FiducialMarkList.push_back(*it );
	}
	m_CriticalSection.Unlock();


	return  FiducialMarkList;
}
/**
@brief Korean: fidutail mark의 지도상의 위치 정보를 가져가는 함수
@brief English: 
*/
list<KuPose> KuFiducialbasedLocalizerPr::getFiducialMarkList()
{

	list<KuPose> FiducialMarkList;
	list<KuPose>::iterator it;

	m_CriticalSection.Lock();	

	for (it = m_FiducialMarkList.begin(); it != m_FiducialMarkList.end(); it++) {
		FiducialMarkList.push_back(*it );
	}
	m_CriticalSection.Unlock();


	return  FiducialMarkList;
}
/**
@brief Korean: fidutail mark의 정보를 초기화 해주는 함수
@brief English: 
*/
void  KuFiducialbasedLocalizerPr::initFiducialMark()
{
	m_FiducialMarkList.clear();
	m_FiducialMarkImgCoordList.clear();
}

/**
@brief Korean: fidutail mark의 카메라 영상상의 정보를 가져가는 함수
@brief English: 
*/
KuPose KuFiducialbasedLocalizerPr::getFiducialMarkImgCoord()
{

	return  m_FiducialMarkImgCoord;
}
