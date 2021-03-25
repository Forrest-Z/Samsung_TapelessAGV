#include "stdafx.h"
#include "GlobalLocalizationBehavior.h"

GlobalLocalizationBehavior::GlobalLocalizationBehavior()
{
	m_bGlobalLocalizationFlag=false;
	m_bThreadFlag = false;
	m_bStartSURFThread=false;

	m_cvCeilingImage.create(Sensor::CEILING_IMAGE_HEIGHT,Sensor::CEILING_IMAGE_WIDTH,CV_8UC1);
	cout<<"[GlobalLocalizationBehavior]: Instance is created!!!"<<endl;
	m_CeilingCamera=cvLoadImage("./data/path/Image/img_40.bmp",0);//시뮬레이션모드
}

GlobalLocalizationBehavior::~GlobalLocalizationBehavior()
{
	cout<<"[GlobalLocalizationBehavior]: Instance is destroyed!!!"<<endl;
}

/**
 @brief Korean: 초기화 작업을 수행하는 함수.
 @brief English: 
*/
bool GlobalLocalizationBehavior::initialize( )
{
	m_bGlobalLocalizationFlag=false;
	m_bThreadFlag=true;
	m_bStartSURFThread=true;

	m_RobotPos = KuDrawingInfo::getInstance()->getRobotPos();

	GlobalLocalizationSupervisor::getInstance()->doProcessInit();
	startLocalizer(m_RobotPos); //Localizer를 시작한다.

	return true;
}

/**
 @brief Korean: 실시간으로 로봇의 위치를 저장하는 스레드
 @brief English: 
 */
void GlobalLocalizationBehavior::startSavingPathThrerad()
{
	GlobalLocalizationSupervisor::getInstance()->startSavingPathThrerad();
}

/**
 @brief Korean: 실시간으로 로봇의 위치를 저장하는 스레드
 @brief English: 
*/
void GlobalLocalizationBehavior::terminateSavingPathThread()
{
	GlobalLocalizationSupervisor::getInstance()->terminateSavingPathThread();
}

/**
 @brief Korean: 스레드로 돌아가는 함수
 @brief English: 
*/
// void GlobalLocalizationBehavior::doThread(void* arg)
// {
// 	GlobalLocalizationBehavior* pGLB = (GlobalLocalizationBehavior*)arg;
// 
// 	//실시간모드
// // 	if(SensorSupervisor::getInstance()->readSensorData() ==false) return;
// // 	pGLB->m_CeilingCamera = SensorSupervisor::getInstance()->getCeilingImageData();
// 
// 	GlobalLocalizationSupervisor::getInstance()->SURFbasedGlobalLocalization(pGLB->m_CeilingCamera);
// 	
// 	if(GlobalLocalizationSupervisor::getInstance()->checkSelectPathNearLastPos())
// 	{
// 		GlobalLocalizationSupervisor::getInstance()->setTransitionVal(pGLB->m_CeilingCamera);
// 	}
// 
// 	if(GlobalLocalizationSupervisor::getInstance()->determineGLReliability(GlobalLocalizationSupervisor::getInstance()->getSelectPathIdx()))
// 	{
// 		pGLB->m_RobotPos=GlobalLocalizationSupervisor::getInstance()->getRobotPos();
// 		pGLB->terminate();
// 		return;
// 	}
// 	else
// 	{
// 		GlobalLocalizationSupervisor::getInstance()->doProcessInit();
// 	}
// 	
// }
void GlobalLocalizationBehavior::doThread(void* arg)
{
	GlobalLocalizationBehavior* pGLB = (GlobalLocalizationBehavior*)arg;
	//실시간모드
	if(SensorSupervisor::getInstance()->readSensorData()==false) 
	{
		printf("Cameara is not connected!!!\n");
		pGLB->terminate();
	}
	pGLB->m_CeilingCamera = SensorSupervisor::getInstance()->getCeilingImageData();
	IplImage* IplCopyImage = cvCreateImage(cvSize(pGLB->m_CeilingCamera->width,pGLB->m_CeilingCamera->height),pGLB->m_CeilingCamera->depth,pGLB->m_CeilingCamera->nChannels);
	cvCopy(pGLB->m_CeilingCamera,IplCopyImage);
	cvFlip(IplCopyImage,IplCopyImage,0);
	bool bGlobalLocalization=GlobalLocalizationSupervisor::getInstance()->GlobalMapbasedGlobalLocalization(IplCopyImage,pGLB->m_RobotPos);
	cvReleaseImage(&IplCopyImage);
	if(bGlobalLocalization)
	{
		pGLB->terminate();
	}
}
// 
// /**
//  @brief Korean: 스레드로 돌아가는 함수
//  @brief English: 
// */
// void GlobalLocalizationBehavior::doSURFThread(void* arg)
// {
// 	GlobalLocalizationBehavior* pSURFGLB = (GlobalLocalizationBehavior*)arg;
// 
// 	if(!pSURFGLB->m_bGlobalLocalizationFlag)
// 	{
// 		
// 	}
// }

/**
@brief Korean: 주행 관련 정보를 그려주는 함수.
@brief English: 
*/
void GlobalLocalizationBehavior::drawNaviData()
{
	KuDrawingInfo::getInstance()->setRobotPos(m_RobotPos); //로봇 위치 화면에 표시	
	KuDrawingInfo::getInstance()->setCeilingImageData(m_CeilingCamera);
	//KuDrawingInfo::getInstance()->setLaserData181(m_nLaserData181);	
}

/**
 @brief Korean: 현재 사용중인  Localizer를 넘겨주는 함수
 @brief English: 
*/
Localizer* GlobalLocalizationBehavior::getLocalizer()
{
	return KuSURFbasedGlobalLocalizerPr::getInstance();

}

/**
 @brief Korean: 현재 Behavior의 상태를 나타내는 함수
 @brief English: 
*/
bool GlobalLocalizationBehavior::getBehaviorStates()
{
	return m_bThreadFlag;
}

/**
 @brief Korean: 클래스를 종료한다.
 @brief English: 
*/
void GlobalLocalizationBehavior::terminate()
{
	drawNaviData();
	m_doThread.terminate();

	m_bGlobalLocalizationFlag=false;
	m_bStartSURFThread=true;
	m_bThreadFlag = false;
}

/**
 @brief Korean: Localizer를 시작한다.
 @brief English: 
*/
void GlobalLocalizationBehavior::startLocalizer(KuPose RobotPos)
{

}

/**
 @brief Korean: GlobalLocalization를 실행시키는 함수
 @brief English: 
*/
bool GlobalLocalizationBehavior::execute(KuCommandMessage CMessage )
{
	
	switch(CMessage.getCommandName()){

	case KuCommandMessage::START_THREAD:	
		if(initialize()) 
		{
			m_doThread.start(doThread,this,100, "GlobalLocalizationBehavior::execute()"); //메인 스레드 시작	
		}
		break;
	case KuCommandMessage::TERMINATE_THREAD:
		terminate();
		break;
	case KuCommandMessage::SUSPEND_THREAD:
		m_SURFThread.suspend();

		break;
	case KuCommandMessage::RESUME_THREAD:
		m_SURFThread.resume();
		break;
	default:break;
	}

	return true;
}
