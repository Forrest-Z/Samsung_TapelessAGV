#include "stdafx.h"
#include "GlobalLocalizationBehavior.h"

GlobalLocalizationBehavior::GlobalLocalizationBehavior()
{
	m_bGlobalLocalizationFlag=false;
	m_bThreadFlag = false;
	m_bStartSURFThread=false;

	m_cvCeilingImage.create(Sensor::CEILING_IMAGE_HEIGHT,Sensor::CEILING_IMAGE_WIDTH,CV_8UC1);
	cout<<"[GlobalLocalizationBehavior]: Instance is created!!!"<<endl;
	m_CeilingCamera=cvLoadImage("./data/path/Image/img_40.bmp",0);//�ùķ��̼Ǹ��
}

GlobalLocalizationBehavior::~GlobalLocalizationBehavior()
{
	cout<<"[GlobalLocalizationBehavior]: Instance is destroyed!!!"<<endl;
}

/**
 @brief Korean: �ʱ�ȭ �۾��� �����ϴ� �Լ�.
 @brief English: 
*/
bool GlobalLocalizationBehavior::initialize( )
{
	m_bGlobalLocalizationFlag=false;
	m_bThreadFlag=true;
	m_bStartSURFThread=true;

	m_RobotPos = KuDrawingInfo::getInstance()->getRobotPos();

	GlobalLocalizationSupervisor::getInstance()->doProcessInit();
	startLocalizer(m_RobotPos); //Localizer�� �����Ѵ�.

	return true;
}

/**
 @brief Korean: �ǽð����� �κ��� ��ġ�� �����ϴ� ������
 @brief English: 
 */
void GlobalLocalizationBehavior::startSavingPathThrerad()
{
	GlobalLocalizationSupervisor::getInstance()->startSavingPathThrerad();
}

/**
 @brief Korean: �ǽð����� �κ��� ��ġ�� �����ϴ� ������
 @brief English: 
*/
void GlobalLocalizationBehavior::terminateSavingPathThread()
{
	GlobalLocalizationSupervisor::getInstance()->terminateSavingPathThread();
}

/**
 @brief Korean: ������� ���ư��� �Լ�
 @brief English: 
*/
// void GlobalLocalizationBehavior::doThread(void* arg)
// {
// 	GlobalLocalizationBehavior* pGLB = (GlobalLocalizationBehavior*)arg;
// 
// 	//�ǽð����
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
	//�ǽð����
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
//  @brief Korean: ������� ���ư��� �Լ�
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
@brief Korean: ���� ���� ������ �׷��ִ� �Լ�.
@brief English: 
*/
void GlobalLocalizationBehavior::drawNaviData()
{
	KuDrawingInfo::getInstance()->setRobotPos(m_RobotPos); //�κ� ��ġ ȭ�鿡 ǥ��	
	KuDrawingInfo::getInstance()->setCeilingImageData(m_CeilingCamera);
	//KuDrawingInfo::getInstance()->setLaserData181(m_nLaserData181);	
}

/**
 @brief Korean: ���� �������  Localizer�� �Ѱ��ִ� �Լ�
 @brief English: 
*/
Localizer* GlobalLocalizationBehavior::getLocalizer()
{
	return KuSURFbasedGlobalLocalizerPr::getInstance();

}

/**
 @brief Korean: ���� Behavior�� ���¸� ��Ÿ���� �Լ�
 @brief English: 
*/
bool GlobalLocalizationBehavior::getBehaviorStates()
{
	return m_bThreadFlag;
}

/**
 @brief Korean: Ŭ������ �����Ѵ�.
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
 @brief Korean: Localizer�� �����Ѵ�.
 @brief English: 
*/
void GlobalLocalizationBehavior::startLocalizer(KuPose RobotPos)
{

}

/**
 @brief Korean: GlobalLocalization�� �����Ű�� �Լ�
 @brief English: 
*/
bool GlobalLocalizationBehavior::execute(KuCommandMessage CMessage )
{
	
	switch(CMessage.getCommandName()){

	case KuCommandMessage::START_THREAD:	
		if(initialize()) 
		{
			m_doThread.start(doThread,this,100, "GlobalLocalizationBehavior::execute()"); //���� ������ ����	
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
