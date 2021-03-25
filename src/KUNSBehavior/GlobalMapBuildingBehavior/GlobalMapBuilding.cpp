#include "stdafx.h"
#include "GlobalMapBuilding.h"

GlobalMapBuilding::GlobalMapBuilding()
{

	cout<<"[GlobalMapBuilding]: Instance is created!!!"<<endl;
}

GlobalMapBuilding::~GlobalMapBuilding()
{
	cout<<"[GlobalMapBuilding]: Instance is destroyed!!!"<<endl;
}

/**
 @brief Korean: 초기화 작업을 수행하는 함수.
 @brief English: 
*/
bool GlobalMapBuilding::initialize( )
{
	m_bMapBuilding=true;
	m_bDBBuilding=true;
	GlobalLocalizationSupervisor::getInstance()->doProcessInit();

	//image 경로 loading
	m_vecImagePath.clear();
	m_vecImagePath = KuDrawingInfo::getInstance()->getvecLandmarkPos();

	//전체 database image loading
	m_vecmatCeilingImages.clear();
	GlobalLocalizationSupervisor::getInstance()->loadCeilingImage(m_vecImagePath,m_vecmatCeilingImages);

	//전체 image를 affine image로 변환(전역 위치에 맞게 회전된 천장영상)
	m_vecmatCeilingAffineImages.clear();
	GlobalLocalizationSupervisor::getInstance()->affineCeilingImage(m_vecImagePath,m_vecmatCeilingImages,m_vecmatCeilingAffineImages);

	//전체 database image를 retinex image로 변환
	m_vecmatRetinexImages.clear();
	GlobalLocalizationSupervisor::getInstance()->cvtRetinexImage(m_vecmatCeilingImages,m_vecmatRetinexImages);

	//retinex image를 retinex affine image로 변환(전역 위치에 맞게 회전된 천장영상)
	m_vecmatRetinexAffineImages.clear();
	GlobalLocalizationSupervisor::getInstance()->affineCeilingImage(m_vecImagePath,m_vecmatRetinexImages,m_vecmatRetinexAffineImages);

	return true;
}

/**
 @brief Korean: 천장 전역 지도 생성 쓰레드
 @brief English: Global Ceiling Map Building Thread
*/
void GlobalMapBuilding::doGCMBuildingThread(void* arg)
{
	GlobalMapBuilding* pGCM = (GlobalMapBuilding*)arg;

	if(pGCM->m_bMapBuilding)
	{
		//전역지도 생성
		GlobalLocalizationSupervisor::getInstance()->buildGlobalMap(pGCM->m_vecImagePath,pGCM->m_vecmatCeilingImages,pGCM->m_vecmatCeilingAffineImages, pGCM->m_vecmatRetinexImages, pGCM->m_vecmatRetinexAffineImages);

		pGCM->m_bMapBuilding=false;
	}
	else if(!(pGCM->m_bDBBuilding))//특징 데이터베이스 생성 쓰레드가 false 일때
	{
		pGCM->terminate();
	}
}

/**
 @brief Korean: 특징 데이터베이스 생성 쓰레드
 @brief English:Feature DataBase Building Thread
*/
void GlobalMapBuilding::doFDBBuildingThread(void* arg)
{
	GlobalMapBuilding* pFDB = (GlobalMapBuilding*)arg;
	if(pFDB->m_bDBBuilding)
	{
		for(int i=0; i<pFDB->m_vecImagePath.size(); i++)
		{
			Mat matRetinexImage;
			matRetinexImage.create(Sensor::CEILING_IMAGE_HEIGHT,Sensor::CEILING_IMAGE_WIDTH,CV_8UC1);
			for(int j=0; j<matRetinexImage.cols*matRetinexImage.rows; j++)
			{
				matRetinexImage.data[j]=pFDB->m_vecmatRetinexImages[i].data[j];
			}
			GlobalLocalizationSupervisor::getInstance()->saveCeilingDB(matRetinexImage);
		}
		pFDB->m_bDBBuilding=false;
	}
}

/**
@brief Korean: 주행 관련 정보를 그려주는 함수.
@brief English: 
*/
void GlobalMapBuilding::drawNaviData()
{
	KuDrawingInfo::getInstance()->setRobotPos(m_RobotPos); //로봇 위치 화면에 표시	
	KuDrawingInfo::getInstance()->setCeilingImageData(m_CeilingCamera);
	KuDrawingInfo::getInstance()->setFrontLaserData181(m_nLaserData181);	
}

/**
 @brief Korean: 현재 사용중인  Localizer를 넘겨주는 함수
 @brief English: 
*/
Localizer* GlobalMapBuilding::getLocalizer()
{
	return KuSURFbasedGlobalLocalizerPr::getInstance();

}

/**
 @brief Korean: 현재 Behavior의 상태를 나타내는 함수
 @brief English: 
*/
bool GlobalMapBuilding::getBehaviorStates()
{
	return m_bThreadFlag;
}

/**
 @brief Korean: 클래스를 종료한다.
 @brief English: 
*/
void GlobalMapBuilding::terminate()
{
	GlobalLocalizationSupervisor::getInstance()->loadDB(m_vecImagePath.size());
	m_FDBBuildingThread.terminate();
	m_GCMBuildingThread.terminate();
	m_vecmatCeilingAffineImages.clear();
	m_vecmatRetinexImages.clear();
	m_vecImagePath.clear();
	GlobalLocalizationSupervisor::getInstance()->doProcessInit();
	m_bThreadFlag = false;
}

/**
 @brief Korean: Localizer를 시작한다.
 @brief English: 
*/
void GlobalMapBuilding::startLocalizer(KuPose RobotPos)
{


}

/**
 @brief Korean: GlobalLocalization를 실행시키는 함수
 @brief English: 
*/
bool GlobalMapBuilding::execute(KuCommandMessage CMessage )
{
	
	switch(CMessage.getCommandName()){

	case KuCommandMessage::START_THREAD:	
		if(initialize()) 
		{
			m_FDBBuildingThread.start(doFDBBuildingThread,this,100, "GlobalMapBuilding::execute()_FDBBuilding"); //특징추출 스레디 시작
			m_GCMBuildingThread.start(doGCMBuildingThread,this,100, "GlobalMapBuilding::execute()_GCMBuilding"); //천장 전역지도 작성 스레드 시작	
		}
		break;
	case KuCommandMessage::TERMINATE_THREAD:
		terminate();
		break;
	case KuCommandMessage::SUSPEND_THREAD:
		break;
	case KuCommandMessage::RESUME_THREAD:
		break;
	default:break;
	}

	return true;
}
