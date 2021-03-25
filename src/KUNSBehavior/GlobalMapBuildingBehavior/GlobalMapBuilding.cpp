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
 @brief Korean: �ʱ�ȭ �۾��� �����ϴ� �Լ�.
 @brief English: 
*/
bool GlobalMapBuilding::initialize( )
{
	m_bMapBuilding=true;
	m_bDBBuilding=true;
	GlobalLocalizationSupervisor::getInstance()->doProcessInit();

	//image ��� loading
	m_vecImagePath.clear();
	m_vecImagePath = KuDrawingInfo::getInstance()->getvecLandmarkPos();

	//��ü database image loading
	m_vecmatCeilingImages.clear();
	GlobalLocalizationSupervisor::getInstance()->loadCeilingImage(m_vecImagePath,m_vecmatCeilingImages);

	//��ü image�� affine image�� ��ȯ(���� ��ġ�� �°� ȸ���� õ�念��)
	m_vecmatCeilingAffineImages.clear();
	GlobalLocalizationSupervisor::getInstance()->affineCeilingImage(m_vecImagePath,m_vecmatCeilingImages,m_vecmatCeilingAffineImages);

	//��ü database image�� retinex image�� ��ȯ
	m_vecmatRetinexImages.clear();
	GlobalLocalizationSupervisor::getInstance()->cvtRetinexImage(m_vecmatCeilingImages,m_vecmatRetinexImages);

	//retinex image�� retinex affine image�� ��ȯ(���� ��ġ�� �°� ȸ���� õ�念��)
	m_vecmatRetinexAffineImages.clear();
	GlobalLocalizationSupervisor::getInstance()->affineCeilingImage(m_vecImagePath,m_vecmatRetinexImages,m_vecmatRetinexAffineImages);

	return true;
}

/**
 @brief Korean: õ�� ���� ���� ���� ������
 @brief English: Global Ceiling Map Building Thread
*/
void GlobalMapBuilding::doGCMBuildingThread(void* arg)
{
	GlobalMapBuilding* pGCM = (GlobalMapBuilding*)arg;

	if(pGCM->m_bMapBuilding)
	{
		//�������� ����
		GlobalLocalizationSupervisor::getInstance()->buildGlobalMap(pGCM->m_vecImagePath,pGCM->m_vecmatCeilingImages,pGCM->m_vecmatCeilingAffineImages, pGCM->m_vecmatRetinexImages, pGCM->m_vecmatRetinexAffineImages);

		pGCM->m_bMapBuilding=false;
	}
	else if(!(pGCM->m_bDBBuilding))//Ư¡ �����ͺ��̽� ���� �����尡 false �϶�
	{
		pGCM->terminate();
	}
}

/**
 @brief Korean: Ư¡ �����ͺ��̽� ���� ������
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
@brief Korean: ���� ���� ������ �׷��ִ� �Լ�.
@brief English: 
*/
void GlobalMapBuilding::drawNaviData()
{
	KuDrawingInfo::getInstance()->setRobotPos(m_RobotPos); //�κ� ��ġ ȭ�鿡 ǥ��	
	KuDrawingInfo::getInstance()->setCeilingImageData(m_CeilingCamera);
	KuDrawingInfo::getInstance()->setFrontLaserData181(m_nLaserData181);	
}

/**
 @brief Korean: ���� �������  Localizer�� �Ѱ��ִ� �Լ�
 @brief English: 
*/
Localizer* GlobalMapBuilding::getLocalizer()
{
	return KuSURFbasedGlobalLocalizerPr::getInstance();

}

/**
 @brief Korean: ���� Behavior�� ���¸� ��Ÿ���� �Լ�
 @brief English: 
*/
bool GlobalMapBuilding::getBehaviorStates()
{
	return m_bThreadFlag;
}

/**
 @brief Korean: Ŭ������ �����Ѵ�.
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
 @brief Korean: Localizer�� �����Ѵ�.
 @brief English: 
*/
void GlobalMapBuilding::startLocalizer(KuPose RobotPos)
{


}

/**
 @brief Korean: GlobalLocalization�� �����Ű�� �Լ�
 @brief English: 
*/
bool GlobalMapBuilding::execute(KuCommandMessage CMessage )
{
	
	switch(CMessage.getCommandName()){

	case KuCommandMessage::START_THREAD:	
		if(initialize()) 
		{
			m_FDBBuildingThread.start(doFDBBuildingThread,this,100, "GlobalMapBuilding::execute()_FDBBuilding"); //Ư¡���� ������ ����
			m_GCMBuildingThread.start(doGCMBuildingThread,this,100, "GlobalMapBuilding::execute()_GCMBuilding"); //õ�� �������� �ۼ� ������ ����	
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
