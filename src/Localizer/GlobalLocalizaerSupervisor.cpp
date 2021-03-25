#include "stdafx.h"
#include "GlobalLocalizerSupervisor.h"

GlobalLocalizationSupervisor::GlobalLocalizationSupervisor()
{
	m_nCeilingCamLogIdx=0;
	m_RobotPos.init(); //로봇 위치 초기화
}

GlobalLocalizationSupervisor::~GlobalLocalizationSupervisor()
{
	
}

/**
@brief Korean:  초기 이미지 저장을 위한 사전작업
@brief English: 
*/
void GlobalLocalizationSupervisor::initialize()
{
	m_nCeilingCamLogIdx=0;
}

/**
@brief Korean:  초기 이미지 저장을 위한 사전작업
@brief English: 
*/
void GlobalLocalizationSupervisor::doProcessInit()
{
	KuSURFbasedGlobalLocalizerPr::getInstance()->initialize();
	m_nCeilingCamLogIdx=0;
}

/**
@brief Korea: 천장 이미지와 SURF 를 위한 Look up table을 저장한다.
@brief English:
*/
void GlobalLocalizationSupervisor::saveCeilingData(IplImage* IplCeilingImage)
{

	IplImage* IplCopyImage = cvCreateImage(cvSize(IplCeilingImage->width,IplCeilingImage->height),8,1);
	cvCopy(IplCeilingImage,IplCopyImage);
	cvFlip(IplCopyImage,IplCopyImage,0);
	//--------------천장 이미지 경로----------------------------------
	char cFilePathName[150];
	memset(cFilePathName,0,sizeof(cFilePathName));
	sprintf_s(cFilePathName,"./Data/Path/Image/img_%d.bmp", m_nCeilingCamLogIdx);
	//--------------천장 이미지 경로----------------------------------
	cvSaveImage(cFilePathName,IplCopyImage); 	
	cvReleaseImage(&IplCopyImage);
	m_nCeilingCamLogIdx++;
}

void GlobalLocalizationSupervisor::saveCeilingDB(Mat matCeilingImage)
{
	KuSURFbasedGlobalLocalizerPr::getInstance()->detectSURFDatatoSave(matCeilingImage,m_nCeilingCamLogIdx);
	m_nCeilingCamLogIdx++;
}


/**
@brief Korean: 데이터 베이스의 이미지를 불러와서 keypoints 및 descriptors Look up table을 작성한다.
@brief English: 
*/
void GlobalLocalizationSupervisor::loadDB(int nTotalImagSize)
{
	//FeatureDetector detector;
	Mat descriptors;
	vector<KeyPoint> veckeypoint;
	KeyPoint keypoint;

	m_nImageNum=nTotalImagSize;

	KuSURFbasedGlobalLocalizerPr::getInstance()->initDB();
	int nIdx=0;
	while(nIdx<m_nImageNum){
		KuSURFbasedGlobalLocalizerPr::getInstance()->loadSURFDB(nIdx,  descriptors, keypoint, veckeypoint);

		nIdx++;
	}
}

void GlobalLocalizationSupervisor::loadCeilingImage(vector<KuPose> vecImagePath, vector<Mat>& MatCeilingImages)
{
	char cFilePathName[150];
	int nIdx=0;

	while(nIdx<vecImagePath.size()){
		memset(cFilePathName,0,sizeof(cFilePathName));
		sprintf_s(cFilePathName,"./Data/Path/Image/img_%d.bmp", nIdx);
		Mat MatTempImage = imread(cFilePathName,0);
		MatCeilingImages.push_back(MatTempImage);

		nIdx++;
	}
}


void GlobalLocalizationSupervisor::cvtRetinexImage(vector<Mat>& vecmatImages, vector<Mat>& vecmatRetinexImages)
{
	KuGlobalMapBuildingPr::getInstance()->cvtRetinexImage(vecmatImages,vecmatRetinexImages);
}

void GlobalLocalizationSupervisor::affineCeilingImage(vector<KuPose> vecImagePath, vector<Mat>& vecmatCeilingImages, vector<Mat>& vecmatCeilingAffineImages)
{
	KuGlobalMapBuildingPr::getInstance()->affineCeilingImage(vecImagePath,vecmatCeilingImages,vecmatCeilingAffineImages);
}

void GlobalLocalizationSupervisor::affineCeilingImage(Mat& matCeilingImage, Mat& matCeilingAffineImage, double dTheta)
{
	KuGlobalMapBuildingPr::getInstance()->affineCeilingImage(matCeilingImage,matCeilingAffineImage,dTheta);
}

void GlobalLocalizationSupervisor::buildGlobalMap(vector<KuPose>& vecImagePath, vector<Mat>& vecmatCeilingImages, vector<Mat>& vecmatCeilingAffineImages, vector<Mat>& vecmatRetinexImages, vector<Mat>& vecmatRetinexAffineImages)
{
	m_vecImagePixcelPath.clear();
	KuGlobalMapBuildingPr::getInstance()->buildGlobalMap(vecImagePath,m_vecImagePixcelPath,vecmatCeilingImages,vecmatCeilingAffineImages,vecmatRetinexImages,vecmatRetinexAffineImages);
	KuGlobalMapBuildingPr::getInstance()->saveImagePixcelPath(m_vecImagePixcelPath);
}
/**
@brief Korean: 데이터 베이스의 이미지에 대한 위치를 받아온다
@brief English: 
*/
vector<KuPose> GlobalLocalizationSupervisor::loadImagePath(string strDataPath)
{
	m_vecImagePath.clear();
	ifstream DataLog;
	DataLog.open(strDataPath);
	double dPathX, dPathY,dPathThetaDeg;
	while(!DataLog.eof()){
		KuPose PathPos;
		DataLog >> dPathX >> dPathY>>dPathThetaDeg;
		PathPos.setX(dPathX);
		PathPos.setY(dPathY);
		PathPos.setThetaDeg(dPathThetaDeg);
		m_vecImagePath.push_back(PathPos);
	}	
	m_vecImagePath.pop_back();
	DataLog.close();

	return m_vecImagePath;
}

/**
@brief Korean: 데이터 베이스의 이미지에 대한 위치를 받아온다
@brief English: 
*/
vector<Point2i> GlobalLocalizationSupervisor::loadImagePixcelPath(string strDataPath)
{
	m_vecImagePixcelPath.clear();
	ifstream DataLog;
	DataLog.open(strDataPath);

	int nPtX, nPtY;
	Point2i nptPixcel;
	while(!DataLog.eof()){
		DataLog >> nPtX >> nPtY;
		nptPixcel.x=nPtX;
		nptPixcel.y=nPtY;
		m_vecImagePixcelPath.push_back(nptPixcel);
	}	
	m_vecImagePixcelPath.pop_back();
	DataLog.close();

	return m_vecImagePixcelPath;
}

KuPose GlobalLocalizationSupervisor::SURFbasedGlobalLocalization(IplImage * IplCeilingImage)
{
	m_SURFRobotPos=KuSURFbasedGlobalLocalizerPr::getInstance()->GlobalLocalization(IplCeilingImage);

	return m_SURFRobotPos;
}

bool GlobalLocalizationSupervisor::GlobalMapbasedGlobalLocalization(IplImage* IplCeilingImage, KuPose& RobotPose)
{
	bool bGlobalLocalization = KuGlobalMapbasedGlobalLocalizationPr::getInstance()->doGlobalMapbasedGlobalLocalization(IplCeilingImage,m_vecImagePath,m_vecImagePixcelPath,RobotPose);
	return bGlobalLocalization;
}

void GlobalLocalizationSupervisor::setTransitionVal(IplImage* IplCeilingImage)
{
	m_nSelectPathIdx=KuSURFbasedGlobalLocalizerPr::getInstance()->getSelectPathIdx();
	KuSURFbasedGlobalLocalizerPr::getInstance()->setTransitionVal(IplCeilingImage, m_nSelectPathIdx);

	double dSURFAngle=KuSURFbasedGlobalLocalizerPr::getInstance()->getRotationAngle();
	double dSURFDeltaX=KuSURFbasedGlobalLocalizerPr::getInstance()->getDeltaX();
	double dSURFDeltaY=KuSURFbasedGlobalLocalizerPr::getInstance()->getDeltaY();

	if(KuSURFbasedGlobalLocalizerPr::getInstance()->getRotationTruth())
	{
		m_dRotationAngle=dSURFAngle;
		m_dDeltaX=dSURFDeltaX;
		m_dDeltaY=dSURFDeltaY;
		printf("SURF process is checked\n");
	}
	else
	{
		m_dRotationAngle=-360.0;
		m_dDeltaX=0.0;
		m_dDeltaY=0.0;
		printf("All matching processes are failed!\n");
	}
	double dImagePathX = m_vecImagePath[m_nSelectPathIdx].getX();
	double dImagePathY = m_vecImagePath[m_nSelectPathIdx].getY();
	double dImagePathTheta = m_vecImagePath[m_nSelectPathIdx].getThetaDeg();

	double dImagePathTheta_Radian = dImagePathTheta*D2R;

	double dDeltaX = (m_dDeltaX*cos(dImagePathTheta_Radian)-m_dDeltaY*sin(dImagePathTheta_Radian))*1000.0;
	double dDeltaY = (m_dDeltaX*sin(dImagePathTheta_Radian)+m_dDeltaY*cos(dImagePathTheta_Radian))*1000.0;

	m_dDeltaX=dDeltaX;
	m_dDeltaY=dDeltaY;

	double dDist=sqrt(dDeltaX*dDeltaX+dDeltaY*dDeltaY);

	m_RobotPos.setX(dImagePathX+dDeltaX);
	m_RobotPos.setY(dImagePathY+dDeltaY);
	m_RobotPos.setThetaDeg(dImagePathTheta+m_dRotationAngle);
	m_RobotPos.setID(m_nSelectPathIdx);
}

bool GlobalLocalizationSupervisor::checkSelectPathNearLastPos()
{
	return KuSURFbasedGlobalLocalizerPr::getInstance()->checkSelectPathNearLastPos();
}

bool GlobalLocalizationSupervisor::determineGLReliability(bool bSelectPathIdx)
{
	if(bSelectPathIdx)
	{
		double dRotationAngle=getRotationAngle();
		if(dRotationAngle>=-200.0)
		{
			printf("Global Localization has been finished!!\n");
			printf("Rotation Angle = %f\n",getRotationAngle());
			printf("DeltaX =%f\n",getDeltaX());
			printf("DeltaY =%f\n",getDeltaY());
			return true;
		}
		else
		{
			printf("Global Localization is not truthable\n");
			return false;
		}
	}
	else
	{
		printf("Global Localization Failled!!!!!!\n");
		return false;
	}
}
bool GlobalLocalizationSupervisor::getSelectPathIdx()
{
	return KuSURFbasedGlobalLocalizerPr::getInstance()-> getSelectPathReliabliity();
}
int GlobalLocalizationSupervisor::getSURFSelectPathIdx()
{
	return KuSURFbasedGlobalLocalizerPr::getInstance()->getSelectPathIdx();
}

double GlobalLocalizationSupervisor::getRotationAngle()
{
	return m_dRotationAngle;
}

double GlobalLocalizationSupervisor::getDeltaX()
{
	return m_dDeltaX;
}
double GlobalLocalizationSupervisor::getDeltaY()
{
	return m_dDeltaY;
}
/**
@brief Korean:  로봇 위치 초기화
@brief English: 
*/
void GlobalLocalizationSupervisor::init()
{
	m_RobotPos.init(); //로봇 위치 초기화
}

/**
@brief Korean:  로봇 위치를 설정한다
@brief English: 
*/
void GlobalLocalizationSupervisor::setRobotPos(KuPose RobotPos)
{
	m_RobotPos = RobotPos;

}

/**
@brief Korean:  로봇의 X좌표를 설정한다
@brief English: 
*/
void GlobalLocalizationSupervisor::setRobotPosX(double dPoseX)
{
	m_RobotPos.setX(dPoseX);
}
/**
@brief Korean:  로봇의 Y좌표를 설정한다
@brief English: 
*/
void GlobalLocalizationSupervisor::setRobotPosY(double dPoseY)
{
	m_RobotPos.setY(dPoseY);
}
/**
@brief Korean:  로봇의 각도 Degree를 설정한다
@brief English: 
*/
void GlobalLocalizationSupervisor::setRobotPosDeg(double dPoseDeg)
{
	m_RobotPos.setThetaDeg(dPoseDeg);
}
/**
@brief Korean:  로봇의 각도 Radian를 설정한다
@brief English: 
*/
void GlobalLocalizationSupervisor::setRobotPosRad(double dPoseRad)
{
	m_RobotPos.setThetaRad(dPoseRad);
}

/**
@brief Korean: 지도정보를 설정한다
@brief English: 
*/
void GlobalLocalizationSupervisor::setMap(int nMapSizeX, int nMapSizeY, int** nMap)
{
	if(m_nMap==NULL)
	{
		m_nMapSizeX=nMapSizeX;
		m_nMapSizeY=nMapSizeY;
		m_nMap = new int*[m_nMapSizeX];
		if(m_nMap){
			for(int i = 0 ; i < m_nMapSizeX ; i++){
				m_nMap[i] = new int[m_nMapSizeY];
			}
		}

		for(int i = 0; i<m_nMapSizeX; i++){
			for(int j = 0; j< m_nMapSizeY;j++){
				m_nMap[i][j] = nMap[i][j];
			}
		}
	}


}

/**
@brief Korean:  로봇의 X좌표를 가져 간다
@brief English:
*/
double GlobalLocalizationSupervisor::getRobotPosX()
{
	return m_RobotPos.getX();
}
/**
@brief Korean:  로봇의 Y좌표를 가져 간다
@brief English:
*/
double GlobalLocalizationSupervisor::getRobotPosY()
{
	return m_RobotPos.getY();
}
/**
@brief Korean:  로봇의  각도(Degree)를 가져 간다
@brief English:
*/
double GlobalLocalizationSupervisor::getRobotPosDeg()
{
	return m_RobotPos.getThetaDeg();
}
/**
@brief Korean:  로봇의  각도(Radian)를 가져 간다
@brief English:
*/
double GlobalLocalizationSupervisor::getRobotPosRad()
{
	return m_RobotPos.getThetaRad();
}
/**
@brief Korean:  로봇의  위치 값을 가져 간다
@brief English:
*/
KuPose GlobalLocalizationSupervisor::getRobotPos()
{
	return m_RobotPos;
}

int GlobalLocalizationSupervisor::getTotalImageNum()
{
	return m_nCeilingCamLogIdx;
}

/**
 @brief Korean: 마지막 로봇 위치 불러오는 함수
 @brief English: 
*/
KuPose GlobalLocalizationSupervisor::loadLastRobotPos(string strDataPath )
{
	ifstream DataLog;
	DataLog.open(strDataPath);

	double dPathX,dPathY,dPathThetaDeg;
	bool bDircheck;
	DataLog >> dPathX >> dPathY>>dPathThetaDeg>>bDircheck;
	m_LastRobotPos.setX(dPathX);
	m_LastRobotPos.setY(dPathY);
	m_LastRobotPos.setThetaDeg(dPathThetaDeg);
	KuDrawingInfo::getInstance()->setDirectionofPathflag(bDircheck);
	DataLog.close();
	m_RobotPos=m_LastRobotPos;

	startSavingPathThrerad();

	return m_LastRobotPos;
}

/**
 @brief Korean: 실시간으로 로봇의 위치를 저장하는 스레드
 @brief English: 
*/
void GlobalLocalizationSupervisor::doSavingLastRobotPosThread(void* arg)
{
	GlobalLocalizationSupervisor* pSPT = (GlobalLocalizationSupervisor*)arg;

	string strDataPath =KuRobotParameter::getInstance()->getLastRobotPoseNameNPath();
	ofstream ofDataLog;

	ofDataLog.open(strDataPath);
	KuPose RobotPos= KuDrawingInfo::getInstance()->getRobotPos();
	bool bDircheck=KuDrawingInfo::getInstance()->getDirectionofPathflag( );
	ofDataLog<<RobotPos.getX()<<" "<<RobotPos.getY()<<" "<<RobotPos.getThetaDeg()<<" "<<bDircheck<<endl;
	ofDataLog.close();
}

/**
 @brief Korean: 실시간으로 로봇의 위치를 저장하는 스레드
 @brief English: 
 */
void GlobalLocalizationSupervisor::startSavingPathThrerad()
{
	m_SavingPathThread.start(doSavingLastRobotPosThread,this,1000, "GlobalLocalizationSupervisor::startSavingPathThrerad()");
}

/**
 @brief Korean: 실시간으로 로봇의 위치를 저장하는 스레드
 @brief English: 
*/
void GlobalLocalizationSupervisor::terminateSavingPathThread()
{
	m_SavingPathThread.terminate();
}