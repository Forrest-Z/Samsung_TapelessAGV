#include "stdafx.h"
#include "GlobalLocalizerSupervisor.h"

GlobalLocalizationSupervisor::GlobalLocalizationSupervisor()
{
	m_nCeilingCamLogIdx=0;
	m_RobotPos.init(); //�κ� ��ġ �ʱ�ȭ
}

GlobalLocalizationSupervisor::~GlobalLocalizationSupervisor()
{
	
}

/**
@brief Korean:  �ʱ� �̹��� ������ ���� �����۾�
@brief English: 
*/
void GlobalLocalizationSupervisor::initialize()
{
	m_nCeilingCamLogIdx=0;
}

/**
@brief Korean:  �ʱ� �̹��� ������ ���� �����۾�
@brief English: 
*/
void GlobalLocalizationSupervisor::doProcessInit()
{
	KuSURFbasedGlobalLocalizerPr::getInstance()->initialize();
	m_nCeilingCamLogIdx=0;
}

/**
@brief Korea: õ�� �̹����� SURF �� ���� Look up table�� �����Ѵ�.
@brief English:
*/
void GlobalLocalizationSupervisor::saveCeilingData(IplImage* IplCeilingImage)
{

	IplImage* IplCopyImage = cvCreateImage(cvSize(IplCeilingImage->width,IplCeilingImage->height),8,1);
	cvCopy(IplCeilingImage,IplCopyImage);
	cvFlip(IplCopyImage,IplCopyImage,0);
	//--------------õ�� �̹��� ���----------------------------------
	char cFilePathName[150];
	memset(cFilePathName,0,sizeof(cFilePathName));
	sprintf_s(cFilePathName,"./Data/Path/Image/img_%d.bmp", m_nCeilingCamLogIdx);
	//--------------õ�� �̹��� ���----------------------------------
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
@brief Korean: ������ ���̽��� �̹����� �ҷ��ͼ� keypoints �� descriptors Look up table�� �ۼ��Ѵ�.
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
@brief Korean: ������ ���̽��� �̹����� ���� ��ġ�� �޾ƿ´�
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
@brief Korean: ������ ���̽��� �̹����� ���� ��ġ�� �޾ƿ´�
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
@brief Korean:  �κ� ��ġ �ʱ�ȭ
@brief English: 
*/
void GlobalLocalizationSupervisor::init()
{
	m_RobotPos.init(); //�κ� ��ġ �ʱ�ȭ
}

/**
@brief Korean:  �κ� ��ġ�� �����Ѵ�
@brief English: 
*/
void GlobalLocalizationSupervisor::setRobotPos(KuPose RobotPos)
{
	m_RobotPos = RobotPos;

}

/**
@brief Korean:  �κ��� X��ǥ�� �����Ѵ�
@brief English: 
*/
void GlobalLocalizationSupervisor::setRobotPosX(double dPoseX)
{
	m_RobotPos.setX(dPoseX);
}
/**
@brief Korean:  �κ��� Y��ǥ�� �����Ѵ�
@brief English: 
*/
void GlobalLocalizationSupervisor::setRobotPosY(double dPoseY)
{
	m_RobotPos.setY(dPoseY);
}
/**
@brief Korean:  �κ��� ���� Degree�� �����Ѵ�
@brief English: 
*/
void GlobalLocalizationSupervisor::setRobotPosDeg(double dPoseDeg)
{
	m_RobotPos.setThetaDeg(dPoseDeg);
}
/**
@brief Korean:  �κ��� ���� Radian�� �����Ѵ�
@brief English: 
*/
void GlobalLocalizationSupervisor::setRobotPosRad(double dPoseRad)
{
	m_RobotPos.setThetaRad(dPoseRad);
}

/**
@brief Korean: ���������� �����Ѵ�
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
@brief Korean:  �κ��� X��ǥ�� ���� ����
@brief English:
*/
double GlobalLocalizationSupervisor::getRobotPosX()
{
	return m_RobotPos.getX();
}
/**
@brief Korean:  �κ��� Y��ǥ�� ���� ����
@brief English:
*/
double GlobalLocalizationSupervisor::getRobotPosY()
{
	return m_RobotPos.getY();
}
/**
@brief Korean:  �κ���  ����(Degree)�� ���� ����
@brief English:
*/
double GlobalLocalizationSupervisor::getRobotPosDeg()
{
	return m_RobotPos.getThetaDeg();
}
/**
@brief Korean:  �κ���  ����(Radian)�� ���� ����
@brief English:
*/
double GlobalLocalizationSupervisor::getRobotPosRad()
{
	return m_RobotPos.getThetaRad();
}
/**
@brief Korean:  �κ���  ��ġ ���� ���� ����
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
 @brief Korean: ������ �κ� ��ġ �ҷ����� �Լ�
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
 @brief Korean: �ǽð����� �κ��� ��ġ�� �����ϴ� ������
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
 @brief Korean: �ǽð����� �κ��� ��ġ�� �����ϴ� ������
 @brief English: 
 */
void GlobalLocalizationSupervisor::startSavingPathThrerad()
{
	m_SavingPathThread.start(doSavingLastRobotPosThread,this,1000, "GlobalLocalizationSupervisor::startSavingPathThrerad()");
}

/**
 @brief Korean: �ǽð����� �κ��� ��ġ�� �����ϴ� ������
 @brief English: 
*/
void GlobalLocalizationSupervisor::terminateSavingPathThread()
{
	m_SavingPathThread.terminate();
}