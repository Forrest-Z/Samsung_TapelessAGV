#include <stdafx.h>
#include "ALRecognizer.h"

CALRecognizer::CALRecognizer(void)
{
	//이미지 프로세서 초기화
	m_pFrameThresholder = new FrameThresholder;
	m_pFidtrackFinder = new FidtrackFinder;

	m_nLandMarkNum = 0;
	m_dHeight_Camera2Mark = 0.0;
	m_nHeight_Camera2Mark = 0;
	//랜드마크의 개수 초기화
	m_nLandMarkNum =KuRobotParameter::getInstance()->getLandMarkNum();
	//카메라로부터 랜드마크까지의 높이 초기화
	m_nHeight_Camera2Mark = KuRobotParameter::getInstance()->getHeightCamera2Mark();
	m_dHeight_Camera2Mark = (double)m_nHeight_Camera2Mark/1000;//mm로 얻어오기때문에 m로 단위변환

	loadCeilingInfo2changeHeigh();
}

CALRecognizer::~CALRecognizer(void)
{
	delete m_pFrameThresholder;
	delete m_pFidtrackFinder;

}

//특정 id의 마크의 camera2heigh를 바꿔주는 부분140309(허환)
vector<int> m_vecnMarkIdx2changeHeigh;
vector<double> m_vecdCeilingHeigh2change;//m단위
void CALRecognizer::loadCeilingInfo2changeHeigh()
{
	m_vecnMarkIdx2changeHeigh.clear();
	m_vecdCeilingHeigh2change.clear();

	char cFilePathName[300];
	string strDataImagPath =KuRobotParameter::getInstance()->getTeachingPathNameNPath();
	string strNewPath;
	memset(cFilePathName,0,sizeof(cFilePathName));
	sprintf(cFilePathName,"%s/Camera2Mark.txt",strDataImagPath.c_str());
	strNewPath=cFilePathName;

	ifstream DataLog;
	DataLog.open(strNewPath);
	int nIdx;
	double dHeigh;
	while(!DataLog.eof()){
		KuPose PathPos;
		DataLog >> nIdx >> dHeigh;
		m_vecnMarkIdx2changeHeigh.push_back(nIdx);
		m_vecdCeilingHeigh2change.push_back(dHeigh);
	}
	m_vecnMarkIdx2changeHeigh.pop_back();
	m_vecdCeilingHeigh2change.pop_back();
	DataLog.close();
}

void CALRecognizer::trans_global2img(const float& fRx, const float& fRy, const float& fRth, const float& fFx, const float& fFy, const float& fFz, int& nu, int& nv, const CamContext* pContext)
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

void CALRecognizer::trans_img2global(const float& fRx, const float& fRy, const float& fRth, const int& nu, const int& nv, float& fFx, float& fFy, float& fFz, const CamContext* pContext, double dHeight)
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

KuPose CALRecognizer::calcFiducialLandMarkPosimg2global(list <KuPose> lFiducialLandMarkPos, KuPose RobotPos)
{
	KuPose FiducialLandMarkGolbalPos;
	bool bSendFlag = false;
	//거리, 각도 값이 0,0 일 경우 전송하지 않는다. 

	//랜드마크를 감지 하였을 경우
	for(list <KuPose>::iterator iter_FiducialLandMarkPos = lFiducialLandMarkPos.begin();iter_FiducialLandMarkPos != lFiducialLandMarkPos.end(); iter_FiducialLandMarkPos++ )
	{
		float dFiducialMarkX= .0;
		float dFiducialMarkY= .0;
		float dFiducialMarkZ= .0;

		int nID = iter_FiducialLandMarkPos->getID();
		if(nID<m_nLandMarkNum)
		{
			//인식된 랜드마크가 미리 지정된 랜드마크일경우에만 사용
//			int nu = (int)(iter_FiducialLandMarkPos->getX()); 
//			int nv=  ((int)(Sensor::CEILING_IMAGE_HEIGHT-iter_FiducialLandMarkPos->getY()));
			int nu = (int)(Sensor::CEILING_IMAGE_WIDTH-iter_FiducialLandMarkPos->getX()); //여기
			int nv=  ((int)(iter_FiducialLandMarkPos->getY()));

			const double dRobotPosX = RobotPos.getXm();// mm->m
			const double dRobotPosY = RobotPos.getYm();// mm->m
			const double dRobotPosThetaRad = RobotPos.getThetaRad();
			int nDistance = 0;
			int nThetaDeg = 0;

			bool bchangHeigh=false;
			int nIdx2change=-1;
			for(int i=0; i<m_vecnMarkIdx2changeHeigh.size(); i++)
			{
				if(nID==m_vecnMarkIdx2changeHeigh[i])
				{
					bchangHeigh=true;
					nIdx2change = i;
				}
			}
			if(bchangHeigh)
			{
				if(!(nIdx2change<0))
				{
					trans_img2global(dRobotPosX, dRobotPosY,dRobotPosThetaRad,
						nu,nv,	dFiducialMarkX,dFiducialMarkY,dFiducialMarkZ,
						KuSiriusCameraInterface::getInstance()->getCamContext(),m_vecdCeilingHeigh2change[nIdx2change]);
				}				
			}
			else
			{
				trans_img2global(dRobotPosX, dRobotPosY,dRobotPosThetaRad,
					nu,nv,	dFiducialMarkX,dFiducialMarkY,dFiducialMarkZ,
					KuSiriusCameraInterface::getInstance()->getCamContext(),m_dHeight_Camera2Mark);
			}

			//img2global(KuSiriusCameraInterface::getInstance()->getCamContext(), dRobotPosX, dRobotPosY,dRobotPosThetaRad,dFiducialMarkX,dFiducialMarkY,dFiducialMarkZ,nu,nv);			

			FiducialLandMarkGolbalPos.setPro(m_FiducialLandMarkGolbalPos.getPro()+1);
			FiducialLandMarkGolbalPos.setID(nID);
			FiducialLandMarkGolbalPos.setX(dFiducialMarkX*1000);// m->mm
			FiducialLandMarkGolbalPos.setY(dFiducialMarkY*1000);// m->mm
			FiducialLandMarkGolbalPos.setZ(dFiducialMarkZ*1000);// m->mm
			FiducialLandMarkGolbalPos.setThetaRad(iter_FiducialLandMarkPos->getThetaRad());
			double dDistanceRobot2FiducialMark = _hypot(FiducialLandMarkGolbalPos.getX()-RobotPos.getX(),FiducialLandMarkGolbalPos.getY()-RobotPos.getY());
			FiducialLandMarkGolbalPos.setDist(dDistanceRobot2FiducialMark);
			//랜드마크를 하나만 보도록하는 구문
			bSendFlag = true;
			break;
			//iter_FiducialLandMarkPos = lFiducialLandMarkPos.end();
		}
	}

	if(bSendFlag == false){
		FiducialLandMarkGolbalPos.setID(-1);
		FiducialLandMarkGolbalPos.setX(0);// m->mm
		FiducialLandMarkGolbalPos.setY(0);// m->mm
		FiducialLandMarkGolbalPos.setZ(0);// m->mm
		FiducialLandMarkGolbalPos.setThetaRad(0);
		FiducialLandMarkGolbalPos.setDist(0);
	}
	else if(bSendFlag == true){//인식된 랜드마크가 미리 지정된 랜드마크일경우에만 사용
		FiducialLandMarkGolbalPos = FiducialLandMarkGolbalPos;
	}

	return FiducialLandMarkGolbalPos;
	//확률을 이용해서 하는 구문
	/*
	KuPose FiducialLandMarkGolbalPos;

	//거리, 각도 값이 0,0 일 경우 전송하지 않는다. 
	list <KuPose> lFiducialLandMarkGolbalPosList;

	if(lFiducialLandMarkPos.size()==0){//랜드마크를 감지하지 못했을때
	m_FiducialLandMarkGolbalPos.setID(-1);
	m_FiducialLandMarkGolbalPos.setX(0);// m->mm
	m_FiducialLandMarkGolbalPos.setY(0);// m->mm
	m_FiducialLandMarkGolbalPos.setZ(0);// m->mm
	m_FiducialLandMarkGolbalPos.setThetaRad(0);
	m_FiducialLandMarkGolbalPos.setDist(0);
	m_FiducialLandMarkGolbalPos.setPro(0);

	}
	else{ //랜드마크를 감지 하였을 경우
	for(list <KuPose>::iterator iter_FiducialLandMarkPos = lFiducialLandMarkPos.begin();iter_FiducialLandMarkPos != lFiducialLandMarkPos.end(); iter_FiducialLandMarkPos++ )
	{
	float dFiducialMarkX= .0 ;
	float dFiducialMarkY= .0;
	float dFiducialMarkZ= .0;

	int nID = iter_FiducialLandMarkPos->getID();

	int nu = (int)(iter_FiducialLandMarkPos->getX());
	int nv=  (IMG_HEIGHT-(int)(iter_FiducialLandMarkPos->getY()));

	const double dRobotPosX = RobotPos.getXm();// mm->m
	const double dRobotPosY = RobotPos.getYm();// mm->m
	const double dRobotPosThetaRad = RobotPos.getThetaRad();
	int nDistance = 0;
	int nThetaDeg = 0;

	img2global(m_SLAMContext, dRobotPosX, dRobotPosY,dRobotPosThetaRad,dFiducialMarkX,dFiducialMarkY,dFiducialMarkZ,nu,nv);			
	FiducialLandMarkGolbalPos.setPro(m_FiducialLandMarkGolbalPos.getPro()+1);
	FiducialLandMarkGolbalPos.setID(nID);
	FiducialLandMarkGolbalPos.setX(dFiducialMarkX*1000);// m->mm
	FiducialLandMarkGolbalPos.setY(dFiducialMarkY*1000);// m->mm
	FiducialLandMarkGolbalPos.setZ(dFiducialMarkZ*1000);// m->mm
	FiducialLandMarkGolbalPos.setThetaRad(iter_FiducialLandMarkPos->getThetaRad());
	double dDistanceRobot2FiducialMark = hypot(FiducialLandMarkGolbalPos.getX()-RobotPos.getX(),FiducialLandMarkGolbalPos.getY()-RobotPos.getY());
	FiducialLandMarkGolbalPos.setDist(dDistanceRobot2FiducialMark);


	if((m_FiducialLandMarkGolbalPos.getID() != FiducialLandMarkGolbalPos.getID() ) && (FiducialLandMarkGolbalPos.getID() != -1) ){ //처음이거나 새로운 랜드마크의 정보가 들어온경우
	m_FiducialLandMarkGolbalPos.setPro(0);
	m_FiducialLandMarkGolbalPos.setID(nID);
	m_FiducialLandMarkGolbalPos.setX(dFiducialMarkX*1000);// m->mm
	m_FiducialLandMarkGolbalPos.setY(dFiducialMarkY*1000);// m->mm
	m_FiducialLandMarkGolbalPos.setZ(dFiducialMarkZ*1000);// m->mm
	m_FiducialLandMarkGolbalPos.setThetaRad(iter_FiducialLandMarkPos->getThetaRad());
	double dDistanceRobot2FiducialMark = hypot(FiducialLandMarkGolbalPos.getX()-RobotPos.getX(),FiducialLandMarkGolbalPos.getY()-RobotPos.getY());
	m_FiducialLandMarkGolbalPos.setDist(dDistanceRobot2FiducialMark);
	}
	else if(m_FiducialLandMarkGolbalPos.getID() == FiducialLandMarkGolbalPos.getID() && (FiducialLandMarkGolbalPos.getID() != -1)){ //동일한 랜드마크의 정보가들어온경우
	m_FiducialLandMarkGolbalPos.setPro(m_FiducialLandMarkGolbalPos.getPro()+1);
	m_FiducialLandMarkGolbalPos.setID(nID);
	m_FiducialLandMarkGolbalPos.setX(dFiducialMarkX*1000);// m->mm
	m_FiducialLandMarkGolbalPos.setY(dFiducialMarkY*1000);// m->mm
	m_FiducialLandMarkGolbalPos.setZ(dFiducialMarkZ*1000);// m->mm
	m_FiducialLandMarkGolbalPos.setThetaRad(iter_FiducialLandMarkPos->getThetaRad());
	double dDistanceRobot2FiducialMark = hypot(FiducialLandMarkGolbalPos.getX()-RobotPos.getX(),FiducialLandMarkGolbalPos.getY()-RobotPos.getY());
	m_FiducialLandMarkGolbalPos.setDist(dDistanceRobot2FiducialMark);
	}

	//랜드마크를 하나만 보도록하는 구문
	break;
	//iter_FiducialLandMarkPos = lFiducialLandMarkPos.end();

	}
	}
	//감지된 랜드마크의 확률을 고려해서 반환해주는 부분
	if(m_FiducialLandMarkGolbalPos.getPro()<2){
	FiducialLandMarkGolbalPos.setID(-1);
	FiducialLandMarkGolbalPos.setX(0);// m->mm
	FiducialLandMarkGolbalPos.setY(0);// m->mm
	FiducialLandMarkGolbalPos.setZ(0);// m->mm
	FiducialLandMarkGolbalPos.setThetaRad(0);
	FiducialLandMarkGolbalPos.setDist(0);

	}
	else if(m_FiducialLandMarkGolbalPos.getPro()>2){
	FiducialLandMarkGolbalPos = m_FiducialLandMarkGolbalPos;
	}

	return FiducialLandMarkGolbalPos;
	*/
}

void CALRecognizer::setRecogFiducialLandMarkList(list<KuPose> RecogFiducialLandMarkList)
{
	m_RecogFiducialLandMarkList = RecogFiducialLandMarkList;
}

list<KuPose> CALRecognizer::getRecogFiducialLandMarkList()
{
	return m_RecogFiducialLandMarkList;
}


KuPose CALRecognizer::start(IplImage* inimg, KuPose RobotPos)
{
	for(int ii = 0 ;ii < inimg->height ; ii++)
	{
		for(int jj = 0 ; jj< inimg->width ; jj++)
		{
			m_srcImage[(inimg->height-ii)* inimg ->width + jj ] = (unsigned char)inimg->imageData[ii*inimg->width+jj];
		}
	}

	//영상을 thresholding 한다.
	m_pFrameThresholder->process(m_srcImage,m_buffImage);
	memcpy(m_srcImage,m_buffImage,sizeof(m_buffImage));	
	//thresholding 영상에서 인공 마크를 찾는다.
	list <KuPose> lFiducialLandMarkPosImgList = m_pFidtrackFinder->process(m_srcImage,m_buffImage);
	m_RecogFiducialLandMarkList = lFiducialLandMarkPosImgList;
	KuPose FiducialLandMarkGlobalPos= calcFiducialLandMarkPosimg2global(lFiducialLandMarkPosImgList,RobotPos); // 인식된 mark의 global 3D pose
	//TRACE("ID = %d X = %d  Y = %d  Th = %d Dist = %d Pro = %d\n",FiducialLandMarkGolbalPos.getID(), (int)FiducialLandMarkGolbalPos.getX(),(int)FiducialLandMarkGolbalPos.getY(),(int)FiducialLandMarkGolbalPos.getThetaDeg(),(int)FiducialLandMarkGolbalPos.getDist(),(int)FiducialLandMarkGolbalPos.getPro());
	return FiducialLandMarkGlobalPos;
}
