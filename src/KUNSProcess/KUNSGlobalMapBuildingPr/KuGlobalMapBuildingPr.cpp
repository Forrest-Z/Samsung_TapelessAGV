#include "stdafx.h"
#include "KuGlobalMapBuildingPr.h"

KuGlobalMapBuildingPr::KuGlobalMapBuildingPr()
{
	m_nHalfSizeX = KuRobotParameter::getInstance()->getUpdateValX();
	m_nHalfSizeY = KuRobotParameter::getInstance()->getUpdateValY();
	m_nx = KuRobotParameter::getInstance()->getCheckValX();
	m_ny = KuRobotParameter::getInstance()->getCheckValY();
	m_dfx=KuRobotParameter::getInstance()->getCeilingCameraPrameterFx();
	m_dfy=KuRobotParameter::getInstance()->getCeilingCameraPrameterFy();
	m_dcx=KuRobotParameter::getInstance()->getCeilingCameraPrameterCx();
	m_dcy=/*Sensor::CEILING_IMAGE_HEIGHT-*/KuRobotParameter::getInstance()->getCeilingCameraPrameterCy();
}

KuGlobalMapBuildingPr::~KuGlobalMapBuildingPr()
{

}

void KuGlobalMapBuildingPr::init()
{

}

void KuGlobalMapBuildingPr::cvtRetinexImage(vector<Mat>& vecmatImages, vector<Mat>& vecmatRetinexImages)
{
	KuRetinexPr RetinexPr;
	for(int i=0; i<vecmatImages.size(); i++)
	{
		Mat matTempRetinexImage;
		matTempRetinexImage.create(Sensor::CEILING_IMAGE_HEIGHT,Sensor::CEILING_IMAGE_WIDTH,CV_8UC1);
		RetinexPr.doRetinexProcess(vecmatImages[i],matTempRetinexImage,30);
		vecmatRetinexImages.push_back(matTempRetinexImage);
	}
}

void KuGlobalMapBuildingPr::affineCeilingImage(vector<KuPose> vecImagePath, vector<Mat>& vecmatCeilingImages, vector<Mat>& vecmatCeilingAffineImages)
{
	double dcx = m_dcx;
	double dcy = m_dcy;
	int nIdx=0;

	while(nIdx<vecImagePath.size()){
		double dImageTheta = vecImagePath[nIdx].getThetaDeg();

		Mat matTempImage;
		matTempImage.create(Sensor::CEILING_IMAGE_HEIGHT,Sensor::CEILING_IMAGE_WIDTH,CV_8UC1);
		affineCeilingImage(vecmatCeilingImages[nIdx],matTempImage,dImageTheta);
		vecmatCeilingAffineImages.push_back(matTempImage);

		nIdx++;
	}
}

void KuGlobalMapBuildingPr::affineCeilingImage(Mat& matCeilingImage, Mat& matCeilingAffineImage, double dTheta)
{
	double dcx = m_dcx;
	double dcy = m_dcy;

	double dImageTheta = dTheta;
	if(dImageTheta>=90) dImageTheta=-270+dImageTheta;
	else dImageTheta=90+dImageTheta;
	printf("%f\n",dImageTheta);

	Mat center = getRotationMatrix2D(Point2i((int)(dcx+0.5),(int)(dcy+0.5)),dImageTheta,1);
	warpAffine(matCeilingImage,matCeilingAffineImage,center,matCeilingAffineImage.size());//õ�念��	
}

void KuGlobalMapBuildingPr::affineCeilingImage2(Mat& matCeilingImage, Mat& matCeilingAffineImage, double dTheta)
{
	double dcx = m_dcx;
	double dcy = m_dcy;

	double dImageTheta = dTheta;
	if(dImageTheta>=90) dImageTheta=-270+dImageTheta;
	else dImageTheta=90+dImageTheta;

	Mat matTempImage;
	matTempImage.create(matCeilingAffineImage.rows,matCeilingAffineImage.cols,CV_8UC1);
	for(int i=0; i<matTempImage.cols*matTempImage.rows; i++)
	{
		matTempImage.data[i]=0;
	}
	for(int i=0; i<matCeilingImage.rows; i++)
	{
		for(int j=0; j<matCeilingImage.cols; j++)
		{
			matTempImage.data[i*matTempImage.cols+j]=matCeilingImage.data[i*matCeilingImage.cols+j];
		}
	}

	Mat center = getRotationMatrix2D(Point2f((float)(dcx),(float)(dcy)),dImageTheta,1);
	warpAffine(matTempImage,matCeilingAffineImage,center,matCeilingAffineImage.size());//õ�念��	
}

void KuGlobalMapBuildingPr::buildGlobalMap(vector<KuPose>& vecImagePath, vector<Point2i>& vecImgPixcelPath, vector<Mat>& vecmatCeilingImages, vector<Mat>& vecmatCeilingAffineImages, vector<Mat>& vecmatRetinexImages, vector<Mat>& vecmatRetinexAffineImages)
{
	vecImgPixcelPath.clear();

	//�ʱ� ����
	double dCeilingHeigh = KuRobotParameter::getInstance()->getHeight()/2;
	double dScale = 1.0;

	//find global map properties : �������� ũ��(x,y pixcel ũ��)
	Point2d ptMinXMaxY;//���� �������� x�� �ּҰ� y�� �ִ밪
	Point2i ptInitialPixcel;//
	int nGlobalMapWidth;
	int nGlobalMapHeigh;
	vector<double> vecdDirection;
	findGlobalMapProperties(vecImagePath, dCeilingHeigh, dScale, ptMinXMaxY, ptInitialPixcel, nGlobalMapWidth, nGlobalMapHeigh,vecdDirection);

	//initialize global map
	Mat matGlobalMap,matRetinexGlobalMap, matGlobalSegment;
	matGlobalMap.create(nGlobalMapHeigh,nGlobalMapWidth,CV_8UC1);
	matRetinexGlobalMap.create(nGlobalMapHeigh,nGlobalMapWidth,CV_8UC1);
	matGlobalSegment.create(nGlobalMapHeigh,nGlobalMapWidth,CV_8UC1);

	if(vecImagePath.size()>0&&vecmatCeilingAffineImages.size()>0)
	{
		Point2d ptPreImagePos = Point2d(vecImagePath[0].getX(),vecImagePath[0].getY());

		ptInitialPixcel.x=(ptPreImagePos.x-ptMinXMaxY.x)*m_dfx/(dCeilingHeigh*100)+640;
		ptInitialPixcel.y=(ptMinXMaxY.y-ptPreImagePos.y)*m_dfy/(dCeilingHeigh*100)+480;
		initGlobalMap(vecmatCeilingAffineImages[0], vecmatRetinexAffineImages[0],Point2d(m_dcx,m_dcy),ptInitialPixcel,matGlobalMap,matGlobalSegment,matRetinexGlobalMap);
		vecImgPixcelPath.push_back(ptInitialPixcel);

		CeilingImageBasedParticleFilter CeilingImgbasedParticleFilter;
		
		int nIterationNum = vecImagePath.size();
		for(int i=1; i<nIterationNum; i++)
		{
			printf("Global Map Building Process : %d / %d\n",(i+1),nIterationNum);
			Point2i ptnGlobalPos = Point2i(vecImgPixcelPath[i-1].x,vecImgPixcelPath[i-1].y);
			Point2i ptNewImageCtrPoint;

			double dDelTheta = 0.;
			dDelTheta=CeilingImgbasedParticleFilter.doImagebasedParticlefilter(ptnGlobalPos,vecmatRetinexAffineImages[i],matRetinexGlobalMap,ptNewImageCtrPoint, m_nx, m_ny);
			double dTheta = vecImagePath[i].getThetaDeg()+dDelTheta;
			doMergeGlobalMap(vecmatCeilingImages[i],vecmatRetinexImages[i],matGlobalMap,matGlobalSegment, matRetinexGlobalMap, Point2f(m_dcx,m_dcy),ptNewImageCtrPoint,vecImgPixcelPath[i-1], dTheta, 640, 640, vecdDirection[i]);
			vecImgPixcelPath.push_back(Point2i((int)(ptNewImageCtrPoint.x+0.5),(int)(ptNewImageCtrPoint.y+0.5)));
		}
	}
}

void KuGlobalMapBuildingPr::saveImagePixcelPath(vector<Point2i> vecImgPixcelPath)
{
	string strDataPath =KuRobotParameter::getInstance()->getTeachingPathNameNPath();	
	string strNewPath;
	char cFilePathName[300];
	memset(cFilePathName,0,sizeof(cFilePathName));
	sprintf(cFilePathName,"%s/Imagepixel.txt",strDataPath.c_str());
	strNewPath=cFilePathName;//char �����Ͱ��� string�� ����

	ofstream DataLog;
	DataLog.open(strNewPath);
	//�������
	for(int i=0; i<vecImgPixcelPath.size(); i++){		
		DataLog<<vecImgPixcelPath[i].x<<" "<<vecImgPixcelPath[i].y<<endl;
	}
	DataLog.close();
}

void KuGlobalMapBuildingPr::findGlobalMapProperties(vector<KuPose> vecImagePath, double dCeilingHeigh, double dScale , Point2d& ptMinXMaxY, Point2i& ptInitial, int& nGlobalMapWidth, int& nGlobalMapHeigh, vector<double>& vecptdDirection)
{
	int nXMargin = 640;
	int nYMargin = 480;

	//������ ���� ������ ũ�⸦ �����ϴ� �κ�----------------------------------------------------------------------//
	//x,y�ִ� �ּ�
	double dPoseX_Max=vecImagePath[0].getX();
	double dPoseX_Min=vecImagePath[0].getX();
	double dPoseY_Max=vecImagePath[0].getY();
	double dPoseY_Min=vecImagePath[0].getY();
	for(int i=1; i<vecImagePath.size(); i++)
	{
		double dtempX=vecImagePath[i].getX();
		double dtempY=vecImagePath[i].getY();
		if(dPoseX_Max<dtempX)
			dPoseX_Max=dtempX;
		if(dPoseX_Min>dtempX)
			dPoseX_Min=dtempX;
		if(dPoseY_Max<dtempY)
			dPoseY_Max=dtempY;
		if(dPoseY_Min>dtempY)
			dPoseY_Min=dtempY;
	}

	ptMinXMaxY.y=dPoseY_Max;
	ptMinXMaxY.x=dPoseX_Min;
	
	double dDistfromInitialPTtoMinX = vecImagePath[0].getX() - dPoseX_Min;
	double dDistfromInitialPTtoMinY = vecImagePath[0].getY() - dPoseY_Max;
	ptInitial.x = (int)((dDistfromInitialPTtoMinX*m_dfx/(dCeilingHeigh*100))*dScale+((double)nXMargin)*dScale);
	ptInitial.y = (int)((dDistfromInitialPTtoMinY*m_dfy/(dCeilingHeigh*100))*dScale+((double)nYMargin)*dScale);

	nGlobalMapWidth = (int)(((dPoseX_Max-dPoseX_Min)*m_dfx/(dCeilingHeigh*100))*dScale+2*((double)nXMargin)*dScale);
	nGlobalMapHeigh = (int)(((dPoseY_Max-dPoseY_Min)*m_dfy/(dCeilingHeigh*100))*dScale+2*((double)nYMargin)*dScale);

	//������ ������Ʈ �Ǵ� ������ ����-----------------------------------------------------------------------------//
	vecptdDirection.clear();
	vecptdDirection.push_back(0.0);
	for(int i=1; i<vecImagePath.size(); i++)
	{
		double dDelX=vecImagePath[i].getX()-vecImagePath[i-1].getX();
		double dDelY=vecImagePath[i].getY()-vecImagePath[i-1].getY();
		vecptdDirection.push_back(atan2(dDelY,dDelX));
	}
}

void KuGlobalMapBuildingPr::initGlobalMap(Mat matCeilingImage, Mat matRetinexImage, Point2d ptInitialImageCenter, Point2i ptInitialPixcel, Mat& MatGlobalMap, Mat& MatGlobalSegment, Mat& matRetinexGlobalMap)
{
	//�������� �ʱ�ȭ
	for(int i=0; i<MatGlobalMap.rows*MatGlobalMap.cols; i++)
	{
		MatGlobalMap.data[i]=0;
		MatGlobalSegment.data[i]=0;
		matRetinexGlobalMap.data[i]=0;
	}

	//ȸ���߽�
	Point2i ptcenter;
	ptcenter.x = (int)(ptInitialImageCenter.x+0.5);
	ptcenter.y = (int)(ptInitialImageCenter.y+0.5);

	for(int i=-m_nHalfSizeY;i<(m_nHalfSizeY+1);i++)
	{
		for(int j=-m_nHalfSizeX; j<(m_nHalfSizeX+1);j++)
		{
			int nDBIdx = (ptcenter.y+i)*matRetinexImage.cols+ptcenter.x+j;
			int nGMIdx = (ptInitialPixcel.y+i)*MatGlobalMap.cols+ptInitialPixcel.x+j;
			if(nDBIdx<matRetinexImage.cols*matRetinexImage.rows&&nDBIdx>=0&&
				nGMIdx<MatGlobalMap.cols*MatGlobalMap.rows&&nGMIdx>=0)
			{
				MatGlobalMap.data[nGMIdx]=matCeilingImage.data[nDBIdx];
				matRetinexGlobalMap.data[nGMIdx]=matRetinexImage.data[nDBIdx];
				MatGlobalSegment.data[nGMIdx]=255;
			}
		}
	}
}

void KuGlobalMapBuildingPr::doMergeGlobalMap(Mat matCeilingImage, Mat matCeilingRetinexImage, Mat& matGlobalMap, Mat& matGlobalSegment, Mat& matRetinexGlobalMap, Point2f ptPrincipalPoint, Point2d ptdMergeCtrPoint, Point2i ptPreCtrPoint, double dTheta, int nx, int ny, double dDirection)
{
	Point2i ptMergeCtrPoint = Point2i((int)(ptdMergeCtrPoint.x),(int)(ptdMergeCtrPoint.y));
	
	Mat matImage,matRetinexImage;
	matImage.create(Sensor::CEILING_IMAGE_HEIGHT,Sensor::CEILING_IMAGE_WIDTH,CV_8UC1);
	matRetinexImage.create(Sensor::CEILING_IMAGE_HEIGHT,Sensor::CEILING_IMAGE_WIDTH,CV_8UC1);

	for(int i=0; i<Sensor::CEILING_IMAGE_HEIGHT*Sensor::CEILING_IMAGE_WIDTH;i++)
	{
		matImage.data[i]=matCeilingImage.data[i];
		matRetinexImage.data[i]=matCeilingRetinexImage.data[i];
	}

	Mat matInputAffineImage;
	matInputAffineImage.create(Sensor::CEILING_IMAGE_HEIGHT,Sensor::CEILING_IMAGE_WIDTH,CV_8UC1);
	affineCeilingImage(matImage,matInputAffineImage,dTheta);

	Mat matRetinexAffineImage;
	matRetinexAffineImage.create(Sensor::CEILING_IMAGE_WIDTH,Sensor::CEILING_IMAGE_WIDTH,CV_8UC1);
	affineCeilingImage2(matRetinexImage,matRetinexAffineImage,dTheta);

	double dA = D2R*(dDirection+90);//���� ������Ʈ ���� + 90��
	Point2d ptdA = Point2d(cos(dA),sin(dA));//���� dA�� ���� ���� ����
	printf("A: %f, P(%f, %f)\n",dA,ptdA.x,ptdA.y);
	//double db = -da*((double)ptPreCtrPoint.x)+(double)ptPreCtrPoint.y;
	//���� ������Ʈ
	for(int i=-m_nHalfSizeY;i<(m_nHalfSizeY+1);i++)
	{
		for(int j=-m_nHalfSizeX; j<(m_nHalfSizeX+1);j++)
		{
			int nGlobalMapIdx = ((int)(ptMergeCtrPoint.y+0.5)+i)*matGlobalMap.cols+(int)(ptMergeCtrPoint.x+0.5)+j;
			int nCeilingIdx = ((int)(ptPrincipalPoint.y)+i)*matInputAffineImage.cols+(int)(ptPrincipalPoint.x)+j;
			int nRetinexIdx = ((int)(ptPrincipalPoint.y)+i)*matRetinexAffineImage.cols+(int)(ptPrincipalPoint.x)+j;

			if(nGlobalMapIdx>=0&&nGlobalMapIdx<matGlobalMap.cols*matGlobalMap.rows&&
				nCeilingIdx>=0&&nCeilingIdx<matRetinexImage.cols*matRetinexImage.rows&&
				nRetinexIdx>=0&&nRetinexIdx<matRetinexAffineImage.cols*matRetinexAffineImage.rows)//����ó��
			{
				bool bMerge=false;
				//---------------------------AGV �̵� ���⸸ updata �ϱ����� index-----------------------------------//
				Point2d ptdP = Point2d((double)j,-(double)i);//-��ȣ�� ���� ��ǥ�� ���� ��ǥ�� y���� �ݴ�����̹Ƿ�
				double dDet = (ptdA.x*ptdP.y) - (ptdA.y*ptdP.x);
				if(dDet<0)
				{
					bMerge=true;
				}
				//---------------------------AGV �̵� ���⸸ updata �ϱ����� index-----------------------------------//
				if(bMerge)
				{
					matGlobalMap.data[nGlobalMapIdx]=matInputAffineImage.data[nCeilingIdx];
					matRetinexGlobalMap.data[nGlobalMapIdx]=matRetinexAffineImage.data[nRetinexIdx];
					matGlobalSegment.data[nGlobalMapIdx]=255;
				}
			}
		}
	}
	imwrite("./data/map/GlobalMap.bmp",matGlobalMap);
	imwrite("./data/map/RGlobalMap.bmp",matRetinexGlobalMap);
	imwrite("./data/map/SGlobalMap.bmp",matGlobalSegment);
}