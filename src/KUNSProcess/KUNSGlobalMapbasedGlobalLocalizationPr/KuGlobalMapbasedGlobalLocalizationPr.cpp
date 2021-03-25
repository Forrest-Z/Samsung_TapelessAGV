#include "stdafx.h"
#include "KuGlobalMapbasedGlobalLocalizationPr.h"

KuGlobalMapbasedGlobalLocalizationPr::KuGlobalMapbasedGlobalLocalizationPr()
{
	m_nNumFeatureTh=KuRobotParameter::getInstance()->getNumSURFFeatureTh();
	m_dcx=KuRobotParameter::getInstance()->getCeilingCameraPrameterCx();
	m_dcy=(Sensor::CEILING_IMAGE_HEIGHT-KuRobotParameter::getInstance()->getCeilingCameraPrameterCy());
}

KuGlobalMapbasedGlobalLocalizationPr::~KuGlobalMapbasedGlobalLocalizationPr()
{

}

void KuGlobalMapbasedGlobalLocalizationPr::initialize()
{
	m_dDeltaX=0;
	m_dDeltaY=0;
	m_dDeltaAngle=0;
}

/**
@brief Korean: 전역 지도 기반 전역 위치추정
@brief English:
*/
bool KuGlobalMapbasedGlobalLocalizationPr::doGlobalMapbasedGlobalLocalization(IplImage* IplCeilingImage, vector<KuPose> vecRobotPose, vector<Point2i> vecnptPixcelPath, KuPose& RobotPos)
{
	Mat matCeilingImage;
	matCeilingImage.create(Sensor::CEILING_IMAGE_HEIGHT,Sensor::CEILING_IMAGE_WIDTH,CV_8UC1);
	for(int i=0; i<Sensor::CEILING_IMAGE_HEIGHT*Sensor::CEILING_IMAGE_WIDTH; i++)
	{
		matCeilingImage.data[i]=IplCeilingImage->imageData[i];
	}
	KuRetinexPr RetinexPr;
	RetinexPr.doRetinexProcess(matCeilingImage,matCeilingImage,30);

	Mat matGlobalMap=imread("./Data/map/RGlobalMap.bmp",0);
	Point2i nptRobotPose;
	int nNearestNode = findRobotPose(matCeilingImage, matGlobalMap, vecRobotPose, vecnptPixcelPath, nptRobotPose);
	
	double dX=vecRobotPose[nNearestNode].getX();
	double dY=vecRobotPose[nNearestNode].getY();
	double dRobotAngleDeg=vecRobotPose[0].getThetaDeg()-m_dDeltaAngle;
// 	double dAngle = dRobotAngleDeg*D2R;
// 
// 	double dDeltaX = (getDeltaX()*cos(dAngle)-getDeltaY()*sin(dAngle))*1000.0;
// 	double dDeltaY = (getDeltaX()*sin(dAngle)+getDeltaY()*cos(dAngle))*1000.0;
//	printf("%f, %f, %f\n",dDeltaX, dDeltaY, dRobotAngleDeg);
	double dDeltaX = getDeltaX();
	double dDeltaY = getDeltaY();

	RobotPos.setX(dX-dDeltaX);
	RobotPos.setY(dY-dDeltaY);
	RobotPos.setThetaDeg(dRobotAngleDeg);

	return m_bRobotAngle;
}

int KuGlobalMapbasedGlobalLocalizationPr::findRobotPose(Mat matCeilingImage, Mat& matGlobalMap, vector<KuPose> vecRobotPose, vector<Point2i> vecnptPixcelPath, Point2i& nptRobotPixcel)
{
	int nSelectPath=-1;//현재 위치와 가장 가까운 노드

	setPoseReliability(false);//로봇 위치 탐색 실패 여부 확인 -> 초기화
	setAngleReliability(false);//로봇 각도 탐색 실패 여부 확인 -> 초기화

	//천장 이미지를 전역 지도의 초기 위치에 저장 후 특징점 추출
	Mat matCeilingAffineImg, matCeilingImg;
	matCeilingAffineImg.create(matCeilingImage.rows,matCeilingImage.cols,CV_8UC1);
	affineCeilingImage(matCeilingImage,matCeilingAffineImg,vecRobotPose[0].getThetaDeg());
	matCeilingImg.create(matGlobalMap.rows,matGlobalMap.cols,CV_8UC1);
	for(int i=0; i<matCeilingImg.cols*matCeilingImg.rows; i++)//천장영상 초기화(전역지도 크기)
	{
		matCeilingImg.data[i]=0;
	}
	for(int i=-(int)(m_dcy+0.5)+1; i<matCeilingAffineImg.rows-(int)(m_dcy+0.5); i++)//천장영상(전역지도크기) data 복사
	{
		for(int j=-(int)(m_dcx+0.5)+1; j<matCeilingAffineImg.cols-(int)(m_dcx+0.5); j++)//(cx, cy)와 pixcelpath 처음 위치에 정합
		{
			int nGlobalIdx=(vecnptPixcelPath[0].y+i)*matCeilingImg.cols+vecnptPixcelPath[0].x+j;
			int nCeilingIdx=(i+(int)(m_dcy+0.5)-1)*matCeilingAffineImg.cols+j+(int)(m_dcx+0.5)-1;
			if(nGlobalIdx<(matCeilingImg.rows*matCeilingImg.cols)&&nCeilingIdx<(matCeilingAffineImg.rows*matCeilingAffineImg.cols))
			{
				matCeilingImg.data[nGlobalIdx]=matCeilingAffineImg.data[nCeilingIdx];
			}
		}
	}
	imwrite("1_Ceiling_Image.bmp",matCeilingImg);//없어도 됨
	vector<KeyPoint> CeilingKeypoint;
	Mat matCeilingDescriptor;
	SURFFeatureExtractingProcess(matCeilingImg,CeilingKeypoint,matCeilingDescriptor);//특징점 추출


	//천장 전역지도 특징점 추출
	vector<KeyPoint> GlobaMapKeypoint;
	Mat matGlobalMapDescriptors;
	SURFFeatureExtractingProcess(matGlobalMap,GlobaMapKeypoint,matGlobalMapDescriptors);

	if(!(CeilingKeypoint.size()==0||GlobaMapKeypoint.size()==0))
	{
		//매칭
		vector<DMatch> Matches;
		FeatureMatchingProcess(CeilingKeypoint,matCeilingDescriptor,GlobaMapKeypoint,matGlobalMapDescriptors,Matches);

		if(Matches.size()>5)
		{
			//Ransac
			vector<DMatch> RansacMatches;
			Mat H = doRansac(CeilingKeypoint,GlobaMapKeypoint,Matches,RansacMatches);

			if(checkMatchingReliability(H,vecnptPixcelPath[0],nptRobotPixcel))
			{
				setPoseReliability(true);//로봇 위치 탐색 성공
				showRansacResult(CeilingKeypoint,GlobaMapKeypoint,RansacMatches,matCeilingImg,matGlobalMap,H);//호모그래피 정합결과(없어도됨)
				
				//현재 위치와 가장 가까운 노드(pixel path) 탐색
				findNearesstNode(nptRobotPixcel,vecnptPixcelPath,nSelectPath);

				//로봇 x,y 탐색
				findInterpolationPose(nptRobotPixcel,nSelectPath,vecRobotPose,vecnptPixcelPath);
				
				//---------------------천장 높이 사용----------------------------//
				//가까운 노드와 현재 위치의 pixcel 변화량
				int nDeltaX = nptRobotPixcel.x - vecnptPixcelPath[nSelectPath].x;
				int nDeltaY = nptRobotPixcel.y - vecnptPixcelPath[nSelectPath].y;

				//projection
				double dfx=KuRobotParameter::getInstance()->getCeilingCameraPrameterFx();
				double dfy=KuRobotParameter::getInstance()->getCeilingCameraPrameterFy();
				double dZ=KuRobotParameter::getInstance()->getHeight();

				double dDeltaY=-(dZ/dfx)*nDeltaX;
				double dDeltaX=(dZ/dfy)*nDeltaY;
				printf("Using Ceiling Inforamtion DeltaX : %f, DeltaY : %f\n",dDeltaX,dDeltaY);
				//---------------------------------------------------------------//
				
				/*-----------------------------------Transition------------------------------------------*/

				/*-----------------------------------Rotation Angle--------------------------------------*/
				vector<double> vecdDeltaAngle;
				for(int i=0;i<(Matches.size()-1);i++){
					detectReliableAngle(CeilingKeypoint, GlobaMapKeypoint, Matches[i+1], Matches[i], vecdDeltaAngle);
				}
				if(vecdDeltaAngle.size()>5)
				{
					for(int i=0;i<vecdDeltaAngle.size();i++){
						for(int j=vecdDeltaAngle.size()-1;j>i;j--){
							if(vecdDeltaAngle[j-1]>vecdDeltaAngle[j])
							{
								double dtemp=vecdDeltaAngle[j-1];
								vecdDeltaAngle[j-1]=vecdDeltaAngle[j];
								vecdDeltaAngle[j]=dtemp;
							}
						}
					}
					double dDeltaAngle=vecdDeltaAngle[(int)vecdDeltaAngle.size()/2]*180/(3.1415926535);
					setDeltaAngle(dDeltaAngle);
					setAngleReliability(true);//로봇 각도 탐색 성공
				}
			}
		}
	}

	return nSelectPath;
}

void KuGlobalMapbasedGlobalLocalizationPr::findInterpolationPose(Point2i nptRobotPixcel, int nSelectPath, vector<KuPose> vecRobotPose, vector<Point2i> vecnptPixcelPath)
{
	Point2d dptNearestPixcel = Point2d((double)(vecnptPixcelPath[nSelectPath].x),(double)(vecnptPixcelPath[nSelectPath].y));
	Point2d dptNearestPos = Point2d(vecRobotPose[nSelectPath].getX(),vecRobotPose[nSelectPath].getY());
	vector<Point2d> vecdptDeltaRobotPose;
	vector<int> vecnRefIdx;

	//calculate the ratio of ratio of u and X and ratio of v and Y
	vector<double> vecdRatioUXandVY;
	for(int i=0; i<vecRobotPose.size(); i++)
	{
		if(i<vecnptPixcelPath.size())
		{
			double dDelX =dptNearestPos.x-vecRobotPose[i].getX();
			double dDelY =dptNearestPos.y-vecRobotPose[i].getY();
			double dDelU =dptNearestPixcel.x-((double)(vecnptPixcelPath[i].x));
			double dDelV =dptNearestPixcel.y-((double)(vecnptPixcelPath[i].y));
			
			double dRatioUXandVY = (dDelU/(dDelX+0.00001))/((dDelV/(dDelY+0.00001))+0.00001);//0.00001은 분모가 0이되지 않게 하기 위함
			vecdRatioUXandVY.push_back(dRatioUXandVY);
		}
	}

	//---------------find the 2 indexes whose ratio is closest to 1--------------------
	double *dRatio = new double[vecdRatioUXandVY.size()];
	int *nDesOrderIdx = new int[vecdRatioUXandVY.size()];
	for(int i=0; i<vecdRatioUXandVY.size(); i++)
	{
		dRatio[i]=abs(vecdRatioUXandVY[i]-1);//initialization
		nDesOrderIdx[i]=i;
	}
	//bubble sort
	for(int i=0; i<vecdRatioUXandVY.size(); i++)
	{
		for(int j=0; j<(vecdRatioUXandVY.size()-1); j++)
		{
			if(dRatio[j]>dRatio[j+1])
			{
				double dTemp = dRatio[j];
				int nTemp = nDesOrderIdx[j];
				dRatio[j]=dRatio[j+1];
				nDesOrderIdx[j]=nDesOrderIdx[j+1];
				dRatio[j+1]=dTemp;
				nDesOrderIdx[j+1]=nTemp;
			}
		}
	}
	vecnRefIdx.push_back(nDesOrderIdx[0]);
	vecnRefIdx.push_back(nDesOrderIdx[1]);
	delete dRatio;
	delete nDesOrderIdx;
	//---------------find the 2 indexes whose ratio is closest to 1--------------------
	
	//----------------------------find the delta Robot Pose-------------------------------------
	double dDeltaURobotandNearest = dptNearestPixcel.x - ((double)(nptRobotPixcel.x));
	double dDeltaVRobotandNearest = dptNearestPixcel.y - ((double)(nptRobotPixcel.y));
	double dPoseX=vecRobotPose[nSelectPath].getX();
	double dPoseY=vecRobotPose[nSelectPath].getY();
	for(int i=0; i<vecnRefIdx.size(); i++ )
	{
		double dDelX = dptNearestPos.x-vecRobotPose[vecnRefIdx[i]].getX();
		double dDelY = dptNearestPos.y-vecRobotPose[vecnRefIdx[i]].getY();
		double dDelU = dptNearestPixcel.x-vecnptPixcelPath[vecnRefIdx[i]].x;
		double dDelV = dptNearestPixcel.y-vecnptPixcelPath[vecnRefIdx[i]].y;

		double dDelRobotPoseX = dDelX*dDeltaURobotandNearest/(dDelU+0.00001);
		double dDelRobotPoseY = dDelY*dDeltaVRobotandNearest/(dDelV+0.00001);

		vecdptDeltaRobotPose.push_back(Point2d(dDelRobotPoseX,dDelRobotPoseY));
	}
	double dTempX=0.0;
	double dTempY=0.0;
	for(int i=0; i<vecdptDeltaRobotPose.size(); i++)
	{
		dTempX+=vecdptDeltaRobotPose[i].x;
		dTempY+=vecdptDeltaRobotPose[i].y;
	}

	setDeltaX(dTempX/vecdptDeltaRobotPose.size());
	setDeltaY(dTempY/vecdptDeltaRobotPose.size());
	printf("DeltaX : %f\nDeltaY : %f\n",getDeltaX(),getDeltaY());
	//----------------------------find the delta Robot Pose-------------------------------------
}

bool KuGlobalMapbasedGlobalLocalizationPr::checkMatchingReliability(Mat& Homography, Point2i nptPixcelPath, Point2i& nptRobotPixcel)
{
	bool bReliability=false;

	double dH11 = Homography.at<double>(0,0);
	double dH12 = Homography.at<double>(0,1);
	double dH13 = Homography.at<double>(0,2);
	double dH21 = Homography.at<double>(1,0);
	double dH22 = Homography.at<double>(1,1);
	double dH23 = Homography.at<double>(1,2);
	double dH31 = Homography.at<double>(2,0);
	double dH32 = Homography.at<double>(2,1);
	double dH33 = Homography.at<double>(2,2);

	double dTempXPos = (dH11*nptPixcelPath.x+dH12*nptPixcelPath.y+dH13)/(dH31*nptPixcelPath.x+dH32*nptPixcelPath.y+dH33);
	double dTempYPos = (dH21*nptPixcelPath.x+dH22*nptPixcelPath.y+dH23)/(dH31*nptPixcelPath.x+dH32*nptPixcelPath.y+dH33);
	nptRobotPixcel.x = (int)(dTempXPos+0.5);
	nptRobotPixcel.y = (int)(dTempYPos+0.5);

	double D = dH11*dH22-dH12*dH21;
	double sx = sqrt(dH11*dH11+dH21*dH21);
	double sy = sqrt(dH12*dH12+dH22*dH22);
	double p = sqrt(dH31*dH31+dH32*dH32);

	if(D<=0||sx<0.1||sx>4||sy<0.1||sy>4||p>0.002)
	{
		bReliability = false;
	}
	else
	{
		bReliability = true;
	}

	return bReliability;
}

void KuGlobalMapbasedGlobalLocalizationPr::showRansacResult(vector<KeyPoint>& CeilingKeypoint, vector<KeyPoint>& DBKeypoint, vector<DMatch>& Matches, Mat& matCeilingImage, Mat& matDBImage, Mat& matHomography)
{
	if(Matches.size()>0)
	{
		vector<KeyPoint> ShowKeyPoint;
		vector<Point2f> obj;
		for(int i=0;i<CeilingKeypoint.size();i++)
		{
			ShowKeyPoint.push_back(CeilingKeypoint[i]);
			obj.push_back( CeilingKeypoint[i].pt );
		}


		vector<Point2f> perspectivetransformedPoints(obj.size());
		perspectiveTransform(obj, perspectivetransformedPoints, matHomography);
		for( int i = 0; i < perspectivetransformedPoints.size(); i++ )
		{
			//--Get the keypoints from the good matches
			double dX=perspectivetransformedPoints[i].x;
			double dY=perspectivetransformedPoints[i].y;
			{
				ShowKeyPoint[i].pt.x=dX;
				ShowKeyPoint[i].pt.y=dY;
			}
		}
		vector<DMatch> PerspectiveMatche;
		for(int i=0; i<Matches.size(); i++)
		{
			double dX= ShowKeyPoint[Matches[i].queryIdx].pt.x;
			double dY= ShowKeyPoint[Matches[i].queryIdx].pt.y;
			//if(dX>0&&dY>0&&dX<320&&dY<240)
			{
				PerspectiveMatche.push_back(Matches[i]);
			}
		}
		Mat MatFirstPerspectiveImage;
		warpPerspective(matCeilingImage,MatFirstPerspectiveImage,matHomography,Size(matCeilingImage.cols,matCeilingImage.rows));

		Mat ShowMatches;
		drawMatches(MatFirstPerspectiveImage,ShowKeyPoint,matDBImage,DBKeypoint,PerspectiveMatche,ShowMatches);
		imwrite("3.bmp",ShowMatches);
	}
}

void KuGlobalMapbasedGlobalLocalizationPr::affineCeilingImage(Mat& matCeilingImage, Mat& matCeilingAffineImage, double dTheta)
{
	double dImageTheta = dTheta;
	if(dImageTheta>=90) dImageTheta=-270+dImageTheta;
	else dImageTheta=90+dImageTheta;

	Mat center = getRotationMatrix2D(Point2i((int)(m_dcx+0.5),(int)(m_dcy+0.5)),dImageTheta,1);
	warpAffine(matCeilingImage,matCeilingAffineImage,center,matCeilingAffineImage.size());//천장영상
}

/**
@brief Korean: 전역 위치추정 결과와 가장 가까운 pixel 좌표상의 노드(경로)를 찾는 함수
@brief English:
*/
void KuGlobalMapbasedGlobalLocalizationPr::findNearesstNode(Point2i nptRobotPixcel, vector<Point2i> vecPixcelPath, int& nSelectPath)
{
	double dMinDist = DBL_MAX;
	for(int i=0; i<vecPixcelPath.size(); i++)
	{
		double dDistX = vecPixcelPath[i].x-nptRobotPixcel.x;
		double dDistY = vecPixcelPath[i].y-nptRobotPixcel.y;
		double dDist = sqrt(dDistX*dDistX+dDistY*dDistY);
		if(dDist<dMinDist)
		{
			dMinDist=dDist;
			nSelectPath=i;
		}
	}
}

void KuGlobalMapbasedGlobalLocalizationPr::detectReliableAngle(vector<KeyPoint>& SURFkeypoints_Ceiling, vector<KeyPoint>& DBSURFkeypoint, DMatch& PreMatche, DMatch& Matche, vector<double>& vecdDeltaAngle)
{
	double dfirstAngle=0;
	double ddeltaX1 =0;
	double ddeltaY1 =0;
	double dDistance1=0;

	double dsecondAngle=0;
	double ddeltaX2 =0;
	double ddeltaY2 =0;
	double dDistance2=0;

	double dD1perD2=0;
	double dtempdeltaAngle =0;

	//ceiling image 각도
	ddeltaX1 = SURFkeypoints_Ceiling[PreMatche.queryIdx].pt.x-SURFkeypoints_Ceiling[Matche.queryIdx].pt.x;
	ddeltaY1 = SURFkeypoints_Ceiling[PreMatche.queryIdx].pt.y-SURFkeypoints_Ceiling[Matche.queryIdx].pt.y;
	dDistance1 = _hypot(ddeltaX1,ddeltaY1);

	//database image 각도
	ddeltaX2 = DBSURFkeypoint[PreMatche.trainIdx].pt.x-DBSURFkeypoint[Matche.trainIdx].pt.x;
	ddeltaY2 = DBSURFkeypoint[PreMatche.trainIdx].pt.y-DBSURFkeypoint[Matche.trainIdx].pt.y;
	dDistance2 = _hypot(ddeltaX2,ddeltaY2);

	dD1perD2=dDistance1/dDistance2;
	if(dD1perD2<1.3&&dD1perD2>0.7&&(!(dDistance1==0)))
	{
		dfirstAngle=atan2(ddeltaY1,ddeltaX1);
		dsecondAngle=atan2(ddeltaY2,ddeltaX2);

		dtempdeltaAngle = dsecondAngle-dfirstAngle;

		if(dtempdeltaAngle>=3.1415926535)
		{
			dtempdeltaAngle=(3.1415926535*2)-dtempdeltaAngle;
		}
		if(dtempdeltaAngle<-3.1415926535)
		{
			dtempdeltaAngle=dtempdeltaAngle+(3.1415926535*2);
		}
		vecdDeltaAngle.push_back(dtempdeltaAngle);
	}
}

Mat KuGlobalMapbasedGlobalLocalizationPr::doRansac(vector<KeyPoint>& CeilingKeypoint, vector<KeyPoint>& DBKeypint, vector<DMatch>& Matches, vector<DMatch>& RansacMatches)
{
	vector<Point2f> CeilingPoints;
	vector<Point2f> DBPoints;

	for( int i = 0; i < Matches.size(); i++ )
	{
		//-- Get the keypoints from the good matches
		CeilingPoints.push_back( CeilingKeypoint[ Matches[i].queryIdx ].pt );
		DBPoints.push_back( DBKeypint[ Matches[i].trainIdx ].pt );
	}
	Mat H_Ransac = findHomography( CeilingPoints, DBPoints, CV_RANSAC );

	vector<Point2f> perspectivetransformedPoints(CeilingPoints.size());
	if(!H_Ransac.empty())
	{
		perspectiveTransform(CeilingPoints, perspectivetransformedPoints, H_Ransac);
		vector<double> vecdDist;
		vector<double> vecdDist2;
		for(size_t i=0; i<Matches.size(); i++)
		{
			double dDX=DBPoints[i].x-perspectivetransformedPoints[i].x;
			double dDY=DBPoints[i].y-perspectivetransformedPoints[i].y;
			double dDist=sqrt(dDX*dDX+dDY*dDY);
			vecdDist.push_back(dDist);
		}
		double dDistThresVal= findRansacDistanceThresholdVal(vecdDist);
		for(size_t i=0; i<Matches.size(); i++)
		{
			if(vecdDist[i]<dDistThresVal)
			{
				double dDX=DBPoints[i].x-perspectivetransformedPoints[i].x;
				double dDY=DBPoints[i].y-perspectivetransformedPoints[i].y;
				double dDist=sqrt(dDX*dDX+dDY*dDY);
				vecdDist2.push_back(dDist);
			}
		}
		double dDistThresVal2= findRansacDistanceThresholdVal(vecdDist2);
		for(size_t i=0; i<Matches.size(); i++)
		{
			if(vecdDist[i]<dDistThresVal2)
			{
				RansacMatches.push_back(Matches[i]);
			}
		}
	}

	return H_Ransac;
}

double KuGlobalMapbasedGlobalLocalizationPr::findRansacDistanceThresholdVal(vector<double>& vecdDist)
{
	if(vecdDist.size()>=30)
	{
		double dMaxZ=1.3;
		double dDistThresholdVal=0;

		double dDistSum=0;
		double dDistAverage=0;
		double dDistVar=0;
		double dDistDev=0;
		double dNumDist=(double)vecdDist.size();

		//average
		for(size_t i=0; i<vecdDist.size(); i++)
		{
			dDistSum+=vecdDist[i];
		}
		dDistAverage=dDistSum/dNumDist;

		//variance
		double dDistVarSum=0;
		for(size_t i=0; i<vecdDist.size(); i++)
		{
			dDistVarSum+=(vecdDist[i]-dDistAverage)*(vecdDist[i]-dDistAverage);
		}
		dDistVar=dDistVarSum/(dNumDist*dNumDist);

		//deviation
		dDistDev=sqrt(dDistVar);

		//Ransac distance threshold value
		dDistThresholdVal=dMaxZ*dDistDev+dDistAverage+5;

		return dDistThresholdVal;
	}
	else
	{
		int nNumDist=vecdDist.size();
		double dTAlpha=0;

		switch(nNumDist)
		{
		case 1:dTAlpha=0.325;break;
		case 2:dTAlpha=0.289;break;
		case 3:dTAlpha=0.277;break;
		case 4:dTAlpha=0.271;break;
		case 5:dTAlpha=0.267;break;
		case 6:dTAlpha=0.265;break;
		case 7:dTAlpha=0.263;break;
		case 8:dTAlpha=0.262;break;
		case 9:dTAlpha=0.261;break;
		case 10:dTAlpha=0.260;break;
		case 11:dTAlpha=0.260;break;
		case 12:dTAlpha=0.259;break;
		case 13:dTAlpha=0.259;break;
		case 14:dTAlpha=0.258;break;
		case 15:dTAlpha=0.258;break;
		case 16:dTAlpha=0.258;break;
		case 17:dTAlpha=0.257;break;
		case 18:dTAlpha=0.257;break;
		case 19:dTAlpha=0.257;break;
		case 20:dTAlpha=0.257;break;
		case 21:dTAlpha=0.257;break;
		case 22:dTAlpha=0.256;break;
		case 23:dTAlpha=0.256;break;
		case 24:dTAlpha=0.256;break;
		case 25:dTAlpha=0.256;break;
		case 26:dTAlpha=0.256;break;
		case 27:dTAlpha=0.256;break;
		case 28:dTAlpha=0.256;break;
		case 29:dTAlpha=0.256;break;
		case 30:dTAlpha=0.256;break;
		}

		double dDistThresholdVal=0;

		double dDistSum=0;
		double dDistAverage=0;
		double dDistVar=0;
		double dDistDev=0;
		double dNumDist=(double)vecdDist.size();

		//average
		for(size_t i=0; i<vecdDist.size(); i++)
		{
			dDistSum+=vecdDist[i];
		}
		dDistAverage=dDistSum/dNumDist;

		//variance
		double dDistVarSum=0;
		for(size_t i=0; i<vecdDist.size(); i++)
		{
			dDistVarSum+=(vecdDist[i]-dDistAverage)*(vecdDist[i]-dDistAverage);
		}
		dDistVar=dDistVarSum/(dNumDist*dNumDist);

		//deviation
		dDistDev=sqrt(dDistVar);

		//second degree of equation
		double dB=-2*dDistAverage-(dDistVar*dTAlpha*dTAlpha)/dNumDist;
		dDistThresholdVal=(-dB-sqrt(dB*dB-4*dDistAverage*dDistAverage))/2+5;

		return dDistThresholdVal;
	}
}

void KuGlobalMapbasedGlobalLocalizationPr::SURFFeatureExtractingProcess(Mat& matImage,vector<KeyPoint>& ORBKeypoint, Mat& ORBDescriptor)
{
	SurfFeatureDetector surf(m_nNumFeatureTh);
	surf.detect(matImage,ORBKeypoint);
	SurfDescriptorExtractor SURFExtractor;
	SURFExtractor.compute(matImage,ORBKeypoint,ORBDescriptor);
}

void KuGlobalMapbasedGlobalLocalizationPr::FeatureMatchingProcess(vector<KeyPoint>& FirstKeypoint, Mat& FirstDescriptor, vector<KeyPoint>& SecondKeypoint, Mat& SecondDescriptor, vector<DMatch>& OutMatches)
{
	vector<DMatch> Match1, Match2, CrosscheckMatches;
	BruteForceMatcher <L2<float>> SURFmatcher;
	SURFmatcher.match(FirstDescriptor,SecondDescriptor,Match1);
	SURFmatcher.match(SecondDescriptor,FirstDescriptor,Match2);
	CrosscheckMatches.clear();
	symmetryTest(Match1,Match2,CrosscheckMatches);
	if(CrosscheckMatches.size()!=0)	calculateRT(CrosscheckMatches,FirstKeypoint,SecondKeypoint,OutMatches);
}

/**
@brief Korean: 이중 matching된 결과 제거
@brief English: 
*/
void KuGlobalMapbasedGlobalLocalizationPr::symmetryTest(vector<DMatch>& matches1, vector<DMatch>& matches2, vector<DMatch>& symMatches)
{
	// for all matches image#1 -> image#2
	for(int i=0; i<matches1.size(); i++)
	{
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
			if( ( matches1[i].queryIdx == matches2[j].trainIdx) && ( matches2[j].queryIdx == matches1[i].trainIdx))
			{	
				// add symmetrical match
				symMatches.push_back(DMatch( matches1[i].queryIdx, matches1[i].trainIdx, matches1[i].distance));
				break;		// next match in image#1 -> image#2
			}

		}

	}

}

/**
@brief Korean: 란삭
@brief English: 
*/
void KuGlobalMapbasedGlobalLocalizationPr::calculateRT(const vector<DMatch>& matches, const vector<KeyPoint>& keypoints1, const vector<KeyPoint>& keypoints2, vector<DMatch>& outMatches)
{
	// Convert keypoints into Point2f
	vector<Point2f> points1, points2;
	vector<Point2f> imagePoints1, imagePoints2;
	for(vector<DMatch>::const_iterator it = matches.begin(); it !=matches.end(); ++it)
	{
		float x = keypoints1[it->queryIdx].pt.x;
		float y = keypoints1[it->queryIdx].pt.y;
		points1.push_back(Point2f(x,y));

		x = keypoints2[it->trainIdx].pt.x;
		y = keypoints2[it->trainIdx].pt.y;
		points2.push_back(Point2f(x,y));
	}

	// Compute F matrix using RANSAC
	vector<uchar> inliers(points1.size(), 0);
	Mat fundamental = findFundamentalMat(Mat(points1), Mat(points2), inliers, CV_FM_RANSAC, 3.0f, 0.98f);

	vector<uchar>::const_iterator itIn = inliers.begin();
	vector<DMatch>::const_iterator itM = matches.begin();

	// for all matches
	for( ;itIn != inliers.end(); ++itIn, ++itM)
	{
		if(*itIn)
		{
			outMatches.push_back(*itM);
		}
	}
}

void KuGlobalMapbasedGlobalLocalizationPr::setDeltaX(double dDeltaX)
{
	m_dDeltaX=dDeltaX;
}
void KuGlobalMapbasedGlobalLocalizationPr::setDeltaY(double dDletaY)
{
	m_dDeltaY=dDletaY;
}
void KuGlobalMapbasedGlobalLocalizationPr::setDeltaAngle(double dAngle)
{
	m_dDeltaAngle=dAngle;
}
void KuGlobalMapbasedGlobalLocalizationPr::setPoseReliability(bool bPose)
{
	m_bRobotPose=bPose;
}
void KuGlobalMapbasedGlobalLocalizationPr::setAngleReliability(bool bAngle)
{
	m_bRobotAngle=bAngle;
}

double KuGlobalMapbasedGlobalLocalizationPr::getDeltaX()
{
	return m_dDeltaX;
}
double KuGlobalMapbasedGlobalLocalizationPr::getDeltaY()
{
	return m_dDeltaY;
}
double KuGlobalMapbasedGlobalLocalizationPr::getDeltaAngle()
{
	return m_dDeltaAngle;
}
bool KuGlobalMapbasedGlobalLocalizationPr::getPoseReliability()
{
	return m_bRobotPose;
}
bool KuGlobalMapbasedGlobalLocalizationPr::getAngleReliability()
{
	return m_bRobotAngle;
}