#include "stdafx.h"
#include "KuSURFbasedGlobalLocalizerPr.h"

KuSURFbasedGlobalLocalizerPr::KuSURFbasedGlobalLocalizerPr()
{
	m_RobotPos.init(); //로봇 위치 초기화

	m_nSelectPathIdx=-1;
	m_bSURFTransitionTruth=false;
	m_bSelectPath=false;

	m_nNumFeatureTh=KuRobotParameter::getInstance()->getNumSURFFeatureTh();//SURF 특징점개수를 변경시켜주는 변수 

	m_matDBImage.create(Sensor::CEILING_IMAGE_HEIGHT,Sensor::CEILING_IMAGE_WIDTH,CV_8UC1);//Result image initialization
	m_matCeilingImage.create(Sensor::CEILING_IMAGE_HEIGHT,Sensor::CEILING_IMAGE_WIDTH,CV_8UC1);//Result image initialization

	m_nMap=NULL;
}

KuSURFbasedGlobalLocalizerPr::~KuSURFbasedGlobalLocalizerPr()
{

}

void KuSURFbasedGlobalLocalizerPr::initialize()
{
	m_bSURFTransitionTruth=false;
	m_dRotationAngle=0;
	m_dSURFDeltaX=0;
	m_dSURFDeltaX=0;
	m_dSURFDeltaY=0;
	m_nMatchingPointNum=0;
	m_nSelectPathIdx=-1;
	m_bSelectPath=false;
	m_fDistance=0;
	m_DescriptorCnt=0;
	m_vecnCandidatePath.clear();
}

void KuSURFbasedGlobalLocalizerPr::initDB()
{

}
/**
@brief Korean:  초기 이미지 저장을 위한 사전작업 종료(쓰레드 terminate에 있던거)
@brief English: 
*/
void KuSURFbasedGlobalLocalizerPr::finish()
{
	m_vecDBSURFDescriptors.clear();
	m_vecDBSURFkeypoints.clear();
}

/**
@brief Korean: DataBase를 작성하기 위해서 SURF를 통해 얻은 data를 검출하고 저장하는 함수
@brief English: Extracts and saves the data gained from surf to make the database
*/
void KuSURFbasedGlobalLocalizerPr::detectSURFDatatoSave(Mat matCeilingImage, int nDataIdx)
{
	for(int i = 0; i < Sensor::CEILING_IMAGE_WIDTH * Sensor::CEILING_IMAGE_HEIGHT; i++)
	{
		m_matCeilingImage.data[i]=matCeilingImage.data[i];
	}
	
	vector<KeyPoint> SURFkeypoints;
	Mat SURFdescriptors;
	SURFFeatureExtractingProcess(m_matCeilingImage,SURFkeypoints,SURFdescriptors);
	
	char cSURFFilePathName[200];
	//descriptors(index).xml 파일로 저장
	memset(cSURFFilePathName,0,sizeof(cSURFFilePathName));
	string strSURFDataPath = KuRobotParameter::getInstance()->getSURFDataNamePath();
	sprintf_s(cSURFFilePathName,"%s/descriptors_%d.xml",strSURFDataPath.c_str(),nDataIdx);
	cv::FileStorage fs(cSURFFilePathName,cv::FileStorage::WRITE);
	fs << "Mat" << SURFdescriptors;
	fs.release();
	int nSize= SURFdescriptors.rows;
	//---------------keypoints(index).xml 파일로 저장------------------------------
	//keypoint -> Mat(FileStorage사용을 위한 변수 유형 변경)
	Mat KeypointtoSave(nSize,7,CV_32F);
	for(int i=0; i<nSize; i++){
		KeypointtoSave.at<float>(i,0)= SURFkeypoints[i].pt.x;
		KeypointtoSave.at<float>(i,1)= SURFkeypoints[i].pt.y;
		KeypointtoSave.at<float>(i,2)= SURFkeypoints[i].angle;
		KeypointtoSave.at<float>(i,3)= SURFkeypoints[i].class_id;
		KeypointtoSave.at<float>(i,4)= SURFkeypoints[i].octave;
		KeypointtoSave.at<float>(i,5)= SURFkeypoints[i].response;
		KeypointtoSave.at<float>(i,6)= SURFkeypoints[i].size;
	}
	memset(cSURFFilePathName,0,sizeof(cSURFFilePathName));
	string strSURFKeyPointDataPath = KuRobotParameter::getInstance()->getSURFDataNamePath();
	sprintf_s(cSURFFilePathName,"%s/keypoint_%d.xml",strSURFKeyPointDataPath.c_str(),nDataIdx);
	cv::FileStorage fd(cSURFFilePathName,cv::FileStorage::WRITE);
	fd << "Mat" << KeypointtoSave;
	fd.release();
	//---------------keypoints(index).xml 파일로 저장 end--------------------------
}


void KuSURFbasedGlobalLocalizerPr::loadSURFDB(int nIdx,  Mat& descriptors, KeyPoint& keypoint, vector<KeyPoint>& veckeypoint)
{
	char cFilePathName[100];
	int nAGVID=KuRobotParameter::getInstance()->getRobotID();
	//----------------descriptors 불러오기------------------------------------------
	memset(cFilePathName,0,sizeof(cFilePathName));
	string strSURFDataPath = KuRobotParameter::getInstance()->getSURFDataNamePath();
	sprintf_s(cFilePathName,"%s/descriptors_%d.xml",strSURFDataPath.c_str(),nAGVID,nIdx);
	cv::FileStorage fs(cFilePathName, cv::FileStorage::READ);
	fs [ "Mat" ] >> descriptors;
	fs.release();
	//----------------descriptors 불러오기 end--------------------------------------
	int nSize=descriptors.rows;
	//----------------keypoint 불러오기---------------------------------------------
	Mat PreKeyPoint(nSize,7,CV_32F);
	memset(cFilePathName,0,sizeof(cFilePathName));
	string strKeyPointDataPath = KuRobotParameter::getInstance()->getSURFDataNamePath();
	sprintf_s(cFilePathName,"%s/keypoint_%d.xml",strKeyPointDataPath.c_str(),nAGVID ,nIdx);
	cv::FileStorage fd(cFilePathName, cv::FileStorage::READ);
	fd [ "Mat" ] >> PreKeyPoint;
	fd.release();
	//해당 index keypoint 불러오기
	veckeypoint.clear();
	for(int i=0; i<nSize; i++){
		keypoint.pt.x = PreKeyPoint.at<float>(i,0);
		keypoint.pt.y = PreKeyPoint.at<float>(i,1);
		keypoint.angle = PreKeyPoint.at<float>(i,2);
		keypoint.class_id = PreKeyPoint.at<float>(i,3);
		keypoint.octave = PreKeyPoint.at<float>(i,4);
		keypoint.response = PreKeyPoint.at<float>(i,5);
		keypoint.size = PreKeyPoint.at<float>(i,6);
		veckeypoint.push_back(keypoint);
	}
	//----------------keypoint 불러오기 end-----------------------------------------

	//descriptors와 keypoints 저장
	m_vecDBSURFDescriptors.push_back(descriptors);
	m_vecDBSURFkeypoints.push_back(veckeypoint);
}

/**
@brief Korean: 로봇 주변으로부터 가까운 랜드마크의 위치를 검사한다.
@brief English: 
*/
bool KuSURFbasedGlobalLocalizerPr::checkNearLandMarkfromRobotPos(int nCheckIdx)
{
	m_RobotPos = KuDrawingInfo::getInstance()->getRobotPos();

	double  dImagePathX=0.0;
	double  dImagePathY=0.0;
	double  dDist=1000000;
	int nDistFromPath = KuRobotParameter::getInstance()->getDistanceFromPath();

	dImagePathX = m_vecImagePath[nCheckIdx].getX();
	dImagePathY = m_vecImagePath[nCheckIdx].getY();

	dDist=hypot(m_RobotPos.getX()-dImagePathX, m_RobotPos.getY()- dImagePathY);

	if(dDist<nDistFromPath)
	{
		return true;
	}		

	//printf("Overed distance from path=%f\n",dDist);
	return false;
}

bool KuSURFbasedGlobalLocalizerPr::checkSelectPathNearLastPos()
{
	return m_bSelectPath;
}

/**
@brief Korean: 이중 matching된 결과 제거
@brief English: 
*/
void KuSURFbasedGlobalLocalizerPr::symmetryTest(vector<DMatch>& matches1, vector<DMatch>& matches2, vector<DMatch>& symMatches)
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
void KuSURFbasedGlobalLocalizerPr::calculateRT(const vector<DMatch>& matches, const vector<KeyPoint>& keypoints1, const vector<KeyPoint>& keypoints2, vector<DMatch>& outMatches)
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
	m_DescriptorCnt=0;
	m_fDistance=0;

	// for all matches
	for( ;itIn != inliers.end(); ++itIn, ++itM)
	{
		if(*itIn)
		{
			m_fDistance+=(*itM).distance;
			m_DescriptorCnt++;
			outMatches.push_back(*itM);
		}
	}
}

/**
@brief Korean: 전역위치 추정 SURF로 추정된 로봇의 위치를 가져온다
@brief English: 
*/
KuPose KuSURFbasedGlobalLocalizerPr::GlobalLocalization(IplImage * IplCeilingImage)
{
	m_nSelectPathIdx=-1;
	m_vecnCandidatePath.clear();
	vector<Point2i> vecptMatchingNum;
	m_RobotPos.setID(-1);

	Mat matCeilingImage;
	matCeilingImage.create(Sensor::CEILING_IMAGE_HEIGHT,Sensor::CEILING_IMAGE_WIDTH,CV_8UC1);
	for(int i = 0; i < Sensor::CEILING_IMAGE_WIDTH * Sensor::CEILING_IMAGE_HEIGHT; i++)	//천장이미지 정보 복사
	{
		matCeilingImage.data[i]=IplCeilingImage->imageData[i];
	}

	KuRetinexPr RetinexPr;
	RetinexPr.doRetinexProcess(matCeilingImage,m_matCeilingImage,30);

	vector<KeyPoint> SURFkeypoints_Ceiling;
	Mat SURFDescriptors_Ceiling;
	SURFFeatureExtractingProcess(m_matCeilingImage,SURFkeypoints_Ceiling,SURFDescriptors_Ceiling);

	int nSURFCnt=0;
	int nMaxCnt=0;
	double dSURFFeatureDist=0.0;
	double dMinDistance=DBL_MAX;

	m_vecImagePath = GlobalLocalizationSupervisor::getInstance()->m_vecImagePath;
	vector<vector<DMatch>> vecMatches;
	vector<int> vecCandidatePath;
	int nMinDistPathIdx=-1;
	//로봇 근처 위치에 존재하는 이미지와 로봇의 현 위치의 이미지의 매칭을 통하여 비교하는 부분---------------------------------------------------
	if(!(SURFkeypoints_Ceiling.size()==0))
	{
		for(int nPathIdx=0;nPathIdx<GlobalLocalizationSupervisor::getInstance()->m_vecImagePath.size()-1;nPathIdx++ )
		{
			if(/*true*/checkNearLandMarkfromRobotPos(nPathIdx))//로봇 근처에 있는 경우
			{
				vector<DMatch> Matches;
				FeatureMatchingProcess(SURFkeypoints_Ceiling,SURFDescriptors_Ceiling,m_vecDBSURFkeypoints[nPathIdx],m_vecDBSURFDescriptors[nPathIdx],Matches);
				vecMatches.push_back(Matches);

				//신뢰도--------------------------
				nSURFCnt=0;	dSURFFeatureDist=0.0;
				for(int i=0; i<Matches.size();i++)
				{
					nSURFCnt++;
					dSURFFeatureDist+=sqrt(((m_vecDBSURFkeypoints[nPathIdx][Matches[i].trainIdx].pt.y-SURFkeypoints_Ceiling[Matches[i].queryIdx].pt.y)*
						(m_vecDBSURFkeypoints[nPathIdx][Matches[i].trainIdx].pt.y-SURFkeypoints_Ceiling[Matches[i].queryIdx].pt.y))+
						((m_vecDBSURFkeypoints[nPathIdx][Matches[i].trainIdx].pt.x-SURFkeypoints_Ceiling[Matches[i].queryIdx].pt.x)*
						(m_vecDBSURFkeypoints[nPathIdx][Matches[i].trainIdx].pt.x-SURFkeypoints_Ceiling[Matches[i].queryIdx].pt.x)));
				}
				//매칭 개수
				Point2i ptMatchingNum;
				ptMatchingNum.x=nPathIdx;
				ptMatchingNum.y=Matches.size();
				vecptMatchingNum.push_back(ptMatchingNum);

				dSURFFeatureDist=dSURFFeatureDist/(double)nSURFCnt;
				if(dMinDistance>dSURFFeatureDist)
				{
					dMinDistance=dSURFFeatureDist;
					nMinDistPathIdx=nPathIdx;
					nMaxCnt = nSURFCnt;
				}
			}
		}
	}
	//매칭 개수가 k번째 만큼 많은 index 탐색
	for(int k=0; k<3; k++)
	{
		Point2i ptTemp;
		ptTemp.x=0;//index
		ptTemp.y=0;//matching 개수
		if(k<vecMatches.size())
		{
			for(int i=0; i<vecptMatchingNum.size(); i++)
			{
				if(vecptMatchingNum[i].y>ptTemp.y)
				{
					bool bflag=true;//같은 인덱스가 있는지 확인
					for(int j=0; j<vecCandidatePath.size(); j++)
					{
						if(vecCandidatePath[j]==vecptMatchingNum[i].x)//같은 인덱스가 있으면
						{
							bflag=false;//업데이트 하지 않는다
						}
					}
					if(bflag)
					{
						ptTemp.x=vecptMatchingNum[i].x;
						ptTemp.y=vecptMatchingNum[i].y;
					}
				}
			}
		}
		vecCandidatePath.push_back(ptTemp.x);
	}
	if(nMinDistPathIdx>0)	vecCandidatePath.push_back(nMinDistPathIdx);

	//path 신뢰도 판별 
	m_vecHomography.clear();
	for(int i=0; i<vecCandidatePath.size(); i++)
	{
		vector<DMatch> RansacMatches;
		Mat H = doRansac(SURFkeypoints_Ceiling,m_vecDBSURFkeypoints[vecCandidatePath[i]],vecMatches[vecCandidatePath[i]],RansacMatches);
		if(checkMatchingReliability(H))
		{
			m_vecnCandidatePath.push_back(vecCandidatePath[i]);
			m_vecHomography.push_back(H);
		}
	}
	//가장 신뢰성 있는 H 의 path index --> 유사도가 가장 높은 path index
	m_nSelectPathIdx = m_vecnCandidatePath[findMostReliableMatchingIdx(m_vecHomography)];
	if(m_nSelectPathIdx>=0)
	{
		m_bSelectPath=checkNearLandMarkfromRobotPos(m_nSelectPathIdx);
	}

	//로봇 위치 갱신
	if(m_bSelectPath)
	{
		if(nMaxCnt!=0)
		{
			double	dImagePathX = m_vecImagePath[m_nSelectPathIdx].getX();
			double  dImagePathY = m_vecImagePath[m_nSelectPathIdx].getY();
			double  dImagePathTheta = m_vecImagePath[m_nSelectPathIdx].getThetaDeg();

			m_RobotPos.setX(dImagePathX);
			m_RobotPos.setY(dImagePathY);
			m_RobotPos.setThetaDeg( dImagePathTheta);
			m_RobotPos.setID(m_nSelectPathIdx);
			nMaxCnt=0;
		}
	}
	
	//로봇 위치의 신뢰성(마지막 위치로부터 5m 이내)
	return m_RobotPos;
}

/**
@brief Korean:  Database와 현재 카메라 영상의 회전 각도
@brief English: 
*/
void KuSURFbasedGlobalLocalizerPr::setTransitionVal(IplImage* IplCeilingImage, int nPathIdx)
{
	m_bSURFTransitionTruth=false;
	m_nMatchingPointNum=0;

	vector<double> vecdDeltaAngle;
	vector<double> vecdDeltaX,vecdDeltaY;

	for(int i = 0; i < Sensor::CEILING_IMAGE_WIDTH * Sensor::CEILING_IMAGE_HEIGHT; i++)
	{
		m_matCeilingImage.data[i]=IplCeilingImage->imageData[i];
	}
	KuRetinexPr RetinexPr;
	RetinexPr.doRetinexProcess(m_matCeilingImage,m_matCeilingImage,30);
	vector<KeyPoint> SURFkeypoints_Ceiling;
	Mat SURFdescriptors_Ceiling;
	SURFFeatureExtractingProcess(m_matCeilingImage,SURFkeypoints_Ceiling,SURFdescriptors_Ceiling);

	if(!(SURFkeypoints_Ceiling.size()==0))
	{
		vector<DMatch> Matches;
		FeatureMatchingProcess(SURFkeypoints_Ceiling,SURFdescriptors_Ceiling,m_vecDBSURFkeypoints[nPathIdx],m_vecDBSURFDescriptors[nPathIdx],Matches);
		
		if(!(Matches.size()==0))
		{
			m_bSURFTransitionTruth=true;
			if(m_bSURFTransitionTruth&&Matches.size()>5)
			{
				/*-----------------------------------Transition------------------------------------------*/
				double dAngle_Rad=m_dRotationAngle*3.1415926535/180;
				double dImageDeltaX;
				double dImageDeltaY;
				vector<Point2f> Ceiling4Point(4);
				vector<Point2f> DB4Point(4);

				//find homography
				vector<DMatch> RansacMatches;
				Mat cvMatHomographyMatrix = doRansac(SURFkeypoints_Ceiling, m_vecDBSURFkeypoints[nPathIdx], Matches, RansacMatches);

				int nTranstionVal=10;
				Ceiling4Point[0]=cvPoint(m_matCeilingImage.cols/2-nTranstionVal,m_matCeilingImage.rows/2-nTranstionVal);
				Ceiling4Point[1]=cvPoint(m_matCeilingImage.cols/2+nTranstionVal,m_matCeilingImage.rows/2-nTranstionVal);
				Ceiling4Point[2]=cvPoint(m_matCeilingImage.cols/2+nTranstionVal,m_matCeilingImage.rows/2+nTranstionVal);
				Ceiling4Point[3]=cvPoint(m_matCeilingImage.cols/2-nTranstionVal,m_matCeilingImage.rows/2+nTranstionVal);

				perspectiveTransform(Ceiling4Point, DB4Point, cvMatHomographyMatrix);
				 
				double dDBMeanX=(DB4Point[0].x+DB4Point[1].x+DB4Point[2].x+DB4Point[3].x)/4;
				double dDBMeanY=(DB4Point[0].y+DB4Point[1].y+DB4Point[2].y+DB4Point[3].y)/4;

				dImageDeltaX=dDBMeanX-Sensor::CEILING_IMAGE_WIDTH/2;
				dImageDeltaY=dDBMeanY-Sensor::CEILING_IMAGE_HEIGHT/2;

				//projection
				double dfx=KuRobotParameter::getInstance()->getCeilingCameraPrameterFx();
				double dfy=KuRobotParameter::getInstance()->getCeilingCameraPrameterFy();
				double dcx=KuRobotParameter::getInstance()->getCeilingCameraPrameterCx();
				double dcy=KuRobotParameter::getInstance()->getCeilingCameraPrameterCy();
				double dZ=KuRobotParameter::getInstance()->getHeight();

				m_dSURFDeltaY=-(dZ/dfx)*dImageDeltaX;
				m_dSURFDeltaX=(dZ/dfy)*dImageDeltaY;	
				/*-----------------------------------Transition------------------------------------------*/

				/*-----------------------------------Rotation Angle--------------------------------------*/
				vecdDeltaAngle.clear();
				for(int i=0;i<(Matches.size()-1);i++){
					detectReliableAngle(SURFkeypoints_Ceiling, m_vecDBSURFkeypoints[nPathIdx], Matches[i+1], Matches[i], vecdDeltaAngle);
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
					m_dRotationAngle=vecdDeltaAngle[(int)vecdDeltaAngle.size()/2]*180/(3.1415926535);
				}
				else 
				{
					m_bSURFTransitionTruth=false;
					printf("SURF matchings for computing angles are not sufficient\n");
				}
				/*-----------------------------------Rotation Angle--------------------------------------*/
			}
			else
			{
				m_bSURFTransitionTruth=false;
				printf("SURF matchings for computing transitions are not sufficient\n");
			}
		}
		else printf("SURF matchings are not sufficient\n");
	}
	else printf("SURF keypoints are not sufficient\n");
}

void KuSURFbasedGlobalLocalizerPr::detectReliableAngle(vector<KeyPoint>& SURFkeypoints_Ceiling, vector<KeyPoint>& DBSURFkeypoint, DMatch& PreMatche, DMatch& Matche, vector<double>& vecdDeltaAngle)
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

void KuSURFbasedGlobalLocalizerPr::detectReliablePoint(vector<KeyPoint>& SURFkeypoints_Ceiling, vector<KeyPoint>& DBSURFkeypoint, DMatch& PreMatche, DMatch& Matche, vector<double>& vecdDeltaX, vector<double>& vecdDeltaY)
{
	double ddeltaX1 =0;
	double ddeltaY1 =0;
	double dDistance1=0;

	double ddeltaX2 =0;
	double ddeltaY2 =0;
	double dDistance2=0;

	double dD1perD2=0;
	double dDeltaX=0;
	double dDeltaY=0;

	ddeltaX1 = SURFkeypoints_Ceiling[PreMatche.queryIdx].pt.x-SURFkeypoints_Ceiling[Matche.queryIdx].pt.x;
	ddeltaY1 = SURFkeypoints_Ceiling[PreMatche.queryIdx].pt.y-SURFkeypoints_Ceiling[Matche.queryIdx].pt.y;
	dDistance1 = _hypot(ddeltaX1,ddeltaY1);

	//second image 각도
	ddeltaX2 = DBSURFkeypoint[PreMatche.trainIdx].pt.x-DBSURFkeypoint[Matche.trainIdx].pt.x;
	ddeltaY2 = DBSURFkeypoint[PreMatche.trainIdx].pt.y-DBSURFkeypoint[Matche.trainIdx].pt.y;
	dDistance2 = _hypot(ddeltaX2,ddeltaY2);

	dD1perD2=dDistance1/dDistance2;
	if(dD1perD2<1.3&&dD1perD2>0.7&&(!(dDistance2==0)))
	{
		dDeltaX=SURFkeypoints_Ceiling[Matche.queryIdx].pt.x-DBSURFkeypoint[Matche.trainIdx].pt.x;
		dDeltaY=SURFkeypoints_Ceiling[Matche.queryIdx].pt.y-DBSURFkeypoint[Matche.trainIdx].pt.y;
		vecdDeltaX.push_back(dDeltaX);
		vecdDeltaY.push_back(dDeltaY);
	}
}

void KuSURFbasedGlobalLocalizerPr::showRansacResult(vector<KeyPoint>& CeilingKeypoint, vector<KeyPoint>& DBKeypoint, vector<DMatch>& Matches, Mat& matCeilingImage, Mat& matDBImage, Mat& matHomography)
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
	}
}

Mat KuSURFbasedGlobalLocalizerPr::doRansac(vector<KeyPoint>& CeilingKeypoint, vector<KeyPoint>& DBKeypint, vector<DMatch>& Matches, vector<DMatch>& RansacMatches)
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
		double dDistThresVal= getRansacDistanceThresholdVal(vecdDist);
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
		double dDistThresVal2= getRansacDistanceThresholdVal(vecdDist2);
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

double KuSURFbasedGlobalLocalizerPr::getRansacDistanceThresholdVal(vector<double>& vecdDist)
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

int KuSURFbasedGlobalLocalizerPr::getSelectPathIdx()
{
	return m_nSelectPathIdx;
}

bool KuSURFbasedGlobalLocalizerPr::getRotationTruth()
{
	return m_bSURFTransitionTruth;
}

double KuSURFbasedGlobalLocalizerPr::getRotationAngle()
{
	return m_dRotationAngle;
}
double KuSURFbasedGlobalLocalizerPr::getDeltaX()
{
	return m_dSURFDeltaX;
}
double KuSURFbasedGlobalLocalizerPr::getDeltaY()
{
	return m_dSURFDeltaY;
}

int KuSURFbasedGlobalLocalizerPr::getMatchingPointNum()
{
	return m_nMatchingPointNum;
}
/**
@brief Korean:  로봇 위치 초기화
@brief English: 
*/
void KuSURFbasedGlobalLocalizerPr::init()
{
	m_RobotPos.init(); //로봇 위치 초기화
}

/**
@brief Korean:  로봇 위치를 설정한다
@brief English: 
*/
void KuSURFbasedGlobalLocalizerPr::setRobotPos(KuPose RobotPos)
{
	m_RobotPos = RobotPos;

}

/**
@brief Korean:  로봇의 X좌표를 설정한다
@brief English: 
*/
void KuSURFbasedGlobalLocalizerPr::setRobotPosX(double dPoseX)
{
	m_RobotPos.setX(dPoseX);
}
/**
@brief Korean:  로봇의 Y좌표를 설정한다
@brief English: 
*/
void KuSURFbasedGlobalLocalizerPr::setRobotPosY(double dPoseY)
{
	m_RobotPos.setY(dPoseY);
}
/**
@brief Korean:  로봇의 각도 Degree를 설정한다
@brief English: 
*/
void KuSURFbasedGlobalLocalizerPr::setRobotPosDeg(double dPoseDeg)
{
	m_RobotPos.setThetaDeg(dPoseDeg);
}
/**
@brief Korean:  로봇의 각도 Radian를 설정한다
@brief English: 
*/
void KuSURFbasedGlobalLocalizerPr::setRobotPosRad(double dPoseRad)
{
	m_RobotPos.setThetaRad(dPoseRad);
}

/**
@brief Korean: 지도정보를 설정한다
@brief English: 
*/
void KuSURFbasedGlobalLocalizerPr::setMap(int nMapSizeX, int nMapSizeY, int** nMap)
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
double KuSURFbasedGlobalLocalizerPr::getRobotPosX()
{
	return m_RobotPos.getX();
}
/**
@brief Korean:  로봇의 Y좌표를 가져 간다
@brief English:
*/
double KuSURFbasedGlobalLocalizerPr::getRobotPosY()
{
	return m_RobotPos.getY();
}
/**
@brief Korean:  로봇의  각도(Degree)를 가져 간다
@brief English:
*/
double KuSURFbasedGlobalLocalizerPr::getRobotPosDeg()
{
	return m_RobotPos.getThetaDeg();
}
/**
@brief Korean:  로봇의  각도(Radian)를 가져 간다
@brief English:
*/
double KuSURFbasedGlobalLocalizerPr::getRobotPosRad()
{
	return m_RobotPos.getThetaRad();
}
/**
@brief Korean:  로봇의  위치 값을 가져 간다
@brief English:
*/
KuPose KuSURFbasedGlobalLocalizerPr::getRobotPos()
{
	return m_RobotPos;
}
bool KuSURFbasedGlobalLocalizerPr::getSelectPathReliabliity()
{
	return m_bSelectPath;
}
void KuSURFbasedGlobalLocalizerPr::SURFFeatureExtractingProcess(Mat& matImage,vector<KeyPoint>& ORBKeypoint, Mat& ORBDescriptor)
{
	SurfFeatureDetector surf(m_nNumFeatureTh);
	surf.detect(matImage,ORBKeypoint);
	SurfDescriptorExtractor SURFExtractor;
	SURFExtractor.compute(matImage,ORBKeypoint,ORBDescriptor);
}

void KuSURFbasedGlobalLocalizerPr::FeatureMatchingProcess(vector<KeyPoint>& FirstKeypoint, Mat& FirstDescriptor, vector<KeyPoint>& SecondKeypoint, Mat& SecondDescriptor, vector<DMatch>& OutMatches)
{
	vector<DMatch> Match1, Match2, CrosscheckMatches;
	BruteForceMatcher <L2<float>> SURFmatcher;
	SURFmatcher.match(FirstDescriptor,SecondDescriptor,Match1);
	SURFmatcher.match(SecondDescriptor,FirstDescriptor,Match2);
	CrosscheckMatches.clear();
	symmetryTest(Match1,Match2,CrosscheckMatches);
	if(CrosscheckMatches.size()!=0)	calculateRT(CrosscheckMatches,FirstKeypoint,SecondKeypoint,OutMatches);
}

bool KuSURFbasedGlobalLocalizerPr::checkMatchingReliability(Mat& Homography)
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

int KuSURFbasedGlobalLocalizerPr::findMostReliableMatchingIdx(vector<Mat> vecH)
{
	int nSelectIdx=-1;
	double dMinVal=DBL_MAX;
	for(int i=0; i<vecH.size(); i++)
	{
		double dH11 = vecH[i].at<double>(0,0);
		double dH12 = vecH[i].at<double>(0,1);
		double dH13 = vecH[i].at<double>(0,2);
		double dH21 = vecH[i].at<double>(1,0);
		double dH22 = vecH[i].at<double>(1,1);
		double dH23 = vecH[i].at<double>(1,2);
		double dH31 = vecH[i].at<double>(2,0);
		double dH32 = vecH[i].at<double>(2,1);
		double dH33 = vecH[i].at<double>(2,2);

		double D = dH11*dH22-dH12*dH21;
		double sx = sqrt(dH11*dH11+dH21*dH21);
		double sy = sqrt(dH12*dH12+dH22*dH22);
		double p = sqrt(dH31*dH31+dH32*dH32);

		if(D>0)
		{
			double dTemp = ((sx-1)*(sx-1)+(sy-1)*(sy-1))*p;
			if(dMinVal>dTemp)
			{
				nSelectIdx=i;
				dMinVal=dTemp;
			}
		}
	}
	return nSelectIdx;
}