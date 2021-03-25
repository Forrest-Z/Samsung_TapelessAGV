/*______________________________________________________________________________________________
PROJECT : Intelligent Robotics Lab. Department of Mechanical Engineering Korea University                                    
(c) Copyright 2012 KUME Intelligent Robotics Lab.                                             
All rights reserved                                                                         
$Directed by Jae-Bok Song                                                                     
$Designed by Joong-Tae Park                                                                  
$Description :레이저센서와 particle filter를 사용하여 위치추정을 수행하는 프로세스
$Created on: 2012. 6. 12.                                                                        
$Author: Joong-Tae Park      
$e-mail: jtpark1114@gmail.com
______________________________________________________________________________________________*/


#ifndef KUNS_SURF_BASED_GLOBAL_LOCALIZER_PROCESS_H
#define KUNS_SURF_BASED_GLOBAL_LOCALIZER_PROCESS_H


#include <vector>
#include <iostream>
#include "../../sensor/Sensor.h"
#include "../../KUNSPose/KuPose.h"
#include "../../KUNSUtil/KUNSThread/KuThread.h"
#include "../../KUNSUtil/KUNSSingletone/KuSingletone.h"
#include "../../KUNSUtil/KUNSSmartPointer/KuSmartPointer.h"
#include "../../KUNSGUI/KuDrawingInfo.h"
#include "../../MobileSupervisor/KuRobotParameter.h"
#include "../../Localizer/Localizer.h"
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/legacy/legacy.hpp>
#include "../../Algorithm//KuICP/KuICP.h"
#include "../KUNSICPLocalizerPr/KuICPLocalizerPr.h"
#include "../../Localizer/GlobalLocalizerSupervisor.h"
#include "../KUNSRetinexPr/KuRetinexPr.h"

using namespace cv;
using namespace std;
class KuSURFbasedGlobalLocalizerPr : public KuSingletone <KuSURFbasedGlobalLocalizerPr>,public Localizer
{
private:
	int m_nNumFeatureTh;
	vector<KuPose> m_vecImagePath;

private:
	Mat m_matCeilingImage;
	char m_cFilePathName[100];
	Mat m_matDBImage;
	KuPose m_RobotPos,m_PreRobotPos;
	int m_nSelectPathIdx;
	bool m_bSelectPath;
	vector<int> m_vecnCandidatePath;
	int **m_nMap;
	int m_nMapSizeX;
	int m_nMapSizeY;
	vector<Mat> m_vecDBSURFDescriptors;
	vector<vector<KeyPoint>> m_vecDBSURFkeypoints;
	int m_DescriptorCnt;
	float m_fDistance;
	double m_dRotationAngle;
	bool m_bSURFTransitionTruth;
	double m_dSURFDeltaX,m_dSURFDeltaY;
	int m_nMatchingPointNum;
	vector<Mat> m_vecHomography;

private:
	void symmetryTest(vector<DMatch>& matches1, vector<DMatch>& matches2, vector<DMatch>& symMatches);
	void calculateRT(const vector<DMatch>& matches, const vector<KeyPoint>& keypoints1, const vector<KeyPoint>& keypoints2, vector<DMatch>& outMatches);
	
	void detectReliableAngle(vector<KeyPoint>& SURFkeypoints_Ceiling, vector<KeyPoint>& DBSURFkeypoint, DMatch& PreMatche, DMatch& Matche, vector<double>& vecdDeltaAngle);
	void detectReliablePoint(vector<KeyPoint>& SURFkeypoints_Ceiling, vector<KeyPoint>& DBSURFkeypoint, DMatch& PreMatche, DMatch& Matche, vector<double>& vecdDeltaX, vector<double>& vecdDeltaY);
	
	void showRansacResult(vector<KeyPoint>& CeilingKeypoint, vector<KeyPoint>& DBKeypoint, vector<DMatch>& Matches, Mat& matCeilingImage, Mat& matDBImage, Mat& matHomography);
	Mat doRansac(vector<KeyPoint>& CeilingKeypoint, vector<KeyPoint>& DBKeypint, vector<DMatch>& Matches, vector<DMatch>& RansacMatches);
	int findMostReliableMatchingIdx(vector<Mat> vecH);
public:
	void init();
	//-----------------------------------------------------------------
	KuPose getRobotPos();
	void setRobotPos(KuPose RobotPos);
	void setRobotPosX(double dPoseX);
	void setRobotPosY(double dPoseY);
	void setRobotPosDeg(double dPoseDeg);
	void setRobotPosRad(double dPoseRad);
	double getRobotPosX();
	double getRobotPosY();
	double getRobotPosDeg();
	double getRobotPosRad();
	void setMap(int nMapSizeX, int nMapSizeY, int** nMap);
	//------------------------------------------------------------------
public:
	void SURFFeatureExtractingProcess(Mat& matImage,vector<KeyPoint>& Keypoint, Mat& Descriptor);
	void FeatureMatchingProcess(vector<KeyPoint>& FirstKeypoint, Mat& FirstDescriptor, vector<KeyPoint>& SecondKeypoint, Mat& SecondDescriptor, vector<DMatch>& OutMatches);
	bool checkMatchingReliability(Mat& Homography);

	void initialize();
	void initDB();
	void finish();
	void detectSURFDatatoSave(Mat matCeilingImage, int nDataIdx);
	void loadSURFDB(int nIdx,  Mat& descriptors, KeyPoint& keypoint, vector<KeyPoint>& veckeypoint);

public:
	bool checkSelectPathNearLastPos();
	bool checkNearLandMarkfromRobotPos(int nCheckIdx);	
	KuPose GlobalLocalization(IplImage * IplCeilingImage);

	bool getSelectPathReliabliity();
	int getSelectPathIdx();
	void setTransitionVal(IplImage* IplCeilingImage, int nPathIdx);
	bool getRotationTruth();
	double getRotationAngle();
	double getDeltaX();
	double getDeltaY();
	int getMatchingPointNum();
	double getRansacDistanceThresholdVal(vector<double>& vecdDist);

	KuSURFbasedGlobalLocalizerPr();
	~KuSURFbasedGlobalLocalizerPr();
};

#endif 

