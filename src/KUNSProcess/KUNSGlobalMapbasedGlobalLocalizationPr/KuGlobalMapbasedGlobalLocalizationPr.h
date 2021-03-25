#ifndef KUNS_GLOBAL_BASED_GLOBAL_LOCALIZATION_PROCESS_H
#define KUNS_GLOBAL_BASED_GLOBAL_LOCALIZATION_PROCESS_H

#include <iostream>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <cv.h>
#include <highgui.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/legacy/legacy.hpp>
#include "../../KUNSPose/KuPose.h"
#include <fstream>
#include <math.h>
#include "../../Sensor/Sensor.h"
#include "../../MobileSupervisor/KuRobotParameter.h"
#include "../../KUNSUtil/KUNSSingletone/KuSingletone.h"
#include "../KUNSRetinexPr/KuRetinexPr.h"
#include "../../KUNSMath/KuMath.h"

using namespace std;
using namespace cv;

class KuGlobalMapbasedGlobalLocalizationPr : public KuSingletone<KuGlobalMapbasedGlobalLocalizationPr>
{
private:
	int m_nNumFeatureTh;
	double m_dDeltaX;
	double m_dDeltaY;
	double m_dDeltaAngle;

	double m_dcx;
	double m_dcy;

	bool m_bRobotPose;
	bool m_bRobotAngle;

private:
	void initialize();
	int findRobotPose(Mat matCeilingImage, Mat& matGlobalMap, vector<KuPose> vecRobotPose, vector<Point2i> vecnptPixcelPath, Point2i& dptRobotPose);
	void affineCeilingImage(Mat& matCeilingImage, Mat& matCeilingAffineImage, double dTheta);
	void findNearesstNode(Point2i nptRobotPose, vector<Point2i> vecPixcelPath, int& nSelectPath);
	void findInterpolationPose(Point2i nptRobotPose, int nSelectPath, vector<KuPose> vecRobotPose, vector<Point2i> vecnptPixcelPath);

private:
	void SURFFeatureExtractingProcess(Mat& matImage,vector<KeyPoint>& Keypoint, Mat& Descriptor);
	void FeatureMatchingProcess(vector<KeyPoint>& FirstKeypoint, Mat& FirstDescriptor, vector<KeyPoint>& SecondKeypoint, Mat& SecondDescriptor, vector<DMatch>& OutMatches);
	void symmetryTest(vector<DMatch>& matches1, vector<DMatch>& matches2, vector<DMatch>& symMatches);
	void calculateRT(const vector<DMatch>& matches, const vector<KeyPoint>& keypoints1, const vector<KeyPoint>& keypoints2, vector<DMatch>& outMatches);
	void detectReliableAngle(vector<KeyPoint>& SURFkeypoints_Ceiling, vector<KeyPoint>& DBSURFkeypoint, DMatch& PreMatche, DMatch& Matche, vector<double>& vecdDeltaAngle);
	Mat doRansac(vector<KeyPoint>& CeilingKeypoint, vector<KeyPoint>& DBKeypint, vector<DMatch>& Matches, vector<DMatch>& RansacMatches);
	double findRansacDistanceThresholdVal(vector<double>& vecdDist);
	bool checkMatchingReliability(Mat& Homography, Point2i nptPixcelPath, Point2i& nptRobotPose);

public:
	void setDeltaX(double dDeltaX);
	void setDeltaY(double dDletaY);
	void setDeltaAngle(double dAngle);
	void setPoseReliability(bool bPose);
	void setAngleReliability(bool bAngle);

	double getDeltaX();
	double getDeltaY();
	double getDeltaAngle();
	bool getPoseReliability();
	bool getAngleReliability();
public:
	bool doGlobalMapbasedGlobalLocalization(IplImage* IplCeilingImage, vector<KuPose> vecRobotPose, vector<Point2i> vecnptPixcelPath, KuPose& RobotPos);
	void showRansacResult(vector<KeyPoint>& CeilingKeypoint, vector<KeyPoint>& DBKeypoint, vector<DMatch>& Matches, Mat& matCeilingImage, Mat& matDBImage, Mat& matHomography);

	KuGlobalMapbasedGlobalLocalizationPr();
	~KuGlobalMapbasedGlobalLocalizationPr();
};

#endif 