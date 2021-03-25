#ifndef CEILING_IMAGE_BASED_PARTICLE_FILTER_H
#define CEILING_IMAGE_BASED_PARTICLE_FILTER_H
//////////////////////////////////////////////////////////////////////
// CKinectSonarBasedParticleFilter.h: interface for the MCL class.
// copyright: IRL (Intelligent Robotics Lab.) at Korea university
// Author: Tae-Bum Kwon, Joong-Tae Park (haptics@korea.ac.kr, geullu@korea.ac.kr)
//////////////////////////////////////////////////////////////////////
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <list>
#include "sample.h"
#include <fstream>
#include <cmath>
#include <iostream>
#include "../../KUNSPose/KuPose.h"
#include "../../KUNSMath/KuMath.h"
#include "../../KUNSUtil/KuUtil.h"
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/legacy/legacy.hpp>
#include <math.h>
#include "../../MobileSupervisor/KuRobotParameter.h"
#include "../../Sensor/Sensor.h"

using namespace std;
using namespace cv;

class CeilingImageBasedParticleFilter
{
	static const int MCL_CONVERGED = 0;
	static const int MCL_CONVERGING_YET = 1;
	static const int MCL_DIVERGING_NOW	= 2;
	static const int MCL_ERROR_ALLSAMPLES_DISAPPEARED =11;


private:
	double m_dDeviationforTransConverged; // 병진운동에 대한 불확실성
	double m_dDeviationforRotateConverged; // 회전운동에 대한 불확실성.
	double m_dDeviationforTransRotateConverged; 	// 병진운동이 회전운동에 영향을 주는 불확실성.
	int m_nMinSampleStandardDeviation;  // 추정위치에서 가장 먼 샘플의 위치가 이 값보다 작으면 모션모델만 적용하여 샘플을 좀 퍼지게 함.
	int m_nSampleDensity;// x방향 표준편차가 1m이고, y방향 표준편차가 1m일 때 추출할 샘플의 갯수. 1m*1m 정도의 공간에 뿌릴 샘플의 갯수.


private:
	KuMath m_math;
	KuUtil m_KuUtil;

public:

	int m_nSampleNum;			// 현재 추출된 샘플의 갯수.;
	int m_nMaxSampleNum;		// 샘플의 최대 갯수.
	vector<Sample> getParticle();   // 샘플 list를 전달.
	vector<Sample> GetOldParticle();   // 샘플 list를 전달.
	int m_nLocalizationState;

	//Parameter 조정------------------------------------------------------------------------------------------------
	void setSampleNum(int nMaxSample,int nMinSample);
	void setDeviation(double dDeviationforTrans, double dDeviationforRotate,double dDeviationforTransRotate );
	//----------------------------------------------------------------------------------------------------------------


	//----------------------------------------------------------------------------------------------------------------
	void ResetReservation() {m_bResetSamples = true;};
	int getSampleNum() { return m_nSampleNum;};
	bool setSamplesNearRobot(int dx,int dy,int dSize, Mat& matGlobalImage);
	int getResamplingCnt();
	//----------------------------------------------------------------------------------------------------------------

	double doImagebasedParticlefilter(Point2i ptGlobalPos, Mat matRetinexImg, Mat matGlobalImg, Point2i& ptCeilingCtrPoint, int nx, int ny);
	void init();
	void finish();

private:
	KuPose* m_pSensorConfiguration; //무라타 초음파 센서구성


	vector<Sample> m_vecSample;		// 샘플정보를 저장하는 list.
	vector<Sample> m_vecOldSample;
	vector<Sample> m_vecOldSampleCopy;

	Sample *m_stSample;
	bool m_bResetSamples;

	int m_nSampleRegion[4];	// 샘플이 뿌려지는 영역을 저장하는 변수. x최소값, x최대값, y최소값, y최대값 순서로 저장.

	double m_dBasePro;		// 노말라이징한 확률 값 저장. 1.0/샘플수.
	int m_nMinSampleNum;	// 최소한으로 뿌릴 샘플의 수.
	double m_dSampleSD;
	double m_dSampleSDX;
	double m_dSampleSDY;
	double m_dSampleSDForRotation;


	double m_dEncoderData[3];

	double m_dEstimatedRangeData[181];	// 샘플의 위치에서 예상되는 거리값.
	double m_dMaxDistance;				// 거리센서 값의 최대 값.
	double m_dSamplePos[3];				// 샘플의 평균으로 계산한 로봇의 위치.

	double m_dMinMatchingError;

	bool m_bgenerateSample;
	bool m_bImageModel;

	double m_dfx;
	double m_dfy;
	double m_dcx;
	double m_dcy;
	double m_dGlobalMaxWidth;
	double m_dGlobalMaxHeigh;

	int m_nXMargin;
	int m_nYMargin;
	int m_nMaxIteration;
	int m_nMinIteration;
	double m_dSDThreshold;
	//----------------------------------------------------------------------------------------------------------------
	bool doParticleFiltering(Mat matGlobalImg,vector<Mat> vec_matCeilingImg, int nx, int ny);
	bool ImageModel(int nx,int ny);
	//----------------------------------------------------------------------------------------------------------------


	void initProbability();

	//----------------------------------------------------------------------------------------------------------------
	double GetGaussianValue(double sigma, double x);
	double GetGaussianValue(double sigma, double x, double y);		// sigma와 x를 입력하면 평균이 0인 가우시안 분포에 따른 결과를 출력. 가중치로 사용됨.
	double GetRandValue(double sigma);						// -2*sigma ~ 2*sigma 범위에서 임의의 값 출력.
	void Normalizing();
	void Resampling();
	//----------------------------------------------------------------------------------------------------------------


	//----------------------------------------------------------------------------------------------------------------
	void CalculateNoOfSamples();
	//----------------------------------------------------------------------------------------------------------------


public:
	CeilingImageBasedParticleFilter();
	~CeilingImageBasedParticleFilter();
};

#endif
