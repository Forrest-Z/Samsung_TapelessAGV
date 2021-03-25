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
	double m_dDeviationforTransConverged; // ������� ���� ��Ȯ�Ǽ�
	double m_dDeviationforRotateConverged; // ȸ����� ���� ��Ȯ�Ǽ�.
	double m_dDeviationforTransRotateConverged; 	// ������� ȸ����� ������ �ִ� ��Ȯ�Ǽ�.
	int m_nMinSampleStandardDeviation;  // ������ġ���� ���� �� ������ ��ġ�� �� ������ ������ ��Ǹ𵨸� �����Ͽ� ������ �� ������ ��.
	int m_nSampleDensity;// x���� ǥ�������� 1m�̰�, y���� ǥ�������� 1m�� �� ������ ������ ����. 1m*1m ������ ������ �Ѹ� ������ ����.


private:
	KuMath m_math;
	KuUtil m_KuUtil;

public:

	int m_nSampleNum;			// ���� ����� ������ ����.;
	int m_nMaxSampleNum;		// ������ �ִ� ����.
	vector<Sample> getParticle();   // ���� list�� ����.
	vector<Sample> GetOldParticle();   // ���� list�� ����.
	int m_nLocalizationState;

	//Parameter ����------------------------------------------------------------------------------------------------
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
	KuPose* m_pSensorConfiguration; //����Ÿ ������ ��������


	vector<Sample> m_vecSample;		// ���������� �����ϴ� list.
	vector<Sample> m_vecOldSample;
	vector<Sample> m_vecOldSampleCopy;

	Sample *m_stSample;
	bool m_bResetSamples;

	int m_nSampleRegion[4];	// ������ �ѷ����� ������ �����ϴ� ����. x�ּҰ�, x�ִ밪, y�ּҰ�, y�ִ밪 ������ ����.

	double m_dBasePro;		// �븻����¡�� Ȯ�� �� ����. 1.0/���ü�.
	int m_nMinSampleNum;	// �ּ������� �Ѹ� ������ ��.
	double m_dSampleSD;
	double m_dSampleSDX;
	double m_dSampleSDY;
	double m_dSampleSDForRotation;


	double m_dEncoderData[3];

	double m_dEstimatedRangeData[181];	// ������ ��ġ���� ����Ǵ� �Ÿ���.
	double m_dMaxDistance;				// �Ÿ����� ���� �ִ� ��.
	double m_dSamplePos[3];				// ������ ������� ����� �κ��� ��ġ.

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
	double GetGaussianValue(double sigma, double x, double y);		// sigma�� x�� �Է��ϸ� ����� 0�� ����þ� ������ ���� ����� ���. ����ġ�� ����.
	double GetRandValue(double sigma);						// -2*sigma ~ 2*sigma �������� ������ �� ���.
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
