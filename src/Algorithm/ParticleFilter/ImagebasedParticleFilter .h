#ifndef IMAGE_BASED_PARTICLE_FILTER_H
#define IMAGE_BASED_PARTICLE_FILTER_H
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
#include "../../KUNSMath/KuMath.h"
#include "../../KUNSMap/KuMap.h"
#include "../../KUNSUtil/KuUtil.h"
#include "../../Sensor/Sensor.h"
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/legacy/legacy.hpp>

#include "../LineDetector/LDResult.h"
#include "../FeatureDetector/FeatureDetector.h"
#include "../FeatureDetector/freak.h"
#include "../FeatureDetector/hammingseg.h"
#include "../../Sensor/SiriusCameraInterface/KuSiriusCameraInterface.h"
#include <newmat.h>

using namespace cv;
using namespace std;

class ImagebasedParticleFilter
{
	static const int MCL_CONVERGED = 0;
	static const int MCL_CONVERGING_YET = 1;
	static const int MCL_DIVERGING_NOW	= 2;
	static const int MCL_ERROR_ALLSAMPLES_DISAPPEARED =11;

	static const int STARTPOS_MARGIN=100; //100의 의미는 1m터를 cm로 표현한것.
	static const int ENDPOS_MARGIN=100; //100의 의미는 1m터를 cm로 표현한것.

	static const int GLOBAL_LOCALIZATION = 1;
	static const int LOCAL_TRACKING = 0;
	static const int NO_WORK= 3;
	static const int INITIAL_STATE = 10;

private:
	double m_dDeviationforTransConverged; // 병진운동에 대한 불확실성
	double m_dDeviationforRotateConverged; // 회전운동에 대한 불확실성.
	double m_dDeviationforTransRotateConverged; 	// 병진운동이 회전운동에 영향을 주는 불확실성.
	int m_nMinSampleStandardDeviation;  // 추정위치에서 가장 먼 샘플의 위치가 이 값보다 작으면 모션모델만 적용하여 샘플을 좀 퍼지게 함.
	int m_nSampleDensity;// x방향 표준편차가 1m이고, y방향 표준편차가 1m일 때 추출할 샘플의 갯수. 1m*1m 정도의 공간에 뿌릴 샘플의 갯수.

private:
	int m_EmptyArea;
	int m_OccupiedArea;
	int m_UnkownArea;
	int m_FixedCADArea;
	int m_DynamicEnvArea;
	int m_LuggageArea;

private:
	Mat m_cvMatImage;
	Mat m_cvMatImageBright;
	double m_dVisionWeight;

private:
	KuMath m_math;
	KuUtil m_KuUtil;
	vector<CLAMPData> m_FRD;
	vector<CFREAKData> m_FTD;	
	CFeatureDetector m_FDetectorAlg;
	vector<CLAMPData> m_observedFRD;
	vector<CFREAKData> m_observedFTD;
	double  m_dCovPose;
	double  m_dCovImage;
	double m_dCovTheta;
	double m_dCovArea;
	double  m_dCovResp;
	double  m_dCovRub;

public:
	void setCeilingMap(int sizeX, int sizeY, int** nCeilingMap);
	void copyCeilingImage(Mat cvMatImage);
	bool UpdateUsingCeilingSensor();
	inline double predictCeilingData(double x, double y, double t, int nGap);
	inline int max(int a, int b);
	inline int min(int a, int b);
	void setRegion(vector<CLAMPData>  fRegionresult);
	void setTemplate(vector<CFREAKData> fRegionresult);
	void getFeature(vector<CLAMPData>* FRD,vector<CFREAKData>*FTD);
	void getSampleSD(double *dSampleSD,double *dSampleSDForRotation,double *dVisionWeight);
	void setCameraParameter( double dCam_f,double dHeight,double dWidth);

public:

	int m_nSampleNum;			// 현재 추출된 샘플의 갯수.;
	int m_nOldSampleNum;
	int m_nMaxSampleNum;		// 샘플의 최대 갯수.
	vector<Sample> getParticle();   // 샘플 list를 전달.
	vector<Sample> GetOldParticle();   // 샘플 list를 전달.
	bool m_bCalculationStop;
	int m_nLocalizationState;

	//Parameter 조정------------------------------------------------------------------------------------------------
	void setRangeSensorParameter(int nNo, int nMinAngle, double dInterval, double dMaxDistance, double dRangeSensorOffset);
	void setSampleNum(int nMaxSample,int nMinSample);
	void setDeviation(double dDeviationforTrans, double dDeviationforRotate,double dDeviationforTransRotate );
	//----------------------------------------------------------------------------------------------------------------

	//----------------------------------------------------------------------------------------------------------------
	void setEncoderData(double dEncoderDelta[3]);
	void setRangeData(int nRangeData[Sensor::URG04LX_DATA_NUM181]);
	//----------------------------------------------------------------------------------------------------------------


	//----------------------------------------------------------------------------------------------------------------
	void setMap(int sizeX, int sizeY, int** nGridMap);
	void updateMapData(int sizeX, int sizeY, int **nGridMap); //중태 추가.
	void SetSampleRegion(int minX, int maxX, int minY, int maxY);
	//----------------------------------------------------------------------------------------------------------------

	void MCLStop();

	//----------------------------------------------------------------------------------------------------------------
	void ResetReservation() {m_bResetSamples = true;};
	void setSamples();
	int getSamplePos(double dPos[3], double dEstimatedRangeData[Sensor::URG04LX_DATA_NUM181]);
	int getSampleNum() { return m_nSampleNum;};
	void setSamplesforLocalTracking(double x, double y, double t, double dSize);
	void setSamplesNearRobot(double dx,double dy,double dt,double dSize);
	int getResamplingCnt();
	//----------------------------------------------------------------------------------------------------------------

	void startTimeCheck(LARGE_INTEGER& nStart);
	float finishTimeCheck(const LARGE_INTEGER nStart);
	void setHeight(double dHeight);
	void symmetryTest(vector<DMatch>& matches1, vector<DMatch>& matches2, vector<DMatch>& symMatches);


	inline  void trans_global2img(const float& fRx, const float& fRy, const float& fRth, const float& fFx, const float& fFy, const float& fFz, int& nu, int& nv, const CamContext* pContext);
	inline  void trans_img2global(const float& fRx, const float& fRy, const float& fRth, const int& nu, const int& nv, float& fFx, float& fFy, float& fFz, const CamContext* pContext,double dHeight);


private:
	CLDResult* m_pLDResult;
	double m_dRatio;
	double m_dsensoroffsetX;
	double m_dsensoroffsetY;


private:
	int** m_nGridMap;			// 격자지도를 저장하는 변수.

	KuPose* m_pSensorConfiguration; //무라타 초음파 센서구성

	int m_nMCLReturnValue;		// MCL 수행 결과를 리턴하는 변수
	bool m_bMCLWorkingNow;

	vector<Sample> m_vecSample;		// 샘플정보를 저장하는 list.
	vector<Sample> m_vecOldSample;
	vector<Sample> m_vecOldSampleCopy;

	Sample *m_stSample;
	bool m_bResetSamples;
	bool m_bDoRangeUpdate;		// 거리정보를 이용한 확률갱신 수행 여부를 저장.
	bool m_bDoVisionUpdate;

	double m_dAlphaV;
	double m_dAlphaR;


	double m_dMinProForUpdate;
	double m_dMaxProForUpdate;

	float **m_fRangeMCLPro;
	int m_nMapSizeX;			// 지도의 x 방향 크기.
	int m_nMapSizeY;			// 지도의 y 방향 크기.

	int m_nSampleRegion[4];	// 샘플이 뿌려지는 영역을 저장하는 변수. x최소값, x최대값, y최소값, y최대값 순서로 저장.
	bool m_bMCLWork;
	double m_dBasePro;		// 노말라이징한 확률 값 저장. 1.0/샘플수.
	int m_nMinSampleNum;	// 최소한으로 뿌릴 샘플의 수.
	double m_dSampleSD;
	double m_dSampleSDForRotation;


	double m_dRangeData[181];
	double m_dEncoderData[3];

	double m_dEstimatedRangeData[181];	// 샘플의 위치에서 예상되는 거리값.
	int m_nNoOfRangeSensorPoint;		// 몇 개의 거리센서 값을 사용하는지. 예를 들면 레이저는 181개, IR은 121개.
	double m_dRangeSensorAngleInterval;	// 거리센서 값이 몇 도 단위인지. 예를 들면 레이저는 1.0도, IR은 1.8도.
	int m_nRangeSensorMinAngle;			// 거리센서 값의 스캔 범위 중 최소값. 예를 들면 레이저는 -90도, IR은 -110도.
	double m_dMaxDistance;				// 거리센서 값의 최대 값.
	double m_dRangeSensorOffset;		// 거리센서와 로봇 중심과의 offset.
	double m_dSamplePos[3];				// 샘플의 평균으로 계산한 로봇의 위치.
	double m_dAccumulatedDistance;
	double m_dAccumulatedRotation;
	double m_dMinMatchingError;
	bool m_bgenerateSample;
	double m_dCam_f;

private:

	//----------------------------------------------------------------------------------------------------------------
	bool doParticleFiltering();
	bool UpdateUsingRangeSensor();
	bool MotionModel();
	bool SensorModel();
	//----------------------------------------------------------------------------------------------------------------
	void initProbability();

	void initStateforMap();
	void generateGaussianValue( );
	double getGaussianValue(int nX, int nY);

	//----------------------------------------------------------------------------------------------------------------
	double GetGaussianValue(double sigma, double x);		// sigma와 x를 입력하면 평균이 0인 가우시안 분포에 따른 결과를 출력. 가중치로 사용됨.
	double GetRandValue(double sigma);						// -2*sigma ~ 2*sigma 범위에서 임의의 값 출력.
	void Normalizing();
	void Resampling();
	//----------------------------------------------------------------------------------------------------------------

	//----------------------------------------------------------------------------------------------------------------
	inline void predictRangeData(double x, double y, double t, int nGap);
	inline void doRayTracing(int x_1, int x_2, int y_1, int y_2, int *hit_x, int *hit_y);
	//----------------------------------------------------------------------------------------------------------------
	inline void getRayTracingData(int x_1, int x_2, int y_1, int y_2, int *hit_x, int *hit_y);

	//----------------------------------------------------------------------------------------------------------------
	void CalculateNoOfSamples(bool bCalSampleCnt);
	bool CheckMovementForUpdateOrNot(int nMode);
	void CheckLocalizationState();
	//----------------------------------------------------------------------------------------------------------------


public:
	ImagebasedParticleFilter();
	~ImagebasedParticleFilter();
};

#endif
