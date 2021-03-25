#ifndef KUNS_ICP_H
#define KUNS_ICP_H

#include <iostream>
#include <cmath>
#include <cv.h>
#include <cxcore.h>

using namespace std;

struct IcpPos {
	double x, y,th;
	double error;
	int nMatchNum;


};

// struct sPoint {
// 	double x, y;
// 
// 	sPoint (double x_ = 0., double y_ = 0.) : x(x_), y(y_) { }
// };/
// 
// struct sPointPair {
// 	sPoint p;	// input data
// 	sPoint q;	// model data
// 	sPoint n;	// normal vector of model data
// };
class KuICP
{
private:
	double m_dDeviationforTrans;
	double m_dDeviationforRotate;
	double m_dDeviationforTransRotate;

private:
	inline void findClosestPoint(int* nMatchingPointIndex, double* PreData,int NoPreData, double* NewData, int NoNewData,double dDIstTh ,int *nCnt);
	inline void calDeltaPos(int*nMatchingPointIndex,double* PreData,int NoPreData, double* NewData, int NoNewData,int cnt,double *DeltaX,double *DeltaY,double *DeltaT);
	inline void calTransformedData(double* PreData,int NoPreData,double*TransformedData, int NoNewData,double DeltaX,double DeltaY,double DeltaT,double *accDeltaX,double *accDeltaY,double *accDeltaT); 
	inline double calRMSError(int* nMatchingPointIndex,double* PreData,int NoPreData, double*TransformedData, int NoNewData,int cnt);
	void startTimeCheck(LARGE_INTEGER& nStart);
	float finishTimeCheck(const LARGE_INTEGER nStart);
	inline double GaussRand();
	inline double GetRandValue(double sigma);

public:
	double icp(double R[3][3], double T[3], double* PreData, 
			   int NoPreData, double* NewData, int NoNewData, 
			   int MinIter, int MaxIter, double MinOverrap, 
			   double threshold, double margin,
			   double dEXm,double dEYm,double dETm);

	double icp(double R[3][3], double T[3], double* PreData, 
				int NoPreData, double* NewData, int NoNewData, 
				int MinIter, int MaxIter, double MinOverrap, double threshold, double margin);

	double icp_Mathch(double R[3][3], double T[3], double* PreData, 
				int NoPreData, double* NewData, int NoNewData, 
				int MinIter, int MaxIter, double MinOverrap, double threshold, double margin,
				double dEXm,double dEYm,double dETm);

	void setDeviation(double dDeviationforTrans, double dDeviationforRotate,double dDeviationforTransRotate );

	KuICP();
	~KuICP();
};

#endif