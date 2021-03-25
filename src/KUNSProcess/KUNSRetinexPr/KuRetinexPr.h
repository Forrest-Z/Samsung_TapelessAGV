#ifndef KUNS_RETINEX_PROCESS_H
#define KUNS_RETINEX_PROCESS_H

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv.h>
#include <highgui.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "../../KUNSUtil/KUNSSingletone/KuSingletone.h"

using namespace std;
using namespace cv;

class KuRetinexPr
{
private:


public:
	//Retinex
	void computeCoefofGaussianBlurring(double dCoef[5], double dSigma);
	void findRetinexScaleDistribution(const int nscales/*=3*/, const int nDefaultScale/*=240*/, double dScales[]);
	void doGaussianSmoothing(double *dInputImage, int nWidth, int nHeight, double *dBlurredImage, double dCoef[5]);
	void findImageStatistics(double *dInputImage, int nSize/*=width*height*/, double *dMean, double *dDerivation);
	void rescaleRange(double *dData, int nSize);
	void doRetinexProcess(Mat& MatInputImage, Mat& MatOutputImage, int nDefaultScale);

	KuRetinexPr();
	~KuRetinexPr();
};

#endif