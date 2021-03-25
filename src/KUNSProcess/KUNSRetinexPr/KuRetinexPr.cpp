#include "stdafx.h"
#include "KuRetinexPr.h"

KuRetinexPr::KuRetinexPr()
{

}

KuRetinexPr::~KuRetinexPr()
{

}

//recursive 가우시안 필터의 계수계산
void KuRetinexPr::computeCoefofGaussianBlurring(double dCoef[5], double dSigma)
{
	/*
	* "Recursive Implementation of the gaussian filter.",
	* Ian T.Young, Lucas J.Van Vliet, Signal Processing 44, Elsevier 1995.
	*/
	double q=0;
	if(dSigma>=2.5)
	{
		q = 0.98711*dSigma - 0.96330;
	}
	else if((dSigma>=0.5)&&(dSigma<2.5))
	{
		q = 3.97156 - 4.14554*(double)sqrt((double) 1 - 0.26891*dSigma);
	}
	else
	{
		q=0.1147705018520355224609375;
	}

	double q2=q*q;
	double q3=q*q2;
	dCoef[0]=(1.57825 + (2.44413*q) + (1.4281*q2) + (0.422205*q3));
	dCoef[1]=(			(2.44413*q) + (2.85619*q2) + (1.26661*q3));
	dCoef[2]=(						-((1.4281*q2) + (1.2661*q3)));
	dCoef[3]=(										(0.422205*q3));
	dCoef[4]=1.0-(dCoef[1]+dCoef[2]+dCoef[3])/dCoef[0];//=B*;
}

//멀티스케일 설정
void KuRetinexPr::findRetinexScaleDistribution(const int nScales/*=3*/, const int nDefaultScale/*=240*/, double dScales[])
{
	//ASSERT(nscales>=3);
	double dStepSize = (double) nDefaultScale / (double) nScales;
	for(int i=0; i<nScales; ++i)
	{
		dScales[i] = 2. + (double)i*dStepSize;
	}
}

//가우시안 convolution(horizontal)
void KuRetinexPr::doGaussianSmoothing(double *dInputImage, int nWidth, int nHeight, double *dBlurredImage, double dCoef[5])
{
	int nBufSize = nWidth+3;
	vector<double> w1(nBufSize);
	vector<double> w2(nBufSize);
	double dInvCoef0 = 1./dCoef[0];
	for(int y=0; y<nHeight; y++)
	{
		/*forward pass*/
		double *dInputImage_ptr = &dInputImage[y*nWidth];
		w1[0] = dInputImage_ptr[0];
		w1[1] = dInputImage_ptr[0];
		w1[2] = dInputImage_ptr[0];
		for(int x=0; x<nWidth; x++)
		{
			w1[x+3] = dCoef[4]*dInputImage_ptr[x] + ((dCoef[1]*w1[x+2]+dCoef[2]*w1[x+1]+dCoef[3]*w1[x])*dInvCoef0);
		}
		/*backward pass : out==>transposed;*/
		w2[nWidth+0]=w1[nWidth+2];
		w2[nWidth+1]=w1[nWidth+2];
		w2[nWidth+2]=w1[nWidth+2];
		for(int x=nWidth-1; x>=0; x--)
		{
			w2[x] = dBlurredImage[x*nHeight+y] = dCoef[4]*w1[x]+((dCoef[1]*w2[x+1] + dCoef[2]*w2[x+2] + dCoef[3]*w2[x+3])*dInvCoef0);
			if(dBlurredImage[x*nHeight+y]>255)dBlurredImage[x*nHeight+y]=255;
			if(dBlurredImage[x*nHeight+y]<0)dBlurredImage[x*nHeight+y]=0;
		}
	}
}

//이미지의 평균과 표준편차 계산
void KuRetinexPr::findImageStatistics(double *dInputImage, int nSize/*=width*height*/, double *dMean, double *dDerivation)
{
	double dSum=0, dSum_Square=0;
	for(int i=0; i<nSize; i++)
	{
		double a = dInputImage[i];
		dSum += a;
		dSum_Square += (a*a);
	}
	*dMean =dSum / nSize;
	*dDerivation = sqrt((dSum_Square-dSum*dSum/nSize)/nSize);
}

void KuRetinexPr::rescaleRange(double *dData, int nSize)
{
	double dMean, dDerivation;
	findImageStatistics(&dData[0], nSize, &dMean, &dDerivation);

	double dMaxVal = dMean + 1.2*dDerivation;
	double dMinVal = dMean - 1.2*dDerivation;
	double dRange = dMaxVal - dMinVal;
	if(!dRange) 
	{
		dRange = 1.0;
	}
	dRange = 255./dRange;
	//change the range;
	for(int i=0; i<nSize; i++)
	{
		if(dData[i]>=dMaxVal)
		{
			dData[i] =  255;
		}
		else if(dData[i]<dMaxVal&&dData[i]>=dMinVal)
		{
			dData[i] =  (dData[i]-dMinVal)*dRange;
		}
		else
		{
			dData[i] =  0;
		}
		
	}
}

void KuRetinexPr::doRetinexProcess(Mat& MatInputImage, Mat& MatOutputImage, int nDefaultScale)
{
	int nWidth = MatInputImage.cols;
	int nHeight = MatInputImage.rows;
	double* dInputImage = new double[nWidth*nHeight];
	double* dBlurredImage = new double[nWidth*nHeight];

	for(int i=0; i<nWidth*nHeight; i++)
	{
		dInputImage[i] = (double)MatInputImage.data[i];
		dBlurredImage[i] = (double)MatInputImage.data[i];
	}

	const int nFilter = 3;
	double dSigma[nFilter];
	//int nDefaultScale = 33;
	findRetinexScaleDistribution(nFilter, nDefaultScale, dSigma);

	vector<double> vecdScaleImg[nFilter];
	//allocate filtered image;
	for(int i=0; i<nFilter; i++)
	{
		vecdScaleImg[i].resize(nWidth*nHeight);
	}

	//scale-space gauss_smooth;
	for(int i=0; i<nFilter; i++)
	{
		double dCoef[5];
		computeCoefofGaussianBlurring(dCoef, dSigma[i]);
		//seperable gaussian convolution;
		//(1)horizontal convolution; (wxh-->hxw); and dst as a temp-buffer;
		doGaussianSmoothing(&dInputImage[0],nWidth, nHeight, &dBlurredImage[0],dCoef);
		//(2)vertical convolution;(hxw-->wxh);
		doGaussianSmoothing(&dBlurredImage[0], nHeight, nWidth, &((vecdScaleImg[i])[0]), dCoef);
	}

	//reset dBlurredImage;
	for(int i=0; i<nWidth*nHeight; i++)
	{
		dBlurredImage[i] = 0;
	}

	//accumulate dBlurredImage;
	for(int i=0; i<nFilter; i++)
	{
		vector<double>&filtered=vecdScaleImg[i];
		for(int k=0; k<nWidth*nHeight;k++)
		{
			dBlurredImage[k]+=log(dInputImage[k]+1.)-log(filtered[k]+1.);
		}
	}

	//scale to [0,255];
	rescaleRange(&dBlurredImage[0],nWidth*nHeight);

	for(int i=0; i<nWidth*nHeight; i++)
	{
		MatOutputImage.data[i]=dBlurredImage[i];
	}

	delete []dInputImage;
	delete []dBlurredImage;
}