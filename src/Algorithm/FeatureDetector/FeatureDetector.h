#pragma once

#include "../LineDetector/LDResult.h"
#include "FDResult.h"
#include "harrisDetector.h"
#include <opencv2/core/core.hpp>
#include <opencv2/core/core_c.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/legacy/legacy.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/legacy/compat.hpp"
#include "freak.h"
#include "hammingseg.h"

using namespace cv;

static const Vec3b bcolor[] =
{
	Vec3b(0,0,255),
	Vec3b(0,255,0),
	Vec3b(255,0,0),
	Vec3b(0,255,255),
	Vec3b(255,255,0),
	Vec3b(255,0,255),
	Vec3b(0,128,255),
	Vec3b(255,128,0),
	Vec3b(0,255,128)
};

typedef struct _tagRegionData
{
	int u, v; // center points
	int mu, mv; // center points
	double x, y; // center points
	int area; // area of blob
	double th; // polar coordinate
	double th2; // polar coordinate
	float angle; // polar coordinate
	double width, height; // center points
	double dDir; 
	int idx;
	bool add;
	bool detect;
	int nmatchnum;
	double ceilingheight;
	double Rx, Ry, Rt; // robot pose
	int ntype; //0: 방향 1: 방향 없음

} CLAMPData;

typedef struct _tagTampletData
{
	int u, v; // center points
	double x, y; // center points
	double th; // polar coordinate
	float angle; // polar coordinate
	int idx;
	bool add;
	int MatchNum;
	bool associate;
	double similarity;
	int imageSizeX;
	int imageSizeY;
	Mat sourceImage;
	bool detect;
	int nmatchnum;
	double ceilingheight;
	double Rx, Ry, Rt; // robot pose
	float response;

} CFREAKData;

class AFX_CLASS_EXPORT CFeatureDetector
{

public:
	/* Functions */
	CFeatureDetector(void);
	~CFeatureDetector(void);

	//void readINI(std::string path);

	
	vector<CLAMPData> detectRegion(Mat matImgSrc,bool bmapping);

	void setRegion(vector<CLAMPData>  fRegionresult);
	void init();
	

	int templateMatching(IplImage *sourceImage,IplImage *templateImage,double dthreshold,double dtheta,CvPoint* cvResultsMinPoint,CvPoint* cvResultsMaxPoint,double* dminVal);
	vector<CFREAKData> checktempletRegion(Mat matImgSrc,bool bmapping);
	void setTampletData(vector<CFREAKData> FTData);
	void trainingRegion(Mat matImgSrc);
	void getEllipseData(double *dWidth,double *dHeight);
	void setEllipseData(double dWidth,double dHeight);



	/* Variables */
private:
	void startTimeCheck(LARGE_INTEGER& nStart);
	float finishTimeCheck(const LARGE_INTEGER nStart);

private:
	/* Functions */

	/* Variables */
	CLDResult m_ldresult;
	vector<FDResult> m_fdresult;
	int m_nDBNum;
	bool m_brecData;
	Mat m_cvGrayMatImage;
	Mat m_cvMatImage;
	IplImage* m_IplCeilingImage;
	CvMemStorage* m_storage;
	vector<CLAMPData> m_fRegionresult;
	vector<CFREAKData> m_vecFTData;
	double m_dWidth;
	double m_dHeight;

};

