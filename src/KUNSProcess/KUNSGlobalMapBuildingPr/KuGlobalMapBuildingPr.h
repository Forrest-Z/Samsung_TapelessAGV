#ifndef KUNS_GLOBAL_MAPBUILDING_PROCESS_H
#define KUNS_GLOBAL_MAPBUILDING_PROCESS_H

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
#include "../../KUNSPose/KuPose.h"
#include <fstream>
#include <math.h>
#include "../../Sensor/Sensor.h"
#include "../../MobileSupervisor/KuRobotParameter.h"
#include "../../KUNSUtil/KUNSSingletone/KuSingletone.h"
#include "../KUNSRetinexPr/KuRetinexPr.h"
#include "../../Algorithm/ParticleFilter/CeilingImagebasedParticleFilter.h"

using namespace std;
using namespace cv;

class KuGlobalMapBuildingPr : public KuSingletone<KuGlobalMapBuildingPr>
{
private:
	int m_nHalfSizeX;
	int m_nHalfSizeY;

	double m_dfx;
	double m_dfy;
	double m_dcx;
	double m_dcy;

	int m_nx;
	int m_ny;

	KuRetinexPr m_RetinexPr;
private:
	//GlobalMapBuilding
	void init();
	void findGlobalMapProperties(vector<KuPose> vecImagePath, double dCeilingHeigh, double dScale, Point2d& ptMinXMaxY, Point2i& ptInitial ,int& nGlobalMapWidth, int& nGlobalMapHeigh, vector<double>& vecptdDirection);
	void initGlobalMap(Mat matCeilingImage, Mat matRetinexImages, Point2d PoinptInitialImageCenter, Point2i ptInitialPixcel, Mat& MatGlobalMap, Mat& MatGlobalSegment, Mat& matRetinexGlobalMap);
	void doMergeGlobalMap(Mat matCeilingImage, Mat matCeilingRetinexImage, Mat& matGlobalMap, Mat& matGlobalSegment, Mat& matRetinexGlobalMap, Point2f ptPrincipalPoint, Point2d ptdMergeCtrPoint, Point2i ptPreCtrPoint, double dTheta, int nx, int ny, double dDirection);
	void InitializeMergeGlobalMap(Mat matCeilingImage, Mat matRetinexImage, Mat& matGlobalMap, Mat& matGlobalSegment, Mat& matRetinexGlobalMap, Point2i ptCeilingCtrPoint, Point2i ptMergeCtrPoint, double dTheta, int nx, int ny);
	
public:
	void affineCeilingImage(vector<KuPose> vecImagePath, vector<Mat>& vecmatCeilingImages, vector<Mat>& vecmatCeilingAffineImages);
	void affineCeilingImage(Mat& matCeilingImage, Mat& matCeilingAffineImage, double dTheta);
	void affineCeilingImage2(Mat& matCeilingImage, Mat& matCeilingAffineImage, double dTheta);
	void cvtRetinexImage(vector<Mat>& vecmatImages, vector<Mat>& vecmatRetinexImages);
	void saveImagePixcelPath(vector<Point2i> vecImgPixcelPath);
	void buildGlobalMap(vector<KuPose>& vecImagePath, vector<Point2i>& vecImagePixcelPath, vector<Mat>& vecmatCeilingImages, vector<Mat>& vecmatCeilingAffineImages, vector<Mat>& vecmatRetinexImages, vector<Mat>& vecmatRetinexAffineImages);

	KuGlobalMapBuildingPr();
	~KuGlobalMapBuildingPr();
};

#endif 