

#ifndef GLOBAL_LOCALIZER_SUPERVISOR_H
#define GLOBAL_LOCALIZER_SUPERVISOR_H


#include <vector>
#include <iostream>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "../KUNSPose/KuPose.h"
#include "../KUNSUtil/KUNSThread/KuThread.h"
#include "../KUNSUtil/KUNSSingletone/KuSingletone.h"
#include "../sensor/WheelActuatorInterface/SSAGVWheelActuatorInterface.h"
#include "../sensor/SensorSupervisor.h"
#include "../MobileSupervisor/KuRobotParameter.h"
#include "../KUNSProcess/KUNSSURFbasedGlobalLocalizerPr/KuSURFbasedGlobalLocalizerPr.h"
#include "../KUNSProcess/KUNSGlobalMapBuildingPr/KuGlobalMapBuildingPr.h"
#include "../KUNSProcess/KUNSGlobalMapbasedGlobalLocalizationPr/KuGlobalMapbasedGlobalLocalizationPr.h"

using namespace std;

class GlobalLocalizationSupervisor : public KuSingletone <GlobalLocalizationSupervisor>
{
private:
	KuThread m_SavingPathThread;
private:
	CCriticalSection m_CriticalSection;
	KuPose m_dDelEncoderData;

private:
	int m_nSelectPathIdx;
	KuPose m_SURFRobotPos;
	KuPose m_GlobalMapRobotPos;
private:
	int m_nCeilingCamLogIdx;
	ofstream m_DataImageNum;

private:
	double m_dRotationAngle;
	double m_dDeltaX;
	double m_dDeltaY;

private:
	KuPose m_RobotPos;
	KuPose m_LastRobotPos;
	int **m_nMap;
	int m_nMapSizeX;
	int m_nMapSizeY;

private:
	int_1DArray m_nLaserData181;

public:
	vector<KuPose> m_vecImagePath;
	vector<Point2i> m_vecImagePixcelPath;
	int m_nImageNum;

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
	int getTotalImageNum();
	void setMap(int nMapSizeX, int nMapSizeY, int** nMap);
	//------------------------------------------------------------------
public:
	void initialize();
	void doProcessInit();

	void saveCeilingData(IplImage* IplCeilingImage);
	void saveCeilingDB(Mat matCeilingImage);
	void loadDB(int nTotalImagSize);
	void loadCeilingImage(vector<KuPose> vecImagePath, vector<Mat>& matCeilingImages);
	void buildGlobalMap(vector<KuPose>& vecImagePath, vector<Mat>& vecmatCeilingImages, vector<Mat>& vecmatCeilingAffineImages, vector<Mat>& vecmatRetinexImages, vector<Mat>& vecmatRetinexAffineImages);
	void affineCeilingImage(vector<KuPose> vecImagePath, vector<Mat>& vecmatCeilingImages, vector<Mat>& vecmatCeilingAffineImages);
	void affineCeilingImage(Mat& matCeilingImage, Mat& matCeilingAffineImage, double dTheta);
	void cvtRetinexImage(vector<Mat>& vecmatImages, vector<Mat>& vecmatRetinexImages);
	vector<KuPose> loadImagePath(string strDataPath);
	vector<Point2i> loadImagePixcelPath(string strDataPath);
	KuPose SURFbasedGlobalLocalization(IplImage * IplCeilingImage);
	bool GlobalMapbasedGlobalLocalization(IplImage* IplCeilingImage, KuPose& RobotPose);
	bool determineGLReliability(bool bSelectPathIdx);
	bool checkSelectPathNearLastPos();

	void setTransitionVal(IplImage* IplCeilingImage);

	bool getSelectPathIdx();
	int getSURFSelectPathIdx();
	double getRotationAngle();
	double getDeltaX();
	double getDeltaY();
	KuPose loadLastRobotPos(string strDataPath);

	static void doSavingLastRobotPosThread(void* arg);
	void terminateSavingPathThread();
	void startSavingPathThrerad();

	GlobalLocalizationSupervisor();
	~GlobalLocalizationSupervisor();
};

#endif 
