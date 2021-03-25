/*______________________________________________________________________________________________
PROJECT : Intelligent Robotics Lab. Department of Mechanical Engineering Korea University                                    
(c) Copyright 2012 KUME Intelligent Robotics Lab.                                             
All rights reserved                                                                         
$Directed by Jae-Bok Song                                                                     
$Designed by Joong-Tae Park                                                                  
$Description :그림 그릴 각종 정보들을 저장하고 있는 싱글톤 타입의 클래스.
$Created on: 2012. 6. 4.                                                                        
$Author: Joong-Tae Park      
$e-mail: jtpark1114@gmail.com
______________________________________________________________________________________________*/

#ifndef KUNS_DRAWING_INFO_H
#define KUNS_DRAWING_INFO_H


#include <iostream>
#include <vector>
#include <list>
#include "../KUNSMap/KuMap.h"
#include "../KUNSUtil/KuUtil.h"
#include "../KUNSMath/KuMath.h"
#include "../KUNSUtil/KUNSSmartPointer/KuSmartPointer.h"
#include "../KUNSUtil/KUNSCriticalSection/KuCriticalSection.h"
#include "../KUNSUtil/KUNSSingletone/KuSingletone.h"
#include "../sensor/Sensor.h"
#include "../KUNSPose/KuPose.h"
#include "../Algorithm/ParticleFilter/Sample.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "../Algorithm/FeatureDetector/FDResult.h"
#include "../Algorithm/FeatureDetector/FeatureDetector.h"
#include "../Algorithm/PathBlock/PathBlock.h"


using namespace std;
class KuDrawingInfo : public KuSingletone <KuDrawingInfo>
{
	// CSLAM
private:
	CCriticalSection m_CriticalSection;
private:
	KuUtil m_KuUtil;

	///map 관련 변수
	int m_nMapX;
	int m_nMapY;
	int **  m_nMap; /// 환경에 대한 정보를 정수 형태로 저장하는 변수
	double ** m_dProMap;/// 환경에 대한  확률적인 정보로 저장하는 변수

	int **  m_nCeilingMap; /// 환경에 대한 정보를 정수 형태로 저장하는 변수
	
	int **  m_nZoneMap; /// 환경에 대한 정보를 정수 형태로 저장하는 변수

	//sensor 관련
	int_1DArray m_nLaserDataFront181; ///181개의 레이저 데이터를 저장하는 변수.
	int_1DArray m_nLaserDataRear181; ///181개의 레이저 데이터를 저장하는 변수.
	int_1DArray m_nAlignLaserData181; ///181개의 레이저 데이터를 저장하는 변수.
	int_1DArray m_nKinectRangeData; ///57개의 키넥트 데이터를 저장하는 변수.
	IplImage *m_IplCeilingImage;// 천장 이미지 변수
	IplImage *m_IplKinectImg;// 천장 이미지 변수
	int_1DArray m_nTData; ///181개의 레이저 데이터를 저장하는 변수.


	KuPose m_RobotPos; //로봇 위치값을 저장하는 변수
	KuPose m_RobotPos2; //로봇 위치값을 저장하는 변수
	KuPose m_AuxiliaryRobotPos; //실험 목적의 로봇 위치를 저장하는 변수 예) 엔코더 기반의 로봇 위치, 가상로봇등
	KuPose m_IdealRobotPos; //위치오차가 전혀없는 로봇 위치를 저장하는 변수. 실험목적으로 사용가능함.
	KuPose m_GoalPos; //골 위치값을 저장하는 변수
	KuPose m_TargetPos; //target pose를 저장하는 변수
	int m_nTVByKeyEvt, m_nRVByKeyEvt; //키보드 이벤트를 통해 받은 로봇 속도를 저장하는 변수.
	KuPose m_LocalGoalPos;
	vector<PBlock> m_vecPathBlockPos;
	vector<vector<PBlock>> m_vecGlobalPathBlock;

	//particle 관련 변수
	vector<Sample> m_vecParticle;

	//경로관련 변수
	list<KuPose> m_listPath;
	list<KuPose> m_listWayPoint;
	list<KuPose> m_listLocalPath;
	vector<list<KuPose>> m_veclistPathlist;

	//골 위치관련 변수
	list<KuPose> m_GoalPosList;

	//지도 그리기 관련 변수
	bool m_bRenderMapflag;
	bool m_bRenderBuildingMapflag;
	bool m_bRenderZoneMapflag;

	//레이저 그리기 관련 변수
	bool m_bRendeLaserflag;
	//키넥트 그리기 관련 변수
	bool m_bRenderKinectflag;
	// 경로 그리기 관련 변수
	bool m_bRenderPathflag;
	bool m_bRenderKinectDepthflag;

	// 데이터 전송 관련
	bool m_bDataAccess;

	bool m_bDirectionofPathflag;

	vector<KuPose> m_vecLandmark;

	bool m_bRenderCeilingImagflag;

	bool m_bWayPointflag;
	KuPose m_Global3DPose[Sensor::IMAGE_WIDTH*Sensor::IMAGE_HEIGHT];
	int** m_nCADMap;
	bool m_bRenderCADMapflag;

	bool m_bWaitDiffAGVflag;

	vector<CLAMPData> m_FRD;
	vector<CFREAKData> m_FTD;

	list<KuPose> m_FiducialLandMarkList;

public:
	void setCeilingMap(KuMap *pMap);
	int** getCeilingMap();

	void setKinectGlobal3DPos(KuPose* pGlobal3DPose);
	KuPose*  getKinectGlobal3DPos();

	void setCADMap(int** nCADMap);
	int** getCADMap();

	void setRenderCADMapflag(bool bRenderCADMapflag);
	bool getRenderCADMapflag();

	void setAlignLaserData181(int_1DArray nLaserData181);
	int_1DArray getAlignLaserData181();

	void getRegion(vector<CLAMPData>* fRegionresult);
	void setRegion(vector<CLAMPData> fRegionresult);
	void getTemplateData(vector<CFREAKData>* FTemplate);
	void setTemplateData(vector<CFREAKData> fTemplateresult);

	void drawFiducialMarkInfoToImage(list <KuPose> lFiducialLandMarkImgPos);
	void setFiducialMarkList(list<KuPose> FiducialLandMarkList);
	list<KuPose> getFiducialMarkList();
	void drawFiducialMarkInfoToImage(KuPose lFiducialLandMarkImgPos);

public:

	void initVariable(); ///변수들을 초기화 한다. 

	///map 관련 함수
	void setMap(KuMap *pMap); //지도정보를 저장하는 함수.
	int** getMap(); /// 저장된 지도정보를 얻어가는 함수.
	double** getBuildingMap( int* nX, int* nY);// 작성중인 지도의 정보를 얻어가는 함수
	void setBuildingMap( double** dMap );
	int getMapSizeX();
	int getMapSizeY();


	//센서관련 함수
	void setFrontLaserData181(int_1DArray nLaserData181); /// 레이저 데이터를 저장하는 함수.
	void setRearLaserData181(int_1DArray nLaserData181);
	int_1DArray getLaserDataFront181(); ///저장된 레이저 데이터를 넘겨주는 함수
	int_1DArray getLaserDataRear181();

	void setKinectRangeData(int_1DArray nKinectRangeData); /// 키넥트 데이터를 저장하는 함수.
	int_1DArray getKinectRangeData();///키넥트 데이터를 넘겨주는 함수

	void setCeilingImageData(IplImage* imageData); // 천장 영상 데이터를 저장하는 함수.
	void getCeilingImageData(IplImage* ImageData);// 천장 영상 데이터를  넘겨주는 함수.	

	void getKinectImageData(IplImage* IplKinectImg);
	void setKinectImageData(IplImage* IplKinectImg);


	KuPose  getRobotPos(); ///저장된 로봇 위치를 넘겨주는 함수.
	void setRobotPos(KuPose RobotPos); //로봇 위치를 저장하는 함수

	KuPose  getRobotPos2(); ///저장된 로봇 위치를 넘겨주는 함수.
	void setRobotPos2(KuPose RobotPos); //로봇 위치를 저장하는 함수

	KuPose  getAuxiliaryRobotPos(); /////실험 목적의 로봇 위치를 넘겨주는 함수.
	void setAuxiliaryRobotPos(KuPose auxiliaryRobotPos); //실험 목적의 로봇 위치를 저장하는 함수

	KuPose  getIdealRobotPos(); /////실험 목적의 이상적인 로봇 위치를 넘겨주는 함수.
	void setIdealRobotPos(KuPose IdealRobotPos); //실험 목적의 로봇 위치를 저장하는 함수

	KuPose  getGoalPos(); ///저장된 골 위치를 넘겨주는 함수.
	void setGoalPos(KuPose GoalPos); //골 위치를 저장하는 함수

	list<KuPose> getGoalPosList(); ///저장된 골 위치를 넘겨주는 함수.
	void setGoalPosList(list<KuPose> GoalPosList); //골 위치를 저장하는 함수

	//그리기 관련 함수
	void setRenderMapflag(bool bRenderMapflag);
	bool getRenderMapflag(); 
	void setRenderBuildingMapflag(bool bRendbRenderBuildingMapflagerMapflag);
	bool getRenderBuildingMapflag();
	void setRenderLaserflag(bool bRendeLaserflag);
	bool getRenderLaserflag();
	void setRenderKinectflag(bool bRenderKinectflag);
	bool getRenderKinectflag();
	void setRenderPathflag(bool bRenderPathflag);
	bool getRenderPathflag();

	//가상로봇 제어관련 함수
	void setRobotTRVel(int nTVel, int nRVel);
	void getRobotTRVel(int* nTVel, int* nRVel);

	//파티클 관련 함수
	void setParticle(vector<Sample> vecParticle); ///파티클에 대한 정보를 저장하는 함수
	vector<Sample> getParticle(); ///파티클에 대한 정보를 넘겨주는 함수

	//경로관련 함수
	void setPath(list<KuPose> vecPathPos); ///경로를 저장하는 함수.
	list<KuPose> getPath(); ///경로를 넘겨주는 함수.

	//target pos관련 함수
	void setTargetPos(KuPose TargetPos); //target pos를 저장하는 함수
	KuPose getTargetPos(); //target pos를 넘겨주는 함수.

	void setDirectionofPathflag(bool bDirectionofPathflag);
	bool getDirectionofPathflag();

	vector<KuPose> getvecLandmarkPos();
	void setvecLandmarkPos(vector<KuPose> vecLandmark);

	void setRenderCeilingImageflag(bool bRenderCeilingImagflag);
	bool getRenderCeilingImageflag();

	void setWayPointList(list<KuPose> listWayPoint);
	list<KuPose> getWayPointList();

	void setWayPointflag(bool bWayPointflag);
	bool getWayPointflag();

	void setRenderKinectDepthflag(bool bRenderKinectDepthflag);
	bool getRenderKinectDepthflag();


	void setLocalPath(list<KuPose> listPath);
	list<KuPose> getLocalPath();

	void setLocalGoalPos(KuPose TargetPos);
	KuPose getLocalGoalPos();

	void setWaitforDiffAGV(bool bWaitDiffAGVflag);
	bool getWaitforDiffAGV();

	void setRenderZoneMapflag(bool bRenderZoneMapflag);
	bool getRenderZoneMapflag();

	void setZoneMap(KuMap *pMap);
	int** getZoneMap();

	void  getPathBlockPos(vector<PBlock>*vecPathBlockPos);
	void  setPathBlockPos(vector<PBlock> vecPathBlockPos);
	void clearGlobalPathBlockPos();

	void setTData(int_1DArray nLaserData181); /// 레이저 데이터를 저장하는 함수.
	int_1DArray getTData(); ///저장된 레이저 데이터를 넘겨주는 함수

	void setvecPathlist(vector<list<KuPose>> veclistPathlist);
	vector<list<KuPose>>  getvecPathlist();


public:
	KuDrawingInfo();
	~KuDrawingInfo();
};

#endif /*DRAWING_INFO_H*/