/*______________________________________________________________________________________________
PROJECT : Intelligent Robotics Lab. Department of Mechanical Engineering Korea University                                    
(c) Copyright 2012 KUME Intelligent Robotics Lab.                                             
All rights reserved                                                                         
$Directed by Jae-Bok Song                                                                     
$Designed by Joong-Tae Park                                                                  
$Description :bmp, txt등 여러종류의 지도정보를 저장하고 있는 singletone type의 클래스.
$Created on: 2012. 6. 2.                                                                        
$Author: Joong-Tae Park      
$e-mail: jtpark1114@gmail.com
______________________________________________________________________________________________*/



#ifndef CMAP_REPOSITORY_H_
#define CMAP_REPOSITORY_H_

#include <iostream>
#include <fstream>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "../KUNSUtil/KUNSSingletone/KuSingletone.h"
#include "../KUNSUtil/KUNSSmartPointer/KuSmartPointer.h"
#include "../KUNSGUI/KuDrawingInfo.h"
#include "../KUNSUtil/KuUtil.h"
#include "KuMap.h"
#include "../MobileSupervisor/KuRobotParameter.h"

using namespace std;
class kuMapRepository : public KuSingletone <kuMapRepository>
{
private:
	KuMap* m_smtpMap; //지도정보를 저장할 공간.
	KuMap* m_velocitypMap; //지도정보를 저장할 공간.
	IplImage*	m_IplMapImage; //OpenCV라이브러 타입의 변수로써, BMP 파일을 읽어드려 임시로 저장할 변수
	KuUtil m_KuUtil;
	string m_strMapFilePath;
	KuPose m_poseRobotTemp; // 임시지도 저장 시 같이 저장되는 로봇의 위치
public:
	bool loadMap(string strMapFilePath);
	KuMap* getMap();
	void generateMap(string strMapFilePath, KuMap* pMap);
	void compensateMap(int nSizeX, int nSizeY, int** nMap);
	void filterMap(KuMap* pMap);
	void filterMapForOccupiedGrid(KuMap* pMap);
	void saveMap(KuMap*pMap, bool bFiltering = true, bool bDrawingInfoSetMap = true);
	void saveProbMap(KuMap*pMap, KuPose& poseRobot);
	bool loadProbMap(string strMapFilePath, double** pdMap, KuPose& poseRobot);
	bool loadVelocityMap(string strMapFilePath);
	int **getVelocityMap();
	 void saveMapNavi(KuMap*pMap);

private:
	void doMorphologyClose(KuMap* pMap);
	void doMorphologyOpen(KuMap* pMap);
	void doMorphologyCloseForOccupiedGrid(KuMap* pMap);
	void doMorphologyOpenForOccupiedGrid(KuMap* pMap);


public:
	kuMapRepository();
	~kuMapRepository();
};


#endif