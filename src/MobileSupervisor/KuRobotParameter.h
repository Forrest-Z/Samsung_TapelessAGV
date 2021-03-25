/*______________________________________________________________________________________________
PROJECT : Intelligent Robotics Lab. Department of Mechanical Engineering Korea University                                    
(c) Copyright 2011 KUME Intelligent Robotics Lab.                                             
All rights reserved.

$Directed by Jae-Bok Song                                                                     
$Designed by Joong-Tae Park, jtpark1114@gmail.com                                                                  
$Description : 주행시스템의 모든 parameter들을 담당하는 컴포넌트. 싱글톤으로 작성되어 있음.
$Data: 2011/10                                   
______________________________________________________________________________________________*/

#ifndef KUNS_ROBOT_PARAMETER_H
#define KUNS_ROBOT_PARAMETER_H

#include <conio.h>
#include <iostream>
#include "../KUNSUtil/KUNSINIReadWriter/KuINIReadWriter.h"
#include "../KUNSUtil/KuUtil.h"
#include "../KUNSUtil/KUNSSingletone/KuSingletone.h"
#include "KuCommandMessage.h"

using namespace std;
class KuRobotParameter : public KuSingletone <KuRobotParameter>
{

private:
	KuINIReadWriter* m_pINIReaderWriter;

	int m_nUseISSAC; // ISSAC 사용여부
	string m_sISSACServerIP; // ISSAC server ip address
	int m_nISSACServerPort; // ISSAC server port
	int m_nAGVClientPort; // ISSAC client port

	int m_nUseIoT;
	string m_sIoTServerIP;
	int m_nIoTServerPort;

	string m_strMapNameNPath ;
	string m_strVelocityMapNameNPath ;
	string m_strCeilingMapNameNPath ;
	string m_strCadMapNameNPath ;
	string m_strZoneMapNameNPath ;
	string m_strTempMapPath;

	string m_strMovieNameNPath ;


	string m_strTeachingPathNameNPath ;
	string m_strImagePathNameNPath;
	string m_strTeachingWayPointNameNPath;
	string m_strOutlineWayPointNameNPath;
	string m_strPathteaching;

	string m_strGlobalLocalization;
	string m_strVelocityMap;	
	int m_nMapSizeXm ; //지도 x,y 크기 단위: m로 설정.
	int m_nMapSizeYm ;
	double m_dHeight;
	double m_dSDThres;
	int m_nUpSize_X;
	int m_nUpSize_Y;
	int m_nCheckSize_X;
	int m_nCheckSize_Y;

	int m_nRobotID ;
	int m_nRadiusofRobot; //로봇 반경, 장애물 회피등에 사용된다. 단위: mm 
	int m_nWheelBaseofRobot ;
	int m_nMaxRobotVelocity ;//로봇의 최대 속도, 로봇이 최소 속도.
	int m_nMinRobotVelocity ;
	KuPose m_initRobotPosForMap;
	int m_nUsingLastPose;
	KuPose m_initRobotPosForNav;
	int m_nLocalization;
	int m_nObstacleDetectionTime;
	string m_strLastRobotPoseNameNPath;

	string m_strDataPath ;
	string m_strDataRecoding ;
	string m_strDataPlay ;
	double m_dBlockSizeX;
	double m_dBlockSizeY;
	int m_nPathNum;
	int m_nReversePathNum;

	char m_cWheelCom[10];
	char m_cGyroCom[10];
	char m_cHokuyoURG04LXCom[10];
	char m_cSICKLMSFrontLaserIP[30];
	char m_cSICKLMSRearLaserIP[30];
	char m_cCommunicationCom[10];
	string m_sSVLaserIP;
	int m_nSVLaserPort;
	char m_cSwitchCom[10];
	char m_cZigbeeCom[10];
	char m_cGPIOCom[10];
	int m_nVarietyLaser;

	string m_strdoSonar ;
	int m_nLaserTCPPort;

	int m_nKuPRIMUSCommPort;
	int m_nCartConnecContPort;
	double m_dTimeForOn;
	double m_dTimeForOff;
	int m_nDoorOpen1Port;
	int m_nDoorOpen2Port;
	
	// Monitering
	float m_fLowBatteryAlarm;
	
	//INI 파일로부터 kanayama motion controㅣ을 수행하기 위해 필요한 정보를 얻어온다.------------------------
	int m_nDistToTarget;
	int m_nDesiredVel ;
	int m_nGoalArea;
	double m_dKX ;
	double m_dKY ;
	double m_dKT ;
	int m_nDirectionofRotation;
	//========================================================================================


	//INI 파일로부터 Particle filter을 수행하기 위해 필요한 정보를 얻어온다.------------------------------------
	int m_nMaxParticleNum;  //최대, 최소 particle 갯수. 로봇의 위치추정에 사용된다.
	int m_nMinParticleNum ; 
	double m_dDevationforTrans ;
	double m_dDevationforRotate ;
	double m_dDevationforTransRotate ;
	//=========================================================================================

	//INI 파일로부터------------------------------------
	int m_nFeatureTh; // 매칭을 검사하는 최소값 . 높을 수록 부정확해짐. 낮을 수록 저장//80
	int m_nSiftMatchingTh;// 매칭되는 특징점의 최소 개수//5
	double m_dNumSIFTFeatureTh;//특징점의 개수를 변경시켜주는 변수 낮을수록 많이 뽑히고  느려진다
	int m_nNumSURFFeatureTh;//SURF 특징점의 개수를 변경시켜주는 변수 높으수록 많이 뽑히고 느려진다
	int m_nInteractionPointTh;// 전 이미지와 후 이미지 간의 유사도 관계
	double m_dMatchingAngleTh;// 현재 영상과 데이터 영상간의 매칭되는 특징점들의 각도.
	string m_strSIFTDataNamePath;
	string m_strSURFDataNamePath;
	int m_nDistnaceFromPath;
	double m_dEllipseHeight;
	double m_dEllipseWidth;
	//=========================================================================================

	//laser 관련 파라미터----------------------------------------------------------------------------------
	int m_nURG04LX_LaserMaxDist; //상단 레이저의 최대 탐지거리를 저장하는 변수. mm단위
	int m_nURG04LX_LaserMinDist; //상단 레이저의 최소 탐지거리를 저장하는 변수. mm단위
	int m_nURG04LX_LaserHeight; //틸트 축으로 부터 상단 레이저까지의 높이값을 저장하는 변수. mm단위
	int m_nFrontLaserXOffset; //틸트 축으로 부터 상단 레이저까지의 x offset을 저장하는 변수 mm단위
	int m_nFrontLaserYOffset; //틸트 출으로 부터 상단 레이저까지의 y offset을 저장하는 변수 mm단위
	int m_nRearLaserXOffset; //틸트 축으로 부터 상단 레이저까지의 x offset을 저장하는 변수 mm단위
	int m_nRearLaserYOffset; //틸트 출으로 부터 상단 레이저까지의 y offset을 저장하는 변수 mm단위
	//laser 관련 파라미터 선언 끝===========================================================



	//Kinect 관련 파라미터----------------------------------------------------------------------------------
	int m_nKinectMaxDist; //키넥트의 최대 탐지거리를 저장하는 변수. mm단위
	int m_nKinectMinDist; //키넥트의 최소 탐지거리를 저장하는 변수. mm단위
	int m_nKinectHeight; //바닥에서 키넥트까지의 높이값을 저장하는 변수. mm단위
	int m_nKinectXOffset; //로봇의 중심에서 부터 키넥트까지의 x offset을 저장하는 변수 mm단위
	int m_nKinectYOffset; //로봇의 중심에서 부터 키넥트까지의 y offset을 저장하는 변수 mm단위
	int m_nKinectMaxHeightDist;
	int m_nKinectMinHeightDist;
	//Kinect 관련 파라미터 선언 끝===========================================================


	//천장카메라 관련 파라미터------------------------------------------------------------------------------
	double m_dCam_fx;//천장카메라 왜곡보정 계수, fx
	double m_dCam_fy;//천장카메라 왜곡보정 계수, fy
	double m_dCam_cx;//천장카메라 왜곡보정 계수, cx
	double m_dCam_cy;//천장카메라 왜곡보정 계수, cy
	double m_dCam_d1;//천장카메라 왜곡보정 계수, d1
	double m_dCam_d2;//천장카메라 왜곡보정 계수, d2
	double m_dCam_d3;//천장카메라 왜곡보정 계수, d3
	double m_dCam_d4;//천장카메라 왜곡보정 계수, d4
	double m_dCam_offset_x;
	int m_nCamMode;
	int m_nCamExposure;
	bool m_bCamExposure;
	//천장카메라 관련 파리미터 선언 끝========================================================


	//ALRecognizer 관련 파라미터------------------------------------------------------------------
	string m_strAlFeatureMapNameNPath ;
	int m_nLandMarkNum; //랜드마크의 개수
	int m_nHeight_Camera2Mark; //카메라로부터 랜드마크까지의 높이
	double m_dRecognizingDistThres;
	//ALRecognizer 관련 파라미터 선언 끝===========================================================



public:
	bool initialize(); //시스템 설정에 관련된 모든 변수들을 초기화 시켜주는 함수.
	void saveParameter();

	//set 함수목록-----------------------------------------------------------------------------------------------------------------------
	// ISSAC 관련 파라미터
	void setUsingISSAC(int nUse);
	void setISSACServerIP(string strServerIP);
	void setISSACServerPort(int nServerPort);
	void setISSACClientPort(int nClientPort);

	void setUsingIoT(int nUse);
	void setIoTServerIP(string strServerIP);
	void setIoTServerPort(int nServerPort);

	//robot 관련 파라미터
	void setRobotID(int nID); //로봇의 ID를 설정해주는 함수.
	void setRobotRadius(int nRadius); //로봇 반경을 설정해주는 함수. 단위는 mm
	void setMaxRobotVelocity(int nMaxVelocity); //로봇의 최대속도를 설정하는 함수.
	void setMinRobotVelocity(int nMinVelocity); //로봇의 최소속도를 설정하는 함수.
	void setGlobalLocalization(string sGlobalLocalization);
	void setInitRobotPoseForMap(KuPose initRobotPos);
	void setUsingLastPose(int nUsingLastPose);
	void setInitRobotPoseForNav(KuPose initRobotPos);
	void setWheelBaseofRobot(int nWheelBaseofRobot);
	void setLocalization(int nLocalization);
	void setLastRobotPoseNameNPath(string strLastRobotPoseNameNPath);
	void setObstacleDetectionTime(int nObstacleDetectionTime);

	//Map 관련 파라미터
	void setMapSize(int nSizeXm, int nSizeYm); //지도크기를 설정하는 함수.
	void setMapNameNPath(string sMapName) ;//작성할 지도의 이름.
	void setVelocityMapNameNPath(string sMapName); 
	void setCeilingMapNameNPath(string sMapName) ;
	void setCadMapNameNPath(string sMapName) ;
	void setZoneMapNameNPath(string sMapName);
	void setTempMapPath(string sMapPath);

	void setHeight(double dHeight );
	void setSDValue(double dSDValue);
	void setUpdateValX(int nUpdateHalfSizeX);
	void setUpdateValY(int nUpdateHalfSizeY);
	void setCheckValX(int nCheckValX);
	void setCheckValY(int nCheckValY);
	
	void setMovieNameNPath(string sMapName);


	void setTeachingPathNameNPath(string sMapName);
	void setImagePathNameNPath(string sMapName) ;
	void setTeachingWayPointNameNPath(string sMapName); 
	void setOutlineWayPointNameNPath(string sMapName);
	void setBlockSizeX(double dBlockSizeX);
	void setBlockSizeY(double dBlockSizeY);
	void setPathNum(int nPathNum);
	void setReversePathNum(int nReversePathNum);
	void setPathteaching(string strPathteaching); 


	//Sensor 관련 파라미터
	void setDataPath(string sDataPath) ;
	void setDataRecoding(string sDataRecoding) ;
	void setDataPlay(string sDataPlay) ;


	void setURG04LXLaserComport(char cComport[10]); //URG04LX laser comport를 설정하는 함수.
	void setWheelComport(char cComport[10]); //wheel comport를 설정하는 함수.
	void setKuPrimusCommPort(int nPort);
	void setCartConnecContPort(int nPort);
	void setCartPortTimeOn(double dTime);
	void setCartPortTimeOff(double dTime);
	void setDoorOpen1Port(int dPort);
	void setDoorOpen2Port(int dPort);
	void setGyroComport(char cComport[10]); //wheel comport를 설정하는 함수.
	void setCommunicationComport(char cComport[10]);
	void setSwitchComport(char cComport[10]);
	void setZigbeeComport(char cComport[10]); // Zigbee com port를 설정하는 함수.
	void setGPIOComport(char cComport[10]); // GPIO com port를 설정하는 함수.
	void setdoSonar(string strdoSonar);
	void setSICKLMSLaserComport(char cComport[30]);
	void setLaserTCPPort(int nLaserTCPPort );
	void setVarietyLaser(int nVarietyLaser );


	//laser 관련 파라미터
	void setURG04LXLaserMaxDist(int nMaxDist); // 레이저의 최대 탐지거리를 설정하는 함수.
	void setURG04LXLaserMinDist(int nMinDist); // 레이저의 최소 탐지거리를 설정하는 함수.
	void setURG04LXLaserHeight(int nHeight); // 바닥에서 레이저까지의 높이값을 설정하는 함수.
	void setFrontLaserXOffset(int nXOffset); // 로봇의 중심에서 레이저까지의 x offset을 설정하는 함수.
	void setFrontLaserYOffset(int nYOffset); //로봇의 중심에서 틸트 축으로 부터 상단 레이저까지의 y offset을 설정하는 함수.
	void setRearLaserXOffset(int nXOffset); // 로봇의 중심에서 레이저까지의 x offset을 설정하는 함수.
	void setRearLaserYOffset(int nYOffset); //로봇의 중심에서 틸트 축으로 부터 상단 레이저까지의 y offset을 설정하는 함수.

	//Kinect 관련 파라미터
	void setKinectMaxDist(int nMaxDist); //키넥트의 최대 탐지거리를 설정하는 함수.
	void setKinectMinDist(int nMinDist); //키넥트의 최소 탐지거리를 설정하는 함수.
	void setKinectHeight(int nHeight); //바닥에서 키넥트까지의 높이값을 설정하는 함수.
	void setKinectXOffset(int nXOffset); //로봇의 중심에서 키넥트까지의 x offset을 설정하는 함수.
	void setKinectYOffset(int nYOffset); //로봇의 중심에서 키넥트까지의 y offset을 설정하는 함수.	
	void setKinectMaxHeightDist(int nMaxDist); //키넥트의 최대 높이거리를 설정하는 함수.
	void setKinectMinHeightDist(int nMinDist); //키넥트의 최소 높이거리를 설정하는 함수.

	//천장카메라 관련 파라미터
	void setCeilingCameraPrameterFx(double dFx);
	void setCeilingCameraPrameterFy(double dFy);
	void setCeilingCameraPrameterCx(double dCx);
	void setCeilingCameraPrameterCy(double dCy);
	void setCeilingCameraPrameterD1(double dD1);
	void setCeilingCameraPrameterD2(double dD2);
	void setCeilingCameraPrameterD3(double dD3);
	void setCeilingCameraPrameterD4(double dD4);
	void setCeilingCameraPrameterOffsetX(double doffsetx);
	void setCeilingCameraPrameterMode(int nMode);
	void setCeilingCameraExposure(int nExposure);
	void setCeilingCameraAutoExposure(bool bAuto);

	//주행 성능과 연관된 parameter 설정 함수들-------------------------------------------
	void setTargetDistance(int nDistToTarget);
	void setDesiedVel(int nDesiredVel);
	void setGoalArea(int nGoalArea);
	void setdKX(double  dKX);
	void setdKY(double  dKY);
	void setdKT(double  dKT);
	void setDirectionofRotation(int nDirectionofRotation);

	void setMaxParticleNum(int nMaxNum); //최대 particle 갯수를 설정하는 함수.
	void setMinParticleNUm(int nMinNUm); //최소 particle 갯수를 설정하는 함수.
	void setDeviationforTrans(double  dDevationforTrans);
	void setDeviationforRotae(double  dDevationforRotate);
	void setDeviationforTransRotae(double  dDevationforTransRotate);
	void setFeatureTh(int nFeatureTh);
	void setMatchingTh(int nMatchingTh);
	void setNumSIFTFeatureTh(double dNuMFeatureTh);
	void setInteractionPointTh(int nInteractionPointTh);
	void setMatchingAngleTh(double dMatchingAngleTh);
	
	void setSIFTDataNamePath(string strSIFTDataNamePath) ;
	void setSURFDataNamePath(string strSURFDataNamePath) ;
	void setDistanceFromPath(int nDistnaceFromPath);
	void setNumSURFFeatureTh(int nSURFKeypointTh);
	
	void setEllipseWidth(double dEllipseWidth);
	void setEllipseHeight(double dEllipseHeight);

	
	//ALRecognizer 관련 파라미터
	void setAlFeatureMapNameNPath(string sAlFeatureMapNameNPath) ;
	void setLandMarkNum(int nLandMarkNum);
	void setHeight_Camera2Mark(int nHeight_Camera2Mark);
	void setRecognizingDistTh(double dRecognizingDistTh);

	// Monitoring
	void setLowBatteryAlarmVoltage(float fVoltage);

	//=====================================================================================================================================
	//=====================================================================================================================================


	//get 함수목록--------------------------------------------------------------------------------------------------------------------------
	// ISSAC 관련 파라미터
	int getUsingISSAC(void);
	string getISSACServerIP(void);
	int getISSACServerPort(void);
	int getISSACClientPort(void);

	// IoT
	int getUsingIoT(void);
	string getIoTServerIP(void);
	int getIoTServerPort(void);

	//robot 관련 파라미터
	int getRobotID(); //로봇의 ID를 설정해주는 함수.
	int getRobotRadius(); //로봇 반경을 설정해주는 함수. 단위는 mm
	int getMaxRobotVelocity(); //로봇의 최대속도를 설정하는 함수.
	int getMinRobotVelocity(); //로봇의 최소속도를 설정하는 함수.
	string getGlobalLocalization( );
	KuPose getInitRobotPoseForMap();
	int getUsingLastPose();
	KuPose getInitRobotPoseForNav();
	int getWheelBaseofRobot();
	int getLocalization();
	string getLastRobotPoseNameNPath( );
	int getObstacleDetectionTime();

	//Map 관련 파라미터
	int getMapSizeXm(); //지도크기를 설정하는 함수.
	int getMapSizeYm(); //지도크기를 설정하는 함수.
	string getMapNameNPath( ); //장성할 지도의 이름을 넘겨준다.
	string getVelocityMapNameNPath(	);
	string getCeilingMapNameNPath(	);
	string getCadMapNameNPath( ); 
	string getZoneMapNameNPath( ); 
	string getTempMapPath(	);

	string getTeachingPathNameNPath() ;
	string getImagePathNameNPath() ;
	string getTeachingWayPointNameNPath(); 
	string getOutlineWayPointNameNPath(	);
	double getBlockSizeX();
	double getBlockSizeY();
	int getPathNum();
	int getReversePathNum();
	string getPathteaching(); 

	double getHeight();
	double getSDValue();
	int getUpdateValX();
	int getUpdateValY();
	int getCheckValX();
	int getCheckValY();

	string getMovieNameNPath();

	//Sensor 관련 파라미터
	string getDataPath( ) ;
	string getDataRecoding( ) ;
	string getDataPlay( ) ;

	void getURG04LXLaserComport(char wcComport[10]); // laser comport를 넘겨준다. 
	void getGyroComport(char wcComport[10]); //gyro comport를 넘겨준다. 
	void getWheelComport(char cComport[10]); //wheel comport를 넘겨준다. 
	int getKuPrimusCommPort();
	int getCartConnecContPort();
	double getCartPortTimeOn();
	double getCartPortTimeOFF();
	int getDoorOpen1Port();
	int getDoorOpen2Port();
	void getCommunicationComport(char cComport[10]);
	void getSwitchComport(char cComport[10]);
	void getZigbeeComport(char cComport[10]); // Zigbee com port를 넘겨준다.
	void getGPIOComport(char cComport[10]); // GPIO com port를 넘겨준다.
	string getdoSonar( );
	int getLaserType();
	void getSICKLMSFrontLaserIPAddress(char cComport[30]);
	void getSICKLMSRearLaserIPAddress(char cComport[30]);
	int getLaserTCPPort();


	//laser 관련 파라미터
	int getURG04LXLaserMaxDist(); //상단 레이저의 최대 탐지거리를 설정하는 함수.
	int getURG04LXLaserMinDist(); //상단 레이저의 최소 탐지거리를 설정하는 함수.
	int getURG04LXLaserHeight(); //틸트 축으로 부터 상단 레이저까지의 높이값을 설정하는 함수.
	int getFrontLaserXOffset(); //틸트 축으로 부터 상단 레이저까지의 x offset을 설정하는 함수.
	int getRearLaserXOffset();
	int getFrontLaserYOffset(); //틸트 축으로 부터 상단 레이저까지의 y offset을 설정하는 함수.	
	int getRearLaserYOffset();

	//Kinect 관련 파라미터
	int getKinectMaxDist(); //키넥트의 최대 탐지거리를 설정하는 함수.
	int getKinectMinDist(); //키넥트의 최소 탐지거리를 설정하는 함수.
	int getKinectHeight(); //바닥에서 키넥트까지의 높이값을 설정하는 함수.
	int getKinectXOffset(); //로봇의 중심에서 키넥트까지의 x offset을 설정하는 함수.
	int getKinectYOffset(); //로봇의 중심에서 키넥트까지의 y offset을 설정하는 함수.	
	int getKinectMaxHeightDist();
	int getKinectMinHeightDist( );

	//천장카메라 관련 파라미터
	double getCeilingCameraPrameterFx();
	double getCeilingCameraPrameterFy();
	double getCeilingCameraPrameterCx();
	double getCeilingCameraPrameterCy();
	double getCeilingCameraPrameterD1();
	double getCeilingCameraPrameterD2();
	double getCeilingCameraPrameterD3();
	double getCeilingCameraPrameterD4();
	double getCeilingCameraPrameterOffsetX();
	int getCeilingCameraPrameterMode();
	int getCeilingCameraExposure();
	bool getCeilingCameraAutoExposure();

	//주행 성능과 연관된 parameter 설정 함수들-------------------------------------------

	int getTargetDistance();
	int getDesiedVel( );
	int getGoalArea( );
	double getdKX(  );
	double getdKY(  );
	double getdKT(  );
	int getDirectionofRotation();

	int getMaxParticleNum(); //최대 particle 갯수를 설정하는 함수.
	int getMinParticleNUm(); //최소 particle 갯수를 설정하는 함수.
	double getDeviationforTrans(  );
	double getDeviationforRotate(  );
	double getDeviationforTransRotate(  );
	int getFeatureTh( );
	int getMatchingTh( );
	double getNumSIFTFeatureTh( );
	int getNumSURFFeatureTh();
	int getInteractionPointTh( );
	double getMatchingAngleTh( );	
	string getSIFTDataNamePath();
	string getSURFDataNamePath();
	int getDistanceFromPath();

	double getEllipseWidth( );
	double getEllipseHeight( );

		//ALRecognizer 관련 파라미터 함수들
	int getLandMarkNum();
	int getHeightCamera2Mark();
	string getAlFeatureMapNameNPath( ) ;
	double getRecognizingDistTh();

	// Monitoring
	float getLowBatteryAlarmVoltage(void);

	// SV laser
	std::string getSVLaserIP(void);
	int getSVLaserPort(void);
	//======================================================================================================================================
	//======================================================================================================================================


public:
	KuRobotParameter();
	virtual ~KuRobotParameter();
};

#endif

