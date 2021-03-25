/*______________________________________________________________________________________________
PROJECT : Intelligent Robotics Lab. Department of Mechanical Engineering Korea University                                    
(c) Copyright 2011 KUME Intelligent Robotics Lab.                                             
All rights reserved.

$Directed by Jae-Bok Song                                                                     
$Designed by Joong-Tae Park, jtpark1114@gmail.com                                                                  
$Description : ����ý����� ��� parameter���� ����ϴ� ������Ʈ. �̱������� �ۼ��Ǿ� ����.
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

	int m_nUseISSAC; // ISSAC ��뿩��
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
	int m_nMapSizeXm ; //���� x,y ũ�� ����: m�� ����.
	int m_nMapSizeYm ;
	double m_dHeight;
	double m_dSDThres;
	int m_nUpSize_X;
	int m_nUpSize_Y;
	int m_nCheckSize_X;
	int m_nCheckSize_Y;

	int m_nRobotID ;
	int m_nRadiusofRobot; //�κ� �ݰ�, ��ֹ� ȸ�ǵ ���ȴ�. ����: mm 
	int m_nWheelBaseofRobot ;
	int m_nMaxRobotVelocity ;//�κ��� �ִ� �ӵ�, �κ��� �ּ� �ӵ�.
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
	
	//INI ���Ϸκ��� kanayama motion contro���� �����ϱ� ���� �ʿ��� ������ ���´�.------------------------
	int m_nDistToTarget;
	int m_nDesiredVel ;
	int m_nGoalArea;
	double m_dKX ;
	double m_dKY ;
	double m_dKT ;
	int m_nDirectionofRotation;
	//========================================================================================


	//INI ���Ϸκ��� Particle filter�� �����ϱ� ���� �ʿ��� ������ ���´�.------------------------------------
	int m_nMaxParticleNum;  //�ִ�, �ּ� particle ����. �κ��� ��ġ������ ���ȴ�.
	int m_nMinParticleNum ; 
	double m_dDevationforTrans ;
	double m_dDevationforRotate ;
	double m_dDevationforTransRotate ;
	//=========================================================================================

	//INI ���Ϸκ���------------------------------------
	int m_nFeatureTh; // ��Ī�� �˻��ϴ� �ּҰ� . ���� ���� ����Ȯ����. ���� ���� ����//80
	int m_nSiftMatchingTh;// ��Ī�Ǵ� Ư¡���� �ּ� ����//5
	double m_dNumSIFTFeatureTh;//Ư¡���� ������ ��������ִ� ���� �������� ���� ������  ��������
	int m_nNumSURFFeatureTh;//SURF Ư¡���� ������ ��������ִ� ���� �������� ���� ������ ��������
	int m_nInteractionPointTh;// �� �̹����� �� �̹��� ���� ���絵 ����
	double m_dMatchingAngleTh;// ���� ����� ������ ������ ��Ī�Ǵ� Ư¡������ ����.
	string m_strSIFTDataNamePath;
	string m_strSURFDataNamePath;
	int m_nDistnaceFromPath;
	double m_dEllipseHeight;
	double m_dEllipseWidth;
	//=========================================================================================

	//laser ���� �Ķ����----------------------------------------------------------------------------------
	int m_nURG04LX_LaserMaxDist; //��� �������� �ִ� Ž���Ÿ��� �����ϴ� ����. mm����
	int m_nURG04LX_LaserMinDist; //��� �������� �ּ� Ž���Ÿ��� �����ϴ� ����. mm����
	int m_nURG04LX_LaserHeight; //ƿƮ ������ ���� ��� ������������ ���̰��� �����ϴ� ����. mm����
	int m_nFrontLaserXOffset; //ƿƮ ������ ���� ��� ������������ x offset�� �����ϴ� ���� mm����
	int m_nFrontLaserYOffset; //ƿƮ ������ ���� ��� ������������ y offset�� �����ϴ� ���� mm����
	int m_nRearLaserXOffset; //ƿƮ ������ ���� ��� ������������ x offset�� �����ϴ� ���� mm����
	int m_nRearLaserYOffset; //ƿƮ ������ ���� ��� ������������ y offset�� �����ϴ� ���� mm����
	//laser ���� �Ķ���� ���� ��===========================================================



	//Kinect ���� �Ķ����----------------------------------------------------------------------------------
	int m_nKinectMaxDist; //Ű��Ʈ�� �ִ� Ž���Ÿ��� �����ϴ� ����. mm����
	int m_nKinectMinDist; //Ű��Ʈ�� �ּ� Ž���Ÿ��� �����ϴ� ����. mm����
	int m_nKinectHeight; //�ٴڿ��� Ű��Ʈ������ ���̰��� �����ϴ� ����. mm����
	int m_nKinectXOffset; //�κ��� �߽ɿ��� ���� Ű��Ʈ������ x offset�� �����ϴ� ���� mm����
	int m_nKinectYOffset; //�κ��� �߽ɿ��� ���� Ű��Ʈ������ y offset�� �����ϴ� ���� mm����
	int m_nKinectMaxHeightDist;
	int m_nKinectMinHeightDist;
	//Kinect ���� �Ķ���� ���� ��===========================================================


	//õ��ī�޶� ���� �Ķ����------------------------------------------------------------------------------
	double m_dCam_fx;//õ��ī�޶� �ְ�� ���, fx
	double m_dCam_fy;//õ��ī�޶� �ְ�� ���, fy
	double m_dCam_cx;//õ��ī�޶� �ְ�� ���, cx
	double m_dCam_cy;//õ��ī�޶� �ְ�� ���, cy
	double m_dCam_d1;//õ��ī�޶� �ְ�� ���, d1
	double m_dCam_d2;//õ��ī�޶� �ְ�� ���, d2
	double m_dCam_d3;//õ��ī�޶� �ְ�� ���, d3
	double m_dCam_d4;//õ��ī�޶� �ְ�� ���, d4
	double m_dCam_offset_x;
	int m_nCamMode;
	int m_nCamExposure;
	bool m_bCamExposure;
	//õ��ī�޶� ���� �ĸ����� ���� ��========================================================


	//ALRecognizer ���� �Ķ����------------------------------------------------------------------
	string m_strAlFeatureMapNameNPath ;
	int m_nLandMarkNum; //���帶ũ�� ����
	int m_nHeight_Camera2Mark; //ī�޶�κ��� ���帶ũ������ ����
	double m_dRecognizingDistThres;
	//ALRecognizer ���� �Ķ���� ���� ��===========================================================



public:
	bool initialize(); //�ý��� ������ ���õ� ��� �������� �ʱ�ȭ �����ִ� �Լ�.
	void saveParameter();

	//set �Լ����-----------------------------------------------------------------------------------------------------------------------
	// ISSAC ���� �Ķ����
	void setUsingISSAC(int nUse);
	void setISSACServerIP(string strServerIP);
	void setISSACServerPort(int nServerPort);
	void setISSACClientPort(int nClientPort);

	void setUsingIoT(int nUse);
	void setIoTServerIP(string strServerIP);
	void setIoTServerPort(int nServerPort);

	//robot ���� �Ķ����
	void setRobotID(int nID); //�κ��� ID�� �������ִ� �Լ�.
	void setRobotRadius(int nRadius); //�κ� �ݰ��� �������ִ� �Լ�. ������ mm
	void setMaxRobotVelocity(int nMaxVelocity); //�κ��� �ִ�ӵ��� �����ϴ� �Լ�.
	void setMinRobotVelocity(int nMinVelocity); //�κ��� �ּҼӵ��� �����ϴ� �Լ�.
	void setGlobalLocalization(string sGlobalLocalization);
	void setInitRobotPoseForMap(KuPose initRobotPos);
	void setUsingLastPose(int nUsingLastPose);
	void setInitRobotPoseForNav(KuPose initRobotPos);
	void setWheelBaseofRobot(int nWheelBaseofRobot);
	void setLocalization(int nLocalization);
	void setLastRobotPoseNameNPath(string strLastRobotPoseNameNPath);
	void setObstacleDetectionTime(int nObstacleDetectionTime);

	//Map ���� �Ķ����
	void setMapSize(int nSizeXm, int nSizeYm); //����ũ�⸦ �����ϴ� �Լ�.
	void setMapNameNPath(string sMapName) ;//�ۼ��� ������ �̸�.
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


	//Sensor ���� �Ķ����
	void setDataPath(string sDataPath) ;
	void setDataRecoding(string sDataRecoding) ;
	void setDataPlay(string sDataPlay) ;


	void setURG04LXLaserComport(char cComport[10]); //URG04LX laser comport�� �����ϴ� �Լ�.
	void setWheelComport(char cComport[10]); //wheel comport�� �����ϴ� �Լ�.
	void setKuPrimusCommPort(int nPort);
	void setCartConnecContPort(int nPort);
	void setCartPortTimeOn(double dTime);
	void setCartPortTimeOff(double dTime);
	void setDoorOpen1Port(int dPort);
	void setDoorOpen2Port(int dPort);
	void setGyroComport(char cComport[10]); //wheel comport�� �����ϴ� �Լ�.
	void setCommunicationComport(char cComport[10]);
	void setSwitchComport(char cComport[10]);
	void setZigbeeComport(char cComport[10]); // Zigbee com port�� �����ϴ� �Լ�.
	void setGPIOComport(char cComport[10]); // GPIO com port�� �����ϴ� �Լ�.
	void setdoSonar(string strdoSonar);
	void setSICKLMSLaserComport(char cComport[30]);
	void setLaserTCPPort(int nLaserTCPPort );
	void setVarietyLaser(int nVarietyLaser );


	//laser ���� �Ķ����
	void setURG04LXLaserMaxDist(int nMaxDist); // �������� �ִ� Ž���Ÿ��� �����ϴ� �Լ�.
	void setURG04LXLaserMinDist(int nMinDist); // �������� �ּ� Ž���Ÿ��� �����ϴ� �Լ�.
	void setURG04LXLaserHeight(int nHeight); // �ٴڿ��� ������������ ���̰��� �����ϴ� �Լ�.
	void setFrontLaserXOffset(int nXOffset); // �κ��� �߽ɿ��� ������������ x offset�� �����ϴ� �Լ�.
	void setFrontLaserYOffset(int nYOffset); //�κ��� �߽ɿ��� ƿƮ ������ ���� ��� ������������ y offset�� �����ϴ� �Լ�.
	void setRearLaserXOffset(int nXOffset); // �κ��� �߽ɿ��� ������������ x offset�� �����ϴ� �Լ�.
	void setRearLaserYOffset(int nYOffset); //�κ��� �߽ɿ��� ƿƮ ������ ���� ��� ������������ y offset�� �����ϴ� �Լ�.

	//Kinect ���� �Ķ����
	void setKinectMaxDist(int nMaxDist); //Ű��Ʈ�� �ִ� Ž���Ÿ��� �����ϴ� �Լ�.
	void setKinectMinDist(int nMinDist); //Ű��Ʈ�� �ּ� Ž���Ÿ��� �����ϴ� �Լ�.
	void setKinectHeight(int nHeight); //�ٴڿ��� Ű��Ʈ������ ���̰��� �����ϴ� �Լ�.
	void setKinectXOffset(int nXOffset); //�κ��� �߽ɿ��� Ű��Ʈ������ x offset�� �����ϴ� �Լ�.
	void setKinectYOffset(int nYOffset); //�κ��� �߽ɿ��� Ű��Ʈ������ y offset�� �����ϴ� �Լ�.	
	void setKinectMaxHeightDist(int nMaxDist); //Ű��Ʈ�� �ִ� ���̰Ÿ��� �����ϴ� �Լ�.
	void setKinectMinHeightDist(int nMinDist); //Ű��Ʈ�� �ּ� ���̰Ÿ��� �����ϴ� �Լ�.

	//õ��ī�޶� ���� �Ķ����
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

	//���� ���ɰ� ������ parameter ���� �Լ���-------------------------------------------
	void setTargetDistance(int nDistToTarget);
	void setDesiedVel(int nDesiredVel);
	void setGoalArea(int nGoalArea);
	void setdKX(double  dKX);
	void setdKY(double  dKY);
	void setdKT(double  dKT);
	void setDirectionofRotation(int nDirectionofRotation);

	void setMaxParticleNum(int nMaxNum); //�ִ� particle ������ �����ϴ� �Լ�.
	void setMinParticleNUm(int nMinNUm); //�ּ� particle ������ �����ϴ� �Լ�.
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

	
	//ALRecognizer ���� �Ķ����
	void setAlFeatureMapNameNPath(string sAlFeatureMapNameNPath) ;
	void setLandMarkNum(int nLandMarkNum);
	void setHeight_Camera2Mark(int nHeight_Camera2Mark);
	void setRecognizingDistTh(double dRecognizingDistTh);

	// Monitoring
	void setLowBatteryAlarmVoltage(float fVoltage);

	//=====================================================================================================================================
	//=====================================================================================================================================


	//get �Լ����--------------------------------------------------------------------------------------------------------------------------
	// ISSAC ���� �Ķ����
	int getUsingISSAC(void);
	string getISSACServerIP(void);
	int getISSACServerPort(void);
	int getISSACClientPort(void);

	// IoT
	int getUsingIoT(void);
	string getIoTServerIP(void);
	int getIoTServerPort(void);

	//robot ���� �Ķ����
	int getRobotID(); //�κ��� ID�� �������ִ� �Լ�.
	int getRobotRadius(); //�κ� �ݰ��� �������ִ� �Լ�. ������ mm
	int getMaxRobotVelocity(); //�κ��� �ִ�ӵ��� �����ϴ� �Լ�.
	int getMinRobotVelocity(); //�κ��� �ּҼӵ��� �����ϴ� �Լ�.
	string getGlobalLocalization( );
	KuPose getInitRobotPoseForMap();
	int getUsingLastPose();
	KuPose getInitRobotPoseForNav();
	int getWheelBaseofRobot();
	int getLocalization();
	string getLastRobotPoseNameNPath( );
	int getObstacleDetectionTime();

	//Map ���� �Ķ����
	int getMapSizeXm(); //����ũ�⸦ �����ϴ� �Լ�.
	int getMapSizeYm(); //����ũ�⸦ �����ϴ� �Լ�.
	string getMapNameNPath( ); //�强�� ������ �̸��� �Ѱ��ش�.
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

	//Sensor ���� �Ķ����
	string getDataPath( ) ;
	string getDataRecoding( ) ;
	string getDataPlay( ) ;

	void getURG04LXLaserComport(char wcComport[10]); // laser comport�� �Ѱ��ش�. 
	void getGyroComport(char wcComport[10]); //gyro comport�� �Ѱ��ش�. 
	void getWheelComport(char cComport[10]); //wheel comport�� �Ѱ��ش�. 
	int getKuPrimusCommPort();
	int getCartConnecContPort();
	double getCartPortTimeOn();
	double getCartPortTimeOFF();
	int getDoorOpen1Port();
	int getDoorOpen2Port();
	void getCommunicationComport(char cComport[10]);
	void getSwitchComport(char cComport[10]);
	void getZigbeeComport(char cComport[10]); // Zigbee com port�� �Ѱ��ش�.
	void getGPIOComport(char cComport[10]); // GPIO com port�� �Ѱ��ش�.
	string getdoSonar( );
	int getLaserType();
	void getSICKLMSFrontLaserIPAddress(char cComport[30]);
	void getSICKLMSRearLaserIPAddress(char cComport[30]);
	int getLaserTCPPort();


	//laser ���� �Ķ����
	int getURG04LXLaserMaxDist(); //��� �������� �ִ� Ž���Ÿ��� �����ϴ� �Լ�.
	int getURG04LXLaserMinDist(); //��� �������� �ּ� Ž���Ÿ��� �����ϴ� �Լ�.
	int getURG04LXLaserHeight(); //ƿƮ ������ ���� ��� ������������ ���̰��� �����ϴ� �Լ�.
	int getFrontLaserXOffset(); //ƿƮ ������ ���� ��� ������������ x offset�� �����ϴ� �Լ�.
	int getRearLaserXOffset();
	int getFrontLaserYOffset(); //ƿƮ ������ ���� ��� ������������ y offset�� �����ϴ� �Լ�.	
	int getRearLaserYOffset();

	//Kinect ���� �Ķ����
	int getKinectMaxDist(); //Ű��Ʈ�� �ִ� Ž���Ÿ��� �����ϴ� �Լ�.
	int getKinectMinDist(); //Ű��Ʈ�� �ּ� Ž���Ÿ��� �����ϴ� �Լ�.
	int getKinectHeight(); //�ٴڿ��� Ű��Ʈ������ ���̰��� �����ϴ� �Լ�.
	int getKinectXOffset(); //�κ��� �߽ɿ��� Ű��Ʈ������ x offset�� �����ϴ� �Լ�.
	int getKinectYOffset(); //�κ��� �߽ɿ��� Ű��Ʈ������ y offset�� �����ϴ� �Լ�.	
	int getKinectMaxHeightDist();
	int getKinectMinHeightDist( );

	//õ��ī�޶� ���� �Ķ����
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

	//���� ���ɰ� ������ parameter ���� �Լ���-------------------------------------------

	int getTargetDistance();
	int getDesiedVel( );
	int getGoalArea( );
	double getdKX(  );
	double getdKY(  );
	double getdKT(  );
	int getDirectionofRotation();

	int getMaxParticleNum(); //�ִ� particle ������ �����ϴ� �Լ�.
	int getMinParticleNUm(); //�ּ� particle ������ �����ϴ� �Լ�.
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

		//ALRecognizer ���� �Ķ���� �Լ���
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

