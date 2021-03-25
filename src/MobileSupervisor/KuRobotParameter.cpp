#include "stdafx.h"
#include "KuRobotParameter.h"
#include "../MultiRobotSupervisor/XmldataSetting.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#endif

KuRobotParameter::KuRobotParameter()
{

}

KuRobotParameter::~KuRobotParameter()
{

}

bool  KuRobotParameter::initialize()
{	
	printf("Loading parameters from KUNS.ini file...\n");

	string strInIFileName = "./ini/KUNS.ini";  	//�⺻���� �̸��� kuns.ini���Ͽ� ��õǾ� �ִ�. 
	m_pINIReaderWriter = new KuINIReadWriter(strInIFileName); //�������� �б�

	if (m_pINIReaderWriter->ParseError() < 0) { //������ ���� ���� ��� ���α׷��� �����Ų��.
		cout << "Can't load "<<strInIFileName<<endl;;
		_getch();
		return false;
	}

	const char* constch;

	// ISSAC
	m_nUseISSAC = m_pINIReaderWriter->getIntValue("ISSAC", "USE_ISSAC", 0); // ISSAC ��뿩��
	m_sISSACServerIP = m_pINIReaderWriter->getStringValue("ISSAC", "ISSAC_SERVER_IP", "127.0.0.1"); // ISSAC server IP (������ ���� ��)
	m_nISSACServerPort = m_pINIReaderWriter->getIntValue("ISSAC", "ISSAC_SERVER_PORT", 1234); // ISSAC server port (������ ���� ��)
	m_nAGVClientPort = m_pINIReaderWriter->getIntValue("ISSAC", "AGV_CLIENT_PORT", 1234); // ISSAC client port (������ ���� ��)

	// IoT
	m_nUseIoT = m_pINIReaderWriter->getIntValue("IOT", "USE_IOT", 0);
	m_sIoTServerIP = m_pINIReaderWriter->getStringValue("IOT", "IOT_SERVER_IP", "127.0.0.1"); // ISSAC server IP (������ ���� ��)
	m_nIoTServerPort = m_pINIReaderWriter->getIntValue("IOT", "IOT_SERVER_PORT", 1234); // ISSAC server port (������ ���� ��)

	//
	m_strMapNameNPath = m_pINIReaderWriter->getStringValue("MAP", "PATH&NAME", "no");
	m_strVelocityMapNameNPath = m_pINIReaderWriter->getStringValue("MAP", "VELOCITY_PATH&NAME", "no");
	m_strCeilingMapNameNPath = m_pINIReaderWriter->getStringValue("MAP", "CEILINGMAP_PATH&NAME", "no");
	m_strCadMapNameNPath = m_pINIReaderWriter->getStringValue("MAP", "CADMAP_PATH&NAME", "no");
	m_strZoneMapNameNPath = m_pINIReaderWriter->getStringValue("MAP", "ZONEMAP_PATH&NAME", "no");
	m_strTempMapPath = m_pINIReaderWriter->getStringValue("MAP", "TEMPMAP_PATH&NAME", "no");

	m_nMapSizeXm =	m_pINIReaderWriter->getIntValue("MAP","MAP_SIZE_X",0);
	m_nMapSizeYm =	m_pINIReaderWriter->getIntValue("MAP","MAP_SIZE_Y",0);
	m_dHeight = m_pINIReaderWriter->getDoubleValue("MAP","HEIGHT",1.75);
	m_dSDThres = m_pINIReaderWriter->getDoubleValue("MAP","STANDARD_DEVIATION_TH",7.0);
	m_nUpSize_X = m_pINIReaderWriter->getIntValue("MAP","UPDATAESIZE_X",160);
	m_nUpSize_Y = m_pINIReaderWriter->getIntValue("MAP","UPDATAESIZE_Y",120);
	m_nCheckSize_X = m_pINIReaderWriter->getIntValue("MAP","CHECKSIZE_X",60);
	m_nCheckSize_Y = m_pINIReaderWriter->getIntValue("MAP","CHECKSIZE_Y",40);

	m_strMovieNameNPath = m_pINIReaderWriter->getStringValue("MOVIE", "MOVIE&PATH&NAME", "no");


	m_strTeachingPathNameNPath = m_pINIReaderWriter->getStringValue("PATH", "TEACHINGPATH_PATH&NAME", "no");
	m_strImagePathNameNPath = m_pINIReaderWriter->getStringValue("PATH", "IMAGEPATH_PATH&NAME", "no");
	m_strTeachingWayPointNameNPath = m_pINIReaderWriter->getStringValue("PATH", "TEACHINGWAYPOINT_PATH&NAME", "no");
	m_strOutlineWayPointNameNPath = m_pINIReaderWriter->getStringValue("PATH", "OUTLINEWAYPOINT_PATH&NAME", "no");
	m_dBlockSizeX = m_pINIReaderWriter->getDoubleValue("PATH","BLOCKSIZE_X",1.0);
	m_dBlockSizeY = m_pINIReaderWriter->getDoubleValue("PATH","BLOCKSIZE_Y",1.0);
	m_nPathNum = m_pINIReaderWriter->getIntValue("PATH","PATH_NUM",1);
	m_nReversePathNum = m_pINIReaderWriter->getIntValue("PATH","REVERSE_PATH_NUM",10);
	m_strPathteaching = m_pINIReaderWriter->getStringValue("PATH", "PATHTEACHING", "no");

	m_nRobotID = m_pINIReaderWriter->getIntValue("ROBOT","ROBOT_ID",0);
	m_nRadiusofRobot = m_pINIReaderWriter->getIntValue("ROBOT","ROBOT_RADIUS",0);
	m_nWheelBaseofRobot = m_pINIReaderWriter->getIntValue("ROBOT","ROBOT_WHEEL_BASE",0);
	m_nMaxRobotVelocity = m_pINIReaderWriter->getIntValue("ROBOT","MAX_VELOCITY",0);
	m_nMinRobotVelocity = m_pINIReaderWriter->getIntValue("ROBOT","MIN_VELOCITY",0);
	m_strGlobalLocalization = m_pINIReaderWriter->getStringValue("ROBOT", "GLOBAL_LOCALIZATION", "no");
	m_initRobotPosForMap.setX(m_pINIReaderWriter->getDoubleValue("ROBOT","INIT_XPOSE_MAP",0.0));
	m_initRobotPosForMap.setY(m_pINIReaderWriter->getDoubleValue("ROBOT","INIT_YPOSE_MAP",0.0));
	m_initRobotPosForMap.setThetaDeg(m_pINIReaderWriter->getDoubleValue("ROBOT","INIT_THETADEG_MAP",0.0));
	m_nUsingLastPose = m_pINIReaderWriter->getIntValue("ROBOT","USING_LAST_POSE",0);
	m_initRobotPosForNav.setX(m_pINIReaderWriter->getDoubleValue("ROBOT","INIT_XPOSE_NAV",0.0));
	m_initRobotPosForNav.setY(m_pINIReaderWriter->getDoubleValue("ROBOT","INIT_YPOSE_NAV",0.0));
	m_initRobotPosForNav.setThetaDeg(m_pINIReaderWriter->getDoubleValue("ROBOT","INIT_THETADEG_NAV",0.0));
	m_strLastRobotPoseNameNPath = m_pINIReaderWriter->getStringValue("ROBOT", "LASTROBOTPOSE_PATH&NAME", "no");
	m_nObstacleDetectionTime = m_pINIReaderWriter->getIntValue("ROBOT","OBSTACLEDETECTION_TIME",3);
	m_nLocalization = m_pINIReaderWriter->getIntValue("ROBOT","LOCALIZATION",0);

	m_strDataPath = m_pINIReaderWriter->getStringValue("SENSOR", "DATA_PATH", "no");
	m_strDataRecoding = m_pINIReaderWriter->getStringValue("SENSOR", "DATA_RECORDING", "no");
	m_strDataPlay = m_pINIReaderWriter->getStringValue("SENSOR", "DATA_PLAY", "no");

	string strWheelCom= m_pINIReaderWriter->getStringValue("SENSOR", "WHEEL_ACTUATOR", "no");
	constch=strWheelCom.c_str();
	strcpy(m_cWheelCom,constch);

	m_nKuPRIMUSCommPort = m_pINIReaderWriter->getIntValue("SENSOR","KUNS_PRIMUS_COMM_PORT",2);
	m_nCartConnecContPort = m_pINIReaderWriter->getIntValue("SENSOR","CART_CONNECTION_CONT_PORT",1);
	m_dTimeForOn = m_pINIReaderWriter->getDoubleValue("SENSOR","CART_PORT_TIME_ON",4.0);
	m_dTimeForOff = m_pINIReaderWriter->getDoubleValue("SENSOR","CART_PORT_TIME_OFF",0.0);
	m_nDoorOpen1Port = m_pINIReaderWriter->getIntValue("SENSOR","DOOR_OPEN1_PORT",2);
	m_nDoorOpen2Port = m_pINIReaderWriter->getIntValue("SENSOR","DOOR_OPEN2_PORT",3);

	string strGYROCom= m_pINIReaderWriter->getStringValue("SENSOR", "GYRO", "no");
	constch=strGYROCom.c_str();
	strcpy(m_cGyroCom,constch);

	m_nVarietyLaser = m_pINIReaderWriter->getIntValue("SENSOR","VARIETY_LASER",0);

	string strHokuyoURG04LXCom= m_pINIReaderWriter->getStringValue("SENSOR", "URG04LX_LASER", "no");
	constch=strHokuyoURG04LXCom.c_str();
	strcpy(m_cHokuyoURG04LXCom,constch);

	string strSICKLMSFrontCom= m_pINIReaderWriter->getStringValue("SENSOR", "FRONT_LASER_CONNECTION_IP", "no");
	constch=strSICKLMSFrontCom.c_str();
	strcpy(m_cSICKLMSFrontLaserIP,constch);

	string strSICKLMSRearCom= m_pINIReaderWriter->getStringValue("SENSOR", "REAR_LASER_CONNECTION_IP", "no");
	constch=strSICKLMSRearCom.c_str();
	strcpy(m_cSICKLMSRearLaserIP,constch);
	
	m_nLaserTCPPort = m_pINIReaderWriter->getIntValue("SENSOR","LASER_CONNECTION_PORT",0);

	m_sSVLaserIP = m_pINIReaderWriter->getStringValue("SENSOR", "SV_LASER_IP", "127.0.0.1"); // SV laser scanner IP
	m_nSVLaserPort = m_pINIReaderWriter->getIntValue("SENSOR","SV_LASER_PORT", 0); // SV laser scanner TCP/IP comm. port

	string strCommunicationCom= m_pINIReaderWriter->getStringValue("SENSOR", "COMMUNICATION", "no");
	constch=strCommunicationCom.c_str();
	strcpy(m_cCommunicationCom,constch);
	string strSwitchCom= m_pINIReaderWriter->getStringValue("SENSOR", "SWITCH", "no");
	constch=strSwitchCom.c_str();
	strcpy(m_cSwitchCom,constch);
	string strZigbeeCom= m_pINIReaderWriter->getStringValue("SENSOR", "ZIGBEE_COMM_PORT", "no");
	constch=strZigbeeCom.c_str();
	strcpy(m_cZigbeeCom,constch);
	string strGPIOCom= m_pINIReaderWriter->getStringValue("SENSOR", "GPIO_COMM_PORT", "no");
	constch=strGPIOCom.c_str();
	strcpy(m_cGPIOCom,constch);

	m_strdoSonar = m_pINIReaderWriter->getStringValue("SENSOR", "SONAR", "no");


	m_nURG04LX_LaserMaxDist = m_pINIReaderWriter->getIntValue("SENSOR","URG04LX_LASER_MAX_DISTANCE",0);
	m_nURG04LX_LaserMinDist = m_pINIReaderWriter->getIntValue("SENSOR","URG04LX_LASER_MIN_DISTANCE",0); 
	m_nURG04LX_LaserHeight = m_pINIReaderWriter->getIntValue("SENSOR","URG04LX_LASER_HEIGHT",0);
	m_nFrontLaserXOffset = m_pINIReaderWriter->getIntValue("SENSOR","FRONT_LASER_XOFFSET",0);
	m_nFrontLaserYOffset = m_pINIReaderWriter->getIntValue("SENSOR","FRONT_LASER_YOFFSET",0); 
	m_nRearLaserXOffset = m_pINIReaderWriter->getIntValue("SENSOR","REAR_LASER_XOFFSET",0);
	m_nRearLaserYOffset = m_pINIReaderWriter->getIntValue("SENSOR","REAR_LASER_YOFFSET",0); 

	m_nKinectMaxDist = m_pINIReaderWriter->getIntValue("SENSOR","KINECT_MAX_DISTANCE",0);
	m_nKinectMinDist = m_pINIReaderWriter->getIntValue("SENSOR","KINECT_MIN_DISTANCE",0); 
	m_nKinectHeight = m_pINIReaderWriter->getIntValue("SENSOR","KINECT_HEIGHT",0);
	m_nKinectXOffset = m_pINIReaderWriter->getIntValue("SENSOR","KINECT_XOFFSET",0);
	m_nKinectYOffset = m_pINIReaderWriter->getIntValue("SENSOR","KINECT_YOFFSET",0); 
	m_nKinectMaxHeightDist = m_pINIReaderWriter->getIntValue("SENSOR","KINECT_MAX_HEIGHT_DISTANCE",0);
	m_nKinectMinHeightDist = m_pINIReaderWriter->getIntValue("SENSOR","KINECT_MIN_HEIGHT_DISTANCE",0); 

	m_dCam_fx = m_pINIReaderWriter->getDoubleValue("SENSOR","CEILINGCAM_UNDISTORTION_FX",160);
	m_dCam_fy = m_pINIReaderWriter->getDoubleValue("SENSOR","CEILINGCAM_UNDISTORTION_FY",160);
	m_dCam_cx = m_pINIReaderWriter->getDoubleValue("SENSOR","CEILINGCAM_UNDISTORTION_CX",160);
	m_dCam_cy = m_pINIReaderWriter->getDoubleValue("SENSOR","CEILINGCAM_UNDISTORTION_CY",120);
	m_dCam_d1 = m_pINIReaderWriter->getDoubleValue("SENSOR","CEILINGCAM_UNDISTORTION_D1",0);
	m_dCam_d2 = m_pINIReaderWriter->getDoubleValue("SENSOR","CEILINGCAM_UNDISTORTION_D2",0);
	m_dCam_d3 = m_pINIReaderWriter->getDoubleValue("SENSOR","CEILINGCAM_UNDISTORTION_D3",0);
	m_dCam_d4 = m_pINIReaderWriter->getDoubleValue("SENSOR","CEILINGCAM_UNDISTORTION_D4",0);
	m_dCam_offset_x  = m_pINIReaderWriter->getDoubleValue("SENSOR","CEILINGCAM_OFFSET_X",0);
	m_nCamMode = m_pINIReaderWriter->getIntValue("SENSOR","CEILINGCAM_MODE",0);
	m_nCamExposure = m_pINIReaderWriter->getIntValue("SENSOR","CEILINGCAM_EXPOSURE",0);
	m_bCamExposure = (bool)m_pINIReaderWriter->getIntValue("SENSOR","CEILINGCAM_AUTO_EXPOSURE",0);

	// Monitoring
	m_fLowBatteryAlarm = m_pINIReaderWriter->getDoubleValue("MONITORING", "LOW_BATTERY_ALARM", 0);

	//INI ���Ϸκ��� kanayama motion contro���� �����ϱ� ���� �ʿ��� ������ ���´�.------------------------------------
	m_nDistToTarget = m_pINIReaderWriter->getIntValue("KANAYAMA_MOTION_CONTROL","DISTANCE_TO_TARGET",0);
	m_nDesiredVel = m_pINIReaderWriter->getIntValue("KANAYAMA_MOTION_CONTROL","DESIRED_VELOCITY",0); 
	m_nGoalArea = m_pINIReaderWriter->getIntValue("KANAYAMA_MOTION_CONTROL","GOAL_AREA",0);
	m_dKX = m_pINIReaderWriter->getDoubleValue("KANAYAMA_MOTION_CONTROL","X_GAIN");
	m_dKY = m_pINIReaderWriter->getDoubleValue("KANAYAMA_MOTION_CONTROL","Y_GAIN");
	m_dKT = m_pINIReaderWriter->getDoubleValue("KANAYAMA_MOTION_CONTROL","T_GAIN");
	m_nDirectionofRotation = m_pINIReaderWriter->getIntValue("KANAYAMA_MOTION_CONTROL","DIRECTION_OF_ROTATION",0);	
	//==================================================================================================================

	//INI ���Ϸκ��� Particle filter�� �����ϱ� ���� �ʿ��� ������ ���´�.------------------------------------
	m_nMaxParticleNum = m_pINIReaderWriter->getIntValue("PARTICLE_FILTER","MAX_SAMPLE_NUM",0);
	m_nMinParticleNum = m_pINIReaderWriter->getIntValue("PARTICLE_FILTER","MIN_SAMPLE_NUM",0); 
	m_dDevationforTrans = m_pINIReaderWriter->getDoubleValue("PARTICLE_FILTER","DEVATION_TRANS");
	m_dDevationforRotate = m_pINIReaderWriter->getDoubleValue("PARTICLE_FILTER","DEVATION_ROTATE");
	m_dDevationforTransRotate = m_pINIReaderWriter->getDoubleValue("PARTICLE_FILTER","DEVATION_TRANSROTATE");
	//==================================================================================================================


	//INI ���Ϸκ��� FEATURES�� �����ϱ� ���� �ʿ��� ������ ���´�.------------------------------------
	m_nFeatureTh = m_pINIReaderWriter->getIntValue("FEATURES","MIN_FEATURE_TH", 160);
	m_nSiftMatchingTh = m_pINIReaderWriter->getIntValue("FEATURES","MATCHING_TH", 20); 
	m_nInteractionPointTh = m_pINIReaderWriter->getIntValue("FEATURES","INTERACTION_POINT_TH", 15);
	m_dNumSIFTFeatureTh = m_pINIReaderWriter->getDoubleValue("FEATURES","NUM_SIFTFEATURE_TH", 0.01);
	m_nNumSURFFeatureTh = m_pINIReaderWriter->getIntValue("FEATURES","NUM_SURFFEATURE_TH",400);
	m_dMatchingAngleTh = m_pINIReaderWriter->getDoubleValue("FEATURES","MATHING_ANGLE_TH", 0.15);
	m_strSIFTDataNamePath = m_pINIReaderWriter->getStringValue("FEATURES", "SIFTDATA_PATH&NAME", "no");
	m_strSURFDataNamePath = m_pINIReaderWriter->getStringValue("FEATURES", "SURFDATA_PATH&NAME", "no");
	m_nDistnaceFromPath = m_pINIReaderWriter->getDoubleValue("FEATURES","DISTANCE_FROM_PATH", 1000);
	m_dEllipseWidth = m_pINIReaderWriter->getDoubleValue("FEATURES","ELLIPSE_WIDTH", 100.0);
	m_dEllipseHeight = m_pINIReaderWriter->getDoubleValue("FEATURES","ELLIPSE_HEIGHT", 50.0);

	//==================================================================================================================


	//INI ���Ϸκ��� Recognizer�� �����ϱ� ���� �ʿ��� ������ ���´�.------------------------------------
	m_strAlFeatureMapNameNPath = m_pINIReaderWriter->getStringValue("RECOGNIZER", "AL_FEATURE_PATH&NAME", "no");
	m_nLandMarkNum = m_pINIReaderWriter->getIntValue("RECOGNIZER","TOTAL_FIDUCIAL_MARK_NUM", 1);
	m_nHeight_Camera2Mark = m_pINIReaderWriter->getIntValue("RECOGNIZER","HEIGHT_CAMERA_MARK", 2000); 
	m_dRecognizingDistThres = m_pINIReaderWriter->getDoubleValue("RECOGNIZER","RECOGNIZING_DIST_TH",2000);
	//==================================================================================================================

	// ISSAC /////////////////////////////////////////////////////////////////////////////////////
	if(getUsingISSAC() == 0) // ISSAC �̻��
	{
		XmldataSetting::getInstance()->setMode(XmldataSetting::AGV_MODE);
	}
	else // ISSAC ���
	{
		XmldataSetting::getInstance()->setMode(XmldataSetting::ISSAC_MODE);
	}
	// ISSAC /////////////////////////////////////////////////////////////////////////////////////

	//saveParameter();

	return true;

}
void KuRobotParameter::saveParameter()
{

	string strInIFileName = "./ini/KUNS.ini";  	//�⺻���� �̸��� kuns.ini���Ͽ� ��õǾ� �ִ�. 

	CString strTemp;

	strTemp.Format(L"%d",m_nUseISSAC);
	WritePrivateProfileString(_T("ISSAC"), _T("USE_ISSAC"), strTemp,_T("./ini/KUNS.ini"));
	strTemp=m_sISSACServerIP.c_str();
	WritePrivateProfileString(_T("ISSAC"), _T("ISSAC_SERVER_IP"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nISSACServerPort);
	WritePrivateProfileString(_T("ISSAC"), _T("ISSAC_SERVER_PORT"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nAGVClientPort);
	WritePrivateProfileString(_T("ISSAC"), _T("AGV_CLIENT_PORT"), strTemp,_T("./ini/KUNS.ini"));

	strTemp.Format(L"%d",m_nUseIoT);
	WritePrivateProfileString(_T("IOT"), _T("USE_IOT"), strTemp,_T("./ini/KUNS.ini"));
	strTemp=m_sIoTServerIP.c_str();
	WritePrivateProfileString(_T("IOT"), _T("IOT_SERVER_IP"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nIoTServerPort);
	WritePrivateProfileString(_T("IOT"), _T("IOT_SERVER_PORT"), strTemp,_T("./ini/KUNS.ini"));

	strTemp=m_strMapNameNPath.c_str();
	WritePrivateProfileString(_T("MAP"), _T("PATH&NAME"), strTemp,_T("./ini/KUNS.ini"));
	strTemp=m_strVelocityMapNameNPath.c_str();
	WritePrivateProfileString(_T("MAP"), _T("VELOCITY_PATH&NAME"), strTemp,_T("./ini/KUNS.ini"));
	strTemp=m_strCeilingMapNameNPath.c_str();
	WritePrivateProfileString(_T("MAP"), _T("CEILINGMAP_PATH&NAME"), strTemp,_T("./ini/KUNS.ini"));
	strTemp=m_strCadMapNameNPath.c_str();
	WritePrivateProfileString(_T("MAP"), _T("CADMAP_PATH&NAME"), strTemp,_T("./ini/KUNS.ini"));
	strTemp=m_strZoneMapNameNPath.c_str();
	WritePrivateProfileString(_T("MAP"), _T("ZONEMAP_PATH&NAME"), strTemp,_T("./ini/KUNS.ini"));
	strTemp=m_strTempMapPath.c_str();
	WritePrivateProfileString(_T("MAP"), _T("TEMPMAP_PATH&NAME"), strTemp,_T("./ini/KUNS.ini"));
	
	strTemp.Format(L"%d",m_nMapSizeXm);
	WritePrivateProfileString(_T("MAP"), _T("MAP_SIZE_X"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nMapSizeYm);
	WritePrivateProfileString(_T("MAP"), _T("MAP_SIZE_Y"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%0.2f",m_dHeight);
	WritePrivateProfileString(_T("MAP"), _T("HEIGHT"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%0.2f",m_dSDThres);
	WritePrivateProfileString(_T("MAP"), _T("STANDARD_DEVIATION_TH"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nUpSize_X);
	WritePrivateProfileString(_T("MAP"), _T("UPDATAESIZE_X"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nUpSize_Y);
	WritePrivateProfileString(_T("MAP"), _T("UPDATAESIZE_Y"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nCheckSize_X);
	WritePrivateProfileString(_T("MAP"), _T("CHECKSIZE_X"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nCheckSize_Y);
	WritePrivateProfileString(_T("MAP"), _T("CHECKSIZE_Y"), strTemp,_T("./ini/KUNS.ini"));

	strTemp=m_strMovieNameNPath.c_str();
	WritePrivateProfileString(_T("MOVIE"), _T("MOVIE&PATH&NAME"), strTemp,_T("./ini/KUNS.ini"));
	
	strTemp=m_strTeachingPathNameNPath.c_str();
	WritePrivateProfileString(_T("PATH"), _T("TEACHINGPATH_PATH&NAME"), strTemp,_T("./ini/KUNS.ini"));
	strTemp=m_strImagePathNameNPath.c_str();
	WritePrivateProfileString(_T("PATH"), _T("IMAGEPATH_PATH&NAME"), strTemp,_T("./ini/KUNS.ini"));
	strTemp=m_strTeachingWayPointNameNPath.c_str();
	WritePrivateProfileString(_T("PATH"), _T("TEACHINGWAYPOINT_PATH&NAME"), strTemp,_T("./ini/KUNS.ini"));
	strTemp=m_strOutlineWayPointNameNPath.c_str();
	WritePrivateProfileString(_T("PATH"), _T("OUTLINEWAYPOINT_PATH&NAME"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_dBlockSizeX);
	WritePrivateProfileString(_T("PATH"), _T("BLOCKSIZE_X"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_dBlockSizeY);
	WritePrivateProfileString(_T("PATH"), _T("BLOCKSIZE_Y"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nPathNum);
	WritePrivateProfileString(_T("PATH"), _T("PATH_NUM"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nReversePathNum);
	WritePrivateProfileString(_T("PATH"), _T("REVERSE_PATH_NUM"), strTemp,_T("./ini/KUNS.ini"));
	strTemp=m_strPathteaching.c_str();
	WritePrivateProfileString(_T("PATH"), _T("PATHTEACHING"), strTemp,_T("./ini/KUNS.ini"));


	strTemp.Format(L"%d",m_nRobotID);
	WritePrivateProfileString(_T("ROBOT"), _T("ROBOT_ID"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nRadiusofRobot);
	WritePrivateProfileString(_T("ROBOT"), _T("ROBOT_RADIUS"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nWheelBaseofRobot);
	WritePrivateProfileString(_T("ROBOT"), _T("ROBOT_WHEEL_BASE"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nMaxRobotVelocity);
	WritePrivateProfileString(_T("ROBOT"), _T("MAX_VELOCITY"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nMinRobotVelocity);
	WritePrivateProfileString(_T("ROBOT"), _T("MIN_VELOCITY"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%f",m_initRobotPosForMap.getX());
	WritePrivateProfileString(_T("ROBOT"), _T("INIT_XPOSE_MAP"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%0.2f",m_initRobotPosForMap.getY());
	WritePrivateProfileString(_T("ROBOT"), _T("INIT_YPOSE_MAP"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%0.2f",m_initRobotPosForMap.getThetaDeg());
	WritePrivateProfileString(_T("ROBOT"), _T("INIT_THETADEG_MAP"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nUsingLastPose);
	WritePrivateProfileString(_T("ROBOT"), _T("USING_LAST_POSE"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%f",m_initRobotPosForNav.getX());
	WritePrivateProfileString(_T("ROBOT"), _T("INIT_XPOSE_NAV"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%0.2f",m_initRobotPosForNav.getY());
	WritePrivateProfileString(_T("ROBOT"), _T("INIT_YPOSE_NAV"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%0.2f",m_initRobotPosForNav.getThetaDeg());
	WritePrivateProfileString(_T("ROBOT"), _T("INIT_THETADEG_NAV"), strTemp,_T("./ini/KUNS.ini"));
	strTemp=m_strLastRobotPoseNameNPath.c_str();
	WritePrivateProfileString(_T("ROBOT"), _T("LASTROBOTPOSE_PATH&NAME"), strTemp,_T("./ini/KUNS.ini"));
	strTemp=m_strGlobalLocalization.c_str();
	WritePrivateProfileString(_T("ROBOT"), _T("GLOBAL_LOCALIZATION"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nObstacleDetectionTime);
	WritePrivateProfileString(_T("ROBOT"), _T("OBSTACLEDETECTION_TIME"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nLocalization);
	WritePrivateProfileString(_T("ROBOT"), _T("LOCALIZATION"), strTemp,_T("./ini/KUNS.ini"));
	

	strTemp=m_strDataPath.c_str();
	WritePrivateProfileString(_T("SENSOR"), _T("DATA_PATH"), strTemp,_T("./ini/KUNS.ini"));
	strTemp=m_strDataRecoding.c_str();
	WritePrivateProfileString(_T("SENSOR"), _T("DATA_RECORDING"), strTemp,_T("./ini/KUNS.ini"));
	strTemp=m_strDataPlay.c_str();
	WritePrivateProfileString(_T("SENSOR"), _T("DATA_PLAY"), strTemp,_T("./ini/KUNS.ini"));

	strTemp=m_cWheelCom;
	WritePrivateProfileString(_T("SENSOR"), _T("WHEEL_ACTUATOR"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nKuPRIMUSCommPort);
	WritePrivateProfileString(_T("SENSOR"), _T("KUNS_PRIMUS_COMM_PORT"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nCartConnecContPort);
	WritePrivateProfileString(_T("SENSOR"), _T("CART_CONNECTION_CONT_PORT"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%0.1f",m_dTimeForOn);
	WritePrivateProfileString(_T("SENSOR"), _T("CART_PORT_TIME_ON"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%0.1f",m_dTimeForOff);
	WritePrivateProfileString(_T("SENSOR"), _T("CART_PORT_TIME_OFF"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nDoorOpen1Port);
	WritePrivateProfileString(_T("SENSOR"), _T("DOOR_OPEN1_PORT"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nDoorOpen2Port);
	WritePrivateProfileString(_T("SENSOR"), _T("DOOR_OPEN2_PORT"), strTemp,_T("./ini/KUNS.ini"));
	
	strTemp=m_cGyroCom;
	WritePrivateProfileString(_T("SENSOR"), _T("GYRO"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nVarietyLaser);
	WritePrivateProfileString(_T("SENSOR"), _T("VARIETY_LASER"), strTemp,_T("./ini/KUNS.ini"));

	strTemp=m_cHokuyoURG04LXCom;
	WritePrivateProfileString(_T("SENSOR"), _T("URG04LX_LASER"), strTemp,_T("./ini/KUNS.ini"));
	
	strTemp=m_cSICKLMSFrontLaserIP;
	WritePrivateProfileString(_T("SENSOR"), _T("FRONT_LASER_CONNECTION_IP"), strTemp,_T("./ini/KUNS.ini"));	
	strTemp=m_cSICKLMSRearLaserIP;
	WritePrivateProfileString(_T("SENSOR"), _T("REAR_LASER_CONNECTION_IP"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nLaserTCPPort);
	WritePrivateProfileString(_T("SENSOR"), _T("LASER_CONNECTION_PORT"), strTemp,_T("./ini/KUNS.ini"));

	strTemp=m_cCommunicationCom;
	WritePrivateProfileString(_T("SENSOR"), _T("COMMUNICATION"), strTemp,_T("./ini/KUNS.ini"));
	strTemp=m_cSwitchCom;
	WritePrivateProfileString(_T("SENSOR"), _T("SWITCH"), strTemp,_T("./ini/KUNS.ini"));
	strTemp=m_cZigbeeCom;
	WritePrivateProfileString(_T("SENSOR"), _T("ZIGBEE_COMM_PORT"), strTemp,_T("./ini/KUNS.ini"));
	strTemp=m_cGPIOCom;
	WritePrivateProfileString(_T("SENSOR"), _T("GPIO_COMM_PORT"), strTemp,_T("./ini/KUNS.ini"));

	strTemp=m_strdoSonar.c_str();
	WritePrivateProfileString(_T("SENSOR"), _T("SONAR"), strTemp,_T("./ini/KUNS.ini"));

	strTemp.Format(L"%d",m_nURG04LX_LaserMaxDist);
	WritePrivateProfileString(_T("SENSOR"), _T("URG04LX_LASER_MAX_DISTANCE"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nURG04LX_LaserMinDist);
	WritePrivateProfileString(_T("SENSOR"), _T("URG04LX_LASER_MIN_DISTANCE"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nURG04LX_LaserHeight);
	WritePrivateProfileString(_T("SENSOR"), _T("URG04LX_LASER_HEIGHT"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nFrontLaserXOffset);
	WritePrivateProfileString(_T("SENSOR"), _T("FRONT_LASER_XOFFSET"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nFrontLaserYOffset);
	WritePrivateProfileString(_T("SENSOR"), _T("FRONT_LASER_YOFFSET"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nRearLaserXOffset);
	WritePrivateProfileString(_T("SENSOR"), _T("REAR_LASER_XOFFSET"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nRearLaserYOffset);
	WritePrivateProfileString(_T("SENSOR"), _T("REAR_LASER_YOFFSET"), strTemp,_T("./ini/KUNS.ini"));

	strTemp.Format(L"%d",m_nKinectMaxDist);
	WritePrivateProfileString(_T("SENSOR"), _T("KINECT_MAX_DISTANCE"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nKinectMinDist);
	WritePrivateProfileString(_T("SENSOR"), _T("KINECT_MIN_DISTANCE"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nKinectHeight);
	WritePrivateProfileString(_T("SENSOR"), _T("KINECT_HEIGHT"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nKinectXOffset);
	WritePrivateProfileString(_T("SENSOR"), _T("KINECT_XOFFSET"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nKinectYOffset);
	WritePrivateProfileString(_T("SENSOR"), _T("KINECT_YOFFSET"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nKinectMaxHeightDist);
	WritePrivateProfileString(_T("SENSOR"), _T("KINECT_MAX_HEIGHT_DISTANCE"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nKinectMinHeightDist);
	WritePrivateProfileString(_T("SENSOR"), _T("KINECT_MIN_HEIGHT_DISTANCE"), strTemp,_T("./ini/KUNS.ini"));

	strTemp.Format(L"%0.5f",m_dCam_fx);
	WritePrivateProfileString(_T("SENSOR"), _T("CEILINGCAM_UNDISTORTION_FX"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%0.5f",m_dCam_fy);
	WritePrivateProfileString(_T("SENSOR"), _T("CEILINGCAM_UNDISTORTION_FY"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%0.5f",m_dCam_cx);
	WritePrivateProfileString(_T("SENSOR"), _T("CEILINGCAM_UNDISTORTION_CX"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%0.5f",m_dCam_cy);
	WritePrivateProfileString(_T("SENSOR"), _T("CEILINGCAM_UNDISTORTION_CY"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%0.5f",m_dCam_d1);
	WritePrivateProfileString(_T("SENSOR"), _T("CEILINGCAM_UNDISTORTION_D1"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%0.5f",m_dCam_d2);
	WritePrivateProfileString(_T("SENSOR"), _T("CEILINGCAM_UNDISTORTION_D2"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%0.5f",m_dCam_d3);
	WritePrivateProfileString(_T("SENSOR"), _T("CEILINGCAM_UNDISTORTION_D3"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%0.5f",m_dCam_d4);
	WritePrivateProfileString(_T("SENSOR"), _T("CEILINGCAM_UNDISTORTION_D4"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%0.5f",m_dCam_offset_x  );
	WritePrivateProfileString(_T("SENSOR"), _T("CEILINGCAM_OFFSET_X"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nCamMode);
	WritePrivateProfileString(_T("SENSOR"), _T("CEILINGCAM_MODE"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d", m_nCamExposure);
	WritePrivateProfileString(_T("SENSOR"), _T("CEILINGCAM_EXPOSURE"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d", (int)m_bCamExposure);
	WritePrivateProfileString(_T("SENSOR"), _T("CEILINGCAM_AUTO_EXPOSURE"), strTemp,_T("./ini/KUNS.ini"));

	strTemp.Format(L"%f", m_fLowBatteryAlarm);
	WritePrivateProfileString(_T("MONITORING"), _T("LOW_BATTERY_ALARM"), strTemp,_T("./ini/KUNS.ini"));

	strTemp.Format(L"%d",m_nDistToTarget);
	WritePrivateProfileString(_T("KANAYAMA_MOTION_CONTROL"), _T("DISTANCE_TO_TARGET"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nDesiredVel);
	WritePrivateProfileString(_T("KANAYAMA_MOTION_CONTROL"), _T("DESIRED_VELOCITY"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nGoalArea);
	WritePrivateProfileString(_T("KANAYAMA_MOTION_CONTROL"), _T("GOAL_AREA"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%0.2f",m_dKX);
	WritePrivateProfileString(_T("KANAYAMA_MOTION_CONTROL"), _T("X_GAIN"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%0.2f",m_dKY);
	WritePrivateProfileString(_T("KANAYAMA_MOTION_CONTROL"), _T("Y_GAIN"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%0.2f",m_dKT);
	WritePrivateProfileString(_T("KANAYAMA_MOTION_CONTROL"), _T("T_GAIN"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nDirectionofRotation);
	WritePrivateProfileString(_T("KANAYAMA_MOTION_CONTROL"), _T("DIRECTION_OF_ROTATION"), strTemp,_T("./ini/KUNS.ini"));

	strTemp.Format(L"%d",m_nMaxParticleNum);
	WritePrivateProfileString(_T("PARTICLE_FILTER"), _T("MAX_SAMPLE_NUM"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nMinParticleNum);
	WritePrivateProfileString(_T("PARTICLE_FILTER"), _T("MIN_SAMPLE_NUM"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%0.4f",m_dDevationforTrans);
	WritePrivateProfileString(_T("PARTICLE_FILTER"), _T("DEVATION_TRANS"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%0.4f",m_dDevationforRotate);
	WritePrivateProfileString(_T("PARTICLE_FILTER"), _T("DEVATION_ROTATE"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%0.4f",m_dDevationforTransRotate);
	WritePrivateProfileString(_T("PARTICLE_FILTER"), _T("DEVATION_TRANSROTATE"), strTemp,_T("./ini/KUNS.ini"));

	strTemp.Format(L"%d",m_nFeatureTh);
	WritePrivateProfileString(_T("FEATURES"), _T("MIN_FEATURE_TH"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nSiftMatchingTh);
	WritePrivateProfileString(_T("FEATURES"), _T("MATCHING_TH"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nInteractionPointTh);
	WritePrivateProfileString(_T("FEATURES"), _T("INTERACTION_POINT_TH"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%0.4f",m_dNumSIFTFeatureTh);
	WritePrivateProfileString(_T("FEATURES"), _T("NUM_SIFTFEATURE_TH"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nNumSURFFeatureTh);
	WritePrivateProfileString(_T("FEATURES"), _T("NUM_SURFFEATURE_TH"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%0.4f",m_dMatchingAngleTh);
	WritePrivateProfileString(_T("FEATURES"), _T("MATHING_ANGLE_TH"), strTemp,_T("./ini/KUNS.ini"));
	
	strTemp=m_strSIFTDataNamePath.c_str();
	WritePrivateProfileString(_T("FEATURES"), _T("SIFTDATA_PATH&NAME"), strTemp,_T("./ini/KUNS.ini"));
	strTemp=m_strSURFDataNamePath.c_str();
	WritePrivateProfileString(_T("FEATURES"), _T("SURFDATA_PATH&NAME"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nDistnaceFromPath);
	WritePrivateProfileString(_T("FEATURES"), _T("DISTANCE_FROM_PATH"), strTemp,_T("./ini/KUNS.ini"));

	strTemp.Format(L"%0.4f",m_dEllipseWidth);
	WritePrivateProfileString(_T("FEATURES"), _T("ELLIPSE_WIDTH"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%0.4f",m_dEllipseHeight);
	WritePrivateProfileString(_T("FEATURES"), _T("ELLIPSE_HEIGHT"), strTemp,_T("./ini/KUNS.ini"));
	
	strTemp=m_strAlFeatureMapNameNPath.c_str();
	WritePrivateProfileString(_T("RECOGNIZER"), _T("AL_FEATURE_PATH&NAME"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nLandMarkNum );
	WritePrivateProfileString(_T("RECOGNIZER"), _T("TOTAL_FIDUCIAL_MARK_NUM"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nHeight_Camera2Mark  );
	WritePrivateProfileString(_T("RECOGNIZER"), _T("HEIGHT_CAMERA_MARK"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%0.1f",m_dRecognizingDistThres);
	WritePrivateProfileString(_T("RECOGNIZER"), _T("RECOGNIZING_DIST_TH"), strTemp,_T("./ini/KUNS.ini"));
}
void KuRobotParameter::setRobotID(int nID)
{
	//�κ��� ID�� �������ִ� �Լ�.
	m_nRobotID = nID;
}

void KuRobotParameter::setLocalization(int nLocalization)
{
	//�κ��� ID�� �������ִ� �Լ�.
	m_nLocalization = nLocalization;
}
void KuRobotParameter::setObstacleDetectionTime(int nObstacleDetectionTime)
{
	//�κ��� ID�� �������ִ� �Լ�.
	m_nObstacleDetectionTime = nObstacleDetectionTime;
}

void KuRobotParameter::setInitRobotPoseForMap(KuPose initRobotPos)
{
	m_initRobotPosForMap=initRobotPos;
}
void KuRobotParameter::setUsingLastPose(int nUsingLastPose)
{
	m_nUsingLastPose = nUsingLastPose;
}
void KuRobotParameter::setInitRobotPoseForNav(KuPose initRobotPos)
{
	m_initRobotPosForNav=initRobotPos;
}
void KuRobotParameter::setRobotRadius(int nRadius)
{
	//�κ� �ݰ��� �������ִ� �Լ�. ������ mm
	m_nRadiusofRobot = nRadius;
}

void KuRobotParameter::setWheelBaseofRobot(int nWheelBaseofRobot)
{
	//�κ� �ݰ��� �������ִ� �Լ�. ������ mm
	m_nWheelBaseofRobot = nWheelBaseofRobot;
}

void KuRobotParameter::setMaxRobotVelocity(int nMaxVelocity)
{
	//�κ��� �ִ�ӵ��� �Է¹޴´�.
	m_nMaxRobotVelocity = nMaxVelocity;
}
void KuRobotParameter::setMinRobotVelocity(int nMinVelocity)
{
	//�κ��� �ּҼӵ��� �Է¹޴´�.
	m_nMinRobotVelocity = nMinVelocity;
}
void KuRobotParameter::setLastRobotPoseNameNPath(string strLastRobotPoseNameNPath)
{
	m_strLastRobotPoseNameNPath=strLastRobotPoseNameNPath;
}
void KuRobotParameter::setMapSize(int nSizeXm, int nSizeYm)
{
	//�ۼ��� ������ ũ�⸦ �����Ѵ�. 
	m_nMapSizeXm = nSizeXm; //���� x,y ũ�� ����: m�� ����
	m_nMapSizeYm = nSizeYm; //���� x,y ũ�� ����: m�� ����

}

void KuRobotParameter::setHeight(double dHeight )
{
	m_dHeight = dHeight; // m�� ����

}
void KuRobotParameter::setSDValue(double dSDValue)
{
	m_dSDThres = dSDValue;
}
void KuRobotParameter::setUpdateValX(int nUpdateHalfSizeX)
{
	m_nUpSize_X = nUpdateHalfSizeX;
}
void KuRobotParameter::setUpdateValY(int nUpdateHalfSizeY)
{
	m_nUpSize_Y = nUpdateHalfSizeY;
}
void KuRobotParameter::setCheckValX(int nCheckValX)
{
	m_nCheckSize_X = nCheckValX;
}
void KuRobotParameter::setCheckValY(int nCheckValY)
{
	m_nCheckSize_Y = nCheckValY;
}
void KuRobotParameter::setVelocityMapNameNPath(string sMapName) 
{
	//�强�� ������ �̸��� �����Ѵ�. .	
	m_strVelocityMapNameNPath=sMapName;
}
void KuRobotParameter::setTeachingPathNameNPath(string sMapName) 
{
	//�强�� ������ �̸��� �����Ѵ�. .	
	m_strTeachingPathNameNPath=sMapName;
}
void KuRobotParameter::setTeachingWayPointNameNPath(string sMapName) 
{
	//�强�� ������ �̸��� �����Ѵ�. .	
	m_strTeachingWayPointNameNPath=sMapName;
}
void KuRobotParameter::setOutlineWayPointNameNPath(string sMapName) 
{
	//�强�� ������ �̸��� �����Ѵ�. .	
	m_strOutlineWayPointNameNPath=sMapName;
}
void KuRobotParameter::setBlockSizeX(double dBlockSizeX)
{
	m_dBlockSizeX = dBlockSizeX;
}
void KuRobotParameter::setBlockSizeY(double dBlockSizeY)
{
	m_dBlockSizeY = dBlockSizeY;
}
void KuRobotParameter::setPathNum(int nPathNum)
{
	m_nPathNum = nPathNum;
}
void KuRobotParameter::setReversePathNum(int nReversePathNum)
{
	m_nReversePathNum = nReversePathNum;
}
void KuRobotParameter::setPathteaching(string strPathteaching) 
{
	//�ۼ��� ������ ��� ����
	m_strPathteaching=strPathteaching;
}
void KuRobotParameter::setImagePathNameNPath(string sMapName) 
{
	//�ۼ��� ������ ��� ����
	m_strImagePathNameNPath=sMapName;
}
void KuRobotParameter::setMapNameNPath(string sMapName) 
{
	//�ۼ��� ������ ��� ����
	m_strMapNameNPath=sMapName;
}
void KuRobotParameter::setCeilingMapNameNPath(string sMapName) 
{
	//�ۼ��� ������ ��� ����
	m_strCeilingMapNameNPath=sMapName;
}

void KuRobotParameter::setCadMapNameNPath(string sMapName) 
{
	//�ۼ��� ������ ��� ����
	m_strCadMapNameNPath=sMapName;
}
void KuRobotParameter::setZoneMapNameNPath(string sMapName) 
{
	//�ۼ��� ������ ��� ����
	m_strZoneMapNameNPath=sMapName;
}
void KuRobotParameter::setMovieNameNPath(string sMapName) 
{
	//�ۼ��� ������ ��� ����
	m_strMovieNameNPath=sMapName;
}
void KuRobotParameter::setTempMapPath(string sMapPath)
{
	//�ۼ��� ������ ��� ����
	m_strTempMapPath=sMapPath;
}
void KuRobotParameter::setDataPath(string sDataPath) 
{
	m_strDataPath=sDataPath;
}
void KuRobotParameter::setDataRecoding(string sDataRecoding)
{
	m_strDataRecoding=sDataRecoding;
}
void KuRobotParameter::setGlobalLocalization(string sGlobalLocalization)
{
	m_strGlobalLocalization=sGlobalLocalization;
}

void KuRobotParameter::setDataPlay(string sDataPlay) 
{
	m_strDataPlay=sDataPlay;
}

void KuRobotParameter::setWheelComport(char cComport[10])
{
	//Wheel comport�� �����ϴ� �Լ�.
	strcpy(m_cWheelCom, cComport); //���ڿ� ���� �Լ�.
}

void KuRobotParameter::setKuPrimusCommPort(int nPort)
{
	m_nKuPRIMUSCommPort = nPort;
}
void KuRobotParameter::setCartConnecContPort(int nPort)
{
	m_nCartConnecContPort = nPort;
}
void KuRobotParameter::setCartPortTimeOn(double dTime)
{
	m_dTimeForOn = dTime;
}
void KuRobotParameter::setCartPortTimeOff(double dTime)
{
	m_dTimeForOff = dTime;
}
void KuRobotParameter::setDoorOpen1Port(int nPort)
{
	m_nDoorOpen1Port = nPort;
}
void KuRobotParameter::setDoorOpen2Port(int nPort)
{
	m_nDoorOpen2Port = nPort;
}
void KuRobotParameter::setVarietyLaser(int nVarietyLaser )
{
	m_nVarietyLaser = nVarietyLaser; // m�� ����

}
void KuRobotParameter::setURG04LXLaserComport(char cComport[10])
{
	//URG04LX laser comport�� �����ϴ� �Լ�.
	strcpy(m_cHokuyoURG04LXCom, cComport); //���ڿ� ���� �Լ�.

}

void KuRobotParameter::setSICKLMSLaserComport(char cComport[30])
{
	//URG04LX laser comport�� �����ϴ� �Լ�.
	strcpy(m_cSICKLMSFrontLaserIP, cComport); //���ڿ� ���� �Լ�.

}
void KuRobotParameter::setLaserTCPPort(int nLaserTCPPort )
{
	m_nLaserTCPPort = nLaserTCPPort; // m�� ����

}

void KuRobotParameter::setCommunicationComport(char cComport[10])
{
	//URG04LX laser comport�� �����ϴ� �Լ�.
	strcpy(m_cCommunicationCom, cComport); //���ڿ� ���� �Լ�.

}
void KuRobotParameter::setGyroComport(char cComport[10])
{
	//Wheel comport�� �����ϴ� �Լ�.
	strcpy(m_cGyroCom, cComport); //���ڿ� ���� �Լ�.
}
void KuRobotParameter::setSwitchComport(char cComport[10])
{
	//URG04LX laser comport�� �����ϴ� �Լ�.
	strcpy(m_cSwitchCom, cComport); //���ڿ� ���� �Լ�.

}
void KuRobotParameter::setZigbeeComport(char cComport[10])
{
	//Zigbee comport�� �����ϴ� �Լ�.
	strcpy(m_cZigbeeCom, cComport); //���ڿ� ���� �Լ�.

}
void KuRobotParameter::setGPIOComport(char cComport[10])
{
	//GPIO comport�� �����ϴ� �Լ�.
	strcpy(m_cGPIOCom, cComport); //���ڿ� ���� �Լ�.

}
void KuRobotParameter::setdoSonar(string strdoSonar)
{
	m_strdoSonar =strdoSonar ;
}

//laser ���� �Ķ����---------------------------------------------------------------------------------
void KuRobotParameter::setURG04LXLaserMaxDist(int nMaxDist)
{
	//��� �������� �ִ� Ž���Ÿ��� �����ϴ� �Լ�.
	m_nURG04LX_LaserMaxDist = nMaxDist;
}

void KuRobotParameter::setURG04LXLaserMinDist(int nMinDist)
{
	//��� �������� �ּ� Ž���Ÿ��� �����ϴ� �Լ�.
	m_nURG04LX_LaserMinDist = nMinDist;
}

void KuRobotParameter::setURG04LXLaserHeight(int nHeight)
{
	//ƿƮ ������ ���� ��� ������������ ���̰��� �����ϴ� �Լ�.
	m_nURG04LX_LaserHeight = nHeight;
}

void KuRobotParameter::setFrontLaserXOffset(int nXOffset)
{
	//ƿƮ ������ ���� ��� ������������ x offset�� �����ϴ� �Լ�.
	m_nFrontLaserXOffset = nXOffset;
}

void KuRobotParameter::setFrontLaserYOffset(int nYOffset)
{
	//ƿƮ ������ ���� ��� ������������ y offset�� �����ϴ� �Լ�.
	m_nFrontLaserYOffset = nYOffset;
}

void KuRobotParameter::setRearLaserXOffset(int nXOffset)
{
	//ƿƮ ������ ���� ��� ������������ x offset�� �����ϴ� �Լ�.
	m_nRearLaserXOffset = nXOffset;
}

void KuRobotParameter::setRearLaserYOffset(int nYOffset)
{
	//ƿƮ ������ ���� ��� ������������ y offset�� �����ϴ� �Լ�.
	m_nRearLaserYOffset = nYOffset;
}

//Kinect ���� �Ķ����---------------------------------------------------------------------------------
void KuRobotParameter::setKinectMaxDist(int nMaxDist)
{
	//��� �������� �ִ� Ž���Ÿ��� �����ϴ� �Լ�.
	m_nKinectMaxDist = nMaxDist;
}

void KuRobotParameter::setKinectMinDist(int nMinDist)
{
	//��� �������� �ּ� Ž���Ÿ��� �����ϴ� �Լ�.
	m_nKinectMinDist = nMinDist;
}

void KuRobotParameter::setKinectHeight(int nHeight)
{
	//ƿƮ ������ ���� ��� ������������ ���̰��� �����ϴ� �Լ�.
	m_nKinectHeight = nHeight;
}

void KuRobotParameter::setKinectXOffset(int nXOffset)
{
	//ƿƮ ������ ���� ��� ������������ x offset�� �����ϴ� �Լ�.
	m_nKinectXOffset = nXOffset;
}

void KuRobotParameter::setKinectYOffset(int nYOffset)
{
	//ƿƮ ������ ���� ��� ������������ y offset�� �����ϴ� �Լ�.
	m_nKinectYOffset = nYOffset;
}
void KuRobotParameter::setKinectMaxHeightDist(int nMaxDist)
{
	//��� �������� �ִ� Ž���Ÿ��� �����ϴ� �Լ�.
	m_nKinectMaxHeightDist = nMaxDist;
}

void KuRobotParameter::setKinectMinHeightDist(int nMinDist)
{
	//��� �������� �ּ� Ž���Ÿ��� �����ϴ� �Լ�.
	m_nKinectMinHeightDist = nMinDist;
}

void KuRobotParameter::setCeilingCameraPrameterFx(double dFx)
{
	m_dCam_fx = dFx;
}
void KuRobotParameter::setCeilingCameraPrameterFy(double dFy)
{
	m_dCam_fy = dFy;
}
void KuRobotParameter::setCeilingCameraPrameterCx(double dCx)
{
	m_dCam_cx = dCx;
}
void KuRobotParameter::setCeilingCameraPrameterCy(double dCy)
{
	m_dCam_cy = dCy;
}
void KuRobotParameter::setCeilingCameraPrameterD1(double dD1)
{
	m_dCam_d1 = dD1;
}
void KuRobotParameter::setCeilingCameraPrameterD2(double dD2)
{
	m_dCam_d2 = dD2;
}
void KuRobotParameter::setCeilingCameraPrameterD3(double dD3)
{
	m_dCam_d3 = dD3;
}
void KuRobotParameter::setCeilingCameraPrameterD4(double dD4)
{
	m_dCam_d4 = dD4;
}
void KuRobotParameter::setCeilingCameraPrameterOffsetX(double doffsetx)
{
	m_dCam_offset_x = doffsetx;
}
void KuRobotParameter::setCeilingCameraPrameterMode(int nMode)
{
	m_nCamMode = nMode;
}
void KuRobotParameter::setCeilingCameraExposure(int nExposure)
{
	m_nCamExposure = nExposure;
}
void KuRobotParameter::setCeilingCameraAutoExposure(bool bAuto)
{
	m_bCamExposure = bAuto;
}

//���� ���ɰ� ������ parameter ���� �Լ���-------------------------------------------
void KuRobotParameter::setTargetDistance(int nDistToTarget)
{
	m_nDistToTarget=nDistToTarget;
}
void KuRobotParameter::setDesiedVel(int nDesiredVel)
{
	m_nDesiredVel=nDesiredVel;
}
void KuRobotParameter::setGoalArea(int nGoalArea)
{
	m_nGoalArea=nGoalArea;
}
void KuRobotParameter::setdKX(double  dKX)
{
	m_dKX=dKX;
}
void KuRobotParameter::setdKY(double  dKY)
{
	m_dKY=dKY;
}
void KuRobotParameter::setdKT(double  dKT)
{
	m_dKT=dKT;
}
void KuRobotParameter::setDirectionofRotation(int nDirectionofRotation)
{
	m_nDirectionofRotation=nDirectionofRotation;
}

void KuRobotParameter::setMaxParticleNum(int nMaxNum)
{
	m_nMaxParticleNum=nMaxNum;
}//�ִ� particle ������ �����ϴ� �Լ�.
void KuRobotParameter::setMinParticleNUm(int nMinNUm)
{
	m_nMinParticleNum=nMinNUm;
} //�ּ� particle ������ �����ϴ� �Լ�.
void KuRobotParameter::setDeviationforTrans(double  dDevationforTrans)
{
	m_dDevationforTrans=dDevationforTrans;
}
void KuRobotParameter::setDeviationforRotae(double  dDevationforRotate)
{
	m_dDevationforRotate=dDevationforRotate;
}
void KuRobotParameter::setDeviationforTransRotae(double  dDevationforTransRotate)
{
	m_dDevationforTransRotate=dDevationforTransRotate;
}

void KuRobotParameter::setFeatureTh(int nFeatureTh)
{
	m_nFeatureTh =nFeatureTh;
}
void KuRobotParameter::setMatchingTh(int nMatchingTh)
{
	m_nSiftMatchingTh =nMatchingTh;
}
void KuRobotParameter::setNumSIFTFeatureTh(double dNuMFeatureTh)
{
	m_dNumSIFTFeatureTh =dNuMFeatureTh;
}
void KuRobotParameter::setNumSURFFeatureTh(int nSURFKeypointTh)
{
	m_nNumSURFFeatureTh = nSURFKeypointTh;
}
void KuRobotParameter::setInteractionPointTh(int nInteractionPointTh)
{
	m_nInteractionPointTh =nInteractionPointTh;
}
void KuRobotParameter::setMatchingAngleTh(double dMatchingAngleTh)
{
	m_dMatchingAngleTh =dMatchingAngleTh;
}

void KuRobotParameter::setSIFTDataNamePath(string strSIFTDataNamePath) 
{
	//�强�� ������ �̸��� �����Ѵ�. .
	 m_strSIFTDataNamePath=strSIFTDataNamePath;
}
void KuRobotParameter::setSURFDataNamePath(string strSURFDataNamePath) 
{
	//�强�� ������ �̸��� �����Ѵ�. .
	m_strSURFDataNamePath=strSURFDataNamePath;
}
void KuRobotParameter::setDistanceFromPath(int nDistnaceFromPath)
{
	 m_nDistnaceFromPath=nDistnaceFromPath;
}


void KuRobotParameter::setEllipseWidth(double dEllipseWidth)
{
	m_dEllipseWidth =dEllipseWidth;
}
void KuRobotParameter::setEllipseHeight(double dEllipseHeight)
{
	m_dEllipseHeight =dEllipseHeight;
}


/**
@brief Korean: Land mark ������ �����ϴ� �Լ�.
@brief English: 
*/
void KuRobotParameter::setLandMarkNum(int nLandMarkNum)
{
	m_nLandMarkNum = nLandMarkNum;
}

/**
@brief Korean: ī�޶��� ��ġ�κ��� fidutial mark������ ���̸� �����ϴ� �Լ�.
@brief English: 
*/
void KuRobotParameter::setHeight_Camera2Mark(int nHeight_Camera2Mark)
{
	m_nHeight_Camera2Mark = nHeight_Camera2Mark;
}
/**
@brief Korean: �ۼ��� ������ �̸��� ���� �ּ� ���� �Լ�
@brief English: 
*/
void KuRobotParameter::setAlFeatureMapNameNPath(string sAlFeatureMapNameNPath) 
{
	m_strAlFeatureMapNameNPath=sAlFeatureMapNameNPath;
}
void KuRobotParameter::setRecognizingDistTh(double dRecognizingDistTh)
{
	m_dRecognizingDistThres = dRecognizingDistTh;
}

/**
 * @brief ISSAC ��뿩��
 * @date 2014/05/17
 * @param nUse : 0 - �̻��, 1 - ���
 * @return void
 */
void KuRobotParameter::setUsingISSAC(int nUse)
{
	m_nUseISSAC = nUse;
}

/**
 * @brief ISSAC server ip ����
 * @date 2014/05/17
 * @param strServerIP
 * @return void
 */
void KuRobotParameter::setISSACServerIP(string strServerIP)
{
	m_sISSACServerIP = strServerIP;
}

/**
 * @brief ISSAC server port ����
 * @date 2014/05/17
 * @param strServerPort
 * @return void
 */
void KuRobotParameter::setISSACServerPort(int nServerPort)
{
	m_nISSACServerPort = nServerPort;
}

/**
 * @brief ISSAC client port ����
 * @date 2014/12/08
 * @param strClientPort
 * @return void
 */
void KuRobotParameter::setISSACClientPort(int nClientPort)
{
	m_nAGVClientPort = nClientPort;
}

/**
 * @brief IoT ��뿩��
 * @date 2014/12/08
 * @param nUse : 0 - �̻��, 1 - ���
 * @return void
 */
void KuRobotParameter::setUsingIoT(int nUse)
{
	m_nUseIoT = nUse;
}

/**
 * @brief IoT server ip ����
 * @date 2014/12/08
 * @param strServerIP
 * @return void
 */
void KuRobotParameter::setIoTServerIP(string strServerIP)
{
	m_sIoTServerIP = strServerIP;
}

/**
 * @brief IoT server port ����
 * @date 2014/12/08
 * @param strServerPort
 * @return void
 */
void KuRobotParameter::setIoTServerPort(int nServerPort)
{
	m_nIoTServerPort = nServerPort;
}

/**
 * @brief ���͸� ������ �溸 ����
 * @date 2014/11/24
 * @param fVoltage
 * @return void
 */
void KuRobotParameter::setLowBatteryAlarmVoltage(float fVoltage)
{
	m_fLowBatteryAlarm = fVoltage;
}


//=============================================================================================================================


//get �Լ����--------------------------------------------------------------------------------------------------------------------------
//Robot �Ķ����---------------------------------------
int KuRobotParameter::getRobotID()
{
	//�� ���� RobotID ������ �Ѱ��ش�.
	return m_nRobotID;
}
int KuRobotParameter::getLocalization()
{
	//�� ���� RobotID ������ �Ѱ��ش�.
	return m_nLocalization;
}
int KuRobotParameter::getObstacleDetectionTime()
{
	//�� ���� RobotID ������ �Ѱ��ش�.
	return m_nObstacleDetectionTime;
}
KuPose KuRobotParameter::getInitRobotPoseForMap()
{
	return m_initRobotPosForMap;
}
int KuRobotParameter::getUsingLastPose()
{
	return m_nUsingLastPose;
}
KuPose KuRobotParameter::getInitRobotPoseForNav()
{
	return m_initRobotPosForNav;
}
int KuRobotParameter::getRobotRadius()
{
	//�κ��� �������� �Ѱ��ش�.
	return m_nRadiusofRobot;
}

int KuRobotParameter::getWheelBaseofRobot()
{
	//�κ��� �������� �Ѱ��ش�.
	return m_nWheelBaseofRobot;
}

int KuRobotParameter::getMaxRobotVelocity()
{
	//�κ��� �ִ� �ӵ��� �Ѱ��ش�.
	return m_nMaxRobotVelocity;
}
int KuRobotParameter::getMinRobotVelocity()
{
	//�κ��� �ּ� �ӵ��� �Ѱ��ش�.
	return m_nMinRobotVelocity;
}
string KuRobotParameter::getLastRobotPoseNameNPath( )
{
	return m_strLastRobotPoseNameNPath;
}
int KuRobotParameter::getMapSizeXm()
{
	//���� x,y ũ�⸦ �Ѱ��ش�.
	return m_nMapSizeXm;
}
int KuRobotParameter::getMapSizeYm()
{
	//���� x,y ũ�⸦ �Ѱ��ش�.
	return m_nMapSizeYm;
}

double KuRobotParameter::getHeight()
{
	return m_dHeight;
}
double KuRobotParameter::getSDValue()
{
	return m_dSDThres;
}
int KuRobotParameter::getUpdateValX()
{
	return m_nUpSize_X;
}
int KuRobotParameter::getUpdateValY()
{
	return m_nUpSize_Y;
}
int KuRobotParameter::getCheckValX()
{
	return m_nCheckSize_X;
}
int KuRobotParameter::getCheckValY()
{
	return m_nCheckSize_Y;
}
string KuRobotParameter::getMapNameNPath(	) 
{
	//�ۼ��� ������ ��θ� ����
	return m_strMapNameNPath;
}
string KuRobotParameter::getCadMapNameNPath( ) 
{
	//�ۼ��� ������ ��θ� ����
	return m_strCadMapNameNPath;
}
string KuRobotParameter::getZoneMapNameNPath( ) 
{
	//�ۼ��� ������ ��θ� ����
	return m_strZoneMapNameNPath;
}
string KuRobotParameter::getCeilingMapNameNPath( ) 
{
	//�ۼ��� ������ ��θ� ����
	return m_strCeilingMapNameNPath;
}
string KuRobotParameter::getVelocityMapNameNPath(	) 
{
	//�ۼ��� ������ ��θ� ����
	return m_strVelocityMapNameNPath;
}
string KuRobotParameter::getMovieNameNPath(	) 
{
	//�ۼ��� ������ ��θ� ����
	return m_strMovieNameNPath;
}
string KuRobotParameter::getTeachingPathNameNPath(	) 
{
	//�ۼ��� ������ ��θ� ����
	return m_strTeachingPathNameNPath;
}
string KuRobotParameter::getTeachingWayPointNameNPath(	) 
{
	//�ۼ��� ������ ��θ� ����
	return m_strTeachingWayPointNameNPath;
}
string KuRobotParameter::getOutlineWayPointNameNPath(	) 
{
	//�ۼ��� ������ ��θ� ����
	return m_strOutlineWayPointNameNPath;
}
string KuRobotParameter::getTempMapPath(	) 
{
	//�ۼ��� ������ ��θ� ����
	return m_strTempMapPath;
}
double KuRobotParameter::getBlockSizeX()
{
	return m_dBlockSizeX;
}
double KuRobotParameter::getBlockSizeY()
{
	return m_dBlockSizeY;
}
int KuRobotParameter::getPathNum()
{
	return m_nPathNum;
}
int KuRobotParameter::getReversePathNum()
{
	return m_nReversePathNum;
}
string KuRobotParameter::getPathteaching() 
{
	//�强�� ������ �̸��� �����Ѵ�. .
	return m_strPathteaching;
}
string KuRobotParameter::getImagePathNameNPath(	) 
{
	//�强�� ������ �̸��� �����Ѵ�. .
	return m_strImagePathNameNPath;
}
string KuRobotParameter::getDataPath( )
{
	return m_strDataPath;
}
string KuRobotParameter::getDataRecoding( )
{
	return m_strDataRecoding;
}
string KuRobotParameter::getGlobalLocalization( )
{
	return m_strGlobalLocalization;
}

string KuRobotParameter::getDataPlay( )
{
	return m_strDataPlay;
}

void KuRobotParameter::getWheelComport(char cComport[10])
{
	//Wheel comport�� �����ϴ� �Լ�.
	strcpy(cComport, m_cWheelCom); //���ڿ� ���� �Լ�.
}
int KuRobotParameter::getKuPrimusCommPort()
{
	return m_nKuPRIMUSCommPort;
}
int KuRobotParameter::getCartConnecContPort()
{
	return m_nCartConnecContPort;
}
double KuRobotParameter::getCartPortTimeOn()
{
	return m_dTimeForOn;
}
double KuRobotParameter::getCartPortTimeOFF()
{
	return m_dTimeForOff;
}
int KuRobotParameter::getDoorOpen1Port()
{
	return m_nDoorOpen1Port;
}
int KuRobotParameter::getDoorOpen2Port()
{
	return m_nDoorOpen2Port;
}
int KuRobotParameter::getLaserType()
{
	//���� x,y ũ�⸦ �Ѱ��ش�.
	return m_nVarietyLaser;
}
void KuRobotParameter::getURG04LXLaserComport(char cComport[10])
{
	//URG04LX laser comport�� �����ϴ� �Լ�.
	strcpy( cComport,m_cHokuyoURG04LXCom); //���ڿ� ���� �Լ�.
}
void KuRobotParameter::getSICKLMSFrontLaserIPAddress(char cComport[30])
{
	//�ϴ� laser IP address�� �Ѱ��ش�.
	strcpy( cComport,m_cSICKLMSFrontLaserIP); //���ڿ� ���� �Լ�.
}
void KuRobotParameter::getSICKLMSRearLaserIPAddress(char cComport[30])
{
	//�ϴ� laser IP address�� �Ѱ��ش�.
	strcpy(cComport,m_cSICKLMSRearLaserIP); //���ڿ� ���� �Լ�.
}
int KuRobotParameter::getLaserTCPPort()
{
	//���� x,y ũ�⸦ �Ѱ��ش�.
	return m_nLaserTCPPort;
}
void KuRobotParameter::getCommunicationComport(char cComport[10])
{
	//URG04LX laser comport�� �����ϴ� �Լ�.
	strcpy( cComport,m_cCommunicationCom); //���ڿ� ���� �Լ�.
}
void KuRobotParameter::getGyroComport(char cComport[10])
{
	//Gyro comport�� �����ϴ� �Լ�.
	strcpy( cComport,m_cGyroCom); //���ڿ� ���� �Լ�.
}
void KuRobotParameter::getSwitchComport(char cComport[10])
{
	//URG04LX laser comport�� �����ϴ� �Լ�.
	strcpy( cComport,m_cSwitchCom); //���ڿ� ���� �Լ�.
}
void KuRobotParameter::getZigbeeComport(char cComport[10])
{
	//Zigbee comport�� �����ϴ� �Լ�.
	strcpy( cComport,m_cZigbeeCom); //���ڿ� ���� �Լ�.
}
void KuRobotParameter::getGPIOComport(char cComport[10])
{
	//GPIO comport�� �����ϴ� �Լ�.
	strcpy( cComport,m_cGPIOCom); //���ڿ� ���� �Լ�.
}
string KuRobotParameter::getdoSonar( )
{
	return m_strdoSonar;
}

//laser ���� �Ķ����
int KuRobotParameter:: getURG04LXLaserMaxDist()
{ 
	//��� �������� �ִ� Ž���Ÿ��� �޾ƿ��� �Լ�.
	return m_nURG04LX_LaserMaxDist; 
}
int KuRobotParameter:: getURG04LXLaserMinDist()
{ 
	//��� �������� �ּ� Ž���Ÿ��� �޾ƿ��� �Լ�.
	return m_nURG04LX_LaserMinDist; 

}
int KuRobotParameter::getURG04LXLaserHeight()
{ 
	//ƿƮ ������ ���� ��� ������������ ���̰��� �޾ƿ��� �Լ�.
	return m_nURG04LX_LaserHeight; 

}
int KuRobotParameter::getFrontLaserXOffset()
{ 
	//ƿƮ ������ ���� ��� ������������ x offset�� �޾ƿ��� �Լ�.
	return m_nFrontLaserXOffset; 
}
int KuRobotParameter::getFrontLaserYOffset()
{
	//ƿƮ ������ ���� ��� ������������ y offset�� �޾ƿ��� �Լ�.
	return m_nFrontLaserYOffset; 
}
int KuRobotParameter::getRearLaserXOffset()
{ 
	//ƿƮ ������ ���� ��� ������������ x offset�� �޾ƿ��� �Լ�.
	return m_nRearLaserXOffset;
}
int KuRobotParameter::getRearLaserYOffset()
{
	//ƿƮ ������ ���� ��� ������������ y offset�� �޾ƿ��� �Լ�.
	return m_nRearLaserYOffset; 
}

//Kinect ���� �Ķ����
int KuRobotParameter::getKinectMaxDist()
{
	return m_nKinectMaxDist;
	//Ű��Ʈ�� �ִ� Ž���Ÿ��� �����ϴ� �Լ�.
}
int KuRobotParameter::getKinectMinDist( )
{
	return m_nKinectMinDist;
	//Ű��Ʈ�� �ּ� Ž���Ÿ��� �����ϴ� �Լ�.
}
int KuRobotParameter::getKinectHeight( )
{
	return m_nKinectHeight;
	//�ٴڿ��� Ű��Ʈ������ ���̰��� �����ϴ� �Լ�.
}
int KuRobotParameter::getKinectXOffset( )
{
	return m_nKinectXOffset;
	//�κ��� �߽ɿ��� Ű��Ʈ������ x offset�� �����ϴ� �Լ�.
}
int KuRobotParameter::getKinectYOffset( )
{
	return m_nKinectYOffset;
	//�κ��� �߽ɿ��� Ű��Ʈ������ y offset�� �����ϴ� �Լ�.	
}
int KuRobotParameter::getKinectMaxHeightDist()
{
	return m_nKinectMaxHeightDist;
	//Ű��Ʈ�� �ִ� Ž���Ÿ��� �����ϴ� �Լ�.
}
int KuRobotParameter::getKinectMinHeightDist( )
{
	return m_nKinectMinHeightDist;
	//Ű��Ʈ�� �ּ� Ž���Ÿ��� �����ϴ� �Լ�.
}

double KuRobotParameter::getCeilingCameraPrameterFx()
{
	return m_dCam_fx;
}
double KuRobotParameter::getCeilingCameraPrameterFy()
{
	return m_dCam_fy;
}
double KuRobotParameter::getCeilingCameraPrameterCx()
{
	return m_dCam_cx;
}
double KuRobotParameter::getCeilingCameraPrameterCy()
{
	return m_dCam_cy;
}
double KuRobotParameter::getCeilingCameraPrameterD1()
{
	return m_dCam_d1;
}
double KuRobotParameter::getCeilingCameraPrameterD2()
{
	return m_dCam_d2;
}
double KuRobotParameter::getCeilingCameraPrameterD3()
{
	return m_dCam_d3;
}
double KuRobotParameter::getCeilingCameraPrameterD4()
{
	return m_dCam_d4;
}
double KuRobotParameter::getCeilingCameraPrameterOffsetX()
{
	return m_dCam_offset_x  ;
}
int KuRobotParameter::getCeilingCameraPrameterMode()
{
	return m_nCamMode;
}
int KuRobotParameter::getCeilingCameraExposure()
{
	return m_nCamExposure;
}
bool KuRobotParameter::getCeilingCameraAutoExposure()
{
	return m_bCamExposure;
}
//���� ���ɰ� ������ parameter get �Լ�----------------------------------------------------------------

int KuRobotParameter::getTargetDistance()
{
	return m_nDistToTarget;
}
int KuRobotParameter::getDesiedVel( )
{
	return m_nDesiredVel;
}
int KuRobotParameter::getGoalArea( )
{
	return m_nGoalArea;
}
double KuRobotParameter::getdKX(  )
{
	return m_dKX;
}
double KuRobotParameter::getdKY(  )
{
	return m_dKY;
}
double KuRobotParameter::getdKT(  )
{
	return m_dKT;
}
int KuRobotParameter::getDirectionofRotation()
{
	return m_nDirectionofRotation;
}

int KuRobotParameter::getMaxParticleNum()
{
	return m_nMaxParticleNum;
}//�ִ� particle ������ �����ϴ� �Լ�.
int KuRobotParameter::getMinParticleNUm()
{
	return m_nMinParticleNum;
} //�ּ� particle ������ �����ϴ� �Լ�.
double KuRobotParameter::getDeviationforTrans(  )
{
	return m_dDevationforTrans;
}
double KuRobotParameter::getDeviationforRotate(  )
{
	return m_dDevationforRotate;
}
double KuRobotParameter::getDeviationforTransRotate(  )
{
	return m_dDevationforTransRotate;
}
int KuRobotParameter::getFeatureTh( )
{
	return m_nFeatureTh ;
}
int KuRobotParameter::getMatchingTh( )
{
	return m_nSiftMatchingTh ;
}
double KuRobotParameter::getNumSIFTFeatureTh( )
{
	return m_dNumSIFTFeatureTh ;
}
int KuRobotParameter::getNumSURFFeatureTh()
{
	return m_nNumSURFFeatureTh;
}
int KuRobotParameter::getInteractionPointTh( )
{
	return m_nInteractionPointTh ;
}
double KuRobotParameter::getMatchingAngleTh( )
{
	return  m_dMatchingAngleTh ;
}
string KuRobotParameter::getSIFTDataNamePath() 
{
	//�强�� ������ �̸��� �����Ѵ�. .
	return m_strSIFTDataNamePath;
}
string KuRobotParameter::getSURFDataNamePath() 
{
	//�强�� ������ �̸��� �����Ѵ�. .
	return m_strSURFDataNamePath;
}
int KuRobotParameter::getDistanceFromPath()
{
	return m_nDistnaceFromPath;
}

/**
@brief Korean: Land mark ������ �Ѱ��ִ� �Լ�
@brief English: 
*/
int KuRobotParameter::getLandMarkNum()
{
	return m_nLandMarkNum;
}

/**
@brief Korean: ī�޶��� ��ġ�κ��� fidutial mark������ ���̸� �Ѱ��ִ� �Լ�
@brief English: 
*/
int KuRobotParameter::getHeightCamera2Mark()
{
	return m_nHeight_Camera2Mark;
}

double KuRobotParameter::getRecognizingDistTh()
{
	return m_dRecognizingDistThres;
}
/**
@brief Korean: �ۼ��� ������ �̸��� ���� �ּ� ���� �Լ�
@brief English: 
*/
string KuRobotParameter::getAlFeatureMapNameNPath() 
{
	return m_strAlFeatureMapNameNPath;
}

double KuRobotParameter::getEllipseWidth( )
{
	return  m_dEllipseWidth ;
}
double KuRobotParameter::getEllipseHeight( )
{
	return  m_dEllipseHeight;
}

/**
 * @brief ISSAC ��뿩�� ���� (0: �̻��, 1: ���)
 * @date 2014/05/17
 * @param 
 * @return int
 */
int KuRobotParameter::getUsingISSAC(void)
{
	return m_nUseISSAC;
}

/**
 * @brief ISSAC server ip ����
 * @date 2014/05/17
 * @param 
 * @return cv::string
 */
string KuRobotParameter::getISSACServerIP(void)
{
	return m_sISSACServerIP;
}

/**
 * @brief ISSAC server port ����
 * @date 2014/05/17
 * @param 
 * @return int
 */
int KuRobotParameter::getISSACServerPort(void)
{
	return m_nISSACServerPort;
}

/**
 * @brief IoT ��뿩�� ���� (0: �̻��, 1: ���)
 * @date 2014/12/08
 * @param 
 * @return int
 */
int KuRobotParameter::getUsingIoT(void)
{
	return m_nUseIoT;
}

/**
 * @brief IoT server ip ����
 * @date 2014/12/08
 * @param 
 * @return cv::string
 */
string KuRobotParameter::getIoTServerIP(void)
{
	return m_sIoTServerIP;
}

/**
 * @brief IoT server port ����
 * @date 2014/12/08
 * @param 
 * @return int
 */
int KuRobotParameter::getIoTServerPort(void)
{
	return m_nIoTServerPort;
}

/**
 * @brief ISSAC client port ����
 * @date 2014/05/17
 * @param 
 * @return int
 */
int KuRobotParameter::getISSACClientPort(void)
{
	return m_nAGVClientPort;
}

/**
 * @brief ���͸� ������ �溸�� ���� �� ����
 * @date 2014/11/24
 * @return float
 */
float KuRobotParameter::getLowBatteryAlarmVoltage(void)
{
	return m_fLowBatteryAlarm;
}

/**
 * @brief SV laser scanner IP �ּ� ����
 * @date 2014/12/19
 * @return std::string
 */
std::string KuRobotParameter::getSVLaserIP(void)
{
	return m_sSVLaserIP;
}

/**
 * @brief SV laser scanner ��� ��Ʈ ����
 * @date 2014/12/19
 * @return int
 */
int KuRobotParameter::getSVLaserPort(void)
{
	return m_nSVLaserPort;
}
//======================================================================================================================================

