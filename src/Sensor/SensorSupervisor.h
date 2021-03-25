#ifndef C_SENSOR_SUPERVISOR_H
#define C_SENSOR_SUPERVISOR_H


#include <cv.h>
#include <highgui.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include<iostream>
#include<fstream>
#include "../KUNSPose/KuPose.h"
#include "Sensor.h"
#include "../KUNSUtil/KuUtil.h"
#include "../KUNSUtil/KUNSSingletone/KuSingletone.h"
#include "./HokuyoURG04LXInterface/HokuyoURG04LXInterface.h" //팬틸트 레이저
#include "./SiriusCameraInterface/KuSiriusCameraInterface.h"
#include "./E2BoxIMU9DOFInterface/E2BoxIMU9DOFInterface.h"
#include "./WheelActuatorInterface./SSAGVWheelActuatorInterface.h"
#include "./GyroSensorInterface/GyroSensorInterface.h"
#include "./SwitchInterface/SwitchInterface.h"
#include "../KUNSUtil/KUNSINIReadWriter/KuINIReadWriter.h"
#include "../MobileSupervisor/KuRobotParameter.h"
#include "./SickLMS100Interface/SickLMS100Interface.h"
#include "./SickLMS100Interface/SickTIM551Interface.h"
#include "./SVLaserScannerInterface/SVLaserScannerInterface.h"
using namespace std;
using namespace cv;

#define USE_KINECT_VER_1 // use Kinect for windows ver. 1

#ifdef USE_KINECT_VER_1
#include "KinectSensorInterface/Kinect.h"
#else
#include "./KinectSensorInterface/KinectSensorInterface.h"
#endif

class SensorSupervisor: public KuSingletone <SensorSupervisor>
{
	
	static const int HOKUYO = 0;
	static const int SickLMS = 1;
	static const int SickTIM = 2;
	static const int SV = 3;

private:
	KuUtil m_KuUtil;
private:
	int m_CeilingCamLogIdx;
	int m_KinectLogIdx;

	//sensor data
	SickLMS100Interface m_sick_front;
	SickLMS100Interface m_sick_rear;
	KuPose m_EncoderDelPos;
	KuPose m_tempEncoderDelPos;
	int_1DArray m_nLaserDataFront; //laser data;
	int_1DArray m_nLaserDataRear; //laser data;
	int_1DArray m_nTempLaserDataFront; //laser data
	int_1DArray m_nTempLaserDataRear; //laser data
	int_1DArray m_nKinectRangeData;	
	int_1DArray m_nHeightDataOfRangeData;
	int_1DArray m_nObsState;

	IplImage* m_IplKinectImage;
	IplImage* m_IplKinectDepthImg;
	KuPose* m_pKinectDataPos; 

	double m_dGyroData;
	IplImage* m_IpldisparityImg;	
	IplImage* m_IplImage;

	bool m_bCeilingCamUse; // 천장카메라
	IplImage* m_IplCeilingImage;

	Mat m_cvMatImage;

	bool m_bDataRecoding;
	bool m_bDataPlay;
	ofstream m_DataLog;
	ifstream m_PlayLog;

	double 	*m_d3DKinectX;
	double 	*m_d3DKinectY;
	double  *m_d3DKinectZ;
	int* m_n3DKinectPixX;
	int* m_n3DKinectPixY;
	int* m_n3DKinectDataValidation;
	float	*m_f3DX;
	float	*m_f3DY;
	float	*m_f3DZ;
	bool	*m_bFlag3D;
	float* m_fKinectDistanceImage;

	// connection status
	bool m_bRobotConnected;
	bool m_bCameraConnected;
	bool m_bFrontLRFConnected;
	bool m_bRearLRFConnected;
	bool m_bKinectConnected;
	bool m_bGyroConnected;
	bool m_bButtonBoxConnected;
	bool m_bISSACConnected;
	bool m_bGPIOConnected;
	bool m_bZigbeeConnected;
private:
	void init();

	void dataParsing();

	void recordEncoderDelPosData(KuPose EncoderDelPos);

	KuPose playEncoderDelPosData();

	void recordGyroData(double dGyroData);

	void playGyroData(double* dGyroData);
	void recordLaserData(int_1DArray  nLaserData);
	void playLaserData(int_1DArray  nData);
	void recordKinectRangeData(int_1DArray nKinectRangeData);
	void recordHeightDataOfKinectRangeData(int_1DArray nHightDataOfKinectRangeData);
	void recordKinectData(IplImage* IplKinectImage, IplImage* IplKinectDepthImg, KuPose* pKinectDataPos);

	bool playKinectData();
	void PlayHeightDataOfKinectRangeData(int_1DArray nHightDataOfKinectRangeData);
	void PlayKinectRangeData(int_1DArray nKinectRangeData);
	void recordCeilingCamData(IplImage* CeilingImageData);
	void playCeilingCamData();

	void startTimeCheck(LARGE_INTEGER& nStart);
	float finishTimeCheck(const LARGE_INTEGER nStart);

	inline bool getKinectV1Data(int_1DArray& nKinectRangeData, int_1DArray& nHeightDataOfRangeData);

public:
	//센서의 연결을 모두 종료하는 부분
	void stopAllSensor();

	bool connectionSensor();

	//센서 값 저장을 종료하는 부분
	void completeDataRecording();

	//센서의 데이터를 읽는 부분
	bool readSensorData();

	//엔코더를 받아옴
	KuPose getEncoderDelPos();
	//레이저의 정보를 받아옴
	int_1DArray getLaserDataFront();
	int_1DArray getLaserDataRear();
	bool isRobotConnected(void);
	bool isCameraConnected(void);
	bool isFrontLRFConnected(void);
	bool isRearLRFConnected(void);
	bool isKinectConnected(void);
	bool isGyroConnected(void);
	bool isButtonBoxConnected(void);
	bool isISSACConnected(void);
	bool isGPIOConnected(void);
	bool isZigbeeConnected(void);
	//키넥트의 2D 거리 값을 받아옴
	int_1DArray getKinectRangeData();
	//키넥트의 2D 거리값에 대한 높이 값을 받아옴
	int_1DArray getHeightDataOfKinectRangeData();
	//키넥트의 이미지 값을 받아옴
	IplImage* getKinectImageData();
	//키넥트의  뎁스 이미지 값을 받아옴
	IplImage* getKinectDepthImageData();
	//키넥트의 3D  값의 거리 값을 받아옴
	KuPose* getKinectDataPos();
	//자이로센서의 값을 받아옴
	double getGyroData();
	int_1DArray selLaserData();

	//천장카메라의 이미지 값을 받아옴
	IplImage* getCeilingImageData();

	int_1DArray getObsData();

	float* getKinectDistanceImage();

	int getSwitchState();

	bool readOnlySensorData();
	void DataRecodingNPlay();
	bool connectLaserScanner();
	bool connectionKinect();
	bool connectionCeilingcamera();
	bool connectionGYRO();
	bool connectionWheelactuator();
	bool connectSwitch();

	bool executeSwitch( );
	void  initLampflag( );

	SensorSupervisor();
	~SensorSupervisor();

};

#endif