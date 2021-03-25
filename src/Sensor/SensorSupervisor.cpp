#include <stdafx.h>
#include "SensorSupervisor.h"

SensorSupervisor::SensorSupervisor()
{
	HokuyoURG04LXInterface::getInstance();

#ifdef USE_KINECT_VER_1
	CKinect::getInstance();
#else
	KinectSensorInterface::getInstance();
#endif

	KuSiriusCameraInterface::getInstance();
	init();
}

SensorSupervisor::~SensorSupervisor()
{
	if(m_IplCeilingImage!=NULL)
		cvReleaseImage(&m_IplCeilingImage);
	if(m_IplKinectImage!=NULL)
		cvReleaseImage(&m_IplKinectImage);

	if(m_f3DX!=NULL)
		delete [] m_f3DX;
	if(m_f3DY!=NULL)
		delete [] m_f3DY;
	if(m_f3DZ!=NULL)
		delete [] m_f3DZ;
	if(m_bFlag3D!=NULL)
		delete [] m_bFlag3D;
	if(m_pKinectDataPos!=NULL)
		delete [] m_pKinectDataPos;	
	if(m_d3DKinectX!=NULL)
		delete [] m_d3DKinectX;
	if(m_d3DKinectY!=NULL)
		delete [] m_d3DKinectY;
	if(m_d3DKinectZ!=NULL)
		delete [] m_d3DKinectZ;
	if(m_n3DKinectPixX!=NULL)
		delete [] m_n3DKinectPixX;
	if(m_n3DKinectPixY!=NULL)
		delete [] m_n3DKinectPixY;
	if(m_n3DKinectDataValidation!=NULL)
		delete [] m_n3DKinectDataValidation;
}

/**
@brief Korean: ����Ÿ�� �ʱ�ȭ
@brief English: write in English
*/
void SensorSupervisor::init()
{
	m_bRobotConnected = false;
	m_bCameraConnected = false;
	m_bFrontLRFConnected = false;
	m_bRearLRFConnected = false;
	m_bKinectConnected = false;
	m_bGyroConnected = false;
	m_bButtonBoxConnected = false;
	m_bISSACConnected = false;
	m_bGPIOConnected = false;
	m_bZigbeeConnected = false;

	m_nLaserDataFront=m_KuUtil.generateIntType1DArray(Sensor::URG04LX_DATA_NUM181,0);
	m_nLaserDataRear=m_KuUtil.generateIntType1DArray(Sensor::URG04LX_DATA_NUM181,0);
	m_nKinectRangeData=m_KuUtil.generateIntType1DArray(Sensor::KINECT_SENSOR_FOV,0);
	m_nHeightDataOfRangeData=m_KuUtil.generateIntType1DArray(Sensor::KINECT_SENSOR_FOV,0);
	m_IplCeilingImage= cvCreateImage(cvSize(Sensor::CEILING_IMAGE_WIDTH,Sensor::CEILING_IMAGE_HEIGHT),8,1);
	m_IplKinectImage = cvCreateImage(cvSize(Sensor::CEILING_IMAGE_WIDTH,Sensor::CEILING_IMAGE_HEIGHT),8,3);
	m_nObsState = m_KuUtil.generateIntType1DArray(Sensor::SONAR_NUM,0); //�Ÿ����� ������ �����ϴ� ���� �ʱ�ȭ
	m_nTempLaserDataFront=m_KuUtil.generateIntType1DArray(Sensor::URG04LX_DATA_NUM181,0);
	m_nTempLaserDataRear=m_KuUtil.generateIntType1DArray(Sensor::URG04LX_DATA_NUM181,0);

	for(int i = 0; i < Sensor::URG04LX_DATA_NUM181; i++)
	{
		m_nTempLaserDataRear[i] = 0;
	}


	//Ű��Ʈ ���� ������ �ʱ�ȭ

	m_IplKinectDepthImg = NULL;
	m_d3DKinectX = new double[Sensor::IMAGE_WIDTH*Sensor::IMAGE_HEIGHT];
	m_d3DKinectY = new double[Sensor::IMAGE_WIDTH*Sensor::IMAGE_HEIGHT];
	m_d3DKinectZ = new double[Sensor::IMAGE_WIDTH*Sensor::IMAGE_HEIGHT];
	m_n3DKinectPixX = new int[Sensor::IMAGE_WIDTH*Sensor::IMAGE_HEIGHT];
	m_n3DKinectPixY = new int[Sensor::IMAGE_WIDTH*Sensor::IMAGE_HEIGHT];
	m_n3DKinectDataValidation = new int[Sensor::IMAGE_WIDTH*Sensor::IMAGE_HEIGHT];
	m_KinectLogIdx = 0;
	m_pKinectDataPos = new KuPose[Sensor::IMAGE_WIDTH*Sensor::IMAGE_HEIGHT];

	//���� �������ʱ�ȭ
	m_f3DX = new float[Sensor::IMAGE_WIDTH*Sensor::IMAGE_HEIGHT];
	m_f3DY = new float[Sensor::IMAGE_WIDTH*Sensor::IMAGE_HEIGHT];
	m_f3DZ = new float[Sensor::IMAGE_WIDTH*Sensor::IMAGE_HEIGHT];
	m_bFlag3D = new bool[Sensor::IMAGE_WIDTH*Sensor::IMAGE_HEIGHT];
	m_fKinectDistanceImage = new float[Sensor::IMAGE_WIDTH*Sensor::IMAGE_HEIGHT];

	m_KinectLogIdx = 0;
	m_bCeilingCamUse = false;
	m_CeilingCamLogIdx =0;
	m_bDataRecoding=false;
	m_bDataPlay=false;
	m_EncoderDelPos.init();
	m_tempEncoderDelPos.init();
}
/**
@brief Korean: ������ ���� �����͸� Ȱ�뿩�θ� �����ϴ� �Լ�
@brief English: write in English
*/
void SensorSupervisor::DataRecodingNPlay()
{

	string strDataPath =KuRobotParameter::getInstance()->getDataPath();
	string strDataRecoding =KuRobotParameter::getInstance()->getDataRecoding();

	if( strDataRecoding=="yes" ){ //������ ���ڵ��� �����ϴ� ����̴�.

		m_bDataRecoding = true;
		m_DataLog.open(strDataPath);

	}
	else if(strDataRecoding=="no" ){ //������ ���ڵ��� �������� �ʴ� ����̴�.

		string strDataPlay = KuRobotParameter::getInstance()->getDataPlay();

		if(strDataPlay=="yes" ){ //���ڵ��� �����͸� �о�� play�ϴ� ����̴�.
			m_bDataPlay = true;
			m_PlayLog.open(strDataPath);
		}
	}
}
bool SensorSupervisor::connectionSensor()
{


	while(!connectLaserScanner())
	{
		Sleep(200);
	}
	printf("Success Laser!!\n");
	while(!connectionKinect())
	{
		Sleep(200);
	}
	printf("Success Kinect!!\n");

	while(!connectionCeilingcamera())
	{
		Sleep(200);
	}
	printf("Success Ceiling!!\n");

	while(!connectionGYRO())
	{
		Sleep(200);
	}
	printf("Success GYRO!!\n");

	while(!connectionWheelactuator())
	{
		Sleep(200);
	}
	printf("Success Wheelactuator!!\n");

	while(!connectSwitch())
	{
		Sleep(200);
	}
	printf("Success SwitchInterface!!\n");

	return true;
}

/**
@brief Korean: ������������ �����ϴ� �Լ�
@brief English: write in English
*/
bool SensorSupervisor::connectLaserScanner()
{
	bool bConnectionResultFront(false);
	bool bConnectionResultRear(false);
	int LowerLaserConnectionPort;
	int nLaserType = KuRobotParameter::getInstance()->getLaserType();
	char cSICKLMSFrontLaserIP[30]; //�ϴ� ������ ���� IP address�� ������ ����.
	char cSICKLMSRearLaserIP[30];

	switch(nLaserType)
	{	

	case HOKUYO:
		char  HokuyoURGCom[10];
		KuRobotParameter::getInstance()->getURG04LXLaserComport(HokuyoURGCom);	
		HokuyoURG04LXInterface::getInstance()->setComPort(HokuyoURGCom);	
		while(!HokuyoURG04LXInterface::getInstance()->connectLaserScanner())
		{
			Sleep(200);
		}
		return true;
		break;
	case SickLMS:
		KuRobotParameter::getInstance()->getSICKLMSFrontLaserIPAddress(cSICKLMSFrontLaserIP);
		KuRobotParameter::getInstance()->getSICKLMSRearLaserIPAddress(cSICKLMSRearLaserIP);
		LowerLaserConnectionPort = KuRobotParameter::getInstance()->getLaserTCPPort();
		bConnectionResultFront = m_sick_front.connect(cSICKLMSFrontLaserIP, LowerLaserConnectionPort);
//		bConnectionResultRear = m_sick_rear.connect(cSICKLMSRearLaserIP, LowerLaserConnectionPort);

		if(bConnectionResultRear == TRUE)
		{
			cout<<"Rear LRF is connected. Server IP :"<< (cSICKLMSRearLaserIP)<<" TCP port :"<< LowerLaserConnectionPort<<endl;
			m_bRearLRFConnected = true;
		}
		else
		{
			m_bRearLRFConnected = false;
		}

		if(TRUE == bConnectionResultFront){ //connection�� ���������� �̷���� ���.
			cout<<"Front LRF is connected. Server IP :"<< (cSICKLMSFrontLaserIP)<<" TCP port :"<< LowerLaserConnectionPort<<endl;
			cout<<"Starting LRF setting procedures."<<endl;

			m_bFrontLRFConnected = true;

			return true;		
		}
		else{
			cout<<"Connection error: Unable to connect the front LRF. Please check the IP address."<<endl;

			m_bFrontLRFConnected = false;

			return false;
		}

		break;
	case SickTIM:
		KuRobotParameter::getInstance()->getSICKLMSFrontLaserIPAddress(cSICKLMSFrontLaserIP);
		LowerLaserConnectionPort = KuRobotParameter::getInstance()->getLaserTCPPort();
		bConnectionResultFront = SickTIM551Interface::getInstance()->connect(cSICKLMSFrontLaserIP, LowerLaserConnectionPort);

		if(TRUE == bConnectionResultFront){ //connection�� ���������� �̷���� ���.
			cout<<"LRF is opened. server IP :"<< (cSICKLMSFrontLaserIP)<<" TCP port :"<< LowerLaserConnectionPort<<endl;
			cout<<"LRF setting was started...."<<endl;
			return true;		
		}
		else{
			cout<<"Connection error: LRF Cannot be connected. check address..."<<endl;
			return false;
		}
		break;

	case SV:
		char SVLaserScannerCom[] = "COM3";
		//KuRobotParameter::getInstance()->getSVLaserScannerComport(SVLaserScannerCom);	
		bConnectionResultFront = CSVLaserScannerInterface::getInstance()->connect(SVLaserScannerCom);

		if(bConnectionResultFront)
		{
			cout << "SV laser scanner is connected (" << SVLaserScannerCom << ")" << endl;
			return true;
		}
		else
		{
			cout << "Unable to connect SV laser scanner." << endl;
			return false;
		}

		break;
	}
}
/**
@brief Korean: Ű��Ʈ ������ �����ϴ� �Լ�
@brief English: write in English
*/
bool SensorSupervisor::connectionKinect()
{
#ifdef USE_KINECT_VER_1
	if(!CKinect::getInstance()->init())
	{
		printf("Unable to connect Kinect sensor.\n");

		Sleep(3000);

		return false;
	}
#else
	while(!KinectSensorInterface::getInstance()->connect())
	{
		Sleep(200);
	}

	KinectSensorInterface::getInstance()->start();
#endif

	m_bKinectConnected = true;

	return true;	

}
/**
@brief Korean: õ�� ī�޶� �����ϴ� �Լ�
@brief English: write in English
*/
bool SensorSupervisor::connectionCeilingcamera()
{
	if(KuSiriusCameraInterface::getInstance()->connect())	
	{		
		m_bCameraConnected = true;

		return true;
	}
 	return false;	

}
/**
@brief Korean: ���̷� ������ �����ϴ� �Լ�
@brief English: write in English
*/
bool SensorSupervisor::connectionGYRO()
{		
//	connectionSwitchInterface();

	char  GYROCom[10];
	KuRobotParameter::getInstance()->getGyroComport(GYROCom);	
	if(GyroSensorInterface::getInstance()->connect(GYROCom))
	{
		m_bGyroConnected = true;

		return true;
	}

	return false;
}
/**
@brief Korean: ���ڴ���  �����ϴ� �Լ�
@brief English: write in English
*/
bool SensorSupervisor::connectionWheelactuator()
{
	char  WheelCom[10];
	KuRobotParameter::getInstance()->getWheelComport(WheelCom);	
	if(SSAGVWheelActuatorInterface::getInstance()->connect(WheelCom))
	{
		m_bRobotConnected = true;

		return true;
	}
	return false;
}


/**
@brief Korean: 
@brief English: write in English
*/
bool SensorSupervisor::connectSwitch()
{
	char  SwitchCom[10];
	KuRobotParameter::getInstance()->getSwitchComport(SwitchCom);	
	if(SwitchInterface::getInstance()->connect(SwitchCom))
	{
		SwitchInterface::getInstance()->execute( );

		m_bButtonBoxConnected = true;

		return true;
	}
	return false;
}
/**
@brief Korean: ��� ������ ������
@brief English: write in English
*/
void SensorSupervisor::stopAllSensor()
{
#ifdef USE_KINECT_VER_1
	CKinect::getInstance()->close_connection();
#else
	KinectSensorInterface::getInstance()->terminate();
#endif
	KuSiriusCameraInterface::getInstance()->disconnection();
	HokuyoURG04LXInterface::getInstance()->disconnectLaserScanner();
	m_sick_front.disconnectLaserScanner();
	m_sick_rear.disconnectLaserScanner();
	CSVLaserScannerInterface::getInstance()->disconnect();
}
/**
@brief Korean: ���ڴ� �����͸� ������
@brief English: write in English
*/
KuPose SensorSupervisor::getEncoderDelPos()
{
	return m_EncoderDelPos;
}
/**
@brief Korean: ���̷� �����͸� ������
@brief English: write in English
*/
double SensorSupervisor::getGyroData()
{
	return m_dGyroData;
}
/**
@brief Korean: �������� �Ÿ��� �����͸� ������
@brief English: write in English
*/
int_1DArray SensorSupervisor::getLaserDataFront()
{
	return m_nLaserDataFront;
}

/**
@brief Korean: �������� �Ÿ��� �����͸� ������
@brief English: write in English
*/
int_1DArray SensorSupervisor::getLaserDataRear()
{
	return m_nLaserDataRear;
}

/**
@brief Korean: �Ĺ� ������ ���� ����
@brief English: write in English
*/
bool SensorSupervisor::isRobotConnected(void)
{
	return m_bRobotConnected;
}

bool SensorSupervisor::isCameraConnected(void)
{
	return m_bCameraConnected;
}

bool SensorSupervisor::isFrontLRFConnected(void)
{
	return m_bFrontLRFConnected;
}

bool SensorSupervisor::isRearLRFConnected(void)
{
	return m_bRearLRFConnected;
}

bool SensorSupervisor::isKinectConnected(void)
{
	return m_bKinectConnected;
}

bool SensorSupervisor::isGyroConnected(void)
{
	return m_bGyroConnected;
}

bool SensorSupervisor::isButtonBoxConnected(void)
{
	return m_bButtonBoxConnected;
}

bool SensorSupervisor::isISSACConnected(void)
{
	return m_bISSACConnected;
}

bool SensorSupervisor::isGPIOConnected(void)
{
	return m_bGPIOConnected;
}

bool SensorSupervisor::isZigbeeConnected(void)
{
	return m_bZigbeeConnected;
}

/**
@brief Korean: Ű��Ʈ�� 2D �Ÿ��� �����͸� ������
@brief English: write in English
*/
int_1DArray SensorSupervisor::getKinectRangeData()
{
	return m_nKinectRangeData;
}
/**
@brief Korean: Ű��Ʈ�� 2D �Ÿ����� ���� ���� �����͸� ������
@brief English: write in English
*/
int_1DArray SensorSupervisor::getHeightDataOfKinectRangeData()
{
	return m_nHeightDataOfRangeData;
}
/**
@brief Korean: Ű��Ʈ�� ���� �����͸� ������
@brief English: write in English
*/
IplImage* SensorSupervisor::getKinectImageData()
{
	return m_IplKinectImage;
}
/**
@brief Korean: Ű��Ʈ�� ���� �����͸� ������
@brief English: write in English
*/
IplImage* SensorSupervisor::getKinectDepthImageData()
{
	return m_IplKinectDepthImg;
}
/**
@brief Korean: Ű��Ʈ�� 3D �Ÿ� �����͸� ������
@brief English: write in English
*/
KuPose* SensorSupervisor::getKinectDataPos()
{
	return m_pKinectDataPos;
}
/**
@brief Korean: Ű��Ʈ�� 3D �Ÿ� �����͸� ������
@brief English: write in English
*/
float* SensorSupervisor::getKinectDistanceImage()
{
	return m_fKinectDistanceImage;
}
/**
@brief Korean: õ�� ���� ������ ������
@brief English: write in English
*/
IplImage* SensorSupervisor::getCeilingImageData()
{
	return m_IplCeilingImage;
}
/**
@brief Korean: 
@brief English: write in English
*/
int SensorSupervisor::getSwitchState( )
{
	return SwitchInterface::getInstance()->getState();

}
int_1DArray SensorSupervisor::getObsData()
{
	int_1DArray nObsState;
	nObsState=SwitchInterface::getInstance()->getObsData();

	for(int i=0; i<Sensor::SONAR_NUM;i++)
	{
		m_nObsState[i]=nObsState[i];
	}
	return m_nObsState;
}

/**
@brief Korean: 
@brief English: write in English
*/
bool SensorSupervisor::executeSwitch( )
{
	return SwitchInterface::getInstance()->execute();
}	
/**
@brief Korean: 
@brief English: write in English
*/
void  SensorSupervisor::initLampflag( )
{
	SwitchInterface::getInstance()->initLampflag();
}

/**
@brief Korean: �ҿ� �ð��� �����ϱ� ���ؼ� �ʱ�ȭ�ϴ� �Լ�
@brief English: Initializes to count the duration
*/
void SensorSupervisor::startTimeCheck(LARGE_INTEGER& nStart)
{
	QueryPerformanceCounter(&nStart);
}
/**
@brief Korean: ������ �ҿ� �ð��� �޾ƿ��� �Լ�
@brief English: Gets the estimated duration
*/
float SensorSupervisor::finishTimeCheck(const LARGE_INTEGER nStart)
{
	LARGE_INTEGER nFinish, freq;

	QueryPerformanceFrequency(&freq);

	QueryPerformanceCounter(&nFinish);

	return (float)(1000.0 * (nFinish.QuadPart - nStart.QuadPart) / freq.QuadPart);
}

int_1DArray SensorSupervisor::selLaserData()
{
	int nVarietyLaser=KuRobotParameter::getInstance()->getLaserType();
	int_1DArray nLaserDataFront, nLaserDataRear;

	switch(nVarietyLaser)
	{	
	case HOKUYO:
		 nLaserDataFront = HokuyoURG04LXInterface::getInstance()->getData();
		for(int i=0; i<Sensor::URG04LX_DATA_NUM181;i++){m_nTempLaserDataFront[i] = nLaserDataFront[i];}
		break;
	case SickLMS:
//		nLaserDataFront = SickLMS100Interface::getInstance()->getData( );
		nLaserDataFront = m_sick_front.getData( );

		for(int i=0; i<Sensor::URG04LX_DATA_NUM181;i++){m_nTempLaserDataFront[i] = nLaserDataFront[i];}

		if(m_bRearLRFConnected)
		{
			nLaserDataRear = m_sick_rear.getData();

			for(int i=0; i<Sensor::URG04LX_DATA_NUM181;i++){m_nTempLaserDataRear[i] = nLaserDataRear[i];}
		}
		
		break;
	case SickTIM:
		nLaserDataFront =SickTIM551Interface::getInstance()->getData( );
		for(int i=0; i<Sensor::URG04LX_DATA_NUM181;i++){m_nTempLaserDataFront[i] = nLaserDataFront[i];}
		break;
	case SV:
		nLaserDataFront = CSVLaserScannerInterface::getInstance()->get_data( );
		for(int i=0; i<Sensor::URG04LX_DATA_NUM181;i++){m_nTempLaserDataFront[i] = nLaserDataFront[i];}
		break;
	}
	return m_nTempLaserDataFront;
}

/**
@brief Korean: ��� ������ ���� ���� ������ �޾ƿ�
@brief English: write in English
*/
bool SensorSupervisor::readSensorData()
{	
	if(m_bDataPlay == true && m_PlayLog.eof()==true){ 
		return false; 
	} //�ùķ��̼� �����͸� �� �о���.


	//���ڴ� ������ ������ ����----------------------------------------------------
	if( m_bDataPlay == false){ //���� ����Ÿ ���

		m_EncoderDelPos= SSAGVWheelActuatorInterface::getInstance()->getDelEncoderData();

		if(m_bDataRecoding == true){ //���� �����͸� ���ڵ��Ѵ�.
			recordEncoderDelPosData(m_EncoderDelPos);
		}
	}
	else{
		//���ڵ� �� �����͸� ���Ϸκ��� �о�ͼ� �����Ѵ�.
		m_EncoderDelPos = playEncoderDelPosData();
	}
	//-------------------------------------------------------------------------------��


	//���̷� ���� ������ ������ ����--------------------------------------------
	if( m_bDataPlay == false){ //���� ����Ÿ ���
		//m_dGyroData=E2BoxIMU9DOFInterface::getInstance()->getThetaDeg();
		m_dGyroData=GyroSensorInterface::getInstance()->getThetaDeg();

		if(m_bDataRecoding == true){ //���� �����͸� ���ڵ��Ѵ�.
			recordGyroData(m_dGyroData);
		}
	}
	else{  //���ڵ� �� �����͸� ���Ϸκ��� �о�ͼ� �÷����Ѵ�..
		playGyroData(&m_dGyroData);
	}
	//-------------------------------------------------------------------------------��


	//������ ������ ������ ����----------------------------------------------------
	if( m_bDataPlay == false){ //���� ����Ÿ ���
		int_1DArray nLaserDataFront = selLaserData();//HokuyoURG04LXInterface::getInstance()->getData();

		for(int i=0; i<Sensor::URG04LX_DATA_NUM181;i++){
			m_nLaserDataFront[i] = nLaserDataFront[i]; //������ ������ ����	
			m_nLaserDataRear[i] = m_nTempLaserDataRear[i];
		}
		if(m_bDataRecoding == true){ //���� �����͸� ���ڵ��Ѵ�.
			recordLaserData(m_nLaserDataFront);
		}

	}
	else{ //���ڵ� �� �����͸� ���Ϸκ��� �о�ͼ� �����Ѵ�.
		playLaserData(m_nLaserDataFront);	
	}
	//������ ������ ������ ����----------------------------------------------------��

	//kinect ���� ������ ������ ����--------------------------------------------------
	if( m_bDataPlay == false  ){ //���� ����Ÿ ���

#ifdef USE_KINECT_VER_1
		int_1DArray nKinectRangeData = m_KuUtil.generateIntType1DArray(Sensor::KINECT_SENSOR_FOV, 1000000);
		int_1DArray nHeightDataOfRangeData = m_KuUtil.generateIntType1DArray(Sensor::KINECT_SENSOR_FOV, 1000000);

// 		m_IplKinectImage = KinectSensorInterface::getInstance()->get320ColorImage();
		getKinectV1Data(nKinectRangeData, nHeightDataOfRangeData);
#else
 		m_IplKinectImage = KinectSensorInterface::getInstance()->get320ColorImage();
		m_pKinectDataPos = KinectSensorInterface::getInstance()->getGlobal3DPose();
		int_1DArray nKinectRangeData = KinectSensorInterface::getInstance()->getRangeData();
		int_1DArray nHeightDataOfRangeData = KinectSensorInterface::getInstance()->getHeightDataOfRangeData(); //�Ÿ����� ���� ���̰� 
#endif

		for(int i=0; i<Sensor::KINECT_SENSOR_FOV;i++){
			m_nKinectRangeData[i] = nKinectRangeData[i]; //kinect range data ����
		}

		for(int i=0; i<Sensor::KINECT_SENSOR_FOV;i++){
			m_nHeightDataOfRangeData[i] = nHeightDataOfRangeData[i]; //�Ÿ����� ���� ���̰� ����
		}

		if(m_bDataRecoding == true){ //���� �����͸� ���ڵ��Ѵ�.
			if(m_IplKinectImage!=NULL)
			{
			//	recordKinectData(m_IplKinectImage, m_IplKinectDepthImg, m_pKinectDataPos);
			//	recordKinectRangeData(m_nKinectRangeData);
			//	recordHightDataOfKinectRangeData(m_nHeightDataOfRangeData);
			}
		}
	}
	else{
		if(playKinectData())
		{
			PlayKinectRangeData(m_nKinectRangeData);
			PlayHeightDataOfKinectRangeData(m_nHeightDataOfRangeData);
		}
	}
	//kinect ���� ������ ������ ����------------------------------------------------------------��

	//ī�޶� ������ ������ ����----------------------------------------------------
	if( m_bDataPlay == false ){ //���� ����Ÿ ���

		m_IplCeilingImage = KuSiriusCameraInterface::getInstance()->calcUndistortedImage();

		//	cvFlip(m_IplCeilingImage,m_IplCeilingImage,-1);
		cvFlip(m_IplCeilingImage,m_IplCeilingImage,1);

		if(m_bDataRecoding == true){ //���� �����͸� ���ڵ��Ѵ�.
			recordCeilingCamData(m_IplCeilingImage);
		}
	}
	else{
		playCeilingCamData();
	}
	//ī�޶� ������ ������ ����----------------------------------------------------��


	return true;
}

bool SensorSupervisor::getKinectV1Data(int_1DArray& nKinectRangeData, int_1DArray& nHeightDataOfRangeData)
{
#ifdef USE_KINECT_VER_1
	CANSArray2D<CANSPoint3D>* p3DPoints = CKinect::getInstance()->get_depth_data();
	const int nMaxDist=KuRobotParameter::getInstance()->getKinectMaxDist(); // mm
	const int nMinDist=KuRobotParameter::getInstance()->getKinectMinDist(); // mm
	const int nMaxHeight=KuRobotParameter::getInstance()->getKinectMaxHeightDist(); // mm
	const int nMinHeight=KuRobotParameter::getInstance()->getKinectMinHeightDist(); // mm

	if(p3DPoints) // not empty
	{
		for(int i = 0; i < CKinect::getInstance()->get_parameters().depth_width; i++)
		{
			int nAngle = -1;
			float fMinDistX = 999999; // max. value
			float fHeight(0);

			for(int j = 0; j < CKinect::getInstance()->get_parameters().depth_height; j++)
			{
				if(p3DPoints->get(i, j).x != 0 && p3DPoints->get(i, j).y != 0 && p3DPoints->get(i, j).z != 0)
				{
					float fx = p3DPoints->get(i, j).z; // ��ǥ�谡 �ٸ�
					float fy = p3DPoints->get(i, j).x; // ��ǥ�谡 �ٸ�
					float fz = p3DPoints->get(i, j).y; // ��ǥ�谡 �ٸ�

					if(fz * 1000 < nMinHeight) continue;
					if(fz * 1000 > nMaxHeight) continue;
					if(fx * 1000 > nMaxDist) continue;

					// 3D points
					m_pKinectDataPos[j * CKinect::getInstance()->get_parameters().depth_width + j].setXm(fx);
					m_pKinectDataPos[j * CKinect::getInstance()->get_parameters().depth_width + j].setYm(fy);
					m_pKinectDataPos[j * CKinect::getInstance()->get_parameters().depth_width + j].setZm(fz);
					m_pKinectDataPos[j * CKinect::getInstance()->get_parameters().depth_width + j].setID(1);
					m_pKinectDataPos[j * CKinect::getInstance()->get_parameters().depth_width + j].setPixX(i);
					m_pKinectDataPos[j * CKinect::getInstance()->get_parameters().depth_width + j].setPixY(j);

					// Range data
					if(fx < fMinDistX)
					{
						fMinDistX = sqrt(fx * fx + fy * fy); // m
						fHeight = fz; // m

						nAngle = (int)(atan2((double)fy, (double)fx) * R2D) + Sensor::KINECT_SENSOR_FOV / 2;
					}
				}
			}

			if(nAngle >= 0 && nAngle < Sensor::KINECT_SENSOR_FOV)
			{
				if(nKinectRangeData[nAngle] > fMinDistX * 1000)
				{
					nKinectRangeData[nAngle] = fMinDistX * 1000; // mm
					nHeightDataOfRangeData[nAngle] = fHeight * 1000; // mm
				}
			}
		}

		return true;
	}
#endif
	return false;
}

/**
@brief Korean: ��� ������ ���� ���� ������ �޾ƿ�(������ ������ �޾ƿ�)
@brief English: write in English
*/
bool SensorSupervisor::readOnlySensorData()
{	
	//���ڴ� ������ ������ ����----------------------------------------------------
	m_EncoderDelPos= SSAGVWheelActuatorInterface::getInstance()->getDelEncoderData();
	//-------------------------------------------------------------------------------��

	//���̷� ���� ������ ������ ����--------------------------------------------
	m_dGyroData=GyroSensorInterface::getInstance()->getThetaDeg();
	//----------------------------------------------------------------------------------��


	//������ ������ ������ ����----------------------------------------------------
	//int_1DArray nLaserData = HokuyoURG04LXInterface::getInstance()->getData();
	int_1DArray nLaserData = selLaserData();
	for(int i=0; i<Sensor::URG04LX_DATA_NUM181;i++){
		m_nLaserDataFront[i] = nLaserData[i]; //������ ������ ����	
	}
	//������ ������ ������ ����----------------------------------------------------��


	//kinect ���� ������ ������ ����------------------------------------------------------------
#ifdef USE_KINECT_VER_1
	int_1DArray nKinectRangeData = m_KuUtil.generateIntType1DArray(Sensor::KINECT_SENSOR_FOV, 1000000);
	int_1DArray nHeightDataOfRangeData = m_KuUtil.generateIntType1DArray(Sensor::KINECT_SENSOR_FOV, 1000000);

	// 		m_IplKinectImage = KinectSensorInterface::getInstance()->get320ColorImage();
	getKinectV1Data(nKinectRangeData, nHeightDataOfRangeData);
#else
	m_IplKinectImage = KinectSensorInterface::getInstance()->get320ColorImage();
	m_IplKinectDepthImg = KinectSensorInterface::getInstance()->get320DepthImage();
	m_pKinectDataPos = KinectSensorInterface::getInstance()->getGlobal3DPose();
	m_fKinectDistanceImage=KinectSensorInterface::getInstance()->getDistanceImage();
	int_1DArray nKinectRangeData = KinectSensorInterface::getInstance()->getRangeData();
	int_1DArray nHeightDataOfRangeData = KinectSensorInterface::getInstance()->getHeightDataOfRangeData(); //�Ÿ����� ���� ���̰� 
#endif

	for(int i=0; i<Sensor::KINECT_SENSOR_FOV;i++){
		m_nKinectRangeData[i] = nKinectRangeData[i]; //kinect range data ����
	}

	for(int i=0; i<Sensor::KINECT_SENSOR_FOV;i++){
		m_nHeightDataOfRangeData[i] = nHeightDataOfRangeData[i]; //�Ÿ����� ���� ���̰� ����
	}
	//kinect ���� ������ ������ ����------------------------------------------------------------��


	//ī�޶� ������ ������ ����----------------------------------------------------

	m_IplCeilingImage = KuSiriusCameraInterface::getInstance()->calcUndistortedImage();

	//cvFlip(m_IplCeilingImage,m_IplCeilingImage,-1);
	cvFlip(m_IplCeilingImage,m_IplCeilingImage,1);
	//ī�޶� ������ ������ ����----------------------------------------------------��

	return true;
}

/**
@brief Korean: log ���� �κ��� ���ϴ� �κ��� ���ö� ���� Parsing ���ִ�  �Լ�
@brief English: write in English
*/
void SensorSupervisor::dataParsing()
{
	char cData;	
	while(true){
		m_PlayLog >> cData;	
		if( cData ==':' ){ //������ �Ľ� ����
			break;
		}
	}

}

/**
@brief Korean: log ���Ͽ� ���ڴ� ������ �����ϴ� �Լ�
@brief English: write in English
*/
void SensorSupervisor::recordEncoderDelPosData(KuPose EncoderDelPos)
{
	m_DataLog<<endl;
	m_DataLog<<"EncoderDelPos(x,y,deg unit/mm): "<<EncoderDelPos.getX()<<" "<<EncoderDelPos.getY()<<" "<<EncoderDelPos.getThetaDeg()<<endl;
}

/**
@brief Korean: log ���Ϸκ���  ���ڴ� ������ �ҷ����� �Լ�
@brief English: write in English
*/
KuPose SensorSupervisor::playEncoderDelPosData()
{
	KuPose DelEncoderPos; 
	double dX=0.0, dY=0.0,dDeg=0.0;	

	dataParsing();
	m_PlayLog >> dX >> dY >> dDeg;
	DelEncoderPos.setX( dX); 
	DelEncoderPos.setY( dY); 
	DelEncoderPos.setThetaDeg(dDeg);
	return DelEncoderPos;
}
/**
@brief Korean: log ���Ͽ� ���̷μ����� ������ �����ϴ� �Լ�
@brief English: write in English
*/
void SensorSupervisor::recordGyroData(double dGyroData)
{
	int i=0;
	m_DataLog<<"Gyro data: ";
	m_DataLog<<dGyroData<<endl;
}
/**
@brief Korean: log ���Ϸκ���  ���̷μ����� ������ �ҷ����� �Լ�
@brief English: write in English
*/
void SensorSupervisor::playGyroData(double* dGyroData)
{
	dataParsing();
	m_PlayLog >> *dGyroData;

}
/**
@brief Korean: log ���Ͽ� ������ ������ ������ �����ϴ� �Լ�
@brief English: write in English
*/
void SensorSupervisor::recordLaserData(int_1DArray nLaserData)
{
	int i=0;
	m_DataLog<<"Laser data("<<Sensor::URG04LX_DATA_NUM181<<" scan idx unit/mm): ";
	for(i=0; i<Sensor::URG04LX_DATA_NUM181 -1 ; i++){
		m_DataLog<<nLaserData[i]<<" ";
	}
	m_DataLog<<nLaserData[i]<<endl;
}
/**
@brief Korean: log ���Ϸκ���  ������ ������ ������ �ҷ����� �Լ�
@brief English: write in English
*/
void SensorSupervisor::playLaserData(int_1DArray nData)
{
	dataParsing();
	for(int i=0; i<Sensor::URG04LX_DATA_NUM181; i++){
		m_PlayLog >> m_nLaserDataFront[i];
	}
}
/**
@brief Korean: log ���Ͽ� Ű��Ʈ ������ ������ ������ �����ϴ� �Լ�
@brief English: write in English
*/
void SensorSupervisor::recordKinectRangeData(int_1DArray nKinectRangeData)
{
	int i=0;
	m_DataLog<<"Kinect range data("<<Sensor::KINECT_SENSOR_FOV<<" scan idx unit/mm): ";
	for(i=0; i<Sensor::KINECT_SENSOR_FOV-1; i++){
		m_DataLog<<nKinectRangeData[i]<<" ";
	}
	m_DataLog<<nKinectRangeData[i]<<endl;
}
/**
@brief Korean: log ���Ͽ� Ű��Ʈ ������ �������� ���� ���� ������ �����ϴ� �Լ�
@brief English: write in English
*/
void SensorSupervisor::recordHeightDataOfKinectRangeData(int_1DArray nHeightDataOfKinectRangeData)
{
	int i=0;
	m_DataLog<<"Kinect Height data("<<Sensor::KINECT_SENSOR_FOV<<" scan idx unit/mm): ";
	for(i=0; i<Sensor::KINECT_SENSOR_FOV-1; i++){
		m_DataLog<<nHeightDataOfKinectRangeData[i]<<" ";
	}
	m_DataLog<<nHeightDataOfKinectRangeData[i];
}

/**
@brief Korean: Ű��Ʈ ������  ������ �����ϴ� �Լ�
@brief English: write in English
*/
void SensorSupervisor::recordKinectData(IplImage* IplKinectImage, IplImage* IplKinectDepthImg, KuPose* pKinectDataPos)
{

	char cFilePathName[150];
	//char cFilePathName2[150];
	memset(cFilePathName,0,sizeof(cFilePathName));
	//memset(cFilePathName2,0,sizeof(cFilePathName2));
	sprintf_s(cFilePathName,"./Data/log/kinect/Image/%d.jpg",m_KinectLogIdx);
	//sprintf_s(cFilePathName2,"./Data/log/kinect/DisparityImage/%d.jpg",m_KinectLogIdx);
	cvSaveImage(cFilePathName,IplKinectImage);
	//cvSaveImage(cFilePathName2,IplKinectDepthImg);


	// 	//////////////////////////////////////////////////////////////////////////
	// 	//3���� ������ ����
	// 	//////////////////////////////////////////////////////////////////////////
	char c1[150], c2[150], c3[150], c4[150], c5[150], c6[150];
	sprintf_s(c1, "./Data/log/kinect/3D/x/%d.log", m_KinectLogIdx);
	sprintf_s(c2, "./Data/log/kinect/3D/y/%d.log", m_KinectLogIdx);
	sprintf_s(c3, "./Data/log/kinect/3D/z/%d.log", m_KinectLogIdx);
	sprintf_s(c4, "./Data/log/kinect/3D/pix_x/%d.log", m_KinectLogIdx);
	sprintf_s(c5, "./Data/log/kinect/3D/pix_y/%d.log", m_KinectLogIdx);
	sprintf_s(c6, "./Data/log/kinect/3D/validation/%d.log", m_KinectLogIdx);

	int nSize = Sensor::IMAGE_WIDTH*Sensor::IMAGE_HEIGHT;
	for(int i=0; i< nSize; i++){
		m_d3DKinectX[i] = pKinectDataPos[i].getX();
		m_d3DKinectY[i] = pKinectDataPos[i].getY();
		m_d3DKinectZ[i] = pKinectDataPos[i].getZ();
		m_n3DKinectPixX[i] = pKinectDataPos[i].getPixX();
		m_n3DKinectPixY[i] = pKinectDataPos[i].getPixY();
		m_n3DKinectDataValidation[i] = pKinectDataPos[i].getID(); 

	}

	FILE *fp_3D_X, *fp_3D_Y, *fp_3D_Z, *fp_3D_Pix_X, *fp_3D_Pix_Y, *fp_3D_Validation;
	fp_3D_X = fopen (c1,"wb");
	fp_3D_Y = fopen (c2,"wb");
	fp_3D_Z = fopen (c3,"wb");
	fp_3D_Pix_X = fopen (c4,"wb");
	fp_3D_Pix_Y = fopen (c5,"wb");
	fp_3D_Validation = fopen (c6,"wb");

	fwrite(m_d3DKinectX,sizeof(double),nSize,fp_3D_X);
	fwrite(m_d3DKinectY,sizeof(double),nSize,fp_3D_Y);
	fwrite(m_d3DKinectZ,sizeof(double),nSize,fp_3D_Z);
	fwrite(m_n3DKinectPixX,sizeof(int),nSize,fp_3D_Pix_X);
	fwrite(m_n3DKinectPixY,sizeof(int),nSize,fp_3D_Pix_Y);
	fwrite(m_n3DKinectDataValidation,sizeof(int),nSize,fp_3D_Validation);


	//////////////////////////////////////////////////////////////////////////
	fclose	(fp_3D_X); fclose (fp_3D_Y); fclose (fp_3D_Z); 
	fclose(fp_3D_Pix_X); fclose(fp_3D_Pix_Y); fclose(fp_3D_Validation);

	m_KinectLogIdx++;
}

/**
@brief Korean:  Ű��Ʈ ������ ������ �ҷ����� �Լ�
@brief English: write in English
*/
bool SensorSupervisor::playKinectData()
{
	char cFilePathName[150];
	memset(cFilePathName,0,sizeof(cFilePathName));
	sprintf_s(cFilePathName,"./Data/log/kinect/Image/%d.jpg",m_KinectLogIdx);

	IplImage* IplLoadKinectImage;
	IplLoadKinectImage = cvLoadImage(cFilePathName, CV_LOAD_IMAGE_COLOR);
	if(IplLoadKinectImage==NULL) return false;
	cvCopy(IplLoadKinectImage,m_IplKinectImage);

	cvReleaseImage(&IplLoadKinectImage);


	//////////////////////////////////////////////////////////////////////////
	//3���� ������ �ε�
	//////////////////////////////////////////////////////////////////////////

	char c1[150], c2[150], c3[150], c4[150], c5[150], c6[150];
	sprintf_s(c1, "./Data/log/kinect/3D/x/%d.log", m_KinectLogIdx);
	sprintf_s(c2, "./Data/log/kinect/3D/y/%d.log", m_KinectLogIdx);
	sprintf_s(c3, "./Data/log/kinect/3D/z/%d.log", m_KinectLogIdx);
	sprintf_s(c4, "./Data/log/kinect/3D/pix_x/%d.log", m_KinectLogIdx);
	sprintf_s(c5, "./Data/log/kinect/3D/pix_y/%d.log", m_KinectLogIdx);
	sprintf_s(c6, "./Data/log/kinect/3D/validation/%d.log", m_KinectLogIdx);


	FILE *fp_3D_X, *fp_3D_Y, *fp_3D_Z, *fp_3D_Pix_X, *fp_3D_Pix_Y, *fp_3D_Validation;
	fp_3D_X = fopen (c1,"rb");
	fp_3D_Y = fopen (c2,"rb");
	fp_3D_Z = fopen (c3,"rb");
	fp_3D_Pix_X = fopen (c4,"rb");
	fp_3D_Pix_Y = fopen (c5,"rb");
	fp_3D_Validation = fopen (c6,"rb");

	int nSize = Sensor::IMAGE_WIDTH*Sensor::IMAGE_HEIGHT;
	fread(m_d3DKinectX,sizeof(double),nSize,fp_3D_X);
	fread(m_d3DKinectY,sizeof(double),nSize,fp_3D_Y);
	fread(m_d3DKinectZ,sizeof(double),nSize,fp_3D_Z);
	fread(m_n3DKinectPixX,sizeof(int),nSize,fp_3D_Pix_X);
	fread(m_n3DKinectPixY,sizeof(int),nSize,fp_3D_Pix_Y);
	fread(m_n3DKinectDataValidation,sizeof(int),nSize,fp_3D_Validation);

	for(int i=0; i< nSize; i++){
		m_pKinectDataPos[i].setX(m_d3DKinectX[i]);
		m_pKinectDataPos[i].setY(m_d3DKinectY[i]);
		m_pKinectDataPos[i].setZ(m_d3DKinectZ[i]);
		m_pKinectDataPos[i].setPixX(m_n3DKinectPixX[i]);
		m_pKinectDataPos[i].setPixY(m_n3DKinectPixY[i]);
		m_pKinectDataPos[i].setID(m_n3DKinectDataValidation[i]);
		m_fKinectDistanceImage[i]=pow(m_d3DKinectX[i]*m_d3DKinectX[i]+m_d3DKinectY[i]*m_d3DKinectY[i]+m_d3DKinectZ[i]*m_d3DKinectZ[i] , 1/2.0 )/100;
	}

	/////////////////////////////////////////////////////////////////////
	fclose(fp_3D_X);
	fclose(fp_3D_Y);
	fclose(fp_3D_Z);
	fclose(fp_3D_Pix_X);
	fclose(fp_3D_Pix_Y); 
	fclose(fp_3D_Validation); 

	m_KinectLogIdx++;

	return true;
}
/**
@brief Korean: log ���Ϸκ���  Ű��Ʈ ������ �������� ���� ���� ������ �ҷ����� �Լ�
@brief English: write in English
*/
void SensorSupervisor::PlayHeightDataOfKinectRangeData(int_1DArray nHightDataOfKinectRangeData)
{
	dataParsing();
	for(int i=0; i<Sensor::KINECT_SENSOR_FOV; i++){
		m_PlayLog >> nHightDataOfKinectRangeData[i];
	}
}
/**
@brief Korean: log ���Ϸκ���  Ű��Ʈ ������ ������ ������ �ҷ����� �Լ�
@brief English: write in English
*/
void SensorSupervisor::PlayKinectRangeData(int_1DArray nKinectRangeData)
{
	dataParsing();
	for(int i=0; i<Sensor::KINECT_SENSOR_FOV; i++){
		m_PlayLog >> nKinectRangeData[i];
	}
}
/**
@brief Korean: õ��ī�޶� �̹��� ������ �����ϴ� �Լ�
@brief English: write in English
*/
void SensorSupervisor::recordCeilingCamData(IplImage* CeilingImageData)
{
	char cFilePathName[150];
	memset(cFilePathName,0,sizeof(cFilePathName));
	sprintf_s(cFilePathName,"./Data/log/Image/sim_img_%d.jpg",m_CeilingCamLogIdx);
	cvSaveImage(cFilePathName,CeilingImageData);
	m_CeilingCamLogIdx++;
}
/**
@brief Korean:  ����� õ��ī�޶� �̹��� ������ �ҷ����� �Լ�
@brief English: write in English
*/
void SensorSupervisor::playCeilingCamData()
{
	char cFilePathName[100];
	memset(cFilePathName,0,sizeof(cFilePathName));
	sprintf_s(cFilePathName,"./Data/log/Image/sim_img_%d.jpg",m_CeilingCamLogIdx);
	IplImage* IplLoadCeilingImage;
	IplLoadCeilingImage = cvLoadImage(cFilePathName, CV_LOAD_IMAGE_GRAYSCALE);
	cvCopy(IplLoadCeilingImage,m_IplCeilingImage);
	//cvFlip(IplLoadCeilingImage,m_IplCeilingImage,-1);
 	//cvFlip(IplLoadCeilingImage,m_IplCeilingImage,0);
 	//cvFlip(IplLoadCeilingImage,m_IplCeilingImage,1);

	cvReleaseImage(&IplLoadCeilingImage);

	//flip_mode = 0 : X���� �߽����� ���� (���� ����)
	//flip_mode > 0 (e.g. 1) : Y���� �߽����� ���� (�¿� ����)
	//flip_mode < 0 (e.g. -1) : ������ �߽����� ���� (�����¿� ����)
	m_CeilingCamLogIdx++;

} 