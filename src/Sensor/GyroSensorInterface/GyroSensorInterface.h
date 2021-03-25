#ifndef GYRO_SENSOR_INTERFACE_H
#define GYRO_SENSOR_INTERFACE_H



#include <iostream>
#include "GyroSerialComm.h"
#include <afxmt.h>
#include "../../src/KUNSUtil/KUNSThread/KuThread.h"
#include "../../src/KUNSUtil/KUNSSingletone/KuSingletone.h"
#include "../../src/KUNSUtil/KUNSCriticalSection/KuCriticalSection.h"
#include "../../src/KUNSUtil/KUNSSmartPointer/KuSmartPointer.h"

using namespace std;
class GyroSensorInterface: public KuSingletone <GyroSensorInterface>
{
public:
	static const int IR_SENSOR_NUM = 4;
	static const int IR_DOWN_SENSOR_NUM =4;
	static const int IR_SENSOR_MAX_DISTANCE=3; //3m

private:
	GYROSerialComm m_Serial;
	CCriticalSection m_CriticalSection;

	
	unsigned char rxbuf_for_GYRO[1024];	
	double m_dData;
	double m_dRefData;

	bool m_bValueInitialized;
	double m_dAngle;

public:
	double getThetaDeg();
	bool connect(char* cPort);
	void OnReceiveIRData();

	GyroSensorInterface();
	~GyroSensorInterface();
	bool isGyroInitialized(void);

};

#endif

