/*
 * KUNSThreadTestTutorial.cpp
 *
 * Created on: 2012. 5. 19.
 * Author: Joong-Tae Park
 * e-mail: jtpark1114@gmail.com
 */

#include "stdafx.h"
#include "HokuyoURG04LXInterface.h"

HokuyoURG04LXInterface::HokuyoURG04LXInterface()
{
	m_doThreadFunc=false;
	m_timestamp = 0;
	m_bConnected = false;
	m_bCaptureEndFlag=false;

	m_LRFRangeData_181=	 m_KuUtil.generateIntType1DArray(Sensor::URG04LX_DATA_NUM181,0);

	m_nMaxDistance = MAX_DISTANCE; //단위 mm
	m_nMinDistance = MIN_DISTANCE; //레이저 스캐너의 최대값과 최소값
}
HokuyoURG04LXInterface::~HokuyoURG04LXInterface()
{
	
}
/**
 @brief Korean: Laser scanner의 최대 값을 설정한다. 
 @brief English: 
*/
void HokuyoURG04LXInterface::setMaxDistance(int nDistance)
{
	//최대 탐지거리를 설정하는 함수.
	if(nDistance > MAX_DISTANCE ){
		m_nMaxDistance = MAX_DISTANCE;
	}else{
		m_nMaxDistance = nDistance;
	}
}
/**
 @brief Korean: Laser scanner의 최소 값을 설정한다. 
 @brief English: 
*/
void HokuyoURG04LXInterface::setMinDistance(int nDistance)
{
	//최소 탐지거리를 설정하는 함수.
	if(nDistance < MIN_DISTANCE ){
		m_nMinDistance = MIN_DISTANCE;
	}else{
		m_nMinDistance = nDistance;
	}

}
/**
 @brief Korean: Thread_function 자이로 센서의 값을 주기적으로 재갱신한다. 
 @brief English: 
*/
void HokuyoURG04LXInterface::Thread_function(LPVOID arg)
{
	HokuyoURG04LXInterface *pHLURGI = (HokuyoURG04LXInterface *) arg;	

	KuTimer Ktimer;
	while(pHLURGI->m_doThreadFunc){
		pHLURGI->m_bCaptureEndFlag= false;
		pHLURGI->capture();
		pHLURGI->m_bCaptureEndFlag= true;
		Ktimer.sleepMS(10);
	}

	printf("HokuyoLaserScannerURG04LXInterface is terminate!!!\n");

}
/**
 @brief Korean: 레이저의 거리 값을 레이저센서로 부터 받아온다. 
 @brief English: 
*/
void HokuyoURG04LXInterface::capture()
{

	int nCnt = 0;
	const double rad90 = 90.0 * M_PI / 180.0;

	int_1DArray LRFRangeData_181;
	int_1DArray ReverseLRFRangeData_181;

	LRFRangeData_181=	 m_KuUtil.generateIntType1DArray(Sensor::URG04LX_DATA_NUM181,0);
	ReverseLRFRangeData_181=	 m_KuUtil.generateIntType1DArray(Sensor::URG04LX_DATA_NUM181,0);

	if(m_bConnected ==true){
		m_urg.setCaptureRange(m_urg.rad2index(-rad90), m_urg.rad2index(rad90));
		long timestamp =0;
		vector<long> data;
		int n = m_urg.capture(data, &timestamp);
		double dPreAngleDeg = 0;
		double dCurAngleDeg = 0;
		double dPreIdxDistance = 0;
		double dCurIdxDistance = 0;
		int nAngleIndex=0;
		for (int j = 0; j < n; ++j) 
		{
			if(data[j]>=0){
				dCurAngleDeg = m_urg.index2deg(j);
				if(dPreAngleDeg!=dCurAngleDeg){

					double dCurAngle = 0.351*j; //360/1024 -->0.351
					double dPreAngle = 0.351*(j-1);
					double dCurAnglediff  =abs(dCurAngleDeg-dCurAngle);
					double dPreAnglediff  = abs(dCurAngleDeg-dPreAngle);

					dCurIdxDistance = data[j];
					dPreIdxDistance = data[j-1];

					if(dCurAnglediff>dPreAnglediff)
						LRFRangeData_181[nAngleIndex] = dCurIdxDistance;
					else
						LRFRangeData_181[nAngleIndex] = dPreIdxDistance;

					nAngleIndex++;
					dPreAngleDeg=dCurAngleDeg;
				}
			}			
		}

		for(int i = 0 ; i < Sensor::URG04LX_DATA_NUM181 ; i++){
			if(LRFRangeData_181[i]==0){
				LRFRangeData_181[i]=-1;
			}
			else if(LRFRangeData_181[i] > m_nMaxDistance){
				LRFRangeData_181[i]=-1;
			}	
			else if(LRFRangeData_181[i]<m_nMinDistance){
				LRFRangeData_181[i]=-1;
			}	
		}
		setData(LRFRangeData_181);
	}

}
/**
 @brief Korean: 레이저 센서를 받아오는 부분을 끝낸다. 
 @brief English: 
*/
void HokuyoURG04LXInterface::terminate()
{
	m_Thread.terminate();
}
/**
 @brief Korean: 레이저 센서를 받아오는 부분을 잠시 멈춘다. 
 @brief English: 
*/
void HokuyoURG04LXInterface::suspend()
{
	m_Thread.suspend();
}
/**
 @brief Korean: 레이저 센서를 받아오는 부분을 다시 재개한다. 
 @brief English: 
*/
void HokuyoURG04LXInterface::resume()
{
	m_Thread.resume();

}
/**
 @brief Korean:  다른 클래스에서 Laser의 거리 값(181 개)을 가져간다. 
 @brief English: 
*/
int_1DArray HokuyoURG04LXInterface::getData()
{
	int_1DArray pLRFRangeData_181;
	//m_CriticalSection.Lock();
	m_CriticalSection.Lock();
	pLRFRangeData_181 = m_LRFRangeData_181;
	m_CriticalSection.Unlock();

	return pLRFRangeData_181;
}
/**
 @brief Korean:  다른 클래스에서 Laser의 거리 값(181 개)을 받아온다. 
 @brief English: 
*/
void HokuyoURG04LXInterface::setData(int_1DArray LRFRangeData_181)
{
	//m_CriticalSection.Lock();
	m_CriticalSection.Lock();
	for(int i=0;i <Sensor::URG04LX_DATA_NUM181;i++)
	{
		m_LRFRangeData_181[i]=LRFRangeData_181[i];
	}

	m_CriticalSection.Unlock();
}
/**
 @brief Korean: 레이저센서와 연결되는 포트를 설정한다.  
 @brief English: 
*/
void HokuyoURG04LXInterface::setComPort(char* ComPort)
{
	m_ComPort = ComPort;
}
/**
 @brief Korean: 레이저센서와 연결되는 포트를 받아온다.. 
 @brief English: 
*/
char* HokuyoURG04LXInterface::getComPort()
{
	return m_ComPort;
}
/**
 @brief Korean: 레이저센서와 연결한다.
 @brief English: 
*/
bool HokuyoURG04LXInterface::connectLaserScanner()
{
	char* device = getComPort();
	if (! m_urg.connect(device)) {
		printf("UrgCtrl::connect: %s\n", m_urg.what());
		getchar();
		return false;
		//exit(0);
	}else{
		m_bConnected = true;
		m_doThreadFunc=true;
		m_Thread.start(&Thread_function ,this, 100, "HokuyoURG04LXInterface::connectLaserScanner()");
		return true;
	}

}
/**
 @brief Korean: 레이저센서와 연결을 끊고 레이저 센서의 클래스를 종료 시킨다.
 @brief English: 
*/
void HokuyoURG04LXInterface::disconnectLaserScanner()
{
	m_doThreadFunc = false;
	KuTimer Ktimer;

	if(m_bConnected==true){
		while(1){
			if(m_bCaptureEndFlag==false){
				Ktimer.sleepMS(10);
				continue;
			}
			else{
				m_bConnected = false;
				terminate();
				m_urg.disconnect();
				return;
			}
		}
	}
}