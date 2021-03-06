#include <stdafx.h>
#include "GyroSensorInterface.h"


// initialize pointer
GyroSensorInterface::GyroSensorInterface()
{
	m_dData =0.;
	m_bValueInitialized = false;
	m_dAngle = 0.;
	m_dRefData=0.;
}

GyroSensorInterface::~GyroSensorInterface()
{
	
}
/**
 @brief Korean: 자이로센서와 연결한다.
 @brief English: 
*/
bool GyroSensorInterface::connect(char* cPort)
{
	int nPortNum = atoi(&cPort[3]); //포트번호만 잘라내기..
	bool bconnect=false;
	//시리얼 통신 연결 구문--------------------------------------------------------------------------------
	m_Serial.SetComport(nPortNum, 115200, 8, 0, 0);		//port, baudrate, databit, stopbit, paritybit
	m_Serial.CreateCommInfo();
	bconnect=m_Serial.OpenComport();
	m_Serial.m_bStartFlag = bconnect;
	//시러얼 통신 연결 구문 끝-----------------------------------------------------------------------------

	if(bconnect)
	{
		string sSend = "$MIB,RESET*87";
		m_Serial.WriteCommBlock((char*)sSend.c_str(), sSend.length());
		Sleep(4000); // 초기화를 위해 4초 대기
	}

	return bconnect;
}
/**
 @brief Korean:  자이로 센서의 값을 주기적으로 재갱신한다. 
 @brief English: 
*/
void GyroSensorInterface::OnReceiveIRData()
{
	m_CriticalSection.Lock();
	
	unsigned char chDataBlock[1024];
	memset(chDataBlock,0,sizeof(chDataBlock));
	int nLength=m_Serial.GetBlock(chDataBlock);
	for (int i=0; i<nLength; i++) {
		rxbuf_for_GYRO[i] = chDataBlock[i];
	//	printf("rxbuf_for_GYRO[%d] = %x\n",i,rxbuf_for_GYRO[i]);	
	}
	

	short header;
	short rate;
	short angle;
	short check_sum;
	// Verify data packet header
	memcpy(&header, rxbuf_for_GYRO, sizeof(short));
//	printf("header=%x\n",header);
	if(header != (short)0xFFFF){
	//	printf("Header error !!!\n");
	}

	// Copy values from data string
	memcpy(&rate, rxbuf_for_GYRO+2, sizeof(short));
	memcpy(&angle, rxbuf_for_GYRO+4, sizeof(short));
	memcpy(&check_sum, rxbuf_for_GYRO+6, sizeof(short));
	// Verify checksum
	if(check_sum != (short)(0xFFFF+rate+angle))	{
		//printf("Checksum error!!\n");
	}
	else
	{
		m_dAngle = (double)angle;

		if(!m_bValueInitialized)
		{
			m_bValueInitialized = true;
			m_dRefData= m_dAngle/100.0;
		}
		m_dData =m_dAngle/100.0;
		
	}
	
	m_CriticalSection.Unlock();

}

/**
 @brief Korean:  다른 클래스에서 Theta 값을 가져간다. 
 @brief English: 
*/
double GyroSensorInterface::getThetaDeg()
{	
	double dData=0.0;

	dData=m_dData-m_dRefData;

	if(fabs(dData)>45&&m_dData*m_dRefData>0) return 0.0; //예외 처리

	m_dRefData=m_dData;
	
	return -dData;
}
/**
 @brief Korean:  자이로 센서의 초기 값을 초기화 하는 함수.
 @brief English: 
*/
bool GyroSensorInterface::isGyroInitialized(void)
{
	return m_bValueInitialized;
}

