/*______________________________________________________________________________________________
PROJECT : Intelligent Robotics Lab. Department of Mechanical Engineering Korea University                                    
(c) Copyright 2012 KUME Intelligent Robotics Lab.                                             
All rights reserved                                                                         
$Directed by Jae-Bok Song                                                                     
$Designed by Joong-Tae Park                                                                  
$Description : E2BOX 9������ IMU ������ �������̽��� �����ϴ� Ŭ����
$Created on: 2012. 6. 8.                                                                        
$Author: Joong-Tae Park      
$e-mail: jtpark1114@gmail.com
______________________________________________________________________________________________*/

#ifndef E2BOX_IMU_9DOF_INTERFACE_H
#define E2BOX_IMU_9DOF_INTERFACE_H

#include <iostream>

#include "../../src/KUNSUtil/KUNSThread/KuThread.h"
#include "../../src/KUNSUtil/KUNSSingletone/KuSingletone.h"
#include "../../src/KUNSUtil/KUNSCriticalSection/KuCriticalSection.h"
#include "../../src/KUNSUtil/KUNSSmartPointer/KuSmartPointer.h"
#include <stdio.h>

using namespace std;

#define SERIAL_BAUDRATE  115200
#define ASCII_INITIAL  48

#define NUMBEROFPORT 4      //���� �ִ� ��Ʈ ��
#define RBUF_SIZE 1024		//���Ź���ũ��
#define TBUF_SIZE 1024      //�۽Ź���ũ��

#define READ_SIZE 1000		//�ѹ��� ���� ������

#define   ERR_OK               0
#define   ERR_CREATEFILE      -1
#define   ERR_SETUPCOMM       -2
#define   ERR_GETCOMMSTATE    -3
#define   ERR_SETCOMMSTATE    -4
#define   ERR_GETCOMMTIMEOUT  -5
#define   ERR_WRITEFILE       -6
#define   ERR_READFILE        -7
#define   ERR_CLOSEHANDLE     -8
#define RXTXONTIME 10


typedef struct  {
	unsigned int  nSize;
	char  szData[READ_SIZE];
} SERIALREADDATA;



class E2BoxIMU9DOFInterface : public KuSingletone <E2BoxIMU9DOFInterface>
{


private:
	SERIALREADDATA srd;
	HANDLE hSerialPort[NUMBEROFPORT];
	unsigned int nTxState,nRxState;  //status bar�� rx tx ���ۻ��� Ŭ���� �ð� ī��Ʈ


public:
	int SCnt;
	char buf[1024];
	char SBuf[1000];

private:
	KuThread m_Thread;
	
	char* m_ComPort;
	bool m_bConnected;
	bool m_doThreadFunc;
	bool m_bCaptureEndFlag;
	int nOpenedPort;
	bool bfirstflag;

	double  m_dPitchDeg;
	double  m_dRollDeg;
	double  m_dYawDeg;

	double  m_dRefPitchDeg;
	double  m_dRefRollDeg;
	double  m_dRefYawDeg;

private:
	int OpenSerialPort(int nPort,unsigned long nBaudRate,int nParityBit,int nDataBit,int nStopBit);
	int CloseSerialPort(int nPort);
	int WriteSerialPort(int nPort,unsigned char *szData,unsigned int nBytesToWrite);
	int ReadSerialPort(int nPort,SERIALREADDATA * SerialReadData);
	int FindComma(char * buf);
	void capture();
	static void Thread_function(LPVOID arg);


public:
	void start();
	void terminate();
	void suspend();
	void resume();
	bool connect();
	void disconnect();

	double  getThetaDeg();
	double  getPitchDeg();
	double  getRollDeg();
	double  getYawDeg();


	void setComPort(char* ComPort);
	char* getComPort();
	

	E2BoxIMU9DOFInterface();
	virtual ~E2BoxIMU9DOFInterface();

};

#endif /*E2BOX_IMU_9DOF_INTERFACE_H*/
