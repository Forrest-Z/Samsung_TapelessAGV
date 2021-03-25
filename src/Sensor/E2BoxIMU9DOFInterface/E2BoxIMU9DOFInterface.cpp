
#include"stdAfx.h"
#include "E2BoxIMU9DOFInterface.h"

E2BoxIMU9DOFInterface::E2BoxIMU9DOFInterface()
{
	m_doThreadFunc=false;
	m_bConnected = false;
	m_bCaptureEndFlag=false;
	bfirstflag=true;
	SCnt=0;
	nOpenedPort=0;
	nTxState=0,nRxState=0; 
	m_dRollDeg=0.0;
	m_dYawDeg=0.0;
	m_dRollDeg=0.0;
	m_dRefPitchDeg=0.0;
	m_dRefYawDeg=0.0;
	m_dRefRollDeg=0.0;
}
E2BoxIMU9DOFInterface::~E2BoxIMU9DOFInterface()
{
}
/**
 @brief Korean: Thread_function 자이로 센서의 값을 주기적으로 재갱신한다. 
 @brief English: 
*/
void E2BoxIMU9DOFInterface::Thread_function(LPVOID arg)
{
	E2BoxIMU9DOFInterface *pEEGI = (E2BoxIMU9DOFInterface *) arg;	

	KuTimer	Ktimer;
	while(pEEGI->m_doThreadFunc){
		pEEGI->m_bCaptureEndFlag= false;
		pEEGI->capture();
		pEEGI->m_bCaptureEndFlag= true;
		Ktimer.sleepMS(10);
	}

	printf("E2BoxIMU9DOFInterface is terminate!!!\n");

}
/**
 @brief Korean: roll, ptch, yaw 값을 자이로센서로 부터 받아온다. 
 @brief English: 
*/
void E2BoxIMU9DOFInterface::capture()
{
	unsigned int n;
	int value =0;
	int value2 =0 ;
	if(ReadSerialPort(nOpenedPort,&srd)==ERR_OK)
	{  
		if(srd.nSize)
		{  
			for(n=0;n<srd.nSize;n++) 
			{  

				SBuf[SCnt]=srd.szData[n];
				if(SBuf[SCnt]==0x0a)
				{   
					value=FindComma(SBuf);
					SBuf[value]='\0';
					//DegRoll=atof(SBuf)*-1;
					m_dRollDeg=atof(SBuf);
					if(bfirstflag) m_dRefPitchDeg=m_dRollDeg;
					value++;
					value2=FindComma(&SBuf[value]);
					SBuf[value+value2]='\0';
					//DegPitch=atof(&SBuf[value])*-1;
					m_dPitchDeg=-atof(&SBuf[value]);
					if(bfirstflag)  m_dRefRollDeg=m_dPitchDeg;
					value=value+value2+1;
					value2=FindComma(&SBuf[value]);
					SBuf[value+value2]='\0';
					//DegYaw=atof(&SBuf[value])*-1;
					m_dYawDeg=-atof(&SBuf[value]);
					if(bfirstflag)  m_dRefYawDeg=m_dYawDeg;
				}
				else if(SBuf[SCnt]=='*')
				{   
					SCnt=-1;
				}
				SCnt++;				

			}
		}
	}
	bfirstflag=false;
}
/**
 @brief Korean: 자이로 센서를 받아오는 부분을 끝낸다. 
 @brief English: 
*/
void E2BoxIMU9DOFInterface::terminate()
{
	m_Thread.terminate();
}
/**
 @brief Korean: 자이로 센서를 받아오는 부분을 잠시 멈춘다. 
 @brief English: 
*/
void E2BoxIMU9DOFInterface::suspend()
{
	m_Thread.suspend();
}
/**
 @brief Korean: 자이로 센서를 받아오는 부분을 다시 재개한다. 
 @brief English: 
*/
void E2BoxIMU9DOFInterface::resume()
{
	m_Thread.resume();

}
/**
 @brief Korean: 다른 클래스에서 pitch 값을 가져간다. 
 @brief English: 
*/
double  E2BoxIMU9DOFInterface::getPitchDeg()
{

	return m_dPitchDeg;
}
/**
 @brief Korean: 다른 클래스에서 roll 값을 가져간다. 
 @brief English: 
*/
double  E2BoxIMU9DOFInterface::getRollDeg()
{
	return m_dRollDeg;
}
/**
 @brief Korean:  다른 클래스에서 yaw 값을 가져간다. 
 @brief English: 
*/
double  E2BoxIMU9DOFInterface::getYawDeg()
{

	return m_dYawDeg;
}
/**
 @brief Korean:  다른 클래스에서 yaw 값을 가져간다. 
 @brief English: 
*/
double  E2BoxIMU9DOFInterface::getThetaDeg()
{
	double  dThetaDeg=0;

	if(m_dYawDeg>180){
		m_dYawDeg=m_dYawDeg-180;
		m_dYawDeg=m_dYawDeg-180;
	}
	dThetaDeg=-(m_dRefYawDeg-m_dYawDeg);
	
	m_dRefYawDeg=m_dYawDeg;
	
	return dThetaDeg;
}
/**
 @brief Korean: 자이로센서와 연결되는 포트를 설정한다.  
 @brief English: 
*/
void E2BoxIMU9DOFInterface::setComPort(char* ComPort)
{
	m_ComPort = ComPort;
}
/**
 @brief Korean: 자이로센서와 연결되는 포트를 받아온다.. 
 @brief English: 
*/
char* E2BoxIMU9DOFInterface::getComPort()
{
	return m_ComPort;
}
/**
 @brief Korean: 자이로센서와 연결한다.
 @brief English: 
*/
bool E2BoxIMU9DOFInterface::connect()
{
	
	int nComport=m_ComPort[3]-ASCII_INITIAL;//atoi(m_ComPort);


	if(OpenSerialPort(nComport,SERIAL_BAUDRATE,NOPARITY,8,ONESTOPBIT)==ERR_OK)
	{  
		nOpenedPort=nComport;
		unsigned char  szSTXData[1];
		szSTXData[0]=0x3C;
		WriteSerialPort(nOpenedPort,szSTXData,sizeof(szSTXData)*sizeof(unsigned char));
		Sleep(1000);
		unsigned char  szCommandData[3];
		szCommandData[0]=0x73;	
		szCommandData[1]=0x65;	
		szCommandData[2]=0x6D;	
		WriteSerialPort(nOpenedPort,szCommandData,sizeof(szCommandData)*sizeof(unsigned char));
		unsigned char  szData[1];
		szData[0]=0x30;		
		WriteSerialPort(nOpenedPort,szSTXData,sizeof(szSTXData)*sizeof(unsigned char));
		unsigned char  szEndData[1];
		szEndData[0]=0x3E; 
		WriteSerialPort(nOpenedPort,szEndData,sizeof(szEndData)*sizeof(unsigned char));
		Sleep(1000);	
		
// 		unsigned char  szData[6];
// 		szData[0]=0x3C;
// 		szData[1]=0x73;	
// 		szData[2]=0x65;	
// 		szData[3]=0x6D;	
// 		szData[4]=0x30;		
// 		szData[5]=0x3E; 
// 	if(ERR_OK==WriteSerialPort(nOpenedPort,szData,sizeof(szData)*sizeof(unsigned char))	)
// 	{
// 		printf("set DISABLE MAGNETO\n");
// 	}
// 		Sleep(1000);

		m_bConnected = true;
		m_doThreadFunc=true;
		m_Thread.start(&Thread_function ,this, 100, "E2BoxIMU9DOFInterface::connect()");
		return true;
	}
	else
	{
		return false;
	}
}
/**
 @brief Korean: 자이로센서와 연결을 끊고  자이로 센서의 클래스를 종료 시킨다.
 @brief English: 
*/
void E2BoxIMU9DOFInterface::disconnect()
{

	m_doThreadFunc = false;
	KuTimer	 Ktimer;

	if(m_bConnected==true){
		while(1){
			if(m_bCaptureEndFlag==false){
				Ktimer.sleepMS(10);
				continue;
			}
			else{
				m_bConnected = false;
				CloseSerialPort(nOpenedPort);
				terminate();
				return;
			}
		}
	}
}




int E2BoxIMU9DOFInterface::FindComma(char * buf)
{
	int n;
	for(n=0;n<100;n++)
	{
		if(buf[n]==',') break;
	}

	return n;
}

int E2BoxIMU9DOFInterface::OpenSerialPort(int nPort,unsigned long nBaudRate,int nParityBit,int nDataBit,int nStopBit)
{
	char str[10];
	DCB dcb;
	COMMTIMEOUTS ct;

	sprintf(str,"\\\\.\\COM%d",nPort);
	if((hSerialPort[nPort-1]=CreateFileA(str,GENERIC_READ|GENERIC_WRITE,0,NULL,OPEN_EXISTING,FILE_ATTRIBUTE_NORMAL,NULL))==INVALID_HANDLE_VALUE) return ERR_CREATEFILE;
	if(!SetupComm(hSerialPort[nPort-1],RBUF_SIZE,TBUF_SIZE)) return ERR_SETUPCOMM;
	if(!GetCommState(hSerialPort[nPort-1],&dcb)) return ERR_GETCOMMSTATE;

	dcb.BaudRate = nBaudRate;
	dcb.ByteSize = nDataBit;
	dcb.Parity   = nParityBit;
	dcb.StopBits = nStopBit;
	dcb.fNull    = FALSE;   //0x00 송수신가능
	


     //흐름제어 : 없음
		dcb.fOutxCtsFlow = false;					// Disable CTS monitoring
		dcb.fOutxDsrFlow = false;					// Disable DSR monitoring
		dcb.fDtrControl = DTR_CONTROL_DISABLE;		// Disable DTR monitoring
		dcb.fOutX = false;							// Disable XON/XOFF for transmission
		dcb.fInX = false;							// Disable XON/XOFF for receiving
		dcb.fRtsControl = RTS_CONTROL_DISABLE;		// Disable RTS (Ready To Send)

     /*// 흐름제어 : 하드웨어(DTR,RTS,CTS,DSR)
		dcb.fOutxCtsFlow = true;					// Enable CTS monitoring
		dcb.fOutxDsrFlow = true;					// Enable DSR monitoring
		dcb.fDtrControl = DTR_CONTROL_HANDSHAKE;	// Enable DTR handshaking
		dcb.fOutX = false;							// Disable XON/XOFF for transmission
		dcb.fInX = false;							// Disable XON/XOFF for receiving
		dcb.fRtsControl = RTS_CONTROL_HANDSHAKE;	// Enable RTS handshaking
*/


/*
     // 흐름제어 : 소프트웨어(Xon,Xoff)
		dcb.fOutxCtsFlow = false;					// Disable CTS (Clear To Send)
		dcb.fOutxDsrFlow = false;					// Disable DSR (Data Set Ready)
		dcb.fDtrControl = DTR_CONTROL_DISABLE;		// Disable DTR (Data Terminal Ready)
		dcb.fOutX = true;							// Enable XON/XOFF for transmission
		dcb.fInX = true;							// Enable XON/XOFF for receiving
		dcb.fRtsControl = RTS_CONTROL_DISABLE;		// Disable RTS (Ready To Send)

*/



	if (!SetCommState(hSerialPort[nPort-1],&dcb)) return ERR_SETCOMMSTATE;
	if (!GetCommTimeouts(hSerialPort[nPort-1],&ct)) return ERR_GETCOMMTIMEOUT;

	ct.ReadIntervalTimeout = 100;
	ct.ReadTotalTimeoutMultiplier = 0;
	ct.ReadTotalTimeoutConstant = 10;
	ct.WriteTotalTimeoutMultiplier = 0; 
	ct.WriteTotalTimeoutConstant = 10;

	if(!SetCommTimeouts(hSerialPort[nPort-1],&ct)) return ERR_GETCOMMTIMEOUT;
	

	
	return ERR_OK;
}

int E2BoxIMU9DOFInterface::CloseSerialPort(int nPort)
{	if (!CloseHandle(hSerialPort[nPort-1])) return ERR_CLOSEHANDLE;
	return ERR_OK;
}


int E2BoxIMU9DOFInterface::WriteSerialPort(int nPort,unsigned char *szData,unsigned int nBytesToWrite)
{	unsigned int nBytesWritten;
    unsigned int nBytesCompleted=0;
	nTxState=RXTXONTIME;   // 상태표시만은 위한 코드
    while(nBytesCompleted < nBytesToWrite)
      {
	     if (!WriteFile(hSerialPort[nPort-1],&szData[nBytesCompleted],nBytesToWrite-nBytesCompleted,(unsigned long *)&nBytesWritten,NULL)) return ERR_WRITEFILE;
         nBytesCompleted+=nBytesWritten;
		 WaitForSingleObject(hSerialPort[nPort-1],INFINITE);
      }
	return ERR_OK;
}



int E2BoxIMU9DOFInterface::ReadSerialPort(int nPort,SERIALREADDATA * SerialReadData)
{	if(!ReadFile(hSerialPort[nPort-1],SerialReadData->szData,READ_SIZE,(unsigned long *)&SerialReadData->nSize,NULL) ) return ERR_READFILE;	
	if(SerialReadData->nSize!=0) nRxState=RXTXONTIME;  // 상태표시만은 위한 코드
	return ERR_OK;
}

