

// #ifdef WIN32   // Windows system specific
// #define _AFXDLL
// #include "../../include/stdafx.h"
// #endif
#include "stdafx.h"
#include "KuSerialComm.h"
//#include "../../Sensor/wheelActuatorInterface/SSAGVWheelActuatorInterface.h"
IMPLEMENT_DYNCREATE(KuSerialComm, CObject)

KuSerialComm::KuSerialComm()
{
	hComm=NULL;				// Initialization the Handle for Comport
	bFlowCtrl=FC_XONXOFF;	// Setup the flow_control
	fConnected=FALSE;		// thread stop
//	m_pToServoMsg = _T("");	// Initialization the CString
//	Rev_Sel=0;
//	m_pServoEnableflag = FALSE;

	
	bNotAvail=FALSE;
	local_buf_wr_idx=0;//pseudo
	local_buf_rd_idx=0;//pseudo

}

KuSerialComm::~KuSerialComm()
{
	DestroyComm();
}
void KuSerialComm::init()
{
	hComm=NULL;				// Initialization the Handle for Comport
	bFlowCtrl=FC_XONXOFF;	// Setup the flow_control
	fConnected=FALSE;		// thread stop
	bNotAvail=FALSE;
	local_buf_wr_idx=0;//pseudo
	local_buf_rd_idx=0;//pseudo

}
int KuSerialComm::GetBlock(char* data)
{
	m_CriticalSection.Lock(); 

	if(local_buf_rd_idx == local_buf_wr_idx){ 
		data[0]=0x00;
		return 0; 
	}
	int len=0;
	for(len=0; local_buf_rd_idx!=local_buf_wr_idx;len++)
	{	
		data[len]=local_buff[local_buf_rd_idx++];
		if(local_buf_rd_idx == MAXBLOCK) local_buf_rd_idx=0;
	}
	m_CriticalSection.Unlock();
	return len;	
}


void KuSerialComm::PutData(char data)
{
	m_CriticalSection.Lock(); 

	local_buff[local_buf_wr_idx++]=data;
	if(local_buf_wr_idx == MAXBLOCK) local_buf_wr_idx=0;
	if(local_buf_wr_idx == local_buf_rd_idx) local_buf_rd_idx++;		//buff overflow
	if(local_buf_rd_idx == MAXBLOCK) local_buf_rd_idx=0;
	m_CriticalSection.Unlock();

}

char KuSerialComm::GetData(void)
{
	bNotAvail=FALSE;
	if(local_buf_rd_idx == local_buf_wr_idx) 
	{
		bNotAvail=TRUE;
		return 0; 
	}

	char data=local_buff[local_buf_rd_idx++];
	if(local_buf_rd_idx == MAXBLOCK) local_buf_rd_idx=0;
	if(local_buf_rd_idx == local_buf_wr_idx) bNotAvail=TRUE;
	return data;	
}


// copy receive data to 'data'
void KuSerialComm::SetReadData(char* data, int len)
{
	m_CriticalSection.Lock(); 

	if(m_bStartFlag == TRUE){
	//	cout<<"hehe ";
		for( int i=0; i< len; i++){
			PutData(data[i]);
		//	printf("%02X",data[i]);
		}
	}
	m_CriticalSection.Unlock();

//	cout<<endl;
	//SSAGVWheelActuatorInterface::getInstance()->OnReceiveData();
}


//set up hWnd which is to deliver message
void KuSerialComm::SetHwnd(HWND hwnd)
{
	m_hWnd=hwnd;
}

// set up comport
void KuSerialComm::SetComport(int port, DWORD rate, BYTE byteSize, BYTE stop, BYTE parity)
{
	bPort=port;
	dwBaudRate=rate;
	bByteSize=byteSize;
	bStopBits=ONESTOPBIT;
	bParity=NOPARITY;
}


void KuSerialComm::SetDtrRts(BYTE chk)
{
	bFlowCtrl=chk;
}

// create comport information
// set up after doing SetComport()->SetXonOff()->SetDtrRts()
BOOL KuSerialComm::CreateCommInfo()
{
	osWrite.Offset=0;
	osWrite.OffsetHigh=0;
	osRead.Offset=0;
	osRead.OffsetHigh=0;

	// creat EVENT. manual reset event, initial no-signal condition
	osRead.hEvent=CreateEvent(NULL, TRUE, FALSE, NULL);
	if(osRead.hEvent=NULL) {

		return FALSE;
	}
	osWrite.hEvent=CreateEvent(NULL, TRUE, FALSE, NULL);
	if(osWrite.hEvent=NULL) {
		CloseHandle(osRead.hEvent);

		return FALSE;
	}

	return TRUE;
}

// attemp to connect after opening comport
bool KuSerialComm::OpenComport()
{
	char szPort[15];
	BOOL fRetVal;
	COMMTIMEOUTS CommTimeOuts;
	if(bPort>10)
		lstrcpy((LPWSTR)szPort, _T("error"));
	else 
		wsprintf((LPWSTR)szPort, _T("COM%d"), bPort);

	if((hComm=CreateFile((LPCWSTR)szPort, GENERIC_READ | GENERIC_WRITE,
		0,
		NULL,
		OPEN_EXISTING,
		FILE_ATTRIBUTE_NORMAL |FILE_FLAG_WRITE_THROUGH/*| FILE_FLAG_OVERLAPPED*/,
		NULL)) == (HANDLE)-1)
		return FALSE;
	else {
		// Set data exchange method in comport to char unit to a default
		SetCommMask(hComm, EV_RXCHAR);
		SetupComm(hComm, 4096, 4096);
		// clean completely in case device has wastes
		PurgeComm(hComm, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR);
		CommTimeOuts.ReadIntervalTimeout=0xFFFFFFFF;
		CommTimeOuts.ReadTotalTimeoutMultiplier=0;
		CommTimeOuts.ReadTotalTimeoutConstant=1000;
		CommTimeOuts.WriteTotalTimeoutMultiplier=0;
		CommTimeOuts.WriteTotalTimeoutConstant=1000;
		SetCommTimeouts(hComm, &CommTimeOuts);
	}

	fRetVal = SetupConnection();
/*
	if(fRetVal) {			// When connected, fRetVal is TRUE that
		fConnected=TRUE;	// tell connection has been succeeded
		AfxBeginThread((AFX_THREADPROC)CommWatchProcForIR,(LPVOID)this, THREAD_PRIORITY_NORMAL, 0, 0, NULL);
	}
	else {
		fConnected=FALSE;
		AfxMessageBox(_T("Problem occured with Communication Port."), MB_OK);
		CloseHandle(hComm);
	}
*/
	return fRetVal;
}

//connect 'filed set comport' and acual port 
//must do SetupConnection befor CreateComport
BOOL KuSerialComm::SetupConnection()
{
	BOOL fRetVal;
	//BYTE bSet;
	DCB dcb;
	dcb.DCBlength=sizeof(DCB);
	GetCommState(hComm, &dcb);		// receive dcb's default value.

	dcb.BaudRate=dwBaudRate;
	dcb.ByteSize=bByteSize;
	dcb.Parity=bParity;
	dcb.StopBits=bStopBits;
	
	
	fRetVal=SetCommState(hComm, &dcb);

	return fRetVal;
}

// remove comport completely
BOOL KuSerialComm::DestroyComm()
{
	if(fConnected) CloseConnection();
//	CloseHandle(osRead.hEvent);
//	CloseHandle(osWrite.hEvent);

	return TRUE;
}

// close connection
BOOL KuSerialComm::CloseConnection()
{
	//set connected flag to FALSE;
	fConnected=FALSE;
	//disable event notification and wait for thread to halt
	SetCommMask(hComm, 0);
	EscapeCommFunction(hComm, CLRDTR);
	PurgeComm(hComm, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR);
	CloseHandle(hComm);

	return TRUE;
}



/**
 @brief Korean: 시리얼 통신 클래스에  명령 프로토콜을 보내는 함수
 @brief English: write in English
*/
void KuSerialComm::sendData(char* pSendData, int nDataLength)
{

//	char cSendData[MAXBLOCK]={0};
// 	cout<<endl<<"보내는 데이터:::";
 //	for(int i = 0; i < nDataLength ; i++) {
//		cSendData[i] = pSendData[i];
// 		printf("%02X",cSendData[i]);
// 	}
// 	cout<<endl;
	WriteCommBlock(pSendData, nDataLength);
}

BOOL KuSerialComm::WriteCommBlock(LPSTR lpByte, DWORD dwBytesToWrite)
{
	BOOL fWriteStat;
	DWORD dwBytesWritten=0;

	fWriteStat=WriteFile(hComm, lpByte, dwBytesToWrite, &dwBytesWritten, &osWrite);
	if(!fWriteStat) {
		if(GetLastError()==ERROR_IO_PENDING){
			cout<<"[Serial Comm]::ERROR_IO_PENDING "<<endl;

			bool bResult = GetOverlappedResult(hComm, // Handle to COMM port
				&osWrite, // Overlapped structure
				&dwBytesWritten, // Stores number of bytes sent
				TRUE); // Wait flag

		}else{
			//Error Message
			//cout<<"[Serial Comm]::Write Error "<<endl;
		}


	}

	return TRUE;
}

/**
 @brief Korean: 시리얼 통신 클래스를 통해 데이터를 받아오는 함수
 @brief English: write in English
 */
int KuSerialComm::readData(unsigned char* pRecvdData, int nDataLength)
{
	int nLength=0;	
	unsigned char InData[MAXBLOCK+1];

	nLength = ReadCommBlock((LPSTR)InData, nDataLength);
	for(int i=0; i<nLength; i++){
		//printf("%02X",InData[i]);
		pRecvdData[i] = InData[i];
	}	
	//cout<<endl;
	return nLength;
}



// read data from comprot
int KuSerialComm::ReadCommBlock(LPSTR lpszBlock, int nMaxLength)
{
	BOOL fReadStat;
	COMSTAT ComStat;
	DWORD dwErrorFlags;
	DWORD dwLength=0;
		
	//only try to read number of bytes in queue
	ClearCommError(hComm, &dwErrorFlags, &ComStat);
	dwLength=min( (DWORD)nMaxLength, ComStat.cbInQue);
	if (dwLength>0) {
		fReadStat=ReadFile(hComm, lpszBlock, dwLength, &dwLength, &osRead);
		if (!fReadStat) {
			//Error Message
			//cout<<"[Serial Comm]::Read Error "<<endl;
		}
	}
	return dwLength;
}
