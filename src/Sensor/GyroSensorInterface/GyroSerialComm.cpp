#include "stdafx.h"
#include "GyroSerialComm.h"
#include "GyroSensorInterface.h"
#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#endif
IMPLEMENT_DYNCREATE(GYROSerialComm, CObject)

GYROSerialComm::GYROSerialComm()
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

GYROSerialComm::~GYROSerialComm()
{
	DestroyComm();
}

// Communication procedure, observation routine, this routine connected to procedure when OpenComPort function is executed.
DWORD CommWatchProcForGYRO(LPVOID lpData)
{
	DWORD dwEvtMask;
	OVERLAPPED os;						// Declaration the OVERLAPPED structure
	GYROSerialComm *npComm = (GYROSerialComm *)lpData;	// Declaration the CComm class pointer
	unsigned char InData[MAXBLOCK+1];			// Array for receive data
	int nLength;						// Variable for length for receive data
	if(npComm==NULL) return -1;			// if npComm handle doesn't have no comports attached, it returns error

	memset(&os, 0, sizeof(OVERLAPPED));	// initiaization the structure OVERLAPPED os
    os.hEvent=CreateEvent(NULL,			// no security
		TRUE,							// explicit reset req
		FALSE,							// initial event reset
		NULL);							// no name

	// event creation failure
	if(os.hEvent==NULL) {
		AfxMessageBox(_T("Fail to Create Event!"), MB_OK);

        return FALSE;
	}

	//EV_RXCHAR is set as event, other events are ignored
	if(!SetCommMask(npComm->hComm, EV_RXCHAR)) return FALSE;

	//waiting for EVENT when fConnectedis TRUE
	while(npComm->fConnected) {
		dwEvtMask=0;									// Variable for created EVENT
		WaitCommEvent(npComm->hComm, &dwEvtMask, NULL);	// waiting for EVENT occur
		if((dwEvtMask & EV_RXCHAR) == EV_RXCHAR) {		// if EV_RXCHAR EVENT occurs
			do {
				memset(InData, 0, MAXBLOCK); // Initialize the array InData to 0
				
				if(nLength = npComm->ReadCommBlock((LPSTR)InData, MAXBLOCK)) {	//Check the buffer has any data
					/*for(int i=0; i<8;i++){
						printf("i=%x\n",InData[i]);
					}*/
					npComm->SetReadData(InData,nLength);		// Receive data
					Sleep(50);
				}
			}
			while(nLength>0);							// When data is read, the data disapear from buffer,
														// so read all data in buffer
		}
		Sleep(50);
	}

	CloseHandle(os.hEvent);

	return TRUE;
}

int GYROSerialComm::GetBlock(unsigned char* data)
{
	if(local_buf_rd_idx == local_buf_wr_idx) 
	{
		data[0]=0x00;
		return 0; 
	}
	int len=0;
	for( len=0; local_buf_rd_idx!=local_buf_wr_idx;len++)
	{	
		data[len]=local_buff[local_buf_rd_idx++];
		//printf("getdata data = %x\n",data[len]);
		if(local_buf_rd_idx == MAXBLOCK) local_buf_rd_idx=0;
	}

	
	return len;	
}


void GYROSerialComm::PutData(unsigned char data)
{
	local_buff[local_buf_wr_idx++]=data;
	if(local_buf_wr_idx == MAXBLOCK) local_buf_wr_idx=0;
	if(local_buf_wr_idx == local_buf_rd_idx) local_buf_rd_idx++;		//buff overflow
	if(local_buf_rd_idx == MAXBLOCK) local_buf_rd_idx=0;
}

char GYROSerialComm::GetData(void)
{
	bNotAvail=FALSE;
	if(local_buf_rd_idx == local_buf_wr_idx) 
	{
		bNotAvail=TRUE;
		return 0; 
	}

	unsigned char data=local_buff[local_buf_rd_idx++];
	
	if(local_buf_rd_idx == MAXBLOCK) local_buf_rd_idx=0;
	if(local_buf_rd_idx == local_buf_wr_idx) bNotAvail=TRUE;
	return data;	
}


// copy receive data to 'data'
void GYROSerialComm::SetReadData(unsigned char* data, int len)
{
	if(m_bStartFlag == TRUE){
		for( int i=0; i< len; i++){
			PutData(data[i]);
		}

		GyroSensorInterface::getInstance()->OnReceiveIRData();

	}
}


//set up hWnd which is to deliver message
void GYROSerialComm::SetHwnd(HWND hwnd)
{
	m_hWnd=hwnd;
}

// set up comport
void GYROSerialComm::SetComport(int port, DWORD rate, BYTE byteSize, BYTE stop, BYTE parity)
{
	bPort=port;
	dwBaudRate=rate;
	bByteSize=byteSize;
	bStopBits=stop;
	bParity=parity;
}

void GYROSerialComm::SetXonOff(BOOL chk)
{
	fXonXoff=chk;
}

void GYROSerialComm::SetDtrRts(BYTE chk)
{
	bFlowCtrl=chk;
}

// create comport information
// set up after doing SetComport()->SetXonOff()->SetDtrRts()
BOOL GYROSerialComm::CreateCommInfo()
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
BOOL GYROSerialComm::OpenComport()
{
	char szPort[15];
	BOOL fRetVal;
	COMMTIMEOUTS CommTimeOuts;
	if(bPort>15)
		lstrcpy((LPWSTR)szPort, _T("error"));
	else 
		wsprintf((LPWSTR)szPort, _T("COM%d"), bPort);

	if((hComm=CreateFile((LPCWSTR)szPort, GENERIC_READ | GENERIC_WRITE,
		0,
		NULL,
		OPEN_EXISTING,
		FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED,
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
	if(fRetVal) {			// When connected, fRetVal is TRUE that
		printf("okok\n");
		fConnected=TRUE;	// tell connection has been succeeded
		AfxBeginThread((AFX_THREADPROC)CommWatchProcForGYRO,(LPVOID)this, THREAD_PRIORITY_NORMAL, 0, 0, NULL);
	}
	else {
		fConnected=FALSE;
		AfxMessageBox(_T("Problem occured with Communication Port."), MB_OK);
		CloseHandle(hComm);
	}

	return fRetVal;
}

//connect 'filed set comport' and acual port 
//must do SetupConnection befor CreateComport
BOOL GYROSerialComm::SetupConnection()
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

// read data from comprot
int GYROSerialComm::ReadCommBlock(LPSTR lpszBlock, int nMaxLength)
{
	BOOL fReadStat;
	COMSTAT ComStat;
	DWORD dwErrorFlags;
	DWORD dwLength;

	//only try to read number of bytes in queue
	ClearCommError(hComm, &dwErrorFlags, &ComStat);
	dwLength=min( (DWORD)nMaxLength, ComStat.cbInQue);
	if (dwLength>0) {
		fReadStat=ReadFile(hComm, lpszBlock, dwLength, &dwLength, &osRead);
		if (!fReadStat) {
			//Error Message
		}
	}
	return dwLength;
}

// remove comport completely
BOOL GYROSerialComm::DestroyComm()
{
	if(fConnected) CloseConnection();
//	CloseHandle(osRead.hEvent);
//	CloseHandle(osWrite.hEvent);

	return TRUE;
}

// close connection
BOOL GYROSerialComm::CloseConnection()
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

BOOL GYROSerialComm::WriteCommBlock(LPSTR lpByte, DWORD dwBytesToWrite)
{
	BOOL fWriteStat;
	DWORD dwBytesWritten;
	fWriteStat=WriteFile(hComm, lpByte, dwBytesToWrite, &dwBytesWritten, &osWrite);
	if(!fWriteStat) {
		//Error Message
	}

	return TRUE;
}
