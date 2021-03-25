#ifndef CSERIAL_COMMUNICATION_FOR_IR_H
#define CSERIAL_COMMUNICATION_FOR_IR_H

#define MAXBLOCK 1024
#define MAXPORT 4

//Flow Control flags
#define FC_DTRDSR 0x01
#define FC_RTSCTS 0x02
#define FC_XONXOFF 0x04

//Ascii definitions
#define ASCII_BEL 0x07
#define ASCII_BS 0x08
#define ASCII_LF 0x0A
#define ASCII_CR 0x0D
#define ASCII_XON 0x11
#define ASCII_XOFF 0x13
#define WM_RECEIVEDATA WM_USER+1
#define MAX_PROTOCOL_SIZE 22

#define	CR  13
#define LF  10
#define SPACE 32

#include <iostream>
#ifdef WIN32   // Windows system specific
#define _AFXDLL
#include "../../include/stdafx.h"
//#include  <Windows.h>
#endif
#include "../KUNSCriticalSection/KuCriticalSection.h"

using namespace std;
class KuSerialComm :public CObject
{	
	DECLARE_DYNCREATE(KuSerialComm)
private:
	CCriticalSection m_CriticalSection;

public:
	KuSerialComm();
	virtual ~KuSerialComm();

	void sendData(char* pSendData, int nDataLength);
	int readData(unsigned char* pRecvdData, int nDataLength); //시리얼 통신 클래스를 통해 데이터를 받아오는 함수
	void init();



	void SetComport(int port, DWORD rate, BYTE bytesize, BYTE stop, BYTE parity);	// Comport Setup
	void SetDtrRts(BYTE chk);			//D tr Rts Setup
	BOOL CreateCommInfo();				// Create Comport
	BOOL DestroyComm();					// Destory Comport
	int  ReadCommBlock(LPSTR, int);		// Receive Data from Comport
	BOOL WriteCommBlock(LPSTR, DWORD);	// Write data to Comport
	bool OpenComport();					// Open Comport and try to connect
	BOOL SetupConnection();				// Open Connection
	BOOL CloseConnection();				// Close Connection
	void SetReadData(char* data, int len);		// Save read data to the buffer. 
	void SetHwnd(HWND hwnd);

public:
	BYTE bPort;
	BOOL fXonXoff;
	BYTE bByteSize, bFlowCtrl, bParity, bStopBits;
	DWORD dwBaudRate;
	HANDLE hWatchThread;
	HWND hTermWnd;
	DWORD dwThreadID;
	OVERLAPPED osWrite, osRead;
	unsigned char Rev_Sel;

public:
	HANDLE hComm;
	BOOL fConnected;
	BYTE abIn[MAXBLOCK+1];
	HWND m_hWnd;



public:
	BOOL bNotAvail;
	char local_buff[MAXBLOCK];
	int local_buf_wr_idx;
	int local_buf_rd_idx;
	void PutData(char);
	char GetData(void);
	int GetBlock(char*);


	CString m_pBuffer;
	CString m_pInBuffer;
	BOOL m_bStartFlag;

};
DWORD CommWatchProcForIR(LPVOID lpData);
//IMPLEMENT_DYNCREATE(KuSerialComm, CObject)

#endif
