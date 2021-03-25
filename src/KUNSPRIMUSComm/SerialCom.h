#ifndef __SERIAL_COM_H__
#define __SERIAL_COM_H__

#include "Windows.h"
#include "SerialDef.h"
#include "SendQueue.h"

//---------------------------- 상수 정의 --------------------------//
#define	BUFF_SIZE		8192
#define	SEND_BUFF_SIZE	2048
#define	RECV_BUFF_SIZE	2048
/*
- Comport로 연결, Send, Receive 기능 
- InitializeComm으로 Com 통신 시작 
- Send, Recv 함수로 통신
*/
//	통신 클래스	 CSerialCom
class	CSerialCom
{
public:

	// 생성자 및 소멸자.
    CSerialCom();
    virtual		~CSerialCom();

	// 수신 Buffer
	CSendQueue	m_qRecv; // 수신 Queue
    UCHAR		m_szRecvBuff[RECV_BUFF_SIZE]; // 수신용 임시 Buffer
	ULONG		m_ulReceivedCharCount; // 수신된 Byte 수

    //  송신 Buffer
	CSendQueue	m_qSend; // 송신 Queue
    UCHAR		m_szSendBuff[SEND_BUFF_SIZE]; // 송신용 임시 Buffer
	ULONG		m_ulSendCharCount; // 송신할 Byte 수

	//--------- 통신 환경 변수 -----------------------------------------//
	HANDLE		m_hComm;				// 통신 포트 파일 핸들
    BOOL		m_bConnected;			// 포트가 열렸는지 유무를 나타냄.

	// 통신 함수
	BOOL InitializeComm( CSerialDef* pSerialDef, BOOL bSyncMode = FALSE );
	BOOL TerminateComm();
	DWORD	ReadComm( UCHAR* pBuff, DWORD nToRead );
	DWORD	WriteComm( UCHAR* pBuff, DWORD nToWrite );
	int		Receive( UCHAR* lpszBlock, DWORD nReadLen );
	int		Recv( char* lpszBlock, DWORD nReadLen );
	int		Send( char* lpszBlock, DWORD nWriteLen );

private:
	OVERLAPPED	m_osRead, m_osWrite;	// 포트 파일 Overlapped structure

protected:
	COMMTIMEOUTS	m_ctoTimeout; // Time-out parameters for all read and write operations on a specified communications device
};

#endif