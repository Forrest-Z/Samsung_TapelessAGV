#ifndef __SERIAL_COM_H__
#define __SERIAL_COM_H__

#include "Windows.h"
#include "SerialDef.h"
#include "SendQueue.h"

//---------------------------- ��� ���� --------------------------//
#define	BUFF_SIZE		8192
#define	SEND_BUFF_SIZE	2048
#define	RECV_BUFF_SIZE	2048
/*
- Comport�� ����, Send, Receive ��� 
- InitializeComm���� Com ��� ���� 
- Send, Recv �Լ��� ���
*/
//	��� Ŭ����	 CSerialCom
class	CSerialCom
{
public:

	// ������ �� �Ҹ���.
    CSerialCom();
    virtual		~CSerialCom();

	// ���� Buffer
	CSendQueue	m_qRecv; // ���� Queue
    UCHAR		m_szRecvBuff[RECV_BUFF_SIZE]; // ���ſ� �ӽ� Buffer
	ULONG		m_ulReceivedCharCount; // ���ŵ� Byte ��

    //  �۽� Buffer
	CSendQueue	m_qSend; // �۽� Queue
    UCHAR		m_szSendBuff[SEND_BUFF_SIZE]; // �۽ſ� �ӽ� Buffer
	ULONG		m_ulSendCharCount; // �۽��� Byte ��

	//--------- ��� ȯ�� ���� -----------------------------------------//
	HANDLE		m_hComm;				// ��� ��Ʈ ���� �ڵ�
    BOOL		m_bConnected;			// ��Ʈ�� ���ȴ��� ������ ��Ÿ��.

	// ��� �Լ�
	BOOL InitializeComm( CSerialDef* pSerialDef, BOOL bSyncMode = FALSE );
	BOOL TerminateComm();
	DWORD	ReadComm( UCHAR* pBuff, DWORD nToRead );
	DWORD	WriteComm( UCHAR* pBuff, DWORD nToWrite );
	int		Receive( UCHAR* lpszBlock, DWORD nReadLen );
	int		Recv( char* lpszBlock, DWORD nReadLen );
	int		Send( char* lpszBlock, DWORD nWriteLen );

private:
	OVERLAPPED	m_osRead, m_osWrite;	// ��Ʈ ���� Overlapped structure

protected:
	COMMTIMEOUTS	m_ctoTimeout; // Time-out parameters for all read and write operations on a specified communications device
};

#endif