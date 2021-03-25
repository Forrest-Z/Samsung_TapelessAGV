#include "StdAfx.h"
#include "SerialCom.h"
#include "stdio.h"


#define READ_TIMEOUT		3000   /* ms */

// CSerialCom 생성자
CSerialCom::CSerialCom()
{
	m_hComm = INVALID_HANDLE_VALUE;
	m_bConnected = FALSE;
	m_ulReceivedCharCount = 0;

	m_osRead.hEvent = NULL;
	m_osWrite.hEvent = NULL;

	m_ctoTimeout.ReadIntervalTimeout = 0;
	m_ctoTimeout.ReadTotalTimeoutConstant = 0;
	m_ctoTimeout.ReadTotalTimeoutMultiplier = 0;
	m_ctoTimeout.WriteTotalTimeoutConstant = 0;
	m_ctoTimeout.WriteTotalTimeoutMultiplier = 0;
} // end of CSerialCom()

// CSerialCom 소멸자
CSerialCom::~CSerialCom()
{
} // end of ~CSerialCom()

/*
 * 포트 sPortName을 dwBaud 속도로 연다.
 * Application에서 Initialize시에 ComPort를 열수 있는 인자들을 pSerialDef에 저장하여 호출한다. 
 */
BOOL CSerialCom::InitializeComm( CSerialDef* pSerialDef, BOOL bSyncMode )
{
	// Local 변수.
	DCB				dcb;
	TCHAR			szPortName[12];

	if( pSerialDef->bEnabled == FALSE )
	{
		m_bConnected = FALSE;
	}else{
		if( pSerialDef->Port >= 10 )
			wsprintf( szPortName, TEXT("\\\\.\\COM%d"), pSerialDef->Port );//0217
		else
			wsprintf( szPortName, TEXT("COM%d"), pSerialDef->Port );//0217
		// 포트 open
		m_hComm = CreateFile( szPortName,  // "COM1" or "COM2"
			GENERIC_READ | GENERIC_WRITE, 0, NULL,
			OPEN_EXISTING,               // must be OPEN_EXISTING
			FILE_ATTRIBUTE_NORMAL | (bSyncMode ? 0 : FILE_FLAG_OVERLAPPED),
			NULL);  // must be NULL
		if( m_hComm == INVALID_HANDLE_VALUE ) return FALSE;
		if( bSyncMode != TRUE )
		{
			// overlapped structure 변수 초기화.
			m_osRead.Offset = 0;
			m_osRead.OffsetHigh = 0;
			if( ! (m_osRead.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL)))	
			{
				TerminateComm();
				return FALSE;
			}
			m_osWrite.Offset = 0;
			m_osWrite.OffsetHigh = 0;
			if( ! (m_osWrite.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL)))
			{
				TerminateComm();
				return FALSE;
			}
		}
		// 포트 상태 설정.

		// EV_RXCHAR event 설정
		SetCommMask( m_hComm, EV_RXCHAR);	

		// InQueue, OutQueue 크기 설정.
		SetupComm( m_hComm, RECV_BUFF_SIZE, SEND_BUFF_SIZE );	

		// 포트 비우기.
		PurgeComm( m_hComm,					
			PURGE_TXABORT | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_RXCLEAR);

		// timeout 설정.
		m_ctoTimeout.ReadIntervalTimeout = 0xFFFFFFFF;
		m_ctoTimeout.ReadTotalTimeoutMultiplier = 0;
		m_ctoTimeout.ReadTotalTimeoutConstant = 0;
		m_ctoTimeout.WriteTotalTimeoutMultiplier = 2*CBR_9600 / pSerialDef->BaudRate;
		m_ctoTimeout.WriteTotalTimeoutConstant = 0;
		SetCommTimeouts( m_hComm, &m_ctoTimeout);

		// dcb 설정
		dcb.DCBlength = sizeof(DCB);
		GetCommState( m_hComm, &dcb);	// 예전 값을 읽음.
		/* Specifies whether binary mode is enabled.
		 * The Win32 API does not support non-binary mode transfers,
		 *  so this member must be TRUE. Using FALSE will not work.
		 */
		dcb.fBinary = TRUE;
		/* Specifies whether parity checking is enabled.
		 * If this member is TRUE, parity checking is performed
		 *  and errors are reported.
		 */
		dcb.fParity = TRUE;

		/* Specifies whether the Clear-To-Send signal is monitored
		 *  for output flow control.
		 * If this member is TRUE and CTS is turned off,
		 *  output is suspended until CTS is sent again.
		 */
		dcb.fOutxCtsFlow = FALSE;

		/* Specifies whether the Data-Set-Ready signal is monitored
		 *  for output flow control.
		 * If this member is TRUE and DSR is turned off,
		 *  output is suspended until DSR is sent again.
		 */
		dcb.fOutxDsrFlow = FALSE;
		
		/* Specifies the Data-Terminal-Ready flow control.
		 * This member can be one of the following values:
		 *  DTR_CONTROL_DISABLE   - Disables the DTR line when the device is opened
		 *                          and leaves it disabled.
		 *  DTR_CONTORL_ENABLE    - Enable the DTR line when the device is opened
		 *                          and leaves it on.
		 *  DTR_CONTROL_HANDSHAKE - Enable the DTR handshaking.
		 *                          If handshaking is enabled, it is an error
		 *                          for the application to adjust the line
		 *                          by using the EscapeCommFunction function.
		 */
		dcb.fDtrControl = DTR_CONTROL_ENABLE;

		/* Specifies whether XON/XOFF flow control is used
		 *  during transmission/reception.
		 * If fOutX is TRUE,
		 *  transmission stops when the XoffChar character is received
		 *  and starts again when the XonChar character is received.
		 * If fInX is TRUE,
		 *  the XoffChar character is sent
		 *   when the input buffer comes within XoffLim bytes of being full,
		 *  and the XonChar character is sent
		 *   when the input buffer comes within XonLim bytes of being empty.
		 */
		dcb.fOutX = FALSE;
		dcb.fInX  = FALSE;

		/* Specifies the Request-To-Send flow control.
		 * If this value is zero, the default is RTS_CONTROL_HANDSHAKE.
		 * This member can be one of the following values:
		 *  RTS_CONTROL_DISABLE
		 *  RTS_CONTROL_ENABLE
		 *  RTS_CONTROL_HANDSHAKE
		 *  RTS_CONTROL_TOGGLE
		 */
		dcb.fRtsControl = RTS_CONTROL_ENABLE;	// request-to-send flow control

		/* Specifies the minimum number of bytes allowed in the input buffer
		 *  before the XON character is sent.
		 */
		dcb.XonLim   = 100;
		/* Specifies the maximum number of bytes allowed in the input buffer
		 *  before the XOFF character is sent.
		 * The maximum number of bytes allowed is calculated
		 *  by subtracting this value from the size, in bytes, of the input buffer.
		 */
		dcb.XoffLim  = 100;

		/* Specifies the number of bits in the bytes transmitted and received. */
		dcb.BaudRate = pSerialDef->BaudRate;		
		dcb.ByteSize = pSerialDef->ByteSize;
		/* Specifies the parity scheme to be used.
		 * This member can be one of the following values:
		 *  EVENPARITY
		 *  MARKPARITY
		 *  NOPARITY
		 *  ODDPARITY
		 *  SPACEPARITY
		 */
		dcb.Parity = pSerialDef->ParityChk;
		/* Specifies the number of stop bits to be used.
		 * This member can be one of the following values:
		 *  ONESTOPBIT   - 1 stop bit
		 *  ONE5STOPBITS - 1.5 stop bits
		 *  TWOSTOPBITS  - 2 stop bits
		 */
		if( pSerialDef->StopBits >= 0 && pSerialDef->StopBits <= 1 )
			dcb.StopBits = 0;
		else if( pSerialDef->StopBits == 2 )
			dcb.StopBits = pSerialDef->StopBits;
		else
		{
			TerminateComm();
			return FALSE;
		}
		/* Specifies the value of the XON/XOFF character
		 *  for both transmission and reception.
		 */
		dcb.XonChar  = ASCII_XON;
		dcb.XoffChar = ASCII_XOFF;
		
		if( ! SetCommState( m_hComm, &dcb))
		{
			DWORD n = GetLastError();
			TerminateComm();
			return FALSE;
		}
		m_bConnected = TRUE;
	}

	if( pSerialDef->bEnabled == FALSE )
		return FALSE;
	else
		return TRUE;
} // end of InitializeComm()

// 종료시에 Comport를 닫는다. 
BOOL CSerialCom::TerminateComm()
{
	if( m_osWrite.hEvent != NULL )
	{
		CloseHandle( m_osWrite.hEvent );
		m_osWrite.hEvent = NULL;
	}
	if( m_osRead.hEvent != NULL )
	{
		CloseHandle( m_osRead.hEvent );
		m_osRead.hEvent = NULL;
	}
	if( m_hComm != INVALID_HANDLE_VALUE )
	{
		SetCommMask( m_hComm, 0);
		PurgeComm( m_hComm,	PURGE_TXABORT | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_RXCLEAR);
		CloseHandle( m_hComm);
		m_hComm = INVALID_HANDLE_VALUE;
	}
	m_bConnected = FALSE;
	return TRUE;
} // end of TerminateComm()

// 포트에 pBuff의 내용을 nToWrite만큼 쓴다.
// 실제로 쓰여진 Byte수를 리턴한다.
DWORD CSerialCom::WriteComm( UCHAR* pBuff, DWORD nToWrite )
{
	DWORD	dwWritten, dwError, dwErrorFlags;
	COMSTAT	comstat;

	if( m_hComm == INVALID_HANDLE_VALUE ||
		m_hComm == NULL ) return 0;

	if( ! WriteFile( m_hComm, pBuff, nToWrite, &dwWritten, &m_osWrite))
	{
		if( GetLastError() == ERROR_IO_PENDING)
		{
			// 읽을 문자가 남아 있거나 전송할 문자가 남아 있을 경우 Overapped IO의
			// 특성에 따라 ERROR_IO_PENDING 에러 메시지가 전달된다.
			//timeouts에 정해준 시간만큼 기다려준다.
			while (! GetOverlappedResult( m_hComm, &m_osWrite, &dwWritten, TRUE))
			{
				dwError = GetLastError();
				if( dwError != ERROR_IO_INCOMPLETE)
				{
					ClearCommError( m_hComm, &dwErrorFlags, &comstat);
					break;
				}
			}
		}else{
			ClearCommError( m_hComm, &dwErrorFlags, &comstat);
		}
	}

	return dwWritten;
} // end of WriteComm()

// 포트로부터 pBuff에 nToWrite만큼 읽는다.
// 실제로 읽혀진 Byte수를 리턴한다.
DWORD CSerialCom::ReadComm( UCHAR* pBuff, DWORD nToRead )
{
	DWORD	dwRead, dwError, dwErrorFlags;
	COMSTAT	comstat;

	if( m_hComm == INVALID_HANDLE_VALUE ||
		m_hComm == NULL ) return 0;

	//----------------- system queue에 도착한 Byte수만 미리 읽는다.
	ClearCommError( m_hComm, &dwErrorFlags, &comstat);
	dwRead = comstat.cbInQue;
	
	if( dwRead > 0)
	{
		if( ! ReadFile( m_hComm, pBuff, nToRead, &dwRead, &m_osRead))
		{
			if( GetLastError() == ERROR_IO_PENDING)
			{
				//--------- timeouts에 정해준 시간만큼 기다려준다.
				while (! GetOverlappedResult( m_hComm, &m_osRead, &dwRead, TRUE))
				{
					dwError = GetLastError();
					if( dwError != ERROR_IO_INCOMPLETE)
					{
						ClearCommError( m_hComm, &dwErrorFlags, &comstat);
						break;
					}
				}
			}else{
				dwRead = 0;
				ClearCommError( m_hComm, &dwErrorFlags, &comstat);
			}
		}
	}

	return dwRead;
} // end of ReadComm()


int CSerialCom::Receive( UCHAR* lpszBlock, DWORD nReadLen )
{
	DWORD dwRead;
	DWORD dwReadAll;
	DWORD dwReceiveStartTime;

	if( m_hComm == INVALID_HANDLE_VALUE ||
		m_hComm == NULL ) return 0;

	dwReadAll = 0;
	dwReceiveStartTime = GetTickCount();
	for(;;)
	{
		dwRead = ReadComm( lpszBlock, nReadLen - dwReadAll );
		if( 0 < dwRead )
		{
			dwReadAll += dwRead;
			lpszBlock += dwRead;
		}

		if( dwReadAll == nReadLen )
		{
			return dwReadAll;
		}
		if( GetTickCount() >= dwReceiveStartTime + READ_TIMEOUT )
		{
			return dwReadAll;
		}
		Sleep (10);
	}
} // end of Receive()

int CSerialCom::Recv( char* lpszBlock, DWORD nReadLen )
{
	return	Receive( (UCHAR*)lpszBlock, nReadLen );
} // end of Receive()

int CSerialCom::Send( char* lpszBlock, DWORD nWriteLen )
{
	return WriteComm( (UCHAR*)lpszBlock, nWriteLen );
}
