#include "stdafx.h"
#include "PRIMUSCommunication.h"
//#include "debug.h"
#include "Global.h"

class CSerialDef				PRIMUSSerialDef;

CPRIMUSCommunication::CPRIMUSCommunication(void)
{
	InitializeCriticalSection(&hCrtclSect);
	m_qSend.InitializeQueue( 64, sizeof(SEND_QUEUE_COMMAND) );	// Record�� �ִ� 64 
	m_qRecv.InitializeQueue( 64, MAX_COMM_RECV_BUFF );			// Record�� �ִ� 64 
	m_ahEvnt[0] = INVALID_HANDLE_VALUE;
	m_ahEvnt[1] = INVALID_HANDLE_VALUE;
	m_ahEvnt[2] = INVALID_HANDLE_VALUE;
}

CPRIMUSCommunication::~CPRIMUSCommunication(void)
{
	DeleteCriticalSection(&hCrtclSect);
}

BOOL CPRIMUSCommunication::Initialize(CSerialDef *pSerialDef)
{
	EnterCriticalSection( &hCrtclSect );
	BOOL bRet = true;
	// Local ����.
	CString			cstrTemp;
	// ���� �ʱ�ȭ
	m_bConnected = FALSE;
	m_SendingState = 0;

	// Paramter default;
	ParameterDefault();

	if( pSerialDef->bEnabled == FALSE )
	{
		//debug( "CPRIMUSCommunication::CommInitialize : Communication Port is disabled.");
		m_hComm = INVALID_HANDLE_VALUE;
	}else{
		if( InitializeComm( pSerialDef ) == -1 )
		{
			//debug( "CPRIMUSCommunication::CommInitialize : Com%d Initialize ����.",pSerialDef->Port );
			Terminate();
			LeaveCriticalSection( &hCrtclSect );
			return -1;
		}else{
			//debug( "CPRIMUSCommunication::CommInitialize : Com%d Initialize OK.",pSerialDef->Port );
		}

		// Timeout Event ����
		m_ahEvnt[0] = CreateWaitableTimer( NULL, FALSE, _T("CommandAck") );//0217
		m_ahEvnt[1] = CreateWaitableTimer( NULL, FALSE, _T("Completion") );//0217
		m_ahEvnt[2] = CreateWaitableTimer( NULL, FALSE, _T("Spare") );//0217

		// ��Ʈ Receive Thread ����.
		m_hCommThread = CreateThread( NULL, 0, 
			(LPTHREAD_START_ROUTINE)ComRecvThread, this, 0, &m_dwComThreadID);
		if( m_hCommThread == NULL )
		{
			//debug( "CPRIMUSCommunication::CommInitialize : CreateThread ����." );
			Terminate();
			LeaveCriticalSection( &hCrtclSect );
			return -1;
		}else{
			//debug( "CPRIMUSCommunication::CommInitialize : Serial ��� Receive Thread�� �����Ͽ����ϴ�." );
		}
	}

	if( pSerialDef->bEnabled == FALSE )
	{
		LeaveCriticalSection( &hCrtclSect );
		return 1;
	}
	else
	{
		LeaveCriticalSection( &hCrtclSect );
		return 0;
	}

	LeaveCriticalSection( &hCrtclSect );
	return bRet;
}

void CPRIMUSCommunication::Terminate()
{
	EnterCriticalSection( &hCrtclSect );
	int i;
	DWORD dwExitCode=0;

	m_bConnected = FALSE;

	// Recv Thread�� ���� �غ�
	CancelWaitableTimer(m_ahEvnt[0]);
	CancelWaitableTimer(m_ahEvnt[1]);
	CancelWaitableTimer(m_ahEvnt[2]);

	if(m_hCommThread != INVALID_HANDLE_VALUE){
		dwExitCode = STILL_ACTIVE;
		for(int i=0; i<10 && dwExitCode==STILL_ACTIVE; i++){
			Sleep(100);
			GetExitCodeThread(m_hCommThread,&dwExitCode);
		}
		if(dwExitCode==STILL_ACTIVE){			
			TerminateThread(m_hCommThread,-1);
			//debug("CPRIMUSCommunication:Terminate()::TerminateThread");
			Sleep(100);
		}
		CloseHandle(m_hCommThread);
		m_hCommThread = INVALID_HANDLE_VALUE;
	}

	// Timer Handle�� ���� Close
	for(i =0; i<3; i++)
	{
		if(m_ahEvnt[i] != INVALID_HANDLE_VALUE)
		{
			CloseHandle(m_ahEvnt[i]);
			m_ahEvnt[i] = INVALID_HANDLE_VALUE;
		}
	}
	// Async Serial Communication Port Close
	TerminateComm();
	LeaveCriticalSection( &hCrtclSect );
}

// ��Ʈ�� �����ϰ�, ���� ������ ������ 
// m_ReadData�� ������ �ڿ� MainWnd�� �޽����� ������ Buffer�� ������
// �о��� �Ű��Ѵ�.
DWORD	ComRecvThread(CPRIMUSCommunication* pPRIMUSCom)
{
	DWORD		dwEvent;
	OVERLAPPED	os;
	BOOL		bOk = TRUE;
	UCHAR		szBlock[MAX_COMM_RECV_BUFF];
	UCHAR		szRecv[MAX_COMM_RECV_BUFF];
	DWORD		dwRead;	 // ���� ����Ʈ��.
	ULONG		ulFirstCount = 0, ulLastCount;
	ULONG		ulLength, ulFixedLength;

 	if( pPRIMUSCom->m_hComm == INVALID_HANDLE_VALUE )
	{
		//debug( "CommRecvThread : Com port handle is invalid, thread exit"  );
		return FALSE;
	}

    // Event, OS ����.
	memset( &os, 0, sizeof(OVERLAPPED));
	if (! (os.hEvent = CreateEvent( NULL, TRUE, FALSE, NULL)))
		bOk = FALSE;
	if (! SetCommMask( pPRIMUSCom->m_hComm, EV_RXCHAR))
		bOk = FALSE;
	if (! bOk)
	{
		//debug( "CommRecvThread : Error while creating CommRecvThread" );
		return FALSE;
	}

	// ��Ʈ�� �����ϴ� ����.
	pPRIMUSCom->m_bConnected = TRUE;
	while (pPRIMUSCom->m_bConnected)
	{
		// ��Ʈ�� ���� �Ÿ��� �ö����� ��ٸ���.
		WaitCommEvent( pPRIMUSCom->m_hComm, &dwEvent, NULL);
		if( (dwEvent & EV_RXCHAR) != EV_RXCHAR ) continue;

		dwRead = pPRIMUSCom->ReadComm( &szRecv[pPRIMUSCom->m_ulReceivedCharCount],
			MAX_COMM_RECV_BUFF - pPRIMUSCom->m_ulReceivedCharCount );
		if( dwRead == 0 )
		{
			continue;
		}else{
			pPRIMUSCom->m_ulReceivedCharCount += dwRead;
			szRecv[pPRIMUSCom->m_ulReceivedCharCount] = 0;
		}
		
		ulFixedLength = START_MASK_SIZE + LENGTH_SIZE + CHECK_SUM_SIZE + END_MASK_SIZE;
		for( ulFirstCount = 0, ulLastCount = 0;
			ulLastCount < pPRIMUSCom->m_ulReceivedCharCount; ulLastCount++ )
		{
			if( (szRecv[ulLastCount] == CR) && (ulLastCount >= (ulFixedLength-1)) )
			{
				switch( szRecv[ulFirstCount] )
				{
				case '$': // Response Start Mask
					ulLength = (szRecv[ulFirstCount+1] | (szRecv[ulFirstCount+2] << 8));
					if(ulLastCount >= (ulLength+ulFixedLength-1))
					{
						memcpy( szBlock, &szRecv[ulFirstCount], ulLastCount - ulFirstCount );
						pPRIMUSCom->m_qRecv.AddSendQueue( &szRecv[ulFirstCount], ulLastCount - ulFirstCount);
						pPRIMUSCom->ReceivedBLOCK();
						//debug( "CommRecvThread1 : Recv = %s", szRecv );
						ulFirstCount = ulLastCount + 1;
					}
					break;			
				}
			}
		}
		if( ulFirstCount < ulLastCount )
		{
			// ������ Message �߿� ��ó���� ���� �κ��� ���� ���
			// �� ���� �κ��� ���� Buffer ������ ����, �������� ��� ���Ŵ��
			//debug( "ulFirstCount < ulLastCount" );
			pPRIMUSCom->m_ulReceivedCharCount = ulLastCount - ulFirstCount;
			memcpy( &szRecv[0], &szRecv[ulFirstCount], pPRIMUSCom->m_ulReceivedCharCount );
		}else{
			pPRIMUSCom->m_ulReceivedCharCount = 0; 
		}
	} // end of while()
	
	// ��Ʈ�� Terminate�� ���� ������ m_bConnected �� FALSE�� �Ǿ� ����.
	CloseHandle( os.hEvent);

	return 0;
} // end of CommRecvThread()

void CPRIMUSCommunication::ReceivedBLOCK()
{
	ULONG		ulBlockSize;
	UCHAR		szBlock[MAX_COMM_RECV_BUFF];

	if( m_qRecv.GetSendQueue( szBlock, &ulBlockSize ) == FALSE )
	{
		//debug( "ReceivedBLOCK: GetSendQueue Error" );
		return;
	}
	if( ulBlockSize < 1 )
	{
		// �ʹ� ª�� Message Block ����
		//debug( "ReceivedBLOCK : �ʹ� ª�� Message Block ����, ���� = %d", ulBlockSize );
		return;
	}
	if( ulBlockSize >= MAX_COMM_RECV_BUFF ) 
	{
		// �ʹ� �� Message Block ����
		//debug( "ReceivedBLOCK() : �ʹ� �� Message Block ����, ���� = %d", ulBlockSize );
		return;
	}
	unsigned char  ucResponsePosition;
	ucResponsePosition = START_MASK_SIZE + LENGTH_SIZE;
	switch(szBlock[ucResponsePosition])
	{
	case RESPONSE_PARAMETER_SET:
		switch(szBlock[ucResponsePosition+1]) // Parameter
		{
		case CPS_WRITE:
			S_PRIMUSParameter  *sParaDumy;
			sParaDumy = (S_PRIMUSParameter *)&szBlock[0];
			memcpy(&s_PRIMUSParameter, sParaDumy, sizeof(S_PRIMUSParameter));
			//debug("RESPONSE_PARAMETER_SET,CPS_WRITE");
			break;
		}
		break;
	case RESPONSE_PARAMETER_READ:
		switch(szBlock[ucResponsePosition+1]) // Parameter
		{
		case CPR_READ:
			S_PRIMUSParameter  *sParaDumy;
			sParaDumy = (S_PRIMUSParameter *)&szBlock[0];
			memcpy(&s_PRIMUSParameter, sParaDumy, sizeof(S_PRIMUSParameter));
			//debug("RESPONSE_PARAMETER_SET,CPR_READ");
			break;
		}
		break;
	case RESPONSE_MONITORING:
		switch(szBlock[ucResponsePosition+1])
		{
		case CM_PRIMUS_STATUS:
		case CM_SBC_STATUS:
			S_PRIMUSStatus  *sDumy;
			sDumy = (S_PRIMUSStatus *)&szBlock[0];
			memcpy(&s_PRIMUSStatus, sDumy, sizeof(S_PRIMUSStatus));
			//debug("RESPONSE_MONITORING,CM_PRIMUS_STATUS");
			break;
		}
		break;
	case RESPONSE_ERROR:
		S_ERRORStatus  *sDumy;
		switch(szBlock[ucResponsePosition+1])
		{
		case CE_WARNING:
			sDumy = (S_ERRORStatus *)&szBlock[0];
			memcpy(&s_ERRORStatus, sDumy, sizeof(S_ERRORStatus));
			//debug("RESPONSE_ERROR,CE_WARNING");
			break;
		case CE_ALARM:
			sDumy = (S_ERRORStatus *)&szBlock[0];
			memcpy(&s_WarningStatus, sDumy, sizeof(S_ERRORStatus));
			//debug("RESPONSE_ERROR,CE_ALARM");
			break;
		}
		break;
	}
}

BOOL CPRIMUSCommunication::StartSendCommand(BOOL bRetry)
{
	SEND_QUEUE_COMMAND	sqcSend;
	ULONG	ulTemp;

	if( m_qSend.GetQueueSize() == 0 )
	{
		// ���̻� ���� ����� ���� ���
		if( m_hComm == INVALID_HANDLE_VALUE )
			SetCompletionWaitTime();
		else
			SetCompletionWaitTime();
			//debug( "StartSendCommand: Nothing left to send" );
		return FALSE;
	}
	
	if( bRetry == TRUE )
	{
		// Time Out 
		if( m_SendingState == 1 )
		{
			if( m_qSend.ReadSendQueue( (UCHAR*)&sqcSend, 0, &ulTemp ) == FALSE )
			{
				//debug( "StartSendCommand: ReadSendQueue Error" );
				return FALSE;
			}
			if(m_usLastCommand == sqcSend.usCommand)
			{
				return FALSE;
			}
			WriteComm( sqcSend.szBuff, sqcSend.ulSize );
			SetAckWaitTime(); 
			m_SendingState = 1;
			sqcSend.szBuff[sqcSend.ulSize] = 0;
			m_usLastCommand = sqcSend.usCommand;
			//debug( "StartSendCommand: Sent Command = %s, Sending State = %d", sqcSend.szBuff, m_SendingState );
	
			return TRUE;
		}else{
			//debug( "StartSendCommand: Retry Timeout ������ �۽Ż��� %d, ���� Reset", m_SendingState  );
			m_SendingState = 0;

			SetCompletionWaitTime(); 

			return FALSE;
		}
	}else{
		if( m_SendingState == 0 )
		{
			if( m_qSend.ReadSendQueue( (UCHAR*)&sqcSend, 0, &ulTemp ) == FALSE )
			{
				//debug( "StartSendCommand: ReadSendQueue Error" );
				return FALSE;
			}
			WriteComm( sqcSend.szBuff, sqcSend.ulSize );
			//m_SendingState = 1;
			sqcSend.szBuff[sqcSend.ulSize] = 0;
			m_usLastCommand = sqcSend.usCommand;
			//debug( "StartSendCommand: Sent Command = %s, Sending State = %d", sqcSend.szBuff, m_SendingState );

			return TRUE;
		}else{
			//debug( "StartSendCommand: Command sequence is processing, can not send another command"  );
			return FALSE;
		}
	}
	return FALSE;
}

void CPRIMUSCommunication::SetCompletionWaitTime()
{
	LARGE_INTEGER	liDueTime;

	liDueTime.QuadPart = -50000000; // 5 sec
	BOOL bReturn = SetWaitableTimer( m_ahEvnt[1], &liDueTime, 0, NULL, 0, FALSE );
}

void CPRIMUSCommunication::SetAckWaitTime()
{
	LARGE_INTEGER	liDueTime;

	liDueTime.QuadPart = -50000000; // 5 sec
	BOOL bReturn = SetWaitableTimer( m_ahEvnt[0], &liDueTime, 0, NULL, 0, FALSE );
}

BOOL CPRIMUSCommunication::ManualCommand(unsigned char ucCommand, unsigned char ucParameter, unsigned char ucVariable, unsigned char ucVariable2, unsigned char ucVariable3)
{
	EnterCriticalSection( &hCrtclSect );
	SEND_QUEUE_COMMAND	sqcSend;

	WORD chsum =0;
	WORD		byHighMask = 0xFF00;
	WORD		byLowMask = 0x00FF;
	BYTE		byHighChSum;
	BYTE		byLowChSum;
	char cSyntaxMessage[128];
	int i;
	unsigned short usTotalSize, usLength;
	unsigned char ucSendChar[3] = {'O', 'F', 'A'};

	if(m_bConnected == FALSE)
	{
		//debug("Com port is not opened");
		LeaveCriticalSection( &hCrtclSect );
		return FALSE;
	}

	switch(ucCommand)
	{
	case COMMAND_PARAMETER_SET:
		switch(ucParameter)
		{
		case CPS_WRITE:
			usTotalSize = sizeof(s_PRIMUSParameter);
			s_PRIMUSParameter.StartMask = '$';
			s_PRIMUSParameter.length =  (usTotalSize - START_MASK_SIZE - LENGTH_SIZE - CHECK_SUM_SIZE - END_MASK_SIZE); //��Ʋ �����
			s_PRIMUSParameter.Command = ucCommand;
			s_PRIMUSParameter.Parameter = ucParameter;
			s_PRIMUSParameter.EndMask = CR;
			memcpy(cSyntaxMessage, (char*)&s_PRIMUSParameter, sizeof(s_PRIMUSParameter));
			break;
		case CPS_MODE:
			usTotalSize = START_MASK_SIZE + LENGTH_SIZE + START_COMMAND_PARAMETER_SIZE + 1 + CHECK_SUM_SIZE + END_MASK_SIZE;
			usLength = START_COMMAND_PARAMETER_SIZE + 1;
			cSyntaxMessage[0] = '$';
			cSyntaxMessage[1] = ( usLength & byLowMask);
			cSyntaxMessage[2] = ( usLength & byHighMask) >> 8;
			cSyntaxMessage[3] = ucCommand;
			cSyntaxMessage[4] = ucParameter;
			cSyntaxMessage[5] = ucVariable;
			/*
			enum{
				OP_MODE_READY=0,	// �����
				OP_MODE_RUN,		// ������
				OP_MODE_BOOT,       // Power on��
				OP_MODE_RESET,      // Reset���
				OP_MODE_DEBUG,		// �ý��� ������ AGV������ �������� �ʴ� ����
			};
			*/
			cSyntaxMessage[8] = CR;
			break;
		}
		break;
	case COMMAND_PARAMETER_READ:
		usTotalSize = START_MASK_SIZE + LENGTH_SIZE + START_COMMAND_PARAMETER_SIZE + CHECK_SUM_SIZE + END_MASK_SIZE;
		usLength = START_COMMAND_PARAMETER_SIZE;
		cSyntaxMessage[0] = '$';
		cSyntaxMessage[1] = ( usLength & byLowMask);
		cSyntaxMessage[2] = ( usLength & byHighMask) >> 8;
		cSyntaxMessage[3] = ucCommand;
		cSyntaxMessage[4] = ucParameter;
		cSyntaxMessage[7] = CR;
		break;
	case COMMAND_MONITORING:
		switch(ucParameter)
		{
		case CM_PRIMUS_STATUS:
			usTotalSize = START_MASK_SIZE + LENGTH_SIZE + START_COMMAND_PARAMETER_SIZE + CHECK_SUM_SIZE + END_MASK_SIZE;
			usLength = START_COMMAND_PARAMETER_SIZE;
			cSyntaxMessage[0] = '$';
			cSyntaxMessage[1] = ( usLength & byLowMask);
			cSyntaxMessage[2] = ( usLength & byHighMask) >> 8;
			cSyntaxMessage[3] = ucCommand;
			cSyntaxMessage[4] = ucParameter;
			cSyntaxMessage[7] = CR;
			break;
		case CM_SBC_STATUS:
			usTotalSize = sizeof(S_SBCStatus);
			s_SBCStatus.StartMask = '$';
			s_SBCStatus.length =  (usTotalSize - START_MASK_SIZE - LENGTH_SIZE - CHECK_SUM_SIZE - END_MASK_SIZE); //��Ʋ �����
			s_SBCStatus.Command = ucCommand;
			s_SBCStatus.Parameter = ucParameter;
			s_SBCStatus.EndMask = CR;
			memcpy(cSyntaxMessage, (char*)&s_SBCStatus, sizeof(s_SBCStatus));
			break;
		}
		break;
	case COMMAND_IO_CONTROL:
		switch(ucParameter)
		{
		case CIC_LOW:
			usTotalSize = START_MASK_SIZE + LENGTH_SIZE + START_COMMAND_PARAMETER_SIZE + 3 + CHECK_SUM_SIZE + END_MASK_SIZE;
			usLength = START_COMMAND_PARAMETER_SIZE +3;
			cSyntaxMessage[0] = '$';
			cSyntaxMessage[1] = ( usLength & byLowMask);
			cSyntaxMessage[2] = ( usLength & byHighMask) >> 8;
			cSyntaxMessage[3] = ucCommand;
			cSyntaxMessage[4] = ucParameter;
			cSyntaxMessage[5] = ucVariable; // Port No
			cSyntaxMessage[6] = ucVariable2; // Off time
			cSyntaxMessage[7] = ucVariable3; // On time
			cSyntaxMessage[10] = CR;
			break;
		case CIC_HIGH: // ������� ����
			break;
		case CIC_POWER:
			usTotalSize = START_MASK_SIZE + LENGTH_SIZE + START_COMMAND_PARAMETER_SIZE + 2 + CHECK_SUM_SIZE + END_MASK_SIZE;
			usLength = START_COMMAND_PARAMETER_SIZE +2;
			cSyntaxMessage[0] = '$';
			cSyntaxMessage[1] = ( usLength & byLowMask);
			cSyntaxMessage[2] = ( usLength & byHighMask) >> 8;
			cSyntaxMessage[3] = ucCommand;
			cSyntaxMessage[4] = ucParameter;
			cSyntaxMessage[5] = ucVariable; // Port No
			cSyntaxMessage[6] = ucSendChar[ucVariable2]; // On(O), Off(F), Auto(A)
			cSyntaxMessage[9] = CR;
			break;
		}
		break;
	}

	//CheckSum
	for(i=1; i<(usTotalSize-CHECK_SUM_SIZE-END_MASK_SIZE); i++)
	{
		chsum += (unsigned char)cSyntaxMessage[i];
	}
	byHighChSum = ( chsum & byHighMask) >> 8;
	byLowChSum = ( chsum & byLowMask);
	cSyntaxMessage[usTotalSize-END_MASK_SIZE-CHECK_SUM_SIZE] = byLowChSum ;
	cSyntaxMessage[usTotalSize-END_MASK_SIZE-CHECK_SUM_SIZE+1] = byHighChSum;

	m_qSend.DeleteSendQueue( 0 );

	// Send Queue�� Add
	sqcSend.usCommand = (unsigned short)(ucCommand);
	sqcSend.ulSize = usTotalSize;
	memcpy( sqcSend.szBuff, cSyntaxMessage, usTotalSize);
	if( m_qSend.AddSendQueue( (UCHAR*)&sqcSend, sizeof(SEND_QUEUE_COMMAND) ) == FALSE )
	{
		//debug( "ManualCommand: AddSendQueue Error"  );
	}else{
		StartSendCommand( FALSE );
	}
	LeaveCriticalSection( &hCrtclSect );
	return TRUE;
}

void CPRIMUSCommunication::ParameterDefault()
{
	int i=0;
	s_PRIMUSParameter.main_pwr_warning = 220;        // Main Power ��� ��������, ���� 0.1V
	s_PRIMUSParameter.main_pwr_alarm=200;          // Main Power �˶� ��������, ���� 0.1V    
	
	s_PRIMUSParameter.sub_bat_temp_max_warning=70;// Sub Battery �µ� ��� �ִ밪 , ���� 1��
	s_PRIMUSParameter.sub_bat_temp_max_alarm=80;  // Sub Battery �µ� �˶� �ִ밪 , ���� 1��
	
	s_PRIMUSParameter.sub_bat_temp_min_warning=20;// Sub Battery �µ� ��� �ּҰ� , ���� 1��
	s_PRIMUSParameter.sub_bat_temp_min_alarm=10;  // Sub Battery �µ� �˶� �ּҰ� , ���� 1��
	
	s_PRIMUSParameter.primus_temp_max_warning=60; // PRIMUS �µ� ��� �ִ밪      , ���� 0.1��
	s_PRIMUSParameter.primus_temp_max_alarm=50;   // PRIMUS �µ� �˶� �ִ밪      , ���� 0.1��
	
	s_PRIMUSParameter.primus_temp_min_warning=19; // PRIMUS �µ� ��� �ּҰ�      , ���� 0.1��
	s_PRIMUSParameter.primus_temp_min_alarm=8;   // PRIMUS �µ� �˶� �ּҰ�      , ���� 0.1��
	
	s_PRIMUSParameter.sbc_cpu_warning=72; // SBC CPU ��뷮 ���         
	s_PRIMUSParameter.sbc_cpu_alarm=85;   // SBC CPU ��뷮 �˶�         
	s_PRIMUSParameter.sbc_hdd_warning=68; // SBC HDD ��뷮 ���         
	s_PRIMUSParameter.sbc_hdd_alarm=85;   // SBC HDD ��뷮 �˶�         
	
	s_PRIMUSParameter.sbc_mem_warning=81; // SBC �޸� ��뷮 ���      
	s_PRIMUSParameter.sbc_mem_alarm=92;   // SBC �޸� ��뷮 �˶�      
	s_PRIMUSParameter.power_off_op= 2;    // �����������ܽ� ���ۼ���     
	s_PRIMUSParameter.fan_temp = 24;        // �� ���� �µ�                     

	s_PRIMUSParameter.run_timeout = 10000;      // ��� timeout ��(RUN���)    , ���� 1ms
	s_PRIMUSParameter.ready_timeout = 20000;    // ��� timeout ��(Ready���)  , ���� 1ms
	s_PRIMUSParameter.reset_timeout = 30000;    // ��� timeout ��(Reset���)  , ���� 1ms
	s_PRIMUSParameter.boot_timeout = 50000;     // ��� timeout ��(boot���)   , ���� 1ms
	s_PRIMUSParameter.sensor_time = 1500;      // �µ����� ���� üũ�ð� ���� , ���� 1ms
	s_PRIMUSParameter.input_vlu_time = 2300;   // �Է���Ʈ üũ �ð� ����     , ���� 1ms
	s_PRIMUSParameter.relay_on_time= 9000;    // ��������Ʈ ���� �ð�        , ���� 1ms

	for(i=0; i<12; i++)
	{
		s_PRIMUSParameter.out_op[i] =0x00;      // �����Ʈ ����             
	}
	for(i=0; i<22; i++)
	{
		s_PRIMUSParameter.power_op[22] =0x00;    // ������� ���� ����          
	}
	s_PRIMUSParameter.dummy1 =0;
	s_PRIMUSParameter.dummy2 =0;
}
	