#include "StdAfx.h"
#include "SendQueue.h"

/////////////////////////////////////////////////////////////////////////////
// CSendQueue construction/destruction
 
CSendQueue::CSendQueue()
{
	m_ppBuff = NULL;
	m_pBuffSize = NULL;
	m_ulHead = 0;
	m_ulTail = 0;
	m_ulQty = 0;
	InitializeCriticalSection( &m_hCrtclSect );
} // end of CSendQueue::CSendQueue()

CSendQueue::~CSendQueue( void )
{
	ResetQueue();
	DeleteCriticalSection( &m_hCrtclSect );
	if( m_ppBuff != NULL ) free( m_ppBuff );
	if( m_pBuffSize != NULL ) free( m_pBuffSize );
} // end of CSendQueue::~CSendQueue()

/* 
 * Queue를 사용하기 위해 buffer를 초기화 하는 함수
 */
void	CSendQueue::InitializeQueue( ULONG ulQty, ULONG ulRecordSize )
{
	EnterCriticalSection( &m_hCrtclSect );
	m_ulIncQty = ulQty;
	m_ulMaxQty = ulQty;
	m_ulMaxRecordSize = ulRecordSize;
	m_ppBuff = (UCHAR**)malloc( m_ulMaxQty * sizeof(UCHAR*) );
	m_pBuffSize = (ULONG*)malloc( sizeof(ULONG) * m_ulMaxQty );
	if( m_ppBuff != NULL )
		memset( m_ppBuff, 0, m_ulMaxQty * sizeof(UCHAR*) );
	if( m_pBuffSize != NULL )
		memset( m_pBuffSize, 0, sizeof(UINT) * m_ulMaxQty );
	LeaveCriticalSection( &m_hCrtclSect );
} // end of CSendQueue::InitializeQueue()

/* 
 * Queue의 buffer를  reset하는함수
 */
void CSendQueue::ResetQueue( void )
{
	ULONG	ulLoc;
	ULONG	ulIndex;

	EnterCriticalSection( &m_hCrtclSect );
	for( ulIndex = 0; ulIndex < m_ulQty; ulIndex++ )
	{
		ulLoc = m_ulTail + ulIndex;
		if( ulLoc >= m_ulMaxQty ) ulLoc -= m_ulMaxQty;
		if( m_ppBuff[ulLoc] != NULL )
		{
			delete	m_ppBuff[ulLoc];
			m_ppBuff[ulLoc] = NULL;
			m_pBuffSize[ulLoc] = 0;
		}
	}
	m_ulHead = 0;
	m_ulTail = 0;
	m_ulQty = 0;
	LeaveCriticalSection( &m_hCrtclSect );
} // end of CSendQueue::ResetQueue()

/* 
 * Queue의 buffer에 저장되어있는 값을 pData에 옮기고 Data는 삭제하는 함수
 */
BOOL CSendQueue::GetSendQueue( UCHAR* pData, ULONG* pulSize, ULONG ulReadSize )
{
	ULONG	ulCount;
	UCHAR*	q;

	if( m_ulQty == 0 ) return FALSE;

	EnterCriticalSection( &m_hCrtclSect );
	q = m_ppBuff[m_ulTail];
	if( q == NULL )
	{
		LeaveCriticalSection( &m_hCrtclSect );
		return FALSE;
	}
	if( ulReadSize > 0 )
	{
		*pulSize = min(ulReadSize, m_pBuffSize[m_ulTail]);
	}else{
		*pulSize = m_pBuffSize[m_ulTail];
	}
	for( ulCount = 0; ulCount < *pulSize; ulCount++ )
	{
		*pData++ = *q++;
	}
	delete m_ppBuff[m_ulTail];
	m_ppBuff[m_ulTail] = NULL;
	m_pBuffSize[m_ulTail] = 0;
	if( ++m_ulTail >= m_ulMaxQty ) m_ulTail = 0;
	m_ulQty--;
	LeaveCriticalSection( &m_hCrtclSect );
	return TRUE;
} // end of CSendQueue::GetSendQueue()

/* 
 * Queue의 buffer에 저장되어있는 값을 pData에 저장하는 함수
 */

BOOL CSendQueue::ReadSendQueue( UCHAR* pData, ULONG ulIndex, ULONG* pulSize )
{
	ULONG	ulCount;
	ULONG	ulLoc;
	UCHAR*	q;

	if( m_ulQty == 0 ) return FALSE;

	EnterCriticalSection( &m_hCrtclSect );
	ulLoc = m_ulTail + ulIndex;
	if( ulLoc >= m_ulMaxQty ) ulLoc -= m_ulMaxQty;
	q = m_ppBuff[ulLoc];
	if( q == NULL )
	{
		LeaveCriticalSection( &m_hCrtclSect );
		return FALSE;
	}
	*pulSize = m_pBuffSize[ulLoc];
	for( ulCount = 0; ulCount < *pulSize; ulCount++ )
	{
		*pData++ = *q++;
	}
	LeaveCriticalSection( &m_hCrtclSect );
	return TRUE;
} // end of CSendQueue::ReadSendQueue()

/* 
 * Queue의 buffer에 저장되어있는 값을 삭제하는 함수
 */

BOOL CSendQueue::DeleteSendQueue( ULONG ulIndex )
{
	if( m_ulQty == 0 ) return FALSE;

	EnterCriticalSection( &m_hCrtclSect );
	if( m_ppBuff[m_ulTail] != NULL )
	{
		delete m_ppBuff[m_ulTail];
		m_ppBuff[m_ulTail] = NULL;
		m_pBuffSize[m_ulTail] = 0;
	}
	if( ++m_ulTail >= m_ulMaxQty ) m_ulTail = 0;
	m_ulQty--;
	LeaveCriticalSection( &m_hCrtclSect );
	return TRUE;
} // end of CSendQueue::DeleteSendQueue()

BOOL CSendQueue::ExpandQueue()
{
	ULONG	ulLoc;

	// 새로운 Pointer Buffer를 만든다.
	UCHAR**	ppBuff = (UCHAR**)malloc( (m_ulMaxQty + m_ulIncQty) * sizeof(UCHAR*) );
	if( ppBuff == NULL ) return FALSE;
	ULONG*	pBuffSize = (ULONG*)malloc( (m_ulMaxQty + m_ulIncQty) * sizeof(ULONG) );
	if( pBuffSize == NULL )
	{
		free( ppBuff );
		return FALSE;
	}

	// 새로운 Pointer Buffer로 기존 것을 Re-ordering 배치한다.
	for( unsigned long ulIndex = 0; ulIndex < m_ulQty; ulIndex++ )
	{
		ulLoc = m_ulTail + ulIndex;
		if( ulLoc >= m_ulMaxQty ) ulLoc -= m_ulMaxQty;
		ppBuff[ulIndex] = m_ppBuff[ulLoc];
		pBuffSize[ulIndex] = m_pBuffSize[ulLoc];
	}
	// 기존 Pointer Buffer는 삭제
	delete	m_ppBuff;
	delete	m_pBuffSize;

	// 새로운 Pointer Buffer를 Pointer Buffer 변수에 저장.
	m_ppBuff = ppBuff;
	m_pBuffSize = pBuffSize;
	m_ulTail = 0;
	m_ulHead = m_ulQty;

	// Pointer Buffer의 크기를 늘려잡음
	m_ulMaxQty += m_ulIncQty;
	return TRUE;
} // end of CSendQueue::ExpandQueue()

/* 
 * Queue의 buffer에 Data를 저장하는 함수
 */

BOOL CSendQueue::AddSendQueue( UCHAR* pData, ULONG ulSize )
{
	ULONG	ulCount;
	UCHAR*	q;

	EnterCriticalSection( &m_hCrtclSect );
	if( m_ulQty == m_ulMaxQty )
	{
		if( ExpandQueue() == FALSE )
		{
			LeaveCriticalSection( &m_hCrtclSect );
			return FALSE;
		}
	}
	q = (UCHAR*)malloc( m_ulMaxRecordSize );
	if( q == NULL )
	{
		LeaveCriticalSection( &m_hCrtclSect );
		return FALSE;
	}
	m_ppBuff[m_ulHead] = q;

	// Queue에 넣는 Record의 Size는 m_ulMaxRecordSize보다 크면 안된다.
	if( ulSize > m_ulMaxRecordSize )
		m_pBuffSize[m_ulHead] = m_ulMaxRecordSize;
	else
		m_pBuffSize[m_ulHead] = ulSize;

	for( ulCount = 0; ulCount < ulSize; ulCount++ )
	{
		*q++ = *pData++;
	}
	if( ++m_ulHead >= m_ulMaxQty ) m_ulHead = 0;
	m_ulQty++;
	LeaveCriticalSection( &m_hCrtclSect );
	return TRUE;
} // end of CSendQueue::AddSendQueue()

/* 
 * Queue의 buffer에 저장되어있는 값을 삭제하는 함수(
 */

BOOL CSendQueue::PushSendQueue( UCHAR* pData, ULONG ulSize )
{
	ULONG	ulCount;
	UCHAR*	q;

	EnterCriticalSection( &m_hCrtclSect );
	if( m_ulQty == m_ulMaxQty )
	{
		if( ExpandQueue() == FALSE )
		{
			LeaveCriticalSection( &m_hCrtclSect );
			return FALSE;
		}
	}
	q = (UCHAR*)malloc( m_ulMaxRecordSize );
	if( q == NULL )
	{
		LeaveCriticalSection( &m_hCrtclSect );
		return FALSE;
	}
	if( m_ulTail == 0 ) m_ulTail = m_ulMaxQty - 1;

	m_ppBuff[m_ulTail] = q;

	// Queue에 넣는 Record의 Size는 m_ulMaxRecordSize보다 크면 안된다.
	if( ulSize > m_ulMaxRecordSize )
		m_pBuffSize[m_ulTail] = m_ulMaxRecordSize;
	else
		m_pBuffSize[m_ulTail] = ulSize;

	for( ulCount = 0; ulCount < ulSize; ulCount++ )
	{
		*q++ = *pData++;
	}
	m_ulQty++;
	LeaveCriticalSection( &m_hCrtclSect );
	return TRUE;
} // end of CSendQueue::PushSendQueue()

/* 
 * Queue의 buffer에 저장되어있는 값의 Size를 구하는 함수
 */
ULONG CSendQueue::GetQueueRecordSize( ULONG ulIndex )
{
	ULONG ulLoc = m_ulTail + ulIndex;
	if( ulLoc >= m_ulMaxQty ) ulLoc -= m_ulMaxQty;
	return m_pBuffSize[ulLoc];
} // end of CSendQueue::GetQueueRecordSize()

/* 
 * Queue의 buffer에 저장되어있는 값을 구하는 함수
 */
UCHAR* CSendQueue::GetQueuePtr( ULONG ulIndex )
{
	ULONG ulLoc = m_ulTail + ulIndex;
	if( ulLoc >= m_ulMaxQty ) ulLoc -= m_ulMaxQty;
	return m_ppBuff[ulLoc];
} // end of CSendQueue::GetQueuePtr()

BOOL CSendQueue::AddAnotherQueue( CSendQueue* pAnother )
{
	ULONG	ulIndex;
	ULONG	ulSize;

	EnterCriticalSection( &m_hCrtclSect );
	for( ulIndex = 0; ulIndex < pAnother->GetQueueSize(); ulIndex++ )
	{
		if( m_ulQty == m_ulMaxQty )
		{
			if( ExpandQueue() == FALSE )
			{
				LeaveCriticalSection( &m_hCrtclSect );
				return FALSE;
			}
		}
		m_ppBuff[m_ulHead] = pAnother->GetQueuePtr( ulIndex );
		ulSize = pAnother->GetQueueRecordSize( ulIndex );
		// Queue에 넣는 Record의 Size는 m_ulMaxRecordSize보다 크면 안된다.
		if( ulSize > m_ulMaxRecordSize )
			m_pBuffSize[m_ulHead] = m_ulMaxRecordSize;
		else
			m_pBuffSize[m_ulHead] = ulSize;
		if( ++m_ulHead >= m_ulMaxQty ) m_ulHead = 0;
		m_ulQty++;
	}
	LeaveCriticalSection( &m_hCrtclSect );
	pAnother->ResetQueue();
	return TRUE;
} // end of CSendQueue::AddAnotherQueue()

