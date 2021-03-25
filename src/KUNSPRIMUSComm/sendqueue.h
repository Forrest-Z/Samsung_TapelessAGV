
#ifndef __SEND_QUEUE_H__
#define __SEND_QUEUE_H__
#include "Windows.h"

#define	MAX_COMM_RECV_BUFF			128

typedef	struct
{
	USHORT	usCommand;
	ULONG	ulSize;
	UCHAR	szBuff[128];
}SEND_QUEUE_COMMAND;
 
class	CSendQueue;
class	CSendQueue
{
private:
	CRITICAL_SECTION m_hCrtclSect;
	ULONG	m_ulIncQty;
	ULONG	m_ulMaxQty;
	ULONG	m_ulMaxRecordSize;
	ULONG	m_ulHead;
	ULONG	m_ulTail;
	ULONG	m_ulQty;
	UCHAR	**m_ppBuff;
	ULONG	*m_pBuffSize;

public:
	CSendQueue();
	~CSendQueue( void );

	// Queue Property
	inline ULONG	GetQueueSize() { return m_ulQty; };
	inline ULONG	GetQueueMaxSize() { return m_ulMaxQty; };
	inline ULONG	GetQueueMaxRecordSize() { return m_ulMaxRecordSize; };
	
	// Queue Operation
	void	InitializeQueue( ULONG ulQty, ULONG ulRecordSize );
	void	ResetQueue( void );
	BOOL	ReadSendQueue( UCHAR* pData, ULONG ulIndex, ULONG* pulSize );
	BOOL	DeleteSendQueue( ULONG ulIndex );
	BOOL	GetSendQueue( UCHAR* pData, ULONG* pulSize, ULONG ulReadSize = 0 );
	BOOL	ExpandQueue();
	BOOL	AddSendQueue( UCHAR* pData, ULONG ulSize );
	BOOL	PushSendQueue( UCHAR* pData, ULONG ulSize );

	ULONG	GetQueueRecordSize( ULONG ulIndex );
	UCHAR*	GetQueuePtr( ULONG ulIndex );
	BOOL	AddAnotherQueue( CSendQueue* pAnother );
};

#endif
