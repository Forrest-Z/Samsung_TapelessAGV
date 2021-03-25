// CheckSW.cpp: implementation of the CCheckSW class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "CheckSW.h"

#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CCheckSW::CCheckSW()
{
	m_ulTimeOut = 0;
	m_iCurrentState = SW_STATE_UNKNOWN;
	m_iSWReply = 0;
}

CCheckSW::~CCheckSW()
{

}

int CCheckSW::DoEvents(void)
{
	Sleep(300);
	if(m_ulTimeOut > 0)
	{
		if((m_ulTimeOut <= GetTickCount()) || (m_iSWReply == 0))
		{
			m_iSWReply = 0;
			m_iCurrentState = SW_STATE_TURN_OFF;
		}
		else if(m_iSWReply != 0)
		{
			m_iCurrentState = SW_STATE_TURN_ON;
		}
	}
	return 0;
}