// CheckSW.h: interface for the CCheckSW class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_CHECKSW_H__677C3EBD_97C4_44D7_B2F5_EBA38AFFEA46__INCLUDED_)
#define AFX_CHECKSW_H__677C3EBD_97C4_44D7_B2F5_EBA38AFFEA46__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

// SW의 상태 정의
#define			SW_STATE_MAX_NUMBER			3
#define			SW_STATE_TURN_OFF			0
#define			SW_STATE_TURN_ON			1
#define			SW_STATE_UNKNOWN			2
#define			SW_REPLY_CHECK_TIMEOUT		3000

#include "ControlUnit.h"

class CCheckSW : public CControlUnit
{
public:
	CCheckSW();
	virtual ~CCheckSW();

	int		GetState(void)
	{
		return m_iCurrentState;
	}
	// 외부에서 SW가 살아 있다는 것을 알리기 위해서는 
	// 위의 정의된 SW_REPLY_CHECK_TIMEOUT 보다 빠르게 아래 함수를 사용하여 iState에 0 보다 큰 값을 보내야 한다. 
	void	SetSWReply(int iState)
	{
		m_iSWReply = iState;
		m_ulTimeOut =  GetTickCount()+ SW_REPLY_CHECK_TIMEOUT;
	}
	
	// CheckSW 사용시 처음에는 아래 초기화 함수를 사용해야 한다.
	void	Initialize(int iState)
	{
		m_iCurrentState = iState;
		m_iSWReply = 1;
		m_ulTimeOut = GetTickCount()+ SW_REPLY_CHECK_TIMEOUT;
		ThreadStart();
	}

	// CheckSW 사용시 마지막에는 아래 종료 함수를 사용해야 한다.
	void	Terminate(void)
	{
		ThreadStop();
	}

	int		GetCurrentState(void)
	{
		return m_iCurrentState;
	}

protected:
	virtual int DoEvents(void);
private:
	int		m_iCurrentState;	 // SW의 현재 상태 표시
	int		m_iSWReply;	 // SW의 reply
	unsigned long	m_ulTimeOut; // SW의 reply가 없는지 확인하는 시간
};

#endif // !defined(AFX_CHECKSW_H__677C3EBD_97C4_44D7_B2F5_EBA38AFFEA46__INCLUDED_)
