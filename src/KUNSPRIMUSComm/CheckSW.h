// CheckSW.h: interface for the CCheckSW class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_CHECKSW_H__677C3EBD_97C4_44D7_B2F5_EBA38AFFEA46__INCLUDED_)
#define AFX_CHECKSW_H__677C3EBD_97C4_44D7_B2F5_EBA38AFFEA46__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

// SW�� ���� ����
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
	// �ܺο��� SW�� ��� �ִٴ� ���� �˸��� ���ؼ��� 
	// ���� ���ǵ� SW_REPLY_CHECK_TIMEOUT ���� ������ �Ʒ� �Լ��� ����Ͽ� iState�� 0 ���� ū ���� ������ �Ѵ�. 
	void	SetSWReply(int iState)
	{
		m_iSWReply = iState;
		m_ulTimeOut =  GetTickCount()+ SW_REPLY_CHECK_TIMEOUT;
	}
	
	// CheckSW ���� ó������ �Ʒ� �ʱ�ȭ �Լ��� ����ؾ� �Ѵ�.
	void	Initialize(int iState)
	{
		m_iCurrentState = iState;
		m_iSWReply = 1;
		m_ulTimeOut = GetTickCount()+ SW_REPLY_CHECK_TIMEOUT;
		ThreadStart();
	}

	// CheckSW ���� ���������� �Ʒ� ���� �Լ��� ����ؾ� �Ѵ�.
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
	int		m_iCurrentState;	 // SW�� ���� ���� ǥ��
	int		m_iSWReply;	 // SW�� reply
	unsigned long	m_ulTimeOut; // SW�� reply�� ������ Ȯ���ϴ� �ð�
};

#endif // !defined(AFX_CHECKSW_H__677C3EBD_97C4_44D7_B2F5_EBA38AFFEA46__INCLUDED_)
