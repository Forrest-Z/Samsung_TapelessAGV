// CheckPC.h: interface for the CCheckPC class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_CHECKPC_H__A0E2B967_BF68_4D65_BF50_64A8ACF44554__INCLUDED_)
#define AFX_CHECKPC_H__A0E2B967_BF68_4D65_BF50_64A8ACF44554__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "ControlUnit.h"
#include <comdef.h>
#include <Wbemidl.h>
#pragma comment(lib, "wbemuuid.lib")

// PC�� ���� ����
#define			PC_STATE_MAX_NUMBER		3
#define			PC_STATE_TURN_OFF		0
#define			PC_STATE_TURN_ON		1
#define			PC_STATE_UNKNOWN		2

class CCheckPC : public CControlUnit
{
public:
	CCheckPC();
	virtual ~CCheckPC();

	int		GetID(void)
	{
		return m_iPCID;
	}

	void	SetID(int iID)
	{
		m_iPCID = iID;
	}

	CString&	GetName(void)
	{
		return m_csName;
	}

	void		SetName(const CString&  csname)
	{
		m_csName = csname;
	}

	CString&	DisplayHardDiskSpace(void)
	{
		return m_csHardDiskSpace;
	}

	CString&	DisplayMemorySpace(void)
	{
		return m_csMemory;
	}

	CString&	DisplayBootingTime(void)
	{
		return m_csBootingTime;
	}

	CString&	DisplayRunningTime(void)
	{
		return m_csRunningTime;
	}
	// CheckPC ���� ó������ �Ʒ� �ʱ�ȭ �Լ��� ����ؾ� �Ѵ�.
	void	Initialize(void)
	{
		bInitialized = FALSE;

		ulinitVal = NULL;
		ulVal = NULL;
		dUsage = NULL;

		bInitialized = CheckCPUInit();
		ThreadStart();
	}

	// CheckPC ���� ���������� �Ʒ� ���� �Լ��� ����ؾ� �Ѵ�.
	void	Terminate(void)
	{
		ThreadStop();
		if (bInitialized)
		{
			CheckCPUTerminate();
			if (ulinitVal) delete ulinitVal;
			if (ulVal) delete ulVal;
			if (dUsage) delete dUsage;
		}
	}
	double *dUsage;
	BOOL GetInitValues();

	int inline iNumProc()
	{
		return iNproc;
	}

	BOOL inline bOK()
	{
		return bInitialized;
	}

	unsigned char GetCPUUsage(void);
	unsigned char GetHDDUsage(void);
	unsigned char GetMemoryUsage(void);

protected:
	virtual int DoEvents(void);

private:
	int			m_iPCID;
	CString		m_csName;  // PC�� �̸� ǥ�� 
	int			m_iCurrentState;	 // PC�� ���� ���� ǥ��
	int			m_iConfirmState;	// PC�� ���¸� Ȯ���ϱ� ���� ����
	long		m_lLastTurnONTime;   // ���������� ���õ� �ð�
	long		m_lLastTurnOFFTime;  //���������� ����� �ð�
	// CPU ����
	double		m_dCPUTotalUsage; 
	// HardDisk ����
	double		m_dHardDiskUsage;
	CString		m_csHardDiskSpace; // free/total
	// Memory ����
	double		m_dMemoryUsage;
	CString		m_csMemory; //free/total

	// Booting Time
	CString		m_csBootingTime;
	// Running Time
	CString		m_csRunningTime;

	unsigned __int64	m_ui64HardDiskNotUsed;
	unsigned __int64	m_ui64HardDiskTotal;

	int		m_iMemoryNotUsed;
	int		m_iMemoryTotal;

	// CPU Process Counter
	ULONG64 *ulinitVal;
	ULONG64 *ulVal;
	IWbemServices *pSvc;
	IWbemLocator *pLoc;

	int iNproc;
	int iNproc2;
	BOOL bInitialized;

	BOOL GetValueInt(ULONG64 *ul);
	void ShutDown(void);

	int iGetNumberOfCores(void);

	// CPU Check �ʱ�ȭ
	BOOL CheckCPUInit(void);
	// CPU Check ����
	void CheckCPUTerminate(void);	 
	// CPU ��뷮 ����
	void CheckCPUUsage(void);

	void MyAbsSecondToSystem(DWORD Abs, SYSTEMTIME &st);
	DWORD MyGetAbsSecond(SYSTEMTIME st);

	void CheckBootingTimeandRunningTime(void);
	void CheckMemorySpace(void);
	void CheckHardDiskSpace(void);
};

#endif // !defined(AFX_CHECKPC_H__A0E2B967_BF68_4D65_BF50_64A8ACF44554__INCLUDED_)
