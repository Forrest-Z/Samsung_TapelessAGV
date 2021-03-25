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

// PC의 상태 정의
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
	// CheckPC 사용시 처음에는 아래 초기화 함수를 사용해야 한다.
	void	Initialize(void)
	{
		bInitialized = FALSE;

		ulinitVal = NULL;
		ulVal = NULL;
		dUsage = NULL;

		bInitialized = CheckCPUInit();
		ThreadStart();
	}

	// CheckPC 사용시 마지막에는 아래 종료 함수를 사용해야 한다.
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
	CString		m_csName;  // PC의 이름 표시 
	int			m_iCurrentState;	 // PC의 현재 상태 표시
	int			m_iConfirmState;	// PC의 상태를 확인하기 위한 변수
	long		m_lLastTurnONTime;   // 마지막으로 부팅된 시간
	long		m_lLastTurnOFFTime;  //마지막으로 종료된 시간
	// CPU 사용률
	double		m_dCPUTotalUsage; 
	// HardDisk 사용률
	double		m_dHardDiskUsage;
	CString		m_csHardDiskSpace; // free/total
	// Memory 사용률
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

	// CPU Check 초기화
	BOOL CheckCPUInit(void);
	// CPU Check 종료
	void CheckCPUTerminate(void);	 
	// CPU 사용량 조사
	void CheckCPUUsage(void);

	void MyAbsSecondToSystem(DWORD Abs, SYSTEMTIME &st);
	DWORD MyGetAbsSecond(SYSTEMTIME st);

	void CheckBootingTimeandRunningTime(void);
	void CheckMemorySpace(void);
	void CheckHardDiskSpace(void);
};

#endif // !defined(AFX_CHECKPC_H__A0E2B967_BF68_4D65_BF50_64A8ACF44554__INCLUDED_)
