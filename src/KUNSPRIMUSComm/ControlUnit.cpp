// ControlUnit.cpp: implementation of the CControlUnit class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "ControlUnit.h"

#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

const unsigned int	uiUnitScanTime = 10;

CControlUnit::CControlUnit()
{
	m_hThread = INVALID_HANDLE_VALUE;
	m_strUnitName = "[NULL]";
	m_iContinue = 0;
}

CControlUnit::~CControlUnit()
{

}

int CControlUnit::ThreadLoop()
{
	while(m_iContinue){
		DoEvents ();
		Sleep(uiUnitScanTime);
	}
	return 0;
}

DWORD WINAPI UnitThreadProc(LPVOID lpParam)
{
	CControlUnit *pUnit;

	pUnit = (CControlUnit *)lpParam;
	pUnit->ThreadLoop();
	return 0;
}

int CControlUnit::ThreadStart()
{
	DWORD dwThreadId;

	m_iContinue = 1;

	m_hThread=CreateThread(NULL,0,
		(LPTHREAD_START_ROUTINE)UnitThreadProc,
		(LPVOID)this,CREATE_SUSPENDED,&dwThreadId);

	if(m_hThread==INVALID_HANDLE_VALUE){

		return -1;
	}
	SetThreadPriority(m_hThread,THREAD_PRIORITY_NORMAL);
	ResumeThread(m_hThread);

	return 1;
}

int CControlUnit::ThreadStop()
{
	int i;
	DWORD dwExitCode=0;

	if (m_iContinue) {
		m_iContinue = 0;

		if(m_hThread!=INVALID_HANDLE_VALUE){
			dwExitCode = STILL_ACTIVE;
			for(i=0; i<10 && dwExitCode==STILL_ACTIVE; i++){
				Sleep(100);
				GetExitCodeThread(m_hThread,&dwExitCode);
			}
			if(dwExitCode==STILL_ACTIVE){

				TerminateThread(m_hThread,-1);
				Sleep(100);
			}
			CloseHandle(m_hThread);
			m_hThread = INVALID_HANDLE_VALUE;
		}
	}

	return 0;
}

int CControlUnit::DoEvents()
{
	return 0;
}
