// ControlUnit.h: interface for the CControlUnit class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_CONTROLUNIT_H__78056641_C90C_46E6_A995_FFF2EDDAEB83__INCLUDED_)
#define AFX_CONTROLUNIT_H__78056641_C90C_46E6_A995_FFF2EDDAEB83__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

class CControlUnit  
{
private:
	HANDLE	m_hThread;
	int		m_iContinue;

protected:
	char *m_strUnitName;
	
public:	
	int ThreadStop(void);
	int ThreadStart(void);
	int ThreadLoop(void);
	virtual int DoEvents(void);
	CControlUnit();
	virtual ~CControlUnit();
};

#endif // !defined(AFX_CONTROLUNIT_H__78056641_C90C_46E6_A995_FFF2EDDAEB83__INCLUDED_)
