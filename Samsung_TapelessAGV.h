
// Samsung_TapelessAGV.h : PROJECT_NAME ���� ���α׷��� ���� �� ��� �����Դϴ�.
//

#pragma once

#ifndef __AFXWIN_H__
	#error "PCH�� ���� �� ������ �����ϱ� ���� 'stdafx.h'�� �����մϴ�."
#endif

#include "resource.h"		// �� ��ȣ�Դϴ�.


// CSamsung_TapelessAGVApp:
// �� Ŭ������ ������ ���ؼ��� Samsung_TapelessAGV.cpp�� �����Ͻʽÿ�.
//

class CSamsung_TapelessAGVApp : public CWinApp
{
public:
	CSamsung_TapelessAGVApp();

// �������Դϴ�.
public:
	virtual BOOL InitInstance();

// �����Դϴ�.

	DECLARE_MESSAGE_MAP()
};

extern CSamsung_TapelessAGVApp theApp;