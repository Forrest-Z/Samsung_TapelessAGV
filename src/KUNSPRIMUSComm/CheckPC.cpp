// CheckPC.cpp: implementation of the CCheckPC class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "CheckPC.h"
//#include "debug.h"
#include <math.h>

#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif


//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CCheckPC::CCheckPC()
{
	
}

CCheckPC::~CCheckPC()
{
	
}

int CCheckPC::DoEvents(void)
{
	CheckBootingTimeandRunningTime();
	CheckHardDiskSpace();
	CheckMemorySpace();
	if (bInitialized)
	{
		//내부에 Sleep(2000)이 있음.
	//	CheckCPUUsage();
		Sleep(2000);
	}
	else
	{
		// CPU를 사용량 check에 대한 초기화가 실패한 경우 여기서 Sleep를 정의해서 사용함.
		Sleep(2000);
	}
	return 0;
}

// st 날짜의 절대 초를 구한다.
DWORD CCheckPC::MyGetAbsSecond(SYSTEMTIME st)
{
	INT64	i64;
	FILETIME	fst;

	SystemTimeToFileTime(&st, &fst);
	i64 = (((INT64)fst.dwHighDateTime) << 32) + fst.dwLowDateTime;
	i64 = i64 / 10000000 - (INT64)145731 * 86400;

	return (DWORD)i64;
}

void CCheckPC::MyAbsSecondToSystem(DWORD Abs, SYSTEMTIME &st)
{
	INT64	i64;
	FILETIME	fst;

	i64 = (Abs + (INT64)145731 * 86400)*10000000;
	fst.dwHighDateTime = (DWORD)(i64 >> 32);
	fst.dwLowDateTime = (DWORD)(i64 & 0xffffffff);
	FileTimeToSystemTime(&fst, &st);
}

void CCheckPC::CheckBootingTimeandRunningTime()
{
	int		Sec, Min, Hour, Day;
	SYSTEMTIME	st, boot;
	DWORD	abs;

	DWORD	Count = GetTickCount();
	Sec = Count/1000;
	Day = Sec/86400;
	Hour = (Sec % 86400)/3600;
	Min = (Sec % 3600)/60;

	GetLocalTime(&st);
	abs = MyGetAbsSecond(st);
	MyAbsSecondToSystem(abs-Sec, boot);
	
	m_csBootingTime.Format(_T("Booting Time : %d.%d.%d - %02d:%02d"), boot.wYear, boot.wMonth, boot.wDay, boot.wHour, boot.wMinute);//0217
	m_csRunningTime.Format(_T("Running Time : %d day %d hour %d min %d sec"), Day, Hour, Min, Sec%60);//0217
//	debug("%s %s", m_csBootingTime, m_csRunningTime);
}

void CCheckPC::CheckHardDiskSpace(void)
{
	// Hard disk 용량을 계산한다.
	unsigned __int64 lFreeBytesAvailableToCaller, lTotalNumberOfBytes, lTotalNumberOfFreeBytes;
	m_csHardDiskSpace ="0/0";
	GetDiskFreeSpaceEx((LPCWSTR)(_T("c:\\")), (PULARGE_INTEGER)&lFreeBytesAvailableToCaller, (PULARGE_INTEGER)&lTotalNumberOfBytes, (PULARGE_INTEGER)&lTotalNumberOfFreeBytes);
	m_ui64HardDiskNotUsed = lTotalNumberOfFreeBytes/(1024*1024);
	m_ui64HardDiskTotal = lTotalNumberOfBytes/(1024*1024);
	m_dHardDiskUsage = 100.0*(m_ui64HardDiskTotal-m_ui64HardDiskNotUsed)/m_ui64HardDiskTotal;
	m_csHardDiskSpace.Format(_T("Hard Disk : Free [%I64uMB] / Total [%I64uMB] = %lf"), m_ui64HardDiskNotUsed, m_ui64HardDiskTotal, m_dHardDiskUsage);
//	debug("%s", m_csHardDiskSpace);
}

void CCheckPC::CheckMemorySpace()
{
	//메모리의 정보를 구한다.
	MEMORYSTATUS ms;
    ms.dwLength=sizeof(MEMORYSTATUS);
    ::GlobalMemoryStatus(&ms);

	m_iMemoryNotUsed = (int)(ms.dwAvailPhys / 1024);
	m_iMemoryTotal = (int)(ms.dwTotalPhys / 1024);
    
	m_dMemoryUsage = 100.0*(m_iMemoryTotal - m_iMemoryNotUsed)/m_iMemoryTotal;
	m_csMemory.Format(_T("Memory : Free [%d] / Total [%d] = %lf "), m_iMemoryNotUsed, m_iMemoryTotal,m_dMemoryUsage );//0217
//	debug("%s", m_csMemory);
	
}

// CPU check 초기화
int CCheckPC::CheckCPUInit(void)
{
    // Step 1: --------------------------------------------------
    // Initialize COM. ------------------------------------------
	iNproc = iGetNumberOfCores();
	iNproc2 = (iNproc+1)*2;

	//debug("Number of Process = %d ", iNproc);

    HRESULT hres =  CoInitializeEx(0, COINIT_MULTITHREADED); 
    if (FAILED(hres))
    {
        return FALSE;
    }

	ulinitVal = new ULONG64[iNproc2];//+1 for the total
	ulVal = new ULONG64[iNproc2];//+1 for the total
	dUsage = new double[iNproc+1];//+1 for the total

	int i=0;


	// Step 2: --------------------------------------------------
    // Set general COM security levels --------------------------
    // Note: If you are using Windows 2000, you need to specify -
    // the default authentication credentials for a user by using
    // a SOLE_AUTHENTICATION_LIST structure in the pAuthList ----
    // parameter of CoInitializeSecurity ------------------------

    hres =  CoInitializeSecurity(
        NULL, 
        -1,                          // COM authentication
        NULL,                        // Authentication services
        NULL,                        // Reserved
        RPC_C_AUTHN_LEVEL_DEFAULT,   // Default authentication 
        RPC_C_IMP_LEVEL_IMPERSONATE, // Default Impersonation  
        NULL,                        // Authentication info
        EOAC_NONE,                   // Additional capabilities 
        NULL                         // Reserved
        );

                      
    if (FAILED(hres))
    {
        CoUninitialize();
        return FALSE;
    }


    // Step 3: ---------------------------------------------------
    // Obtain the initial locator to WMI -------------------------

    pLoc = NULL;

    hres = CoCreateInstance(
        CLSID_WbemLocator,             
        0, 
        CLSCTX_INPROC_SERVER, 
        IID_IWbemLocator, (LPVOID *) &pLoc);
 
    if (FAILED(hres))
    {
        CoUninitialize();
        return FALSE;
    }

	// Step 4: -----------------------------------------------------
    // Connect to WMI through the IWbemLocator::ConnectServer method

    pSvc = NULL;
	
    // Connect to the root\cimv2 namespace with
    // the current user and obtain pointer pSvc
    // to make IWbemServices calls.
    hres = pLoc->ConnectServer(
         _bstr_t("ROOT\\CIMV2"),  // Object path of WMI namespace
         NULL,                    // User name. NULL = current user
         NULL,                    // User password. NULL = current
         0,                       // Locale. NULL indicates current
         NULL,                    // Security flags.
         0,                       // Authority (e.g. Kerberos)
         0,                       // Context object 
         &pSvc                    // pointer to IWbemServices proxy
         );
    
    if (FAILED(hres))
    {
        pLoc->Release();     
        CoUninitialize();
        return FALSE;
    }

    // Step 5: --------------------------------------------------
    // Set security levels on the proxy -------------------------

    hres = CoSetProxyBlanket(
       pSvc,                        // Indicates the proxy to set
       RPC_C_AUTHN_WINNT,           // RPC_C_AUTHN_xxx
       RPC_C_AUTHZ_NONE,            // RPC_C_AUTHZ_xxx
       NULL,                        // Server principal name 
       RPC_C_AUTHN_LEVEL_CALL,      // RPC_C_AUTHN_LEVEL_xxx 
       RPC_C_IMP_LEVEL_IMPERSONATE, // RPC_C_IMP_LEVEL_xxx
       NULL,                        // client identity
       EOAC_NONE                    // proxy capabilities 
    );

    if (FAILED(hres))
    {
        pSvc->Release();
        pLoc->Release();     
        CoUninitialize();
        return FALSE;
    }

	return TRUE;
}
// CPU check 종료
void CCheckPC::CheckCPUTerminate(void)
{
	pSvc->Release();
    pLoc->Release();
    CoUninitialize();
}
// CPU 사용량 조사
void CCheckPC::CheckCPUUsage(void)
{
	GetInitValues();
	Sleep(2000);
	GetValueInt(ulVal);

	// 사용량 정리

	for (int i=0; i<=iNproc; i++)
	{
		if((ulVal[i*2+1]- ulinitVal[i*2+1])>0)
		{
			dUsage[i] = fabs(100.0 - ((double)(ulVal[i*2] - ulinitVal[i*2]) / (ulVal[i*2+1]- ulinitVal[i*2+1])) * 100);
		}
		else
		{
			dUsage[i] = -1.0;
		}
		if(i== iNproc)
		{
			m_dCPUTotalUsage = dUsage[i];
//			debug("CPU Total = %lf", dUsage[i]);
		}
		else
		{
//			debug("CPU%d = %lf", i+1, dUsage[i]);
		}
	}
}

BOOL CCheckPC::GetValueInt(ULONG64 *ul)
{
	// Step 6: --------------------------------------------------
	// Use the IWbemServices pointer to make requests of WMI ----

	// For example, get the name of the operating system
	IEnumWbemClassObject* pEnumerator = NULL;
	HRESULT hres = pSvc->ExecQuery(
		bstr_t("WQL"), 
		bstr_t("SELECT * FROM Win32_PerfRawData_PerfOS_Processor"),
		WBEM_FLAG_FORWARD_ONLY | WBEM_FLAG_RETURN_IMMEDIATELY, 
		NULL,
		&pEnumerator);

	if (FAILED(hres))
	{
		return FALSE;
	}


	// Step 7: -------------------------------------------------
	// Get the data from the query in step 6 -------------------

	IWbemClassObject *pclsObj;
	ULONG uReturn = 0;
	int nCtr = 0;

	while (nCtr<iNproc2)
	{
		HRESULT hr = pEnumerator->Next(WBEM_INFINITE, 1, 
			&pclsObj, &uReturn);

		if(0 == uReturn)
		{
			break;
		}

		VARIANT vtProp;
		VariantInit(&vtProp);

		hr = pclsObj->Get(L"PercentProcessorTime", 0, &vtProp, 0, 0);
		ul[nCtr] = _wtoi64((WCHAR*)(vtProp.bstrVal));
		VariantClear(&vtProp);
		hr = pclsObj->Get(L"TimeStamp_Sys100NS", 0, &vtProp, 0, 0);
		ul[nCtr+1] = _wtoi64((WCHAR*)(vtProp.bstrVal));
		VariantClear(&vtProp);
		nCtr+=2;
	}
	pclsObj->Release();
	pEnumerator->Release();

	return TRUE;
}

BOOL CCheckPC::GetInitValues()
{
	return GetValueInt(ulinitVal);
}

int CCheckPC::iGetNumberOfCores()
{
	PSYSTEM_LOGICAL_PROCESSOR_INFORMATION pProcessorInformations = NULL;
	DWORD length = 0;

	BOOL result = GetLogicalProcessorInformation(pProcessorInformations, &length);
	if (!result)
	{
		if (GetLastError() == ERROR_INSUFFICIENT_BUFFER)
		{
			pProcessorInformations = (PSYSTEM_LOGICAL_PROCESSOR_INFORMATION)new BYTE[length];
		}
	}

	result = GetLogicalProcessorInformation(pProcessorInformations, &length);
	if (!result)
	{
		// error
		return -1;
	}

	int numOfCores = 0;
	for (UINT i = 0 ; i < length / sizeof(SYSTEM_LOGICAL_PROCESSOR_INFORMATION) ; i++)
	{
		if (pProcessorInformations[i].Relationship == RelationProcessorCore)
			numOfCores++;
	}

	delete [] pProcessorInformations;

	return numOfCores;
}

unsigned char CCheckPC::GetCPUUsage()
{
	unsigned char ucReturn;
	ucReturn = (unsigned char)(m_dCPUTotalUsage);
	return ucReturn;
}

unsigned char CCheckPC::GetHDDUsage()
{
	unsigned char ucReturn;
	ucReturn = (unsigned char)(m_dHardDiskUsage);
	return ucReturn;
}

unsigned char CCheckPC::GetMemoryUsage()
{
	unsigned char ucReturn;
	ucReturn = (unsigned char)(m_dMemoryUsage);
	return ucReturn;
}