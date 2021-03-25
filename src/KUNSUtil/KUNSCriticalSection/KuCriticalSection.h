/*______________________________________________________________________________________________
PROJECT : Intelligent Robotics Lab. Department of Mechanical Engineering Korea University                                    
(c) Copyright 2012 KUME Intelligent Robotics Lab.                                             
All rights reserved                                                                         
$Directed by Jae-Bok Song                                                                     
$Designed by Joong-Tae Park                                                                  
$Description :크리티컥 섹션 기능을 수행하는 클래스. 윈도우, 리눅스 등에서 모두 사용가능하다.
$Created on: 2012. 6. 5.                                                                        
$Author: Joong-Tae Park      
$e-mail: jtpark1114@gmail.com
______________________________________________________________________________________________*/

#ifndef KUNS_CRITICAL_SECTION_H
#define KUNS_CRITICAL_SECTION_H

#ifdef WIN32
//#define _AFXDLL
#include <windows.h>
//#include <afxmt.h>
#else
#include <pthread.h>
#endif

#include <iostream>
#include "../KUNSSingletone/KuSingletone.h"
using namespace std;

class KuCriticalSection : public KuSingletone <KuCriticalSection>
{
private:
	//CCriticalSection m_CriticalSection;
	 HANDLE m_mutex;
	DWORD m_owner;

public:
	KuCriticalSection();
	~KuCriticalSection();
public:
	void Lock();
	void Unlock();	
	void init();
	void Remove();
};

#ifdef WIN32
static CRITICAL_SECTION cs; /* This is the critical section object -- once initialized,
                               it cannot be moved in memory */
                            /* If you program in OOP, declare this as a non-static member in your class */
#else
/* This is the critical section object (statically allocated). */
static pthread_mutex_t cs_mutex = PTHREAD_MUTEX_INITIALIZER;
#endif



#endif /*KUNS_CRITICAL_SECTION_H*/