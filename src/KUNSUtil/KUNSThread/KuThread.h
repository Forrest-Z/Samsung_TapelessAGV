/*______________________________________________________________________________________________
PROJECT : Intelligent Robotics Lab. Department of Mechanical Engineering Korea University                                    
(c) Copyright 2012 KUME Intelligent Robotics Lab.                                             
All rights reserved                                                                         
$Directed by Jae-Bok Song                                                                     
$Designed by Joong-Tae Park                                                                  
$Description : thread ±‚¥…¿ª ¡¶∞¯«ÿ¡ÿ¥Ÿ.
$Created on: 2012. 5. 1.                                                                        
$Author: Joong-Tae Park      
$e-mail: jtpark1114@gmail.com
______________________________________________________________________________________________*/

#ifndef KUNS_THREAD_H_
#define KUNS_THREAD_H_

#ifdef WIN32   // Windows system specific
#define _AFXDLL

#include <afxmt.h>
#include <process.h>
#include <stdio.h>
#else          // Unix based system specific
#include <pthread.h> //thread 
#include <unistd.h>
#endif

#include <iostream>
#include <stdlib.h>

#include "../KUNSTimer/KuTimer.h"

using namespace std;

class KuThread: public KuTimer
{

private:

#ifdef WIN32
 	DWORD  m_dwThreadID;
 	HANDLE m_hThreadID;	
#else
	pthread_t _thread;
	pthread_attr_t 	_thread_attr;
	pthread_cond_t m_thCond;
	pthread_mutex_t m_muCond;
#endif

	//?§Î†à?úÎ? ?ùÏÑ±?†Îïå, ?ÑÏöî??Î≥Ä?òÏ? ?®Ïàò.


#ifdef WIN32
static DWORD WINAPI  _thread_function(LPVOID arg );
#else
	static void* _thread_function(void* arg );
#endif


	void (*m_funcPtr)();
	void (*m_funcPtrwithParam)(void*);
	void* m_arg;
	int m_nRate;

	bool m_bDoThread;
	bool m_bSuspendFlag;
	string m_sThreadName;

public:

	bool start( void (*funcPtr)(), int nRate );
	bool start(void (*funcPtr)(void*) , void* arg,  int nRate, string sThreadName = "Default");
	bool terminate();
	bool suspend();
	bool resume();

	KuThread();
	~KuThread();

};


#endif /* KUNS_THREAD_H_ */
