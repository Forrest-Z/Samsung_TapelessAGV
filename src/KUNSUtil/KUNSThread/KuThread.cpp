
#include "stdafx.h"
#include "KuThread.h"

KuThread::KuThread()
	: m_sThreadName("")
{
	//?る爤??齑堦赴??--------------------------------------------------------------------------------
	m_nRate = 0;
	m_bDoThread = true;

#ifdef WIN32
 	m_dwThreadID=NULL;
 	m_hThreadID=NULL;

#else
	pthread_attr_init(&_thread_attr);
	pthread_attr_setdetachstate(&_thread_attr,PTHREAD_CREATE_DETACHED); //?る爤??
	pthread_mutex_init(&m_muCond, NULL);
	pthread_cond_init(&m_thCond, NULL);

#endif

	//?る爤??齑堦赴??--------------------------------------------------------------------------------

	m_arg = NULL;

	m_bSuspendFlag = false;

	cout<<"[KuThread]: Instance is created!!!"<<endl;
}

KuThread::~KuThread()
{
	cout<<"[KuThread]: Instance is destroyed!!!"<<endl;
}
#ifdef WIN32
DWORD WINAPI  KuThread::_thread_function( LPVOID arg )
{
	KuThread* pThread= (KuThread*)arg;
	
	while(pThread->m_bDoThread){
		pThread->checkStartTime();

		if(pThread->m_bSuspendFlag){

		}

		if(NULL == pThread->m_arg) pThread->m_funcPtr();
		else pThread->m_funcPtrwithParam(pThread->m_arg);

		int nFuncTime = (int)pThread->checkEndTime();
		int nSleep = pThread->m_nRate - (int)nFuncTime;

		if(nSleep < 10)
		{
			nSleep = 20; // min. 10 ms delay
		}

		pThread->sleepMS(nSleep); // final delay time in ms
	}

	return 0;
}

#else
void *KuThread::_thread_function( void* arg )
{
	KuThread* pThread = (KuThread*)arg;

	while(pThread->m_bDoThread){
		pThread->checkStartTime();

		if(pThread->m_bSuspendFlag){

			pthread_mutex_lock(&pThread->m_muCond);
			pthread_cond_wait(&pThread->m_thCond, &pThread->m_muCond);
			pthread_mutex_unlock( &pThread->m_muCond );

		}

		if(NULL == pThread->m_arg) pThread->m_funcPtr();
		else pThread->m_funcPtrwithParam(pThread->m_arg);

		int nFuncTime = (int)pThread->checkEndTime(); //?垬 齑??橅枆?滉皠???嶋摑
		pThread->sleepMS(pThread->m_nRate - (int)nFuncTime ); //?愴晿???る爤??欤缄赴?愳劀 ?垬 ?橅枆?滉皠???滌櫢??毵岉伡 sleep?滊嫟.

	}

	pthread_exit(NULL);
 return 0;
}
#endif

bool KuThread::start(void (*funcPtr)() , int nRate )
{
	m_bDoThread = true;
	m_funcPtr = funcPtr;
	m_nRate = nRate;

#ifdef WIN32
	m_hThreadID = CreateThread(NULL,0,_thread_function,this,0,&m_dwThreadID);//?半爤???濎劚

	if(NULL != m_hThreadID ){// ?る爤???濎劚 ?标车
		cout <<"[KuThread] : thread Created!!!" << endl;
	}else{
		return false; //?る爤???濎劚 ?ろ尐
	}

#else
	//?标车??0??毽劥
	// int  nResult = pthread_create(&_thread, &_thread_attr,_thread_function, (void*)*funcPtr);
	int  nResult = pthread_create(&_thread, &_thread_attr,_thread_function, (void*)this);

		 if(nResult == 0){// ?る爤???濎劚 ?标车
			 cout <<"[KuThread] : thread Created!!!" << endl;
		 }else{
			 return false; //?る爤???濎劚 ?ろ尐
		 }

#endif

		 return true;
}

bool KuThread::start(void (*funcPtr)(void*) , void* arg,  int nRate, string sThreadName)
{
	m_bDoThread = true;
	m_funcPtrwithParam = funcPtr;
	m_arg = arg;
	m_nRate = nRate;
	int nResult =0;

	m_sThreadName = sThreadName;	

#ifdef WIN32
	m_hThreadID = CreateThread(NULL,0,_thread_function,this,0,&m_dwThreadID);;//?半爤???濎劚

	if(m_hThreadID != NULL){// ?る爤???濎劚 ?标车
		cout <<"[KuThread] : thread Created!!!" << endl;
	}else{
		return false; //?る爤???濎劚 ?ろ尐
	}
#else
	//?标车??0??毽劥
	//int  nResult = pthread_create(&_thread, &_thread_attr,_thread_function, (void*)*funcPtr);
	nResult = pthread_create(&_thread, &_thread_attr,_thread_function, (void*)this);
	if(nResult == 0){// ?る爤???濎劚 ?标车
		cout <<"[KuThread] : thread Created!!!" << endl;
	}else{
		return false; //?る爤???濎劚 ?ろ尐
	}

#endif
	


	return true;
}

bool KuThread::terminate()
{
	cout <<"[KuThread] : thread terminate!!!" << endl;
	m_bDoThread = false;

#ifdef WIN32
 if(m_hThreadID!=NULL)
 {
//		TerminateThread(m_hThreadID, NULL);
//		WaitForSingleObject(m_hThreadID,INFINITE);
		PostThreadMessage(GetThreadId(m_hThreadID), WM_QUIT, 0, 0);
		CloseHandle(m_hThreadID);
		m_hThreadID=NULL;
		m_dwThreadID=NULL;
 }
#else
	
#endif
	m_arg = NULL;

	return true;
}

bool KuThread::suspend()
{
	m_bSuspendFlag = true;
	return true;
}

bool KuThread::resume()
{
	m_bSuspendFlag = false;
#ifdef WIN32

#else
	pthread_cond_signal(&m_thCond);

#endif

	return true;
}

