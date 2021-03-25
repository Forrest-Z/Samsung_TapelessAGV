#include "stdafx.h"
#include "KuCriticalSection.h"

KuCriticalSection::KuCriticalSection()
{
	//cout<<"[KuCriticalSection]: Instance is created!!!"<<endl;

#ifdef WIN32
  //	InitializeCriticalSection(&cs); /// Initialize the critical section before entering multi-threaded context. 
   //EnterCriticalSection(&cs); /// Enter the critical section -- other threads are locked out 
	init();

#else
	pthread_mutex_lock( &cs_mutex ); /// Enter the critical section -- other threads are locked out 
#endif
}

KuCriticalSection::~KuCriticalSection()
{
#ifdef WIN32	
 //	LeaveCriticalSection(&cs); /// Leave the critical section -- other threads can now EnterCriticalSection() 
 //	DeleteCriticalSection(&cs); /// Release system object when all finished -- usually at the end of the cleanup code 
	Remove();

#else
	 pthread_mutex_unlock( &cs_mutex ); ///Leave the critical section -- other threads can now pthread_mutex_lock()  

#endif

	//cout<<"[KuCriticalSection]: Instance is destroyed!!!"<<endl;
}
void KuCriticalSection::init()
{
	m_mutex=CreateMutex(NULL,FALSE,NULL);
	memset(&m_owner,0,sizeof(DWORD));

}
void KuCriticalSection::Remove()
{
	WaitForSingleObject(m_mutex,INFINITE);
	CloseHandle(m_mutex);

}

void KuCriticalSection::Lock()
{
	//m_CriticalSection.Lock();
	DWORD ThreadID= GetCurrentThreadId();
	if(m_owner!=ThreadID)
	{
		WaitForSingleObject(m_mutex,INFINITE);
		m_owner=ThreadID;
	}

}

void KuCriticalSection::Unlock()
{
	//m_CriticalSection.Unlock();
	DWORD ThreadID= GetCurrentThreadId();
	if(m_owner ==ThreadID)
	{
		ReleaseMutex(m_mutex);
		memset(&m_owner,0,sizeof(DWORD));
	}
}