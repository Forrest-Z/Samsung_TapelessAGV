#include "stdafx.h"
#include "KuStandby.h"

KuStandby::KuStandby()
{
#ifdef WIN32
	m_bHoldFlag=false;
#else
	//initialization---------------------------------------------------------------------------------
	pthread_mutex_init(&m_muCond, NULL);
	pthread_cond_init(&m_thCond, NULL);
	//----------------------------------------------------------------------------------------------
#endif
	cout<<"[KuStandby]: Singletone instance is created!!!"<<endl;
}

KuStandby::~KuStandby()
{
	cout<<"[KuStandby]: Singletone instance is destroyed!!!"<<endl;
}

void KuStandby::hold()
{
#ifdef WIN32
	m_bHoldFlag = true;
	while(m_bHoldFlag){
		m_kTimer.sleepMS(500);				
	}
#else
	pthread_mutex_lock(&m_muCond);
	pthread_cond_wait(&m_thCond, &m_muCond);
	pthread_mutex_unlock( &m_muCond );
#endif
}

void KuStandby::release()
{
#ifdef WIN32
	m_bHoldFlag = false;
#else
	pthread_cond_signal(&m_thCond);
#endif
}

