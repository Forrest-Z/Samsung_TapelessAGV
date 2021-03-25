#include "stdafx.h"
#include "KuTimer.h"


///////////////////////////////////////////////////////////////////////////////
// constructor
///////////////////////////////////////////////////////////////////////////////
KuTimer::KuTimer()
{
#ifdef WIN32
    QueryPerformanceFrequency(&m_Frequency);
    m_StartCount.QuadPart = 0;
    m_EndCount.QuadPart = 0;
#else
    m_tvStartCount.tv_sec = m_tvStartCount.tv_usec = 0;                      //
    m_tvEndCount.tv_sec = m_tvEndCount.tv_usec = 0;                           //

#endif

    stopped = 0;
    m_dStartTimeInMicroSec = 0;
    m_dEndTimeInMicroSec = 0;
    start();
}



/**
 @brief Korean: 작성
 @brief English: write in English
*/
KuTimer::~KuTimer()
{
	stop();
}

/**
 @brief Korean: m_StartCount 또는 m_tvStartCount 본 함수에서 설정된다. 
 @brief English: m_StartCount or m_tvStartCount will be set at this point.
*/

void KuTimer::start()
{
    stopped = 0; // reset stop flag
#ifdef WIN32
    QueryPerformanceCounter(&m_StartCount);
#else
   gettimeofday(&m_tvStartCount, NULL);
#endif
}

/**
 @brief Korean: m_EndCount 또는 m_tvEndCount 본 함수에서 설정된다. 
 @brief English: m_EndCount or m_tvEndCount will be set at this point.
*/

void KuTimer::stop()
{
    stopped = 1; // set timer stopped flag

#ifdef WIN32
    QueryPerformanceCounter(&m_EndCount);
#else
    gettimeofday(&m_tvEndCount, NULL);
#endif
}


void KuTimer::sleepMS(int nTimeMS)
{
	if(nTimeMS <=0 ) return;
	if(nTimeMS > 100000) return;

#ifdef WIN32
		Sleep(nTimeMS);
#else
	usleep(nTimeMS*1000);
#endif

}

void KuTimer::checkStartTime()
{
#ifdef WIN32
	QueryPerformanceCounter(&m_StartCount);
	m_dStartTimeInMicroSec = m_StartCount.QuadPart * (1000000.0 / m_Frequency.QuadPart);
#else
	m_dStartTimeInMicroSec = (m_tvStartCount.tv_sec * 1000000.0) + m_tvStartCount.tv_usec;
#endif
}
double KuTimer::checkEndTime()
{
#ifdef WIN32
	 QueryPerformanceCounter(&m_EndCount);
	 m_dEndTimeInMicroSec = m_EndCount.QuadPart * (1000000.0 / m_Frequency.QuadPart);
#else

    gettimeofday(&m_tvEndCount, NULL);
    m_dEndTimeInMicroSec = (m_tvEndCount.tv_sec * 1000000.0) + m_tvEndCount.tv_usec;
#endif
    return (m_dEndTimeInMicroSec - m_dStartTimeInMicroSec)/1000.;

}





