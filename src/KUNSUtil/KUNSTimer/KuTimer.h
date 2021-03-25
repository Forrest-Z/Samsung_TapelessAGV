/*______________________________________________________________________________________________
PROJECT : Intelligent Robotics Lab. Department of Mechanical Engineering Korea University                                    
(c) Copyright 2012 KUME Intelligent Robotics Lab.                                             
All rights reserved                                                                         
$Directed by Jae-Bok Song                                                                     
$Designed by Joong-Tae Park                                                                  
$Description : thread 기능을 제공해준다.
$Created on: 2012. 4. 26.                                                                        
$Author: Joong-Tae Park      
$e-mail: jtpark1114@gmail.com
______________________________________________________________________________________________*/

#ifndef KUNS_TIMER_H_DEF
#define KUNS_TIMER_H_DEF

#ifdef WIN32   // Windows system specific
	#include <windows.h>
#else          // Unix based system specific
	#include <sys/time.h>
#endif
	#include <stdlib.h>
	#include <iostream>

using namespace std;

class KuTimer
{

private:
    double startTimeInMicroSec;                 // starting time in micro-second
    double endTimeInMicroSec;                   // ending time in micro-second
    int    stopped;                             // stop flag 


    double m_dStartTimeInMicroSec;
    double m_dEndTimeInMicroSec;

#ifdef WIN32
    LARGE_INTEGER m_Frequency;                    // ticks per second
    LARGE_INTEGER m_StartCount;                   //
    LARGE_INTEGER m_EndCount;                     //
#else
    timeval m_tvStartCount;                         //
    timeval m_tvEndCount;                           //
#endif


private:
    void   start();                             // start timer
    void   stop();                              // stop the timer

public:
    KuTimer();                                    // default constructor
    ~KuTimer();                                   // default destructor
    void sleepMS(int nTimeMS); //sleep ms sec.
    void checkStartTime();
    double checkEndTime();

protected:




};

#endif // CKUNSTIMER_H_DEF
