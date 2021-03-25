/*______________________________________________________________________________________________
PROJECT : Intelligent Robotics Lab. Department of Mechanical Engineering Korea University                                    
(c) Copyright 2012 KUME Intelligent Robotics Lab.                                             
All rights reserved                                                                         
$Directed by Jae-Bok Song                                                                     
$Designed by Joong-Tae Park                                                                  
$Description :시스템을 대기상태로 만들어 주거나 각종 이벤트 처리를 담당할 수 있는 기능을 제공해는 클래스.
$Created on: 2012. 5. 17.                                                                        
$Author: Joong-Tae Park      
$e-mail: jtpark1114@gmail.com
______________________________________________________________________________________________*/

#ifndef KUNS_STANDBY_H_
#define KUNS_STANDBY_H_

#ifdef WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

#include <iostream>
#include <stdlib.h>
#include "../KUNSTimer/KuTimer.h"
#include "../KUNSSingletone/KuSingletone.h"

using namespace std;

class KuStandby : public KuSingletone <KuStandby>
{
private:
	KuTimer m_kTimer;
#ifdef WIN32
	bool m_bHoldFlag;
#else
	pthread_cond_t m_thCond;
	pthread_mutex_t m_muCond;
#endif

public:
	void hold();
	void release();
	KuStandby();
	virtual ~KuStandby();
};


#endif /* KUNS_STANDBY_H_ */
