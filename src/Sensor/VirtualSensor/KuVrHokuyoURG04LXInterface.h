/*______________________________________________________________________________________________
PROJECT : Intelligent Robotics Lab. Department of Mecanical Engineering Korea University                                    
(c) Copyright 2012 KUME Intelligent Robotics Lab.                                             
All rights reserved                                                                         
$Directed by Jae-Bok Song                                                                     
$Designed by Joong-Tae Park                                                                  
$Description : Hokuyo 사의 UTM30L의 기능을 모사한 가상의 레이저 인터페이스.
$Created on: 2012. 6. 12.                                                                          
$Author: Joong-Tae Park     
$e-mail: jtpark1114@gmail.com
______________________________________________________________________________________________*/


#ifndef KUNS_VIRTUAL_HOKUYO_UTM30LX_INTERFACE_H
#define KUNS_VIRTUAL_HOKUYO_UTM30LX_INTERFACE_H

#include <iostream>
#include "../Sensor.h"

#include "../../KUNSMap/KuMap.h"
#include "../../KUNSUtil/KuUtil.h"
#include "../../KUNSPose/KuPose.h"
#include "../../KUNSMath/KuMath.h"
#include "../../KUNSUtil/KUNSSingletone/KuSingletone.h"
#include "../../KUNSGUI/KuDrawingInfo.h"

using namespace std;
class KuVrHokuyoUTM30LXInterface : public KuSingletone <KuVrHokuyoUTM30LXInterface>
{
private:
	static const int MAX_LASER_DISTANCE = 30000 ;
private:
	KuUtil m_KuUtil;
	int** m_nMap;
	int m_nMapX;
	int m_nMapY;
	int_1DArray m_nLaserData181;
	CCriticalSection m_CriticalSection;
		
public:
	bool connect(int** nMap);
	int_1DArray getData181(KuPose RobotPos);

	KuVrHokuyoUTM30LXInterface();
	virtual ~KuVrHokuyoUTM30LXInterface();

};

#endif /*KUNS_VIRTUAL_HOKUYO_UTM30LX_INTERFACE_H*/
