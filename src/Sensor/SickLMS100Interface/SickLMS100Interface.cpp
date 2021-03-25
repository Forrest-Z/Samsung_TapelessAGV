#include "stdafx.h"
#include "SickLMS100Interface.h"

SickLMS100Interface::SickLMS100Interface()
{
	m_bIntialized = false;
	m_isSockEstablised = false;
	m_LRFRangeData_181=	 m_KuUtil.generateIntType1DArray(Sensor::URG04LX_DATA_NUM181,0);
	m_socketFrontSick=NULL;
}

SickLMS100Interface::~SickLMS100Interface()
{

}

void SickLMS100Interface::disconnectLaserScanner()
{
	if(m_socketFrontSick!=NULL)
		lms100_disconnect(&m_socketFrontSick);
}

bool SickLMS100Interface::connect(char * cIP, int nPort)
{
//	m_isSockEstablised = lms100_connect(&m_socketFrontSick, cIP, nPort);
	if(1)//m_isSockEstablised == true)
	{
									
// 		cout<<"Socket Connection is established"<<endl;

//		CLMSManager * plmsManager = NULL;
//		plmsManager = CLMSManager::getInstance();

		if(m_lms_manager.connect(cIP) == true){
			cout<<"Laser connection success"<<endl;
			m_lms_manager.run_thread();
			return true;
		}
		else{	
			cout<<"Laser connection failure"<<endl;
			return false;
		}
		
	}
	else{
		cout<<"Socket Connection is failed"<<endl;
		return false;
	}
}


int_1DArray SickLMS100Interface::getData( )
{

	float cur_th = 0;
	float distance = 0;
	float height   = 0;
	for(int i=0; i<Sensor::URG04LX_DATA_NUM181;i++)m_LRFRangeData_181[i]=0;

//	CLMSManager * plmsManager = CLMSManager::getInstance();
	if(m_lms_manager.is_lms_running() == false){
		return m_LRFRangeData_181;
	}
	lms100_scan_data * m_pcur_scan_data = m_lms_manager.get_front_scan_data();

	while(m_pcur_scan_data->is_updating == true){
		::Sleep(1);
	}
	
	float start_angle_rad	= m_pcur_scan_data->dist1_starting_angle * D2R;
	float del_th_rad	= m_pcur_scan_data->dist1_angular_step_width *D2R;

	for(int i = 0; i < m_pcur_scan_data->dist1_number_of_data; i++){
		cur_th	 = start_angle_rad + ( i * del_th_rad );

		while(m_pcur_scan_data->is_updating == true){
			//TRACE("Sensor update while rendering (%d) angle\n", (int) to_deg(cur_th));
			::Sleep(1);
		}
		if(m_pcur_scan_data->pdist1_data[i] == 0){
				continue;
		}
	
		//distance = ( (float) m_pcur_scan_data->pdist1_data[i]) / 1000.0f;

		if(i>=90 && i< 451){
			m_LRFRangeData_181[(int)(i-90)/2] =  ( (float) m_pcur_scan_data->pdist1_data[i]);
			//if( nLaserData[(int)(i-90)/2] <= 50 ) nLaserData[(int)(i-90)/2] = 99999; 
			
		}
	}

	return m_LRFRangeData_181;

}

void SickLMS100Interface::startTimeCheck(LARGE_INTEGER& nStart)
{
	QueryPerformanceCounter(&nStart);
}

float SickLMS100Interface::finishTimeCheck(const LARGE_INTEGER nStart)
{
	LARGE_INTEGER nFinish, freq;

	QueryPerformanceFrequency(&freq);

	QueryPerformanceCounter(&nFinish);

	return (float)(1000.0 * (nFinish.QuadPart - nStart.QuadPart) / freq.QuadPart);
}
