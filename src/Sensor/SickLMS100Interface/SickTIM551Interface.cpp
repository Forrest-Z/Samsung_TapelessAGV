#include "stdafx.h"
#include "SickTIM551Interface.h"

SickTIM551Interface::SickTIM551Interface()
{
	m_bIntialized = false;
	m_isSockEstablised = false;
	m_LRFRangeData_181=	 m_KuUtil.generateIntType1DArray(Sensor::URG04LX_DATA_NUM181,0);
	m_socketFrontSick=NULL;
}

SickTIM551Interface::~SickTIM551Interface()
{

}

void SickTIM551Interface::disconnectLaserScanner()
{
	if(m_socketFrontSick!=NULL)
		lms100_disconnect(&m_socketFrontSick);
}

bool SickTIM551Interface::connect(char * cIP, int nPort)
{
//	m_isSockEstablised = lms100_connect(&m_socketFrontSick, cIP, nPort);
	if(1)//m_isSockEstablised == true)
	{
									
		cout<<"Socket Connection is established"<<endl;

//		CLMSManager * plmsManager = NULL;
//		plmsManager = CLMSManager::getInstance();

		if(m_lms_manager.connect(cIP) == true){
			cout<<"Laser Connection Successed"<<endl;
			m_lms_manager.run_thread();
			return true;
		}
		else{	
			cout<<"Laser Connection Failure"<<endl;
			return false;
		}
		
	}
	else{
		cout<<"Socket Connection is failed"<<endl;
		return false;
	}
}


int_1DArray SickTIM551Interface::getData( )
{

	float cur_th = 0;
	float distance = 0;
	float height = 0;
	for(int i=0; i<Sensor::URG04LX_DATA_NUM181;i++) m_LRFRangeData_181[i] = 0;

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

		if(i>=45 && i< 226){ // 181 scan data with 1 degree angle step
			m_LRFRangeData_181[(int)(i-45)] = ((float) m_pcur_scan_data->pdist1_data[i]);
			//if( nLaserData[(int)(i-90)/2] <= 50 ) nLaserData[(int)(i-90)/2] = 99999; 
			
		}
	}

	return m_LRFRangeData_181;

}

void SickTIM551Interface::startTimeCheck(LARGE_INTEGER& nStart)
{
	QueryPerformanceCounter(&nStart);
}

float SickTIM551Interface::finishTimeCheck(const LARGE_INTEGER nStart)
{
	LARGE_INTEGER nFinish, freq;

	QueryPerformanceFrequency(&freq);

	QueryPerformanceCounter(&nFinish);

	return (float)(1000.0 * (nFinish.QuadPart - nStart.QuadPart) / freq.QuadPart);
}
