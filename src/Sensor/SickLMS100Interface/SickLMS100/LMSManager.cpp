#include "stdafx.h"
#include "LMSManager.h"

UINT __cdecl CLMSManager::ThreadFunc_LMS100_Scanner( LPVOID pParam )
{
	CLMSManager * plmsManager = (CLMSManager *) pParam; 
	
	SOCKET		  * psocket_front	= NULL;
	lms100_cfg	    cfg_front;
	lms100_scan_data  * pscan_data_front	= NULL;
	
	plmsManager->m_brun_show_thread_confirmed = false;

	psocket_front	 = plmsManager->get_front_socket();
	cfg_front	 = plmsManager->get_front_cfg();
	pscan_data_front = plmsManager->get_front_scan_data();
	

	while(plmsManager->m_brun_show_thread)
	{
		lms100_read_single_scan(psocket_front, cfg_front, pscan_data_front);
		::Sleep(40);
	}

	plmsManager->m_brun_show_thread_confirmed = true;
	return 0;
}

CLMSManager * CLMSManager::thisInstance = NULL;

CLMSManager * CLMSManager::getInstance(void)
{
	if ( thisInstance == NULL )
	{
		thisInstance = new CLMSManager();
	}
	return thisInstance;
}

void CLMSManager::deleteInstance(void)
{
	if ( thisInstance != NULL )
	{
		thisInstance->clear();
		delete thisInstance;
		thisInstance = NULL;
	}	
}

CLMSManager::CLMSManager()
	: m_brun_show_thread(false)
	, m_brun_show_thread_confirmed(true)
{
	m_bIntialized	= false;
	m_bUseRearLaser = (USE_REAR_LASER == 1) ? true : false;

	m_scan_data_front.pdist1_data = NULL;
	m_scan_data_front.pdist2_data = NULL;
	m_scan_data_front.prssi1_data = NULL;
	m_scan_data_front.prssi2_data = NULL;

	m_scan_data_rear.pdist1_data = NULL;
	m_scan_data_rear.pdist2_data = NULL;
	m_scan_data_rear.prssi1_data = NULL;
	m_scan_data_rear.prssi2_data = NULL;
}

CLMSManager::~CLMSManager()
{

}

bool CLMSManager::connect(char* cIP)
{
	return init(cIP);
}

bool CLMSManager::init(char* cIP)
{
	if(m_bIntialized == true)
	{
		return false;
	}

	m_cfg_front.nscan_freq   = LMS100_SCAN_FREQ;
	m_cfg_front.nseg_cnt     = 1;
	m_cfg_front.fangle_resol = (LMS100_SCAN_FREQ == 50) ? 0.5 : 0.25;
	m_cfg_front.fstart_angle = -45;
	m_cfg_front.fend_angle   = 225;
	
	m_cfg_front.data_output.output_channel	= 3;
	m_cfg_front.data_output.remission	= 1;
	m_cfg_front.data_output.resolution	= 1;
	m_cfg_front.data_output.unit		= 0;
	m_cfg_front.data_output.encoder		= 1;
	
	m_cfg_front.data_output.position	= 0;
	m_cfg_front.data_output.device_name	= 1;
	m_cfg_front.data_output.comment		= 0;
	m_cfg_front.data_output.time		= 1;
	m_cfg_front.data_output.output_interval	= 1;

	//Connect and Initialize Laser
	int  n_query_count    = 0;
	bool b_front_result   = false;
	int  n_setting_result = 0;
	

	//connect via network
//	b_front_result = lms100_connect(&m_socket_front, SICK_LMS100_FRONT_ADDR, SICK_LMS100_FRONT_PORT);
	b_front_result = lms100_connect(&m_socket_front, cIP, SICK_LMS100_FRONT_PORT);
	if(b_front_result == false)
	{
		clear();
		return false;
	}
/*
	//query lms status
	n_query_count  = 0;
	while(n_query_count < SICK_LMS100_MAX_QUERY_COUNT)
	{
		b_front_result = lms100_query_status(&m_socket_front);

		if(b_front_result == true)
		{
			break;
		}
		TRACE("-- 000 LMSManager:: Waiting LMS Ready :: %d -- \n", n_query_count);

		n_query_count++;
		::Sleep(1000);
	}

	if(b_front_result == false)
	{
		clear();
		lms100_disconnect(&m_socket_front);
		return false;
	}
	
	//set lms scan freq and area
	n_setting_result = lms100_set_scan_cfg_freq_resol_area(&m_socket_front, &m_cfg_front);
	if(n_setting_result != 0)
	{
		clear();
		lms100_disconnect(&m_socket_front);
		return false;
	}
	
	//query lms status
	n_query_count  = 0;
	while(n_query_count < SICK_LMS100_MAX_QUERY_COUNT)
	{
		b_front_result = lms100_query_status(&m_socket_front);

		if(b_front_result == true)
		{
			break;
		}
		TRACE("-- 111 LMSManager:: Waiting LMS Ready :: %d -- \n", n_query_count);

		n_query_count++;
		::Sleep(1000);
	}
	if(b_front_result == false)
	{
		clear();
		lms100_disconnect(&m_socket_front);
		return false;
	}
	
	//set lms data output format
	b_front_result = lms100_set_scan_cfg_output_data(&m_socket_front, &m_cfg_front);
	if(b_front_result == false)
	{
		clear();
		lms100_disconnect(&m_socket_front);
		return false;
	}
	
	//query lms status
	n_query_count  = 0;
	while(n_query_count < SICK_LMS100_MAX_QUERY_COUNT)
	{
		b_front_result = lms100_query_status(&m_socket_front);

		if(b_front_result == true)
		{
			break;
		}
		TRACE("-- 222 LMSManager:: Waiting LMS Ready :: %d -- \n", n_query_count);

		n_query_count++;
		::Sleep(1000);
	}
	if(b_front_result == false)
	{
		clear();
		lms100_disconnect(&m_socket_front);
		return false;
	}
	
*/
	//allocate buffer memory for front lms100
	m_cfg_front.nscan_freq   = 50;
	m_cfg_front.nseg_cnt = 1;

	m_cfg_front.fangle_resol = 0.5;
	m_cfg_front.fstart_angle = -45.0;
	m_cfg_front.fend_angle = 225.0;
	m_cfg_front.range_data_cnt = 541;

	m_scan_data_front.pdist1_data = (unsigned int * ) malloc(sizeof(unsigned int) * (m_cfg_front.range_data_cnt + 1));
	m_scan_data_front.pdist2_data = (unsigned int * ) malloc(sizeof(unsigned int) * (m_cfg_front.range_data_cnt + 1));
	m_scan_data_front.prssi1_data = (unsigned int * ) malloc(sizeof(unsigned int) * (m_cfg_front.range_data_cnt + 1));
	m_scan_data_front.prssi2_data = (unsigned int * ) malloc(sizeof(unsigned int) * (m_cfg_front.range_data_cnt + 1));
	
	if(m_bUseRearLaser == true)
	{
		m_cfg_rear.nscan_freq   = LMS100_SCAN_FREQ;
		m_cfg_rear.nseg_cnt     = 1;
		m_cfg_rear.fangle_resol = (LMS100_SCAN_FREQ == 50) ? 0.5 : 0.25;
		m_cfg_rear.fstart_angle = -45;
		m_cfg_rear.fend_angle   = 225;
		
		m_cfg_rear.data_output.output_channel	= 3;
		m_cfg_rear.data_output.remission		= 1;
		m_cfg_rear.data_output.resolution		= 1;
		m_cfg_rear.data_output.unit		= 0;
		m_cfg_rear.data_output.encoder		= 1;
		
		m_cfg_rear.data_output.position		= 0;
		m_cfg_rear.data_output.device_name	= 1;
		m_cfg_rear.data_output.comment		= 0;
		m_cfg_rear.data_output.time		= 1;
		m_cfg_rear.data_output.output_interval	= 1;

		//connect via network
		b_front_result = lms100_connect(&m_socket_rear, SICK_LMS100_REAR_ADDR, SICK_LMS100_REAR_PORT);
		if(b_front_result == false)
		{
			clear();
			return false;
		}

		//query lms status
		n_query_count  = 0;
		while(n_query_count < SICK_LMS100_MAX_QUERY_COUNT)
		{
			b_front_result = lms100_query_status(&m_socket_rear);

			if(b_front_result == true)
			{
				break;
			}
			TRACE("-- LMSManager:: Waiting LMS Ready :: %d -- \n", n_query_count);

			n_query_count++;
			::Sleep(1000);
		}

		if(b_front_result == false)
		{
			clear();
			lms100_disconnect(&m_socket_rear);
			return false;
		}
		
		//set lms scan freq and area
		n_setting_result = lms100_set_scan_cfg_freq_resol_area(&m_socket_rear, &m_cfg_rear);
		if(n_setting_result != 0)
		{
			clear();
			lms100_disconnect(&m_socket_rear);
			return false;
		}
		
		//query lms status
		n_query_count  = 0;
		while(n_query_count < SICK_LMS100_MAX_QUERY_COUNT)
		{
			b_front_result = lms100_query_status(&m_socket_rear);

			if(b_front_result == true)
			{
				break;
			}
			TRACE("-- LMSManager:: Waiting LMS Ready :: %d -- \n", n_query_count);

			n_query_count++;
			::Sleep(1000);
		}

		if(b_front_result == false)
		{
			clear();
			lms100_disconnect(&m_socket_rear);
			return false;
		}

		//set lms data output format
		b_front_result = lms100_set_scan_cfg_output_data(&m_socket_rear, &m_cfg_rear);
		if(b_front_result == false)
		{
			clear();
			lms100_disconnect(&m_socket_rear);
			return false;
		}
		
		//query lms status
		n_query_count  = 0;
		while(n_query_count < SICK_LMS100_MAX_QUERY_COUNT)
		{
			b_front_result = lms100_query_status(&m_socket_rear);

			if(b_front_result == true)
			{
				break;
			}
			TRACE("-- LMSManager:: Waiting LMS Ready :: %d -- \n", n_query_count);

			n_query_count++;
			::Sleep(1000);
		}
		if(b_front_result == false)
		{
			clear();
			lms100_disconnect(&m_socket_rear);
			return false;
		}

		m_scan_data_rear.pdist1_data = (unsigned int * ) malloc(sizeof(unsigned int) * (m_cfg_rear.range_data_cnt + 1));
		m_scan_data_rear.pdist2_data = (unsigned int * ) malloc(sizeof(unsigned int) * (m_cfg_rear.range_data_cnt + 1));
		m_scan_data_rear.prssi1_data = (unsigned int * ) malloc(sizeof(unsigned int) * (m_cfg_rear.range_data_cnt + 1));
		m_scan_data_rear.prssi2_data = (unsigned int * ) malloc(sizeof(unsigned int) * (m_cfg_rear.range_data_cnt + 1));
	}
	m_bIntialized = true;
	return true;
}

void CLMSManager::clear()
{
	if(m_bIntialized == false)
	{
		return;
	}

	m_bIntialized	= false;

	::Sleep(1000);

	m_bUseRearLaser = (USE_REAR_LASER == 1) ? true : false;
	
	lms100_disconnect(&m_socket_front);

	if(m_scan_data_front.pdist1_data) free(m_scan_data_front.pdist1_data);
	if(m_scan_data_front.pdist2_data) free(m_scan_data_front.pdist2_data);
	if(m_scan_data_front.prssi1_data) free(m_scan_data_front.prssi1_data);
	if(m_scan_data_front.prssi2_data) free(m_scan_data_front.prssi2_data);

	m_scan_data_front.pdist1_data = NULL;
	m_scan_data_front.pdist2_data = NULL;
	m_scan_data_front.prssi1_data = NULL;
	m_scan_data_front.prssi2_data = NULL;

	if(m_bUseRearLaser == true)
	{
		lms100_disconnect(&m_socket_rear);

		if(m_scan_data_rear.pdist1_data) free(m_scan_data_rear.pdist1_data);
		if(m_scan_data_rear.pdist2_data) free(m_scan_data_rear.pdist2_data);
		if(m_scan_data_rear.prssi1_data) free(m_scan_data_rear.prssi1_data);
		if(m_scan_data_rear.prssi2_data) free(m_scan_data_rear.prssi2_data);
		
		m_scan_data_rear.pdist1_data = NULL;
		m_scan_data_rear.pdist2_data = NULL;
		m_scan_data_rear.prssi1_data = NULL;
		m_scan_data_rear.prssi2_data = NULL;
	}
}

bool CLMSManager::is_use_rear_laser()
{
	return m_bUseRearLaser;
}

bool CLMSManager::is_lms_running()
{
	if(m_bIntialized == false)
	{
		return false;
	}
	
	return m_brun_show_thread;
}

lms100_cfg CLMSManager::get_front_cfg()
{
	return m_cfg_front;
}

lms100_cfg CLMSManager::get_rear_cfg ()
{
	return m_cfg_rear;
}

lms100_scan_data * CLMSManager::get_front_scan_data()
{
	if(m_bIntialized == false)
	{
		return NULL;
	}

	return &m_scan_data_front;
}

lms100_scan_data * CLMSManager::get_rear_scan_data ()
{
	if(m_bUseRearLaser == false || m_bIntialized == false)
	{
		return NULL;
	}

	return &m_scan_data_rear;
}

SOCKET * CLMSManager::get_front_socket()
{
	if(m_bIntialized == false)
	{
		return NULL;
	}
	return &m_socket_front;
}

SOCKET * CLMSManager::get_rear_socket ()
{
	if(m_bUseRearLaser == false || m_bIntialized == false)
	{
		return NULL;
	}
	return &m_socket_rear;
}

void CLMSManager::run_thread ()
{
	if(m_brun_show_thread == true || m_bIntialized == false)
	{
		return;
	}

	m_brun_show_thread		= true;

	AfxBeginThread(&CLMSManager::ThreadFunc_LMS100_Scanner, this, THREAD_PRIORITY_TIME_CRITICAL);
}

void CLMSManager::stop_thread()
{
	m_brun_show_thread = false;
	
	while(1)
	{
		if(m_brun_show_thread_confirmed == true)
		{
			break;
		}
		::Sleep(40);
	}
}
