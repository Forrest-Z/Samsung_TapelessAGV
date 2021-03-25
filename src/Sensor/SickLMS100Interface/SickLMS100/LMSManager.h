#ifndef LMSMANAGER_H
#define LMSMANAGER_H

#include "LMS100Comm.h"

class CLMSManager
{
public:
	CLMSManager();
	~CLMSManager();

	static CLMSManager *	getInstance(void);
	static void		deleteInstance(void);
	
	bool is_use_rear_laser();
	bool is_lms_running();

	lms100_cfg get_front_cfg();
	lms100_cfg get_rear_cfg ();

	SOCKET * get_front_socket();
	SOCKET * get_rear_socket ();

	lms100_scan_data * get_front_scan_data(); //if null -> no data output or not-initialized
	lms100_scan_data * get_rear_scan_data(); //if null -> no data output or not-initialized

	void run_thread ();
	void stop_thread();
	
	bool connect(char* cIP);

	static UINT __cdecl ThreadFunc_LMS100_Scanner( LPVOID pParam );

	bool m_brun_show_thread;
	bool m_brun_show_thread_confirmed;

protected:
	static CLMSManager * thisInstance;

	
	bool init(char* cIP);
	void clear();

	bool m_bIntialized;
	bool m_bUseRearLaser;
	
	SOCKET m_socket_front;	
	SOCKET m_socket_rear;

	//front scan data
	lms100_cfg	 m_cfg_front;
	lms100_scan_data m_scan_data_front;
	
	//rear scan data
	lms100_cfg	 m_cfg_rear;
	lms100_scan_data m_scan_data_rear;
	
	
};
#endif