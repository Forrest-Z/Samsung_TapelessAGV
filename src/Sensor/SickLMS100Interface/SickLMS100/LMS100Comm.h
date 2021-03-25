#pragma once

#ifndef LMS100COMM_H
#define LMS100COMM_H

#define USE_REAR_LASER		0
#define LMS100_SCAN_FREQ	50

#define SICK_LMS100_WINSOCK_INIT	1

#define SICK_LMS100_FRONT_ADDR "192.168.112.100"
#define SICK_LMS100_FRONT_PORT 2111

#define SICK_LMS100_REAR_ADDR "163.152.57.202"
#define SICK_LMS100_REAR_PORT 2111

#define SICK_LMS100_MAX_QUERY_COUNT 120

#include "Winsock2.h"

typedef struct _tag_lms100_cfg_data_output
{
	unsigned int output_channel;
	
	unsigned int remission;
	unsigned int resolution; //remission resolution(8bit=0 or 16bit=1) :: 
	unsigned int unit;       //remission unit (digit for 0)
	unsigned int encoder;    //encoder data (0 : no, 1 : encoder channel1)

	unsigned int position;		//position data   (0 : no, 1 : yes)
	unsigned int device_name;	//device_name data(0 : no, 1 : yes)
	unsigned int comment;		//comment data	  (0 : no, 1 : yes)
	unsigned int time;		//time data	  (0 : no, 1 : yes)
	
	unsigned int output_interval;	//output interval data(1 : every scan, 2 : every 2nd scan, 50,000: every 50,000th scan)
}lms100_cfg_data_output;

typedef struct _tag_lms100_cfg
{
	unsigned int	nscan_freq;   //scanning herz
	unsigned int	nseg_cnt;     //number of segments
	
	
	float	fangle_resol; //angular resolution : 0.25 ~ 0.5
	float   fstart_angle; //scan start angle in deg
	float   fend_angle;   //scan end angle in deg

	int range_data_cnt;   //computed count of range data 
	                      //e.g. : 0.5deg resolution and -45 to 225 
	                      //       (225 - (-45)) / 0.5 + 1 = 541
	
	lms100_cfg_data_output data_output;
}lms100_cfg;


typedef struct _tag_lms100_scan_data
{
	bool is_updating;
	
	unsigned int number_of_channels;
	
	//dist1 variable
	float		dist1_scaling_factor;
	float		dist1_scaling_offset;
	float		dist1_starting_angle;
	float		dist1_angular_step_width;
	unsigned int	dist1_number_of_data;

	unsigned int *  pdist1_data;

	//dist2 variable
	float		dist2_scaling_factor;
	float		dist2_scaling_offset;
	float		dist2_starting_angle;
	float		dist2_angular_step_width;
	unsigned int	dist2_number_of_data;

	unsigned int *  pdist2_data;

	//rssi1 variable
	float		rssi1_scaling_factor;
	float		rssi1_scaling_offset;
	float		rssi1_starting_angle;
	float		rssi1_angular_step_width;
	unsigned int	rssi1_number_of_data;

	unsigned int *  prssi1_data;

	//rssi2 variable
	float		rssi2_scaling_factor;
	float		rssi2_scaling_offset;
	float		rssi2_starting_angle;
	float		rssi2_angular_step_width;
	unsigned int	rssi2_number_of_data;

	unsigned int *  prssi2_data;
}lms100_scan_data;

bool lms100_connect	(SOCKET * pSocket, char * szAddr, int port);
void lms100_disconnect	(SOCKET * pSocket);

bool lms100_send_comm_msg	(SOCKET * pSocket, char * szmsg, int msg_len);
bool lms100_rcv_comm_msg	(SOCKET * pSocket, char * psz_ret_msg, int max_msg_len, int * pret_msg_len);

bool lms100_start_measure(SOCKET * pSocket);
bool lms100_stop_measure (SOCKET * pSocket);

bool lms100_query_status(SOCKET * pSocket);
bool lms100_read_scan_cfg(SOCKET * pSocket, lms100_cfg * pcfg);

bool lms100_read_single_scan    (SOCKET * pSocket, lms100_cfg cfg, lms100_scan_data *pscan_data);
bool lms100_read_continuous_scan(SOCKET * pSocket, lms100_cfg cfg, lms100_scan_data *pscan_data);

//special functions which require to login as a manager level
bool lms100_login_release	 (SOCKET * pSocket);
bool lms100_login_as_super_user  (SOCKET * pSocket);
bool lms100_save_data_permanently(SOCKET * pSocket); //프로토콜 이상: 저장이 안됨

int  lms100_set_scan_cfg_freq_resol_area(SOCKET * pSocket, lms100_cfg * pcfg);
bool lms100_set_scan_cfg_output_data(SOCKET * pSocket, lms100_cfg * pcfg);

bool lms100_parse_scan_data  (lms100_cfg cfg, lms100_scan_data *pscan_data, char * pmsg_buff, int msg_len);

char * lms100_util_parse_hex_unsigned_int(char * pmsg, int size, unsigned int * pvalue);
char * lms100_util_parse_hex_signed_int	 (char * pmsg, int size, int * pvalue);
char * lms100_util_parse_hex_real	 (char * pmsg, int size, float * pvalue);

#endif