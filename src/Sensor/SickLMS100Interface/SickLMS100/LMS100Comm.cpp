#include "stdafx.h"
#include "math_util.h"
#include "LMS100Comm.h"

#define SICK_LMS_STX 0x02
#define SICK_LMS_ETX 0x03

#define SICK_DATA_TYPE_BUFF_MAX		32  
#define SICK_MAX_RCV_BUFF		20480
#define SICK_HALF_BYTE_SIZE_IN_BIT	4

WSADATA lms_wsad;

bool lms100_connect(SOCKET * pSocket, char * szAddr, int port)
{
#if SICK_LMS100_WINSOCK_INIT
	if (WSAStartup(MAKEWORD(2,2), &lms_wsad)) 
	{
		WSACleanup();
	}
#endif

	(*pSocket) = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (*pSocket == INVALID_SOCKET) 
	{
		//TRACE"Error at socket\n");		
		return false;
	}

	//----------------------
	// The sockaddr_in structure specifies the address family,
	// IP address, and port of the server to be connected to.
	sockaddr_in clientService; 
	clientService.sin_family = AF_INET;
	clientService.sin_addr.s_addr = inet_addr( szAddr );
	clientService.sin_port = htons( port );

	//----------------------
	// Connect to server.
	if ( connect( *pSocket, (SOCKADDR*) &clientService, sizeof(clientService) ) == SOCKET_ERROR) 
	{
		//TRACE "Failed to connect\n" );
		return false;
	}
	return true;
}

void lms100_disconnect(SOCKET * pSocket)
{
	closesocket(*pSocket);

#if SICK_LMS100_WINSOCK_INIT
	WSACleanup();
#endif
}


bool lms100_send_comm_msg(SOCKET * pSocket, char * szmsg, int msg_len)
{
	int send_len = 0;
	send_len = send(*pSocket, szmsg, msg_len, 0); 
	if(send_len != msg_len)
	{
		//TRACE "Send Msg length is not same.\n");
		return false;
	}
	return true;
}

bool lms100_rcv_comm_msg(SOCKET * pSocket, char * psz_ret_msg, int max_msg_len, int * pret_msg_len)
{
	int rcv_len = SOCKET_ERROR;
	rcv_len = recv( *pSocket, psz_ret_msg, max_msg_len, 0 );
	
	if ( rcv_len == 0 || rcv_len == WSAECONNRESET ) 
	{
		*pret_msg_len = 0;
	}
	else
	{
		*pret_msg_len = rcv_len;
	}
	return true;
}

bool lms100_start_measure(SOCKET * pSocket)
{
	bool snd_result, rcv_result;
	
	int  recv_msg_len  = -1;
	char recv_buff[SICK_MAX_RCV_BUFF] = {0};

	int  send_msg_len  = 0;
	char send_buff[32] = {0};
	
	char * psend_buf = send_buff;

	send_buff[0] = SICK_LMS_STX;
	sprintf(++psend_buf, "%s", "sMN LMCstartmeas");
	send_buff[strlen(send_buff)] = SICK_LMS_ETX;
	
	send_msg_len = strlen(send_buff);
	snd_result = lms100_send_comm_msg(pSocket, send_buff, send_msg_len);
	if(snd_result == true)
	{
		rcv_result = lms100_rcv_comm_msg(pSocket, recv_buff, SICK_MAX_RCV_BUFF, &recv_msg_len);
		
		//TRACE"-- LMS LMCstartmeas Response (%d) -- \n %s \n", recv_msg_len, recv_buff);
		
		return rcv_result;
	}
	return false;
}

bool lms100_stop_measure (SOCKET * pSocket)
{
	bool snd_result, rcv_result;
	
	int  recv_msg_len  = -1;
	char recv_buff[SICK_MAX_RCV_BUFF] = {0};

	int  send_msg_len  = 0;
	char send_buff[32] = {0};
	
	char * psend_buf = send_buff;
	send_buff[0] = SICK_LMS_STX;
	sprintf(++psend_buf, "%s", "sMN LMCstopmeas");
	send_buff[strlen(send_buff)] = SICK_LMS_ETX;
	
	send_msg_len = strlen(send_buff);
	snd_result = lms100_send_comm_msg(pSocket, send_buff, send_msg_len);
	if(snd_result == true)
	{
		rcv_result = lms100_rcv_comm_msg(pSocket, recv_buff, SICK_MAX_RCV_BUFF, &recv_msg_len);
		
		//TRACE"-- LMS LMCstopmeas Response (%d) -- \n %s \n", recv_msg_len, recv_buff);
		
		return rcv_result;
	}
	return false;
}

bool lms100_query_status(SOCKET * pSocket)
{
	bool snd_result, rcv_result;
	
	int  recv_msg_len  = -1;
	char recv_buff[SICK_MAX_RCV_BUFF] = {0};

	int  send_msg_len  = 0;
	char send_buff[32] = {0};
	
	char * psend_buf = send_buff;
	
	send_buff[0] = SICK_LMS_STX;
	sprintf(++psend_buf, "%s", "sRN STlms");
	send_buff[strlen(send_buff)] = SICK_LMS_ETX;
	
	send_msg_len = strlen(send_buff);
	
	int retry_count = 0;
	int rcv_prefix_len = strlen("sRA STlms ");
	char * prcv_buf  = NULL;

	snd_result = lms100_send_comm_msg(pSocket, send_buff, send_msg_len);
	if(snd_result == true)
	{
		rcv_result = lms100_rcv_comm_msg(pSocket, recv_buff, SICK_MAX_RCV_BUFF, &recv_msg_len);
		
		//TRACE"-- lms100_query_status response (%d) -- \n %s \n", recv_msg_len, recv_buff);
		
		prcv_buf = strstr(recv_buff, "sRA STlms ");
		
		if( prcv_buf )
		{
			prcv_buf += rcv_prefix_len;
			if(prcv_buf[0] == '7')
			{
				return rcv_result;
			}
		}
	}
	return false;
}

char * lms100_util_parse_hex_real(char * pmsg, int size, float * pvalue)
{
	int template_idx    = -1;
	char * ptemp_dest   = 0;
	char * template_msg = "0123456789abcdefABCDEF";
	
	char buff[SICK_DATA_TYPE_BUFF_MAX] = {0};
	char * pcur_idx = pmsg;
	
	unsigned int cur_value = 0;
	unsigned int tmp_value = 0;
	int max_digit = -1; //최대 자릿수

	unsigned int sign	= 0;
	unsigned int exponent	= 0;
	float        mantisse	= 1.0f;
	
	float result = 0.0f;

	if(pcur_idx == NULL)
	{
		return NULL;
	}

	if(pcur_idx[0] == ' ')
	{
		pcur_idx++;
	}

	for(int i = 0; i < size*2; i++)
	{
		buff[i] = pcur_idx[0];		
		max_digit++;

		pcur_idx++;
		if(pcur_idx[0] == ' ' || pcur_idx[0] == SICK_LMS_ETX)
		{
			break;
		}
	}
	
	//데이터가 잘못되었음
	if(max_digit < 0)
	{
		return NULL;
	}

	if(pcur_idx[0] == ' ')
	{
		pcur_idx++;
	}
	
	for(int i = 0; i <= max_digit; i++)
	{
		ptemp_dest   = strchr(template_msg, buff[i]);
		template_idx = (int) (ptemp_dest - template_msg);
		if(ptemp_dest == NULL)
		{
			//data is wrong
			return NULL;
		}
		tmp_value  = template_idx;
		if(tmp_value > 15)
		{
			tmp_value -= 6;
		}
		cur_value |= tmp_value << (max_digit-i)*SICK_HALF_BYTE_SIZE_IN_BIT;
	}
	if(cur_value == 0)
	{
		*pvalue = 0.0f;
		return pcur_idx;
	}

	sign	 = (cur_value & (0x01 << 31)) >> 31; 
	cur_value = cur_value << 1; cur_value = cur_value >> 1;

	exponent = (cur_value & (0xFF << 23)) >> 23; 
	cur_value = cur_value << 9; cur_value = cur_value >> 9;
	
	for(int i = 0; i < 6; i++)
	{
		mantisse +=  ( (float) ((cur_value & (0x0F << (19-i*4))) >> (19-i*4) )) / ( (float) (0x10 << (i * 4)));
	}
	result = ((sign == 0) ? 1.0f : -1.0f) * mantisse * ( (float) (0x01 << (exponent-127)) );
	
	*pvalue = result;

	return pcur_idx;
}

char * lms100_util_parse_hex_unsigned_int(char * pmsg, int size, unsigned int * pvalue)
{
	int template_idx    = -1;
	char * ptemp_dest   = 0;
	char * template_msg = "0123456789abcdefABCDEF";
	
	char buff[SICK_DATA_TYPE_BUFF_MAX] = {0};
	char * pcur_idx = pmsg;
	
	unsigned int cur_value = 0;
	unsigned int tmp_value = 0;
	int max_digit = -1; //최대 자릿수

	if(pcur_idx == NULL)
	{
		return NULL;
	}

	if(pcur_idx[0] == ' ')
	{
		pcur_idx++;
	}

	for(int i = 0; i < size*2; i++)
	{
		buff[i] = pcur_idx[0];		
		max_digit++;

		pcur_idx++;
		if(pcur_idx[0] == ' ' || pcur_idx[0] == SICK_LMS_ETX)
		{
			break;
		}
	}
	
	//데이터가 잘못되었음
	if(max_digit < 0)
	{
		return NULL;
	}

	if(pcur_idx[0] == ' ')
	{
		pcur_idx++;
	}
	
	for(int i = 0; i <= max_digit; i++)
	{
		ptemp_dest   = strchr(template_msg, buff[i]);
		template_idx = (int) (ptemp_dest - template_msg);
		if(ptemp_dest == NULL)
		{
			//data is wrong
			return NULL;
		}
		tmp_value  = template_idx;
		if(tmp_value > 15)
		{
			tmp_value -= 6;
		}
		cur_value |= tmp_value << (max_digit-i)*SICK_HALF_BYTE_SIZE_IN_BIT;
	}
	*pvalue = cur_value;

	return pcur_idx;
}

char * lms100_util_parse_hex_signed_int(char * pmsg, int size, int * pvalue)
{
	int template_idx    = -1;
	char * ptemp_dest   = 0;
	char * template_msg = "0123456789abcdefABCDEF";
	
	char buff[SICK_DATA_TYPE_BUFF_MAX] = {0};
	char * pcur_idx = pmsg;
	
	int cur_value = 0;
	int tmp_value = 0;
	int max_digit = -1; //최대 자릿수

	if(pcur_idx == NULL)
	{
		return NULL;
	}

	if(pcur_idx[0] == ' ')
	{
		pcur_idx++;
	}

	for(int i = 0; i < size*2; i++)
	{
		buff[i] = pcur_idx[0];		
		max_digit++;
		
		pcur_idx++;
		if(pcur_idx[0] == ' ' || pcur_idx[0] == SICK_LMS_ETX)
		{
			break;
		}
	}
	
	//데이터가 잘못되었음
	if(max_digit < 0)
	{
		return NULL;
	}

	if(pcur_idx[0] == ' ')
	{
		pcur_idx++;
	}
	
	for(int i = 0; i <= max_digit; i++)
	{
		ptemp_dest   = strchr(template_msg, buff[i]);
		template_idx = (int) (ptemp_dest - template_msg);
		if(ptemp_dest == NULL)
		{
			//data is wrong
			return NULL;
		}
		tmp_value  = template_idx;
		if(tmp_value > 15)
		{
			tmp_value -= 6;
		}
		cur_value |= tmp_value << (max_digit-i)*SICK_HALF_BYTE_SIZE_IN_BIT;
	}
	*pvalue = cur_value;

	return pcur_idx;
}

bool lms100_read_scan_cfg(SOCKET * pSocket, lms100_cfg * pcfg)
{
	bool snd_result, rcv_result;
	
	int  recv_msg_len  = -1;
	char recv_buff[SICK_MAX_RCV_BUFF] = {0};

	int  send_msg_len  = 0;
	char send_buff[32] = {0};
	
	char * pcur_rcv_msg = NULL;
	char   parse_buff[16] = {0};

	char * psend_buf = send_buff;
	send_buff[0] = SICK_LMS_STX;
	sprintf(++psend_buf, "%s", "sRN LMPscancfg");
	send_buff[strlen(send_buff)] = SICK_LMS_ETX;
	
	send_msg_len = strlen(send_buff);
	snd_result = lms100_send_comm_msg(pSocket, send_buff, send_msg_len);


	unsigned int tmp_unsigned_int = 0;
	int          tmp_signed_int   = 0;

	if(snd_result == true)
	{
		rcv_result = lms100_rcv_comm_msg(pSocket, recv_buff, SICK_MAX_RCV_BUFF, &recv_msg_len);
		
		//TRACE"-- lms100_read_scan_cfg response (%d) -- \n %s \n", recv_msg_len, recv_buff);
		pcur_rcv_msg = strstr(recv_buff, "LMPscancfg");
		if(pcur_rcv_msg)
		{
			pcur_rcv_msg += strlen("LMPscancfg")+1; //set msg ptr to data start idx

			//parse scanning frequency
			pcur_rcv_msg       = lms100_util_parse_hex_unsigned_int(pcur_rcv_msg, 4, &tmp_unsigned_int);
			pcfg->nscan_freq   = (unsigned int) ((float) tmp_unsigned_int) / 100.0f;

			//parse number of segments
			pcur_rcv_msg   = lms100_util_parse_hex_unsigned_int(pcur_rcv_msg, 2, &tmp_unsigned_int);
			pcfg->nseg_cnt = tmp_unsigned_int;

			//parse angular resolution
			pcur_rcv_msg = lms100_util_parse_hex_unsigned_int(pcur_rcv_msg, 4, &tmp_unsigned_int);
			pcfg->fangle_resol = ((float) tmp_unsigned_int) / 10000.0f;

			//parse start angle in deg
			pcur_rcv_msg = lms100_util_parse_hex_signed_int(pcur_rcv_msg, 4, &tmp_signed_int);
			pcfg->fstart_angle = ((float) tmp_signed_int) / 10000.0f;

			//parse end angle in deg
			pcur_rcv_msg = lms100_util_parse_hex_signed_int(pcur_rcv_msg, 4, &tmp_signed_int);
			pcfg->fend_angle = ((float) tmp_signed_int) / 10000.0f;
			
			pcfg->range_data_cnt = (int) ((pcfg->fend_angle - pcfg->fstart_angle) / pcfg->fangle_resol + 1.0f);
			return true;
		}
	}
	return false;
}

//special functions which require to login as a manager level
bool lms100_login_release(SOCKET * pSocket)
{
	bool snd_result, rcv_result;
	
	int  recv_msg_len  = -1;
	char recv_buff[SICK_MAX_RCV_BUFF] = {0};

	int  send_msg_len  = 0;
	char send_buff[32] = {0};
	
	char * pcur_rcv_msg = NULL;
	char   parse_buff[16] = {0};

	char * psend_buf = send_buff;
	send_buff[0] = SICK_LMS_STX;
	sprintf(++psend_buf, "%s", "sMI 2"); //sM I (아이임, 엘 아님)
	send_buff[strlen(send_buff)] = SICK_LMS_ETX;
	
	send_msg_len = strlen(send_buff);
	snd_result = lms100_send_comm_msg(pSocket, send_buff, send_msg_len);
	
	unsigned int login_result = 0;
	if(snd_result == true)
	{
		rcv_result = lms100_rcv_comm_msg(pSocket, recv_buff, SICK_MAX_RCV_BUFF, &recv_msg_len);
		
		//TRACE"-- lms100_login_release response (%d) -- \n %s \n", recv_msg_len, recv_buff);
		return rcv_result;		
	}
	return false;
}

bool lms100_login_as_super_user(SOCKET * pSocket)
{
	bool snd_result, rcv_result;
	
	int  recv_msg_len  = -1;
	char recv_buff[SICK_MAX_RCV_BUFF] = {0};

	int  send_msg_len  = 0;
	char send_buff[32] = {0};
	
	char * pcur_rcv_msg = NULL;
	char   parse_buff[16] = {0};

	char * psend_buf = send_buff;
	send_buff[0] = SICK_LMS_STX;
	//sprintf(++psend_buf, "%s", "sMI 5 3 F4724744"); //sM I (아이임, 엘 아님)
	sprintf(++psend_buf, "%s", "sMI 0 3 F4724744"); //sM I (아이임, 엘 아님)
	send_buff[strlen(send_buff)] = SICK_LMS_ETX;
	
	send_msg_len = strlen(send_buff);
	snd_result = lms100_send_comm_msg(pSocket, send_buff, send_msg_len);
	
	unsigned int login_result = 0;
	if(snd_result == true)
	{
		rcv_result = lms100_rcv_comm_msg(pSocket, recv_buff, SICK_MAX_RCV_BUFF, &recv_msg_len);
		
		//TRACE"-- lms100_login_as_super_user response (%d) -- \n %s \n", recv_msg_len, recv_buff);
		//pcur_rcv_msg = strstr(recv_buff, "sAI 5"); //에스 에이 아이 (엘이나, 일 아님)
		pcur_rcv_msg = strstr(recv_buff, "sAI 0"); //에스 에이 아이 (엘이나, 일 아님)
		if(pcur_rcv_msg)
		{
			//pcur_rcv_msg += strlen("sAI 5")+1; //set msg ptr to data start idx
			pcur_rcv_msg += strlen("sAI 0")+1; //set msg ptr to data start idx
			if(pcur_rcv_msg == NULL)
			{
				return false;
			}
			pcur_rcv_msg  = lms100_util_parse_hex_unsigned_int(pcur_rcv_msg, 1, &login_result);
			if(login_result == 1)
			{
				return true;
			}
			else
			{
				return false;
			}
		}
		
	}
	return false;
}

bool lms100_save_data_permanently(SOCKET * pSocket)
{
	bool snd_result, rcv_result;
	
	int  recv_msg_len  = -1;
	char recv_buff[SICK_MAX_RCV_BUFF] = {0};

	int  send_msg_len  = 0;
	char send_buff[32] = {0};
	
	char * pcur_rcv_msg = NULL;
	char   parse_buff[16] = {0};

	char * psend_buf = send_buff;
	send_buff[0] = SICK_LMS_STX;
	sprintf(++psend_buf, "%s", "sMN mEEwriteall");
	send_buff[strlen(send_buff)] = SICK_LMS_ETX;
	
	send_msg_len = strlen(send_buff);
	snd_result = lms100_send_comm_msg(pSocket, send_buff, send_msg_len);
	
	unsigned int login_result = 0;
	if(snd_result == true)
	{
		rcv_result = lms100_rcv_comm_msg(pSocket, recv_buff, SICK_MAX_RCV_BUFF, &recv_msg_len);
		
		//TRACE"-- lms100_login_as_super_user response (%d) -- \n %s \n", recv_msg_len, recv_buff);
		pcur_rcv_msg = strstr(recv_buff, "sMN mEEwriteall");
		if(pcur_rcv_msg)
		{
			pcur_rcv_msg += strlen("sMN mEEwriteall")+1; //set msg ptr to data start idx
			if(pcur_rcv_msg == NULL)
			{
				return false;
			}
			pcur_rcv_msg  = lms100_util_parse_hex_unsigned_int(pcur_rcv_msg, 1, &login_result);
			if(login_result == 1)
			{
				return true;
			}
			else
			{
				return false;
			}
		}
		
	}
	return false;
}

int lms100_set_scan_cfg_freq_resol_area(SOCKET * pSocket, lms100_cfg * pcfg)
{
	bool snd_result, rcv_result;
	
	int  recv_msg_len  = -1;
	char recv_buff[SICK_MAX_RCV_BUFF] = {0};

	int  send_msg_len  = 0;
	char send_buff[256] = {0};
	
	char * pcur_rcv_msg = NULL;
	char   parse_buff[16] = {0};
	
	char   * ptmp_msg_ptr  = NULL;
	char   tmp_msg_buf[16] = {0};

	char * psend_buf = send_buff;
	send_buff[0] = SICK_LMS_STX;
	sprintf(++psend_buf, "%s", "sMI 3F ");
	
	//scanning frequency
	sprintf(tmp_msg_buf, "%x ", pcfg->nscan_freq*100);
	ptmp_msg_ptr = _strupr(tmp_msg_buf);
	strcat(send_buff, ptmp_msg_ptr);
	
	//number of segments
	sprintf(tmp_msg_buf, "%x ", pcfg->nseg_cnt);
	ptmp_msg_ptr = _strupr(tmp_msg_buf);
	strcat(send_buff, ptmp_msg_ptr);
	
	//angular resolution
	sprintf(tmp_msg_buf, "%x ", (int) (pcfg->fangle_resol*10000.0f));
	ptmp_msg_ptr = _strupr(tmp_msg_buf);
	strcat(send_buff, ptmp_msg_ptr);

	//start angle
	sprintf(tmp_msg_buf, "%x ", (int) (pcfg->fstart_angle*10000.0f));
	ptmp_msg_ptr = _strupr(tmp_msg_buf);
	strcat(send_buff, ptmp_msg_ptr);

	//end angle
	sprintf(tmp_msg_buf, "%x", (int) (pcfg->fend_angle*10000.0f));
	ptmp_msg_ptr = _strupr(tmp_msg_buf);
	strcat(send_buff, ptmp_msg_ptr);

	send_buff[strlen(send_buff)] = SICK_LMS_ETX;
	
	send_msg_len = strlen(send_buff);


	lms100_login_as_super_user(pSocket);
	
	snd_result = lms100_send_comm_msg(pSocket, send_buff, send_msg_len);

	unsigned int tmp_unsigned_int = 0;
	int          tmp_signed_int   = 0;
	
	int set_result = -1;

	if(snd_result == true)
	{
		rcv_result = lms100_rcv_comm_msg(pSocket, recv_buff, SICK_MAX_RCV_BUFF, &recv_msg_len);
		//TRACE"-- lms100_read_scan_cfg response (%d) -- \n %s \n", recv_msg_len, recv_buff);
		
		pcur_rcv_msg = strstr(recv_buff, "sAI 3F");
		if(pcur_rcv_msg)
		{
			pcur_rcv_msg += strlen("sAI 3F")+1; //set msg ptr to data start idx

			pcur_rcv_msg    = lms100_util_parse_hex_unsigned_int(pcur_rcv_msg, 1, &tmp_unsigned_int);
			set_result	= tmp_unsigned_int;
			if(set_result == 0)
			{
				//parse scanning frequency
				pcur_rcv_msg       = lms100_util_parse_hex_unsigned_int(pcur_rcv_msg, 4, &tmp_unsigned_int);
				pcfg->nscan_freq   = (unsigned int) ((float) tmp_unsigned_int) / 100.0f;		

				//parse number of segments
				pcur_rcv_msg   = lms100_util_parse_hex_unsigned_int(pcur_rcv_msg, 2, &tmp_unsigned_int);
				pcfg->nseg_cnt = tmp_unsigned_int;

				//parse angular resolution
				pcur_rcv_msg = lms100_util_parse_hex_unsigned_int(pcur_rcv_msg, 4, &tmp_unsigned_int);
				pcfg->fangle_resol = ((float) tmp_unsigned_int) / 10000.0f;

				//parse start angle in deg
				pcur_rcv_msg = lms100_util_parse_hex_signed_int(pcur_rcv_msg, 4, &tmp_signed_int);
				pcfg->fstart_angle = ((float) tmp_signed_int) / 10000.0f;

				//parse end angle in deg
				pcur_rcv_msg = lms100_util_parse_hex_signed_int(pcur_rcv_msg, 4, &tmp_signed_int);
				pcfg->fend_angle = ((float) tmp_signed_int) / 10000.0f;
				
				pcfg->range_data_cnt = (int) ((pcfg->fend_angle - pcfg->fstart_angle) / pcfg->fangle_resol + 1.0f);
			}
		}
	}
	lms100_login_release(pSocket);

	return set_result;
}

bool lms100_set_scan_cfg_output_data(SOCKET * pSocket, lms100_cfg * pcfg)
{
	bool snd_result, rcv_result;
	
	int  recv_msg_len  = -1;
	char recv_buff[SICK_MAX_RCV_BUFF] = {0};

	int  send_msg_len  = 0;
	char send_buff[256] = {0};
	
	char * pcur_rcv_msg = NULL;
	
	char   * ptmp_msg_ptr  = NULL;
	char   tmp_msg_buf[16] = {0};

	char * psend_buf = send_buff;
	send_buff[0] = SICK_LMS_STX;
	sprintf(++psend_buf, "%s", "sWI DB ");
	
	//output channel setting
	if(pcfg->data_output.output_channel < 10) //2byte 데이터
	{
		sprintf(tmp_msg_buf, "0%x 00 ", pcfg->data_output.output_channel);
	}
	else
	{
		sprintf(tmp_msg_buf, "%x 00 ", pcfg->data_output.output_channel);
	}

	ptmp_msg_ptr = _strupr(tmp_msg_buf);
	strcat(send_buff, ptmp_msg_ptr);
		
	//remission data receive setting
	sprintf(tmp_msg_buf, "%x ", pcfg->data_output.remission);
	ptmp_msg_ptr = _strupr(tmp_msg_buf);
	strcat(send_buff, ptmp_msg_ptr);
		
	//remission resolution receive setting
	sprintf(tmp_msg_buf, "%x ", pcfg->data_output.resolution);
	ptmp_msg_ptr = _strupr(tmp_msg_buf);
	strcat(send_buff, ptmp_msg_ptr);
		
	//remission unit setting
	sprintf(tmp_msg_buf, "%x ", pcfg->data_output.unit);
	ptmp_msg_ptr = _strupr(tmp_msg_buf);
	strcat(send_buff, ptmp_msg_ptr);
		
	//encoder setting
	if(pcfg->data_output.encoder < 10) //2byte data
	{
		sprintf(tmp_msg_buf, "0%x 00 ", pcfg->data_output.encoder);
	}
	else
	{
		sprintf(tmp_msg_buf, "%x 00 ", pcfg->data_output.encoder);
	}
	
	ptmp_msg_ptr = _strupr(tmp_msg_buf);
	strcat(send_buff, ptmp_msg_ptr);
		
	//position setting
	sprintf(tmp_msg_buf, "%x ", pcfg->data_output.position);
	ptmp_msg_ptr = _strupr(tmp_msg_buf);
	strcat(send_buff, ptmp_msg_ptr);
		
	//device name setting
	sprintf(tmp_msg_buf, "%x ", pcfg->data_output.device_name);
	ptmp_msg_ptr = _strupr(tmp_msg_buf);
	strcat(send_buff, ptmp_msg_ptr);
		
	//comment setting
	sprintf(tmp_msg_buf, "%x ", pcfg->data_output.comment);
	ptmp_msg_ptr = _strupr(tmp_msg_buf);
	strcat(send_buff, ptmp_msg_ptr);
		
	//time
	sprintf(tmp_msg_buf, "%x ", pcfg->data_output.time);
	ptmp_msg_ptr = _strupr(tmp_msg_buf);
	strcat(send_buff, ptmp_msg_ptr);
		
	//output interval
	sprintf(tmp_msg_buf, "%x", pcfg->data_output.output_interval);
	ptmp_msg_ptr = _strupr(tmp_msg_buf);
	strcat(send_buff, ptmp_msg_ptr);
	
	send_buff[strlen(send_buff)] = SICK_LMS_ETX;	
	send_msg_len = strlen(send_buff);


	lms100_login_as_super_user(pSocket);
	
	snd_result = lms100_send_comm_msg(pSocket, send_buff, send_msg_len);

	unsigned int tmp_unsigned_int = 0;
	int          tmp_signed_int   = 0;
	
	bool set_result = false;

	if(snd_result == true)
	{
		rcv_result = lms100_rcv_comm_msg(pSocket, recv_buff, SICK_MAX_RCV_BUFF, &recv_msg_len);
		//TRACE"-- lms100_set_scan_cfg_output_data response (%d) -- \n %s \n", recv_msg_len, recv_buff);
		
		pcur_rcv_msg = strstr(recv_buff, "sWA DB");
		if(pcur_rcv_msg)
		{			
			set_result = true;
		}
		
		
	}
	lms100_login_release(pSocket);

	return set_result;
}

bool lms100_parse_scan_data(lms100_cfg cfg, lms100_scan_data *pscan_data, char * pmsg_buff, int msg_len)
{
	char * pcur_msg_idx = pmsg_buff;

	unsigned int   tmp_unsigned_int = 0;
	         int   tmp_signed_int   = 0;
		 float tmp_float_value  = 0;
	
	pcur_msg_idx = strstr(pcur_msg_idx, "DIST1");
	if(pcur_msg_idx == NULL)
	{
		return false;
	}
	
	//find channel count
	pcur_msg_idx--;
	while(1)
	{
		if(pcur_msg_idx == pmsg_buff)
		{
			//something wrong, find backward order reached at the starting pointer
			return false;
		}

		pcur_msg_idx--;		
		if(pcur_msg_idx[0] == ' ')
		{
			break;
		}
	}
	pscan_data->is_updating = true;

	pcur_msg_idx = lms100_util_parse_hex_unsigned_int(pcur_msg_idx, 2, &tmp_unsigned_int);
	pscan_data->number_of_channels = tmp_unsigned_int;
	
	///////////////////////////////////////////////////////////////////////
	//parse dist1 data
	///////////////////////////////////////////////////////////////////////
	pcur_msg_idx = pmsg_buff;
	pcur_msg_idx = strstr(pcur_msg_idx, "DIST1");
	if(pcur_msg_idx)
	{
		pcur_msg_idx += strlen("DIST1") + 1;

		//parse scaling factor
		pcur_msg_idx = lms100_util_parse_hex_real(pcur_msg_idx, 4, &tmp_float_value);
		pscan_data->dist1_scaling_factor = tmp_float_value;
		
		//parse scling offset
		pcur_msg_idx = lms100_util_parse_hex_real(pcur_msg_idx, 4, &tmp_float_value);
		pscan_data->dist1_scaling_offset = tmp_float_value;
		
		//parse starting angle
		pcur_msg_idx = lms100_util_parse_hex_signed_int(pcur_msg_idx, 4, &tmp_signed_int);
		pscan_data->dist1_starting_angle = ((float) tmp_signed_int) / 10000.0f;
		
		//parse angular step width
		pcur_msg_idx = lms100_util_parse_hex_unsigned_int(pcur_msg_idx, 2, &tmp_unsigned_int);
		pscan_data->dist1_angular_step_width = ((float) tmp_unsigned_int) / 10000.0f;
		
		//parse number of data
		pcur_msg_idx = lms100_util_parse_hex_unsigned_int(pcur_msg_idx, 2, &tmp_unsigned_int);
		pscan_data->dist1_number_of_data = (float) tmp_unsigned_int;
		
		if(pscan_data->dist1_scaling_factor > 1.0f)
		{
			for(int i = 0; i < pscan_data->dist1_number_of_data; i++)
			{
				pcur_msg_idx = lms100_util_parse_hex_unsigned_int(pcur_msg_idx, 4, &tmp_unsigned_int);
				pscan_data->pdist1_data[i] = (int) ( ((float) tmp_unsigned_int) * pscan_data->dist1_scaling_factor );
			}
		}
		else
		{
			for(int i = 0; i < pscan_data->dist1_number_of_data; i++)
			{
				pcur_msg_idx = lms100_util_parse_hex_unsigned_int(pcur_msg_idx, 4, &tmp_unsigned_int);
				pscan_data->pdist1_data[i] = tmp_unsigned_int;
			}
		}
	}

	///////////////////////////////////////////////////////////////////////
	//parse dist2 data
	///////////////////////////////////////////////////////////////////////
	pcur_msg_idx = pmsg_buff;
	pcur_msg_idx = strstr(pcur_msg_idx, "DIST2");
	if(pcur_msg_idx)
	{
		pcur_msg_idx += strlen("DIST2") + 1;

		//parse scaling factor
		pcur_msg_idx = lms100_util_parse_hex_real(pcur_msg_idx, 4, &tmp_float_value);
		pscan_data->dist2_scaling_factor = tmp_float_value;
		
		//parse scling offset
		pcur_msg_idx = lms100_util_parse_hex_real(pcur_msg_idx, 4, &tmp_float_value);
		pscan_data->dist2_scaling_offset = tmp_float_value;
		
		//parse starting angle
		pcur_msg_idx = lms100_util_parse_hex_signed_int(pcur_msg_idx, 4, &tmp_signed_int);
		pscan_data->dist2_starting_angle = ((float) tmp_signed_int) / 10000.0f;
		
		//parse angular step width
		pcur_msg_idx = lms100_util_parse_hex_unsigned_int(pcur_msg_idx, 2, &tmp_unsigned_int);
		pscan_data->dist2_angular_step_width = ((float) tmp_unsigned_int) / 10000.0f;
		
		//parse angular step width
		pcur_msg_idx = lms100_util_parse_hex_unsigned_int(pcur_msg_idx, 2, &tmp_unsigned_int);
		pscan_data->dist2_number_of_data = (float) tmp_unsigned_int;
		
		if(pscan_data->dist2_scaling_factor > 1.0f)
		{
			for(int i = 0; i < pscan_data->dist2_number_of_data; i++)
			{
				pcur_msg_idx = lms100_util_parse_hex_unsigned_int(pcur_msg_idx, 4, &tmp_unsigned_int);
				pscan_data->pdist2_data[i] = (int) ( ((float) tmp_unsigned_int) * pscan_data->dist1_scaling_factor );
			}
		}
		else
		{
			for(int i = 0; i < pscan_data->dist1_number_of_data; i++)
			{
				pcur_msg_idx = lms100_util_parse_hex_unsigned_int(pcur_msg_idx, 4, &tmp_unsigned_int);
				pscan_data->pdist2_data[i] = tmp_unsigned_int;
			}
		}
	}

	///////////////////////////////////////////////////////////////////////
	//parse rssi1 data
	///////////////////////////////////////////////////////////////////////
	int rssi_buffer_size = (cfg.data_output.resolution ==  0) ? 2 : 4;

	pcur_msg_idx = pmsg_buff;
	pcur_msg_idx = strstr(pcur_msg_idx, "RSSI1");
	if(pcur_msg_idx)
	{
		pcur_msg_idx += strlen("RSSI1") + 1;

		//parse scaling factor
		pcur_msg_idx = lms100_util_parse_hex_real(pcur_msg_idx, 4, &tmp_float_value);
		pscan_data->rssi1_scaling_factor = tmp_float_value;
		
		//parse scling offset
		pcur_msg_idx = lms100_util_parse_hex_real(pcur_msg_idx, 4, &tmp_float_value);
		pscan_data->rssi1_scaling_offset = tmp_float_value;
		
		//parse starting angle
		pcur_msg_idx = lms100_util_parse_hex_signed_int(pcur_msg_idx, 4, &tmp_signed_int);
		pscan_data->rssi1_starting_angle = ((float) tmp_signed_int) / 10000.0f;
		
		//parse angular step width
		pcur_msg_idx = lms100_util_parse_hex_unsigned_int(pcur_msg_idx, 2, &tmp_unsigned_int);
		pscan_data->rssi1_angular_step_width = ((float) tmp_unsigned_int) / 10000.0f;
		
		//parse angular step width
		pcur_msg_idx = lms100_util_parse_hex_unsigned_int(pcur_msg_idx, 2, &tmp_unsigned_int);
		pscan_data->rssi1_number_of_data = (float) tmp_unsigned_int;
		
		if(pscan_data->rssi1_scaling_factor > 1.0f)
		{
			for(int i = 0; i < pscan_data->rssi1_number_of_data; i++)
			{
				pcur_msg_idx = lms100_util_parse_hex_unsigned_int(pcur_msg_idx, rssi_buffer_size, &tmp_unsigned_int);
				pscan_data->prssi1_data[i] = (int) ( ((float) tmp_unsigned_int) * pscan_data->dist1_scaling_factor );
			}
		}
		else
		{
			for(int i = 0; i < pscan_data->rssi1_number_of_data; i++)
			{
				pcur_msg_idx = lms100_util_parse_hex_unsigned_int(pcur_msg_idx, rssi_buffer_size, &tmp_unsigned_int);
				pscan_data->prssi1_data[i] = tmp_unsigned_int;
			}
		}
	}

	///////////////////////////////////////////////////////////////////////
	//parse rssi2 data
	///////////////////////////////////////////////////////////////////////
	pcur_msg_idx = pmsg_buff;
	pcur_msg_idx = strstr(pcur_msg_idx, "RSSI2");
	if(pcur_msg_idx)
	{
		pcur_msg_idx += strlen("RSSI2") + 1;

		//parse scaling factor
		pcur_msg_idx = lms100_util_parse_hex_real(pcur_msg_idx, 4, &tmp_float_value);
		pscan_data->rssi2_scaling_factor = tmp_float_value;
		
		//parse scling offset
		pcur_msg_idx = lms100_util_parse_hex_real(pcur_msg_idx, 4, &tmp_float_value);
		pscan_data->rssi2_scaling_offset = tmp_float_value;
		
		//parse starting angle
		pcur_msg_idx = lms100_util_parse_hex_signed_int(pcur_msg_idx, 4, &tmp_signed_int);
		pscan_data->rssi2_starting_angle = ((float) tmp_signed_int) / 10000.0f;
		
		//parse angular step width
		pcur_msg_idx = lms100_util_parse_hex_unsigned_int(pcur_msg_idx, 2, &tmp_unsigned_int);
		pscan_data->rssi2_angular_step_width = ((float) tmp_unsigned_int) / 10000.0f;
		
		//parse angular step width
		pcur_msg_idx = lms100_util_parse_hex_unsigned_int(pcur_msg_idx, 2, &tmp_unsigned_int);
		pscan_data->rssi2_number_of_data = (float) tmp_unsigned_int;
		
		if(pscan_data->rssi2_scaling_factor > 1.0f)
		{
			for(int i = 0; i < pscan_data->rssi2_number_of_data; i++)
			{
				pcur_msg_idx = lms100_util_parse_hex_unsigned_int(pcur_msg_idx, rssi_buffer_size, &tmp_unsigned_int);
				pscan_data->prssi2_data[i] = (int) ( ((float) tmp_unsigned_int) * pscan_data->dist1_scaling_factor );
			}
		}
		else
		{
			for(int i = 0; i < pscan_data->rssi1_number_of_data; i++)
			{
				pcur_msg_idx = lms100_util_parse_hex_unsigned_int(pcur_msg_idx, rssi_buffer_size, &tmp_unsigned_int);
				pscan_data->prssi2_data[i] = tmp_unsigned_int;
			}
		}
	}
	pscan_data->is_updating = false;
	return true;
}

bool lms100_read_single_scan (SOCKET * pSocket, lms100_cfg cfg, lms100_scan_data *pscan_data)
{
	bool snd_result, rcv_result;
	
	int  recv_msg_len  = -1;
	char recv_buff[SICK_MAX_RCV_BUFF] = {0};

	int  send_msg_len  = 0;
	char send_buff[32] = {0};
	
	char * psend_buf = send_buff;
	send_buff[0] = SICK_LMS_STX;
	sprintf(++psend_buf, "%s", "sRN LMDscandata");
	send_buff[strlen(send_buff)] = SICK_LMS_ETX;
	
	send_msg_len = strlen(send_buff);
	snd_result = lms100_send_comm_msg(pSocket, send_buff, send_msg_len);
	if(snd_result == true)
	{
		rcv_result = lms100_rcv_comm_msg(pSocket, recv_buff, SICK_MAX_RCV_BUFF, &recv_msg_len);
		//TRACE("-- lms100_read_single_scan response (%d) -- \n %s \n", recv_msg_len, recv_buff);
		
		/*
		FILE * pLogFile = NULL;
		pLogFile = fopen("d:/lms100_log.txt", "w+");
		if(pLogFile == NULL)
		{
			return false;
		}
		fwrite(recv_buff, sizeof(char), recv_msg_len, pLogFile);

		fclose(pLogFile);
		
		double cur_time, prev_time;
		prev_time = time_milli_sec();
		*/

		lms100_parse_scan_data(cfg, pscan_data, recv_buff, recv_msg_len);
		
		/*
		cur_time = time_milli_sec();
		
		TRACE("\n\n --parse time %d (ms)\n", (int) (cur_time - prev_time));
		
		pLogFile = NULL;
		pLogFile = fopen("d:/parsed_lms100_log.txt", "w+");
		if(pLogFile == NULL)
		{
			return false;
		}
		fprintf(pLogFile, "--- DIST1 Information ---\n");
		fprintf(pLogFile, "HEADER:: SF\t SO\t SA\t ASW\t ND\t\n");
		fprintf(pLogFile, "HEADER:: %2.2f\t %2.2f\t %2.2f\t %2.2f\t %d\t\n", pscan_data->dist1_scaling_factor, pscan_data->dist1_scaling_offset, pscan_data->dist1_starting_angle, pscan_data->dist1_angular_step_width, pscan_data->dist1_number_of_data);
		fprintf(pLogFile, "DATA\n");
		for(int i = 0; i < pscan_data->dist2_number_of_data; i++)
		{
			fprintf(pLogFile, "%d\t", pscan_data->pdist1_data[i]);
		}
		fprintf(pLogFile, "\n\n");
		
		fprintf(pLogFile, "--- RSSI1 Information ---\n");
		fprintf(pLogFile, "HEADER:: SF\t SO\t SA\t ASW\t ND\t\n");
		fprintf(pLogFile, "HEADER:: %2.2f\t %2.2f\t %2.2f\t %2.2f\t %d\t\n", pscan_data->rssi1_scaling_factor, pscan_data->rssi1_scaling_offset, pscan_data->rssi1_starting_angle, pscan_data->rssi1_angular_step_width, pscan_data->rssi1_number_of_data);
		fprintf(pLogFile, "DATA\n");
		for(int i = 0; i < pscan_data->dist2_number_of_data; i++)
		{
			fprintf(pLogFile, "%d\t", pscan_data->prssi1_data[i]);
		}
		fprintf(pLogFile, "\n\n");

		fprintf(pLogFile, "--- DIST2 Information ---\n");
		fprintf(pLogFile, "HEADER:: SF\t SO\t SA\t ASW\t ND\t\n");
		fprintf(pLogFile, "HEADER:: %2.2f\t %2.2f\t %2.2f\t %2.2f\t %d\t\n", pscan_data->dist2_scaling_factor, pscan_data->dist2_scaling_offset, pscan_data->dist2_starting_angle, pscan_data->dist2_angular_step_width, pscan_data->dist2_number_of_data);
		fprintf(pLogFile, "DATA\n");
		for(int i = 0; i < pscan_data->dist2_number_of_data; i++)
		{
			fprintf(pLogFile, "%d\t", pscan_data->pdist2_data[i]);
		}
		fprintf(pLogFile, "\n\n");
		
		fprintf(pLogFile, "--- RSSI2 Information ---\n");
		fprintf(pLogFile, "HEADER:: SF\t SO\t SA\t ASW\t ND\t\n");
		fprintf(pLogFile, "HEADER:: %2.2f\t %2.2f\t %2.2f\t %2.2f\t %d\t\n", pscan_data->rssi2_scaling_factor, pscan_data->rssi2_scaling_offset, pscan_data->rssi2_starting_angle, pscan_data->rssi2_angular_step_width, pscan_data->rssi2_number_of_data);
		fprintf(pLogFile, "DATA\n");
		for(int i = 0; i < pscan_data->dist2_number_of_data; i++)
		{
			fprintf(pLogFile, "%d\t", pscan_data->prssi2_data[i]);
		}
		fprintf(pLogFile, "\n\n");		
		fclose(pLogFile);
		*/


		return rcv_result;
	}
	
	return false;
}

bool lms100_read_continuous_scan(SOCKET * pSocket, lms100_cfg cfg, lms100_scan_data *pscan_data)
{
	bool snd_result, rcv_result;
	
	int  recv_msg_len  = -1;
	char recv_buff[SICK_MAX_RCV_BUFF] = {0};

	int  send_msg_len  = 0;
	char send_buff[32] = {0};
	
	char * pconfirm_msg = NULL;
	char   lms_confirm_msg[32] = {0};
	
	pconfirm_msg		= lms_confirm_msg;
	lms_confirm_msg[0]	= SICK_LMS_STX;
	sprintf(++pconfirm_msg, "%s", "sEA LMDscandata 1");
	lms_confirm_msg[strlen(lms_confirm_msg)] = SICK_LMS_ETX;
	
	char * psend_buf = NULL;
	
	psend_buf = send_buff;
	send_buff[0] = SICK_LMS_STX;
	sprintf(++psend_buf, "%s", "sEN LMDscandata 1");
	send_buff[strlen(send_buff)] = SICK_LMS_ETX;
	
	send_msg_len = strlen(send_buff);
	snd_result = lms100_send_comm_msg(pSocket, send_buff, send_msg_len);
	
	double prev_read_time = time_milli_sec();
	double cur_read_time  = prev_read_time;

	int test_cnt = 0;
	if(snd_result == true)
	{
		//receive lms confirm message
		rcv_result = lms100_rcv_comm_msg(pSocket, recv_buff, SICK_MAX_RCV_BUFF, &recv_msg_len);		

		if(strcmp(recv_buff, lms_confirm_msg) != 0)
		{
			TRACE("-- lms 100 didn't confirmed continuous scan\n");
			return false;
		}
		
		TRACE("\n-- lms100 confirmed scan response (%d) -- \n %s \n", recv_msg_len, recv_buff);
		::Sleep(10);
		
		
		while(1)
		{
			rcv_result = lms100_rcv_comm_msg(pSocket, recv_buff, SICK_MAX_RCV_BUFF, &recv_msg_len);		
			cur_read_time = time_milli_sec();

			TRACE("\n\n-- [%d]ms :: lms100 scan response (%d) -- \n %s \n\n", (int) (cur_read_time - prev_read_time), recv_msg_len, recv_buff);
			prev_read_time = cur_read_time;

			if(test_cnt++ > 20)
			{
				break;
			}

			::Sleep(10);
		}
	}
	
	//test stop
	psend_buf = send_buff;
	send_buff[0] = SICK_LMS_STX;
	sprintf(++psend_buf, "%s", "sEN LMDscandata 0");
	send_buff[strlen(send_buff)] = SICK_LMS_ETX;
	
	send_msg_len = strlen(send_buff);
	snd_result = lms100_send_comm_msg(pSocket, send_buff, send_msg_len);
	return true;
}
