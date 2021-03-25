#ifndef KUNS_SOCKET_COMMUNICATION_H
#define KUNS_SOCKET_COMMUNICATION_H

#define MAXBUFF 1024
#define MAXPORT 4
#define  _AFXDLL
#include <iostream>
#include <Windows.h>
#include "../../include/stdafx.h"
#include "../KUNSCriticalSection/KuCriticalSection.h"
#include <winsock2.h>
#include <string.h>
#pragma comment(lib, "ws2_32")

static const int AGV_NUMBER = 1;

#define BUFSIZE 100

struct time_val{
	long tv_sec;
	long tv_usec;
};
class __declspec(dllexport) KUNSSocketComm
{
public:
	/* Functions */
	KUNSSocketComm(void);
	~KUNSSocketComm(void);

	bool initServer(int nPort = 9001);//SERVER
	int receiveData(void* pData, int nSize);
	int sendData(void* pData, int nSize);
	void close(void);
	bool initClient(char* pchAddr, int nPort);//CLIENT

private:
	/* Functions */

	/* Variables */
	enum {SERVER, CLIENT} m_eType;
	SOCKET m_socket_server;
	SOCKET m_socket_client;
	bool m_initialized;
private:
	fd_set reads, cpy_reads;
	int fd_max;
	int str_len;
	int fd_num;
	int i;
	char buf[BUFSIZE];
	time_val timeout;

//	struct sockaddr_in serv_addr, clnt_addr;
};
#endif