#include "stdafx.h"
#include "KUNSSocketComm.h"
#include <stdio.h>


KUNSSocketComm::KUNSSocketComm(void)
	: m_initialized(false)
{
	WSADATA wsadata;
	WSAStartup(MAKEWORD(2, 2), &wsadata); // start winsock
}

KUNSSocketComm::~KUNSSocketComm(void)
{
	if(m_initialized)
	{
		closesocket(m_socket_server); // close the socket
		closesocket(m_socket_client);
	}

	WSACleanup(); // close winsock
}

bool KUNSSocketComm::initServer(int nPort /*= 9001*/)
{
	// socket---------------------------------------------------------------------------------------------------------------
	m_socket_server = socket(PF_INET, SOCK_STREAM, 0); // IPv4, communication based on byte unit

	if(m_socket_server == -1)
	{
		printf("[Socket comm.] Failed to create server socket.\n");
		return false;
	}

	// bind---------------------------------------------------------------------------------------------------------------
	SOCKADDR_IN server_addr;

	memset(&server_addr, 0, sizeof(server_addr));

	server_addr.sin_family = AF_INET; // IPv4 internet protocol;
	server_addr.sin_port = htons(/*8503*/nPort); // port number//htons 함수== host byte order를 network byte order로 변환해 주는 함수 
	server_addr.sin_addr.s_addr = htonl(INADDR_ANY); // 32-bit IPv4 address

	if(bind(m_socket_server, (SOCKADDR*)&server_addr, sizeof(server_addr)) == SOCKET_ERROR)
	{
		printf("[Socket comm.] Failed to run bind().");
		closesocket(m_socket_server);
		return false;
	}

	SOCKADDR_IN client_addr;
	int nClientAddrSize(sizeof(client_addr));

	printf("[Socket comm.] Waiting for the client...\n");

	// listen---------------------------------------------------------------------------------------------------------------
	if(listen(m_socket_server, 5) == SOCKET_ERROR) // allow AGV_NUM connections
	{
		printf("[Socket comm.] Failed to run listen().");
		closesocket(m_socket_server);
		return false;
	}

	// accept
	m_socket_client = accept(m_socket_server, (SOCKADDR*)&client_addr, &nClientAddrSize);

	if(m_socket_client == SOCKET_ERROR)
	{
		printf("[Socket comm.] Failed to accept the request.");

		closesocket(m_socket_server);

		return false;
	}

	printf("[Socket comm.] Connected to %s\n", inet_ntoa(client_addr.sin_addr));

	m_initialized = true;
	m_eType = SERVER;

	return true;
}

void error_handling(char* message)
{
	fputs(message, stderr);
	fputc('\n', stderr);
	exit(1);
}

int KUNSSocketComm::receiveData(void* pData, int nSize)
{
	int nRes;
	nRes = recv(m_socket_client, (char*)pData, nSize, 0);
	return nRes;
}

int KUNSSocketComm::sendData(void* pData, int nSize)
{
	int nRes(-1);
	nRes = send(m_socket_client, (char*)pData, nSize, 0);
	return nRes;
}

bool KUNSSocketComm::initClient(char* pchAddr, int nPort)//CLIENT생성::socket->connect
{
	// socket---------------------------------------------------------------------------------------------------------------
// 	m_socket_client = socket(PF_INET, SOCK_STREAM, 0); // IPv4, communication based on byte unit
	m_socket_client = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP); // test

	if(m_socket_client == -1)
	{
		printf("[Socket comm.] Failed to create client socket.\n");

		return false;
	}

	// connect---------------------------------------------------------------------------------------------------------------
	SOCKADDR_IN server_addr;

	memset(&server_addr, 0, sizeof(server_addr));

	server_addr.sin_family = AF_INET; // IPv4 internet protocol;
	server_addr.sin_port = htons(nPort); // port number
	server_addr.sin_addr.s_addr = inet_addr(pchAddr); // 32-bit IPv4 address

	// set the socket in non-blocking (recently added)
	unsigned long nMode = 1;
	int nResult = ioctlsocket(m_socket_client, FIONBIO, &nMode);

	if (nResult != NO_ERROR)
	{	
		printf("[Socket comm.] ioctlsocket failed with error: %ld\n", nResult);
	}

	// connect
	int nConnectionRes = connect(m_socket_client, (SOCKADDR*)&server_addr, sizeof(server_addr));

	if(nConnectionRes == false)// SOCKET_ERROR)
	{
		closesocket(m_socket_client);
		printf("[Socket comm.] Failed to run connect().\n");

		return false;
	}

	// restart the socket mode (recently added)
	nMode = 0;
	nResult = ioctlsocket(m_socket_client, FIONBIO, &nMode);
	if (nResult != NO_ERROR)
	{	
		printf("[Socket comm.] ioctlsocket failed with error: %ld\n", nResult);
	}

	TIMEVAL Timeout;
 	Timeout.tv_sec = 1; // 1 second delay when failed to connect
	Timeout.tv_usec = 0;

	fd_set Write, Err;
	FD_ZERO(&Write);
	FD_ZERO(&Err);
	FD_SET(m_socket_client, &Write);
	FD_SET(m_socket_client, &Err);

	// check if the socket is ready
	select(0, NULL, &Write, &Err, &Timeout);

	if(!FD_ISSET(m_socket_client, &Write)) 
	{	
		closesocket(m_socket_client);
		return false;
	}

	m_initialized = true;
	m_eType = CLIENT;

	return true;
}

void KUNSSocketComm::close(void)
{
	if(m_initialized)
	{
		closesocket(m_socket_server); // close the socket
		closesocket(m_socket_client);
	}

	m_initialized = false;
}
