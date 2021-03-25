#include "StdAfx.h"
#include "SerialDef.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif



unsigned int const cBaudRate[NI_BAUD_RATE]={
	CBR_110,   CBR_300,   CBR_600,    CBR_1200,   CBR_2400, 
	CBR_4800,  CBR_9600,  CBR_14400,  CBR_19200,  CBR_38400, 
	CBR_56000, CBR_57600, CBR_115200, CBR_128000, CBR_256000
}; 

const char *sBaudRate[NI_BAUD_RATE]={
	"110",     "300",     "600",      "1200",     "2400",
	"4800",    "9600",   "14400",     "19200",    "38400",
	"56000",   "57600",  "115200",    "128000",   "256000"
};

unsigned char const cByteSize[NI_BYTE_SIZE]={
	4, 5, 6, 7, 8
};

const char *sByteSize[NI_BYTE_SIZE]={
	"4", "5", "6", "7", "8"
};

unsigned char const cParityChk[NI_PARITY_CHK]={
	NOPARITY, ODDPARITY, EVENPARITY, MARKPARITY, SPACEPARITY
};

const char *sParityChk[NI_PARITY_CHK]={
	"None", "Odd", "Even", "Mark", "Space"
};

unsigned char const cStopBits[NI_STOP_BITS]={
	ONESTOPBIT, ONE5STOPBITS, TWOSTOPBITS
};

const char *sStopBits[NI_STOP_BITS]={
	"1", "1.5", "2"
};

unsigned char const cFlowControl[NI_FLOW_CONTROL]={
	FC_NONE, FC_XON_XOFF, FC_RTS_CTS
};

const char *sFlowControl[NI_FLOW_CONTROL]={
	"None", "Xon/Xoff", "RTS/CTS"
};

