#ifndef _SERIAL_DEF_H_
#define _SERIAL_DEF_H_

#include "Windows.h"
//━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
// Serial 통신에 관련된 parameter 
//━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#define MAX_SERIAL_PORT			16

#define DATALEN(x)				(strlen(x)+1)

#define BAUD_RATE				CBR_9600
#define BYTE_SIZE				8
#define PARITY_CHK				NOPARITY
#define STOP_BITS				ONESTOPBIT

#define NI_BAUD_RATE			15
#define NI_PARITY_CHK			5
#define NI_STOP_BITS			3
#define NI_BYTE_SIZE			5
#define NI_FLOW_CONTROL			3

#define FC_NONE					0
#define FC_XON_XOFF				1
#define FC_RTS_CTS				2

#define ASCII_XON		    0x11
#define ASCII_XOFF		    0x13

extern unsigned int const cBaudRate[NI_BAUD_RATE];
extern const char *sBaudRate[NI_BAUD_RATE];
extern unsigned char const cByteSize[NI_BYTE_SIZE];
extern const char *sByteSize[NI_BYTE_SIZE];
extern unsigned char const cParityChk[NI_PARITY_CHK];
extern const char *sParityChk[NI_PARITY_CHK];
extern unsigned char const cStopBits[NI_STOP_BITS];
extern const char *sStopBits[NI_STOP_BITS];
extern unsigned char const cFlowControl[NI_FLOW_CONTROL];
extern const char *sFlowControl[NI_FLOW_CONTROL];

class CSerialDef{
public:
	int ID;
	int	bEnabled;
	unsigned int Port;
	unsigned int BaudRate;
	unsigned char ByteSize;
	unsigned char ParityChk;
	unsigned char StopBits;
	unsigned char FlowControl;

public:
	CSerialDef(int _ID = 0)
	{
		ID=_ID;

		bEnabled = FALSE;
		Port=0;
		BaudRate=BAUD_RATE;
		ByteSize=BYTE_SIZE;
		ParityChk=PARITY_CHK;
		StopBits=STOP_BITS;
		FlowControl=FC_NONE;
	}
	~CSerialDef(void)
	{
	}

	inline void DCB_BaudRate(unsigned int i)
	{
		BaudRate=cBaudRate[i];
	}

	inline void DCB_ParityChk(unsigned char i)
	{
		ParityChk=cParityChk[i];
	}

	inline void DCB_StopBits(unsigned char i)
	{
		StopBits=cStopBits[i];
	}

	inline void DCB_ByteSize(unsigned char i)
	{
		ByteSize=cByteSize[i];
	}

	inline void DCB_FlowControl(unsigned char i)
	{
		FlowControl=cFlowControl[i];
	}

	inline unsigned int ID_BaudRate(void)
	{
		int i;
		for(i=0; BaudRate!=cBaudRate[i] && i<NI_BAUD_RATE; i++);
		return i;
	}

	inline unsigned char ID_ParityChk(void)
	{
		int i;
		for(i=0; ParityChk!=cParityChk[i] && i<NI_PARITY_CHK; i++);
		return i;
	}

	inline unsigned char ID_StopBits(void)
	{
		int i;
		for(i=0; StopBits!=cStopBits[i] && i<NI_STOP_BITS; i++);
		return i;
	}

	inline unsigned char ID_ByteSize(void)
	{
		int i;
		for(i=0; ByteSize!=cByteSize[i] && i<NI_BYTE_SIZE; i++);
		return i;
	}

	inline unsigned char ID_FlowControl(void)
	{
		int i;
		for(i=0; FlowControl!=cFlowControl[i] && i<NI_FLOW_CONTROL; i++);
		return i;
	}
};

/* Control Characters */

#define NUL 0x00
#define SOH 0x01
#define STX 0x02
#define ETX 0x03
#define EOT 0x04
#define ENQ 0x05
#define ACK 0x06
#define BEL 0x07
#define BS  0x08
#define HT  0x09
#define LF  0x0A
#define VT  0x0B
#define FF  0x0C
#define CR  0x0D
#define SO  0x0E
#define SI  0x0F
#define DLE 0x10
#define DC1 0x11
#define XON 0x11
#define DC2 0x12
#define DC3 0x13
#define XOFF 0x13
#define DC4 0x14
#define NAK 0x15
#define SYN 0x16
#define ETB 0x17
#define CAN 0x18
#define EM  0x19
#define SUB 0x1A
#define ESC 0x1B
#define FS  0x1C
#define GS  0x1D
#define RS  0x1E
#define US  0x1F
#define SP  0x20
#define DEL 0x7F

#endif