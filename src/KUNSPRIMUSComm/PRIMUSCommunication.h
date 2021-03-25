#pragma once
#include "Serialcom.h"
#include "SerialDef.h"

extern class CSerialDef				PRIMUSSerialDef;

#define COMMAND_START_MASK			0x24
#define RESPONSE_START_MASK			0x24
#define COMMAND_MONITORING			0x01
#define COMMAND_PARAMETER_READ		0x02
#define COMMAND_PARAMETER_SET		0x03
#define COMMAND_IO_CONTROL			0x04
#define COMMAND_ERROR				0x05
#define RESPONSE_MONITORING			0x81
#define RESPONSE_PARAMETER_READ		0x82
#define RESPONSE_PARAMETER_SET		0x83
#define RESPONSE_IO_CONTROL			0x84
#define RESPONSE_ERROR				0x85

#define CM_PRIMUS_STATUS			0x01
#define CM_SBC_STATUS				0x02

#define CPR_READ					0x01

#define CPS_WRITE					0x01
#define CPS_MODE					0x02

#define CIC_LOW						0x01
#define CIC_HIGH					0x02
#define CIC_POWER					0x03

#define CE_WARNING					0x01
#define CE_ALARM					0x02

#define POWER_PORT_FAN				0x01
#define POWER_PORT_12V_OUT_1		0x02
#define POWER_PORT_12V_OUT_2		0x03
#define POWER_PORT_12V_OUT_3		0x04
#define POWER_PORT_12V_OUT_4		0x05
#define POWER_PORT_5V_OUT_1			0x06
#define POWER_PORT_5V_OUT_2			0x07
#define POWER_PORT_USB_1_2			0x08
#define POWER_PORT_USB_3_4			0x09
#define POWER_PORT_USB_5_6			0x0a

#define START_MASK_SIZE					1
#define START_COMMAND_PARAMETER_SIZE	2
#define LENGTH_SIZE						2
#define CHECK_SUM_SIZE					2
#define END_MASK_SIZE					1

enum{
	OP_MODE_READY=0,	// 대기중
	OP_MODE_RUN,		// 주행중
	OP_MODE_BOOT,       // Power on후
	OP_MODE_RESET,      // Reset명령
	OP_MODE_DEBUG,		// 시스템 설정등 AGV동작을 수행하지 않는 상태
};

class CPRIMUSCommunication :
	public CSerialCom
{
public:
	CPRIMUSCommunication(void);
	~CPRIMUSCommunication(void);
	void Terminate(void);
	BOOL Initialize(CSerialDef *pSerialDef);
	void ReceivedBLOCK(void);
	BOOL ManualCommand(unsigned char ucCommand, unsigned char ucParameter, unsigned char ucVariable=0, unsigned char ucVariable2=0, unsigned char ucVariable3=0);

private:
	CRITICAL_SECTION hCrtclSect;

	HANDLE		m_ahEvnt[3];			// Timer Event Handle
	// Thread Handle
	HANDLE		m_hCommThread;			// 통신 Recv Thread 핸들.
	// Receive Thread ID
	DWORD		m_dwComThreadID;
	BOOL StartSendCommand( BOOL bRetry );
	void SetCompletionWaitTime(void);
	void SetAckWaitTime(void);
	unsigned short m_usLastCommand;
	USHORT		m_SendingState;
	void ParameterDefault(void);
};
// Receive Thread로 사용할 함수 
DWORD	ComRecvThread(CPRIMUSCommunication* pPRIMUSCom);
