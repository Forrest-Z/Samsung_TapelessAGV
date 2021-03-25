#include "stdafx.h"
#include "KuPRIMUSCommSupervisor.h"

// debug 저장을 위한 함수
int ShowMessage(char *szBuff, int n_buff)
{
	CString csbuff;
	csbuff.Format(L"%s", szBuff);	
	return 0;
}

KuPRIMUSCommSupervisor::KuPRIMUSCommSupervisor()
{
	s_PRIMUSStatus.ext_bat_voltage = 0;
}

KuPRIMUSCommSupervisor::~KuPRIMUSCommSupervisor()
{
	terminatecheckStateThread();
	cCheckPC.Terminate();
}

void KuPRIMUSCommSupervisor::init()
{
	char szCreateDir[1024];
	wchar_t swzCreateDir[1024];
	wchar_t swzFilePath[1024];
	char szFilePath[1024];
	int nPos = 0;
	sprintf_s(szCreateDir, "C:\\Debug");
	swprintf(swzCreateDir,L"%hs",szCreateDir);
	CreateDirectory(swzCreateDir, NULL);
	CTime	cTime = CTime::GetCurrentTime();
	swprintf_s(swzFilePath, _T("%s\\debug%s.log"), swzCreateDir,cTime.Format(_T("%Y%m%d%H%M%S")));
	sprintf_s(szFilePath,"%ws",swzFilePath);
	init_debug(szFilePath, true);

	debug("PRIMUS Start(VS2010)");

	cCheckPC.Initialize();
	setSerialDef();

	getCommParam();
	startcheckStateThread();
}

bool KuPRIMUSCommSupervisor::checkConnection()
{
	Sleep(100);
	if((int)s_PRIMUSStatus.i_sel==7)	m_bConnection=false;
	else m_bConnection=true;
	return m_bConnection;
}

void KuPRIMUSCommSupervisor::setSerialDef()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	CString cstrTemp;
	PRIMUSSerialDef.Port = KuRobotParameter::getInstance()->getKuPrimusCommPort();
	PRIMUSSerialDef.bEnabled = TRUE;
	PRIMUSSerialDef.ID = 1;	
	PRIMUSSerialDef.BaudRate = BAUD_RATE;
	PRIMUSSerialDef.ByteSize = BYTE_SIZE;
	PRIMUSSerialDef.ParityChk = PARITY_CHK;
	PRIMUSSerialDef.StopBits = STOP_BITS;
	PRIMUSSerialDef.FlowControl = FC_NONE;

	cPRIMUSCom.Initialize(&PRIMUSSerialDef);
}

void KuPRIMUSCommSupervisor::terminateSerialComm()
{
	cPRIMUSCom.Terminate();
}

void KuPRIMUSCommSupervisor::getCommParam()
{
	cPRIMUSCom.ManualCommand(COMMAND_PARAMETER_READ, CPR_READ);
	Sleep(300);
}

void KuPRIMUSCommSupervisor::showCommParam()
{
	printf("-------------CommParam--------------\n");
	printf(" : %d\n", (int)s_PRIMUSParameter.main_pwr_warning);
	printf(" : %d\n", (int)s_PRIMUSParameter.main_pwr_alarm);
	printf(" : %d\n", (int)s_PRIMUSParameter.sub_bat_temp_max_warning);
	printf(" : %d\n", (int)s_PRIMUSParameter.sub_bat_temp_max_alarm);
	printf(" : %d\n", (int)s_PRIMUSParameter.sub_bat_temp_min_warning);
	printf(" : %d\n", (int)s_PRIMUSParameter.sub_bat_temp_min_alarm);
	printf(" : %d\n", (int)s_PRIMUSParameter.primus_temp_max_warning);
	printf(" : %d\n", (int)s_PRIMUSParameter.primus_temp_max_alarm);
	printf(" : %d\n", (int)s_PRIMUSParameter.primus_temp_min_warning);
	printf(" : %d\n", (int)s_PRIMUSParameter.primus_temp_min_alarm);
	printf(" : %d\n", (int)s_PRIMUSParameter.sbc_cpu_warning);
	printf(" : %d\n", (int)s_PRIMUSParameter.sbc_cpu_alarm);
	printf(" : %d\n", (int)s_PRIMUSParameter.sbc_hdd_warning);
	printf(" : %d\n", (int)s_PRIMUSParameter.sbc_hdd_alarm);
	printf(" : %d\n", (int)s_PRIMUSParameter.sbc_mem_warning);
	printf(" : %d\n", (int)s_PRIMUSParameter.sbc_mem_alarm);
	printf(" : %d\n", (int)s_PRIMUSParameter.power_off_op);
	printf(" : %d\n", (int)s_PRIMUSParameter.fan_temp);
	printf(" : %d\n", (int)s_PRIMUSParameter.run_timeout);
	printf(" : %d\n", (int)s_PRIMUSParameter.ready_timeout);
	printf(" : %d\n", (int)s_PRIMUSParameter.reset_timeout);
	printf(" : %d\n", (int)s_PRIMUSParameter.boot_timeout);
	printf(" : %d\n", (int)s_PRIMUSParameter.sensor_time);
	printf(" : %d\n", (int)s_PRIMUSParameter.input_vlu_time);
	printf(" : %d\n", (int)s_PRIMUSParameter.relay_on_time);
}

void KuPRIMUSCommSupervisor::checkSBCState()
{
	s_SBCStatus.sbc_cpu_usage = cCheckPC.GetCPUUsage();
	s_SBCStatus.sbc_hdd_usage = cCheckPC.GetHDDUsage();
	s_SBCStatus.sbc_memory_usage = cCheckPC.GetMemoryUsage();
	
 	cPRIMUSCom.ManualCommand(COMMAND_MONITORING, CM_SBC_STATUS);
}

void KuPRIMUSCommSupervisor::checkPRIMUSState()
{
	cPRIMUSCom.ManualCommand(COMMAND_MONITORING, CM_PRIMUS_STATUS);
}

void KuPRIMUSCommSupervisor::showPRIMUSState()
{
	printf("-------------PRIMUS status--------------\n");
	printf("Controller temp. : %d\n", (int)s_PRIMUSStatus.temper / 10);
	printf("External temp. : %d\n", (int)s_PRIMUSStatus.ext_temp);
	printf("Internal temp. : %d\n", (int)s_PRIMUSStatus.int_temp);
	printf("Battery voltage (input) : %d\n", (int)s_PRIMUSStatus.ext_bat_voltage / 10);
	printf("DIP switch for power : %d\n", (int)s_PRIMUSStatus.dip_vlu);
	printf("Motor comm. error : %d\n", (int)s_PRIMUSStatus.motor_com_err);
	printf("SBC comm. error : %d\n", (int)s_PRIMUSStatus.sbc_com_err);
	printf("5~12V input port status : %d\n", (int)s_PRIMUSStatus.v5_i);
	printf("i_sel input : %d\n", (int)s_PRIMUSStatus.i_sel);
	printf("Opertion mode : %d\n", (int)s_PRIMUSStatus.op_mode);
	printf("PCU status : %d\n", (int)s_PRIMUSStatus.pcu_status);
	printf("Relay ON time : %d\n", (int)s_PRIMUSStatus.relay_on_time);
	printf("Voltage : %d\n", (int)s_PRIMUSStatus.voltage);
	printf("Main power input enable: %d\n", (int)s_PRIMUSStatus.sg_shdn);
	printf("Charging : %d\n", (int)s_PRIMUSStatus.ltc4012_shdn);
	printf("Relay status : %d\n", (int)s_PRIMUSStatus.relay);
	printf("IO output value : %d\n", (int)s_PRIMUSStatus.io_out_vlu);
	printf("COM2/COM4 occupation : %d\n", (int)s_PRIMUSStatus.com_sel);
	printf("Fan control : %d\n", (int)s_PRIMUSStatus.fan_control);
}

float KuPRIMUSCommSupervisor::getExternalBatteryVoltage(void)
{
	return ((float)s_PRIMUSStatus.ext_bat_voltage / 10);
}

void KuPRIMUSCommSupervisor::sendDisconnecTime()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	CString			cstrTemp;
	unsigned char ucPortNo, ucOffTime, ucOnTime;

	cstrTemp.Format(_T("%d"),KuRobotParameter::getInstance()->getCartConnecContPort());
	ucPortNo = (unsigned char)(atoi((const char*)cstrTemp.operator LPCWSTR()));

	cstrTemp.Format(_T("%f"),KuRobotParameter::getInstance()->getCartPortTimeOFF());
	ucOffTime = (unsigned char)(atoi((const char*)cstrTemp.operator LPCWSTR()));

	cstrTemp.Format(_T("%f"),KuRobotParameter::getInstance()->getCartPortTimeOn());
	ucOnTime = (unsigned char)(atoi((const char*)cstrTemp.operator LPCWSTR()));

	cPRIMUSCom.ManualCommand(COMMAND_IO_CONTROL, CIC_LOW, ucPortNo, ucOffTime, ucOnTime);	
}

void KuPRIMUSCommSupervisor::sendDoorOpen1()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	CString			cstrTemp;
	unsigned char ucPortNo, ucOffTime, ucOnTime;

	cstrTemp.Format(_T("%d"),KuRobotParameter::getInstance()->getDoorOpen1Port());
	ucPortNo = (unsigned char)(atoi((const char*)cstrTemp.operator LPCWSTR()));

	cstrTemp.Format(_T("%f"),0.0);
	ucOffTime = (unsigned char)(atoi((const char*)cstrTemp.operator LPCWSTR()));

	cstrTemp.Format(_T("%f"),2.0);
	ucOnTime = (unsigned char)(atoi((const char*)cstrTemp.operator LPCWSTR()));

	cPRIMUSCom.ManualCommand(COMMAND_IO_CONTROL, CIC_LOW, ucPortNo, ucOffTime, ucOnTime);	
}

void KuPRIMUSCommSupervisor::sendDoorOpen2()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	CString			cstrTemp;
	unsigned char ucPortNo, ucOffTime, ucOnTime;

	cstrTemp.Format(_T("%d"),KuRobotParameter::getInstance()->getDoorOpen2Port());
	ucPortNo = (unsigned char)(atoi((const char*)cstrTemp.operator LPCWSTR()));

	cstrTemp.Format(_T("%f"),0.0);
	ucOffTime = (unsigned char)(atoi((const char*)cstrTemp.operator LPCWSTR()));

	cstrTemp.Format(_T("%f"),2.0);
	ucOnTime = (unsigned char)(atoi((const char*)cstrTemp.operator LPCWSTR()));

	cPRIMUSCom.ManualCommand(COMMAND_IO_CONTROL, CIC_LOW, ucPortNo, ucOffTime, ucOnTime);	
}
void KuPRIMUSCommSupervisor::checkStateThread(void* arg)
{
	KuPRIMUSCommSupervisor* PCS = (KuPRIMUSCommSupervisor*)arg;

	PCS->checkPRIMUSState();
	Sleep(100);
	PCS->checkSBCState();
}

void KuPRIMUSCommSupervisor::startcheckStateThread()
{
	m_checkStateThread.start(checkStateThread,this,1000, "KuPRIMUSCommSupervisor::startcheckStateThread");
}

void KuPRIMUSCommSupervisor::terminatecheckStateThread()
{
	m_checkStateThread.terminate();
}