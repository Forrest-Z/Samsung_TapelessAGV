
#ifndef _PRIMUSVARIABLE_H__INCLUDED_
#define _PRIMUSVARIABLE_H__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif 

#pragma pack(1)
typedef struct {
	unsigned char StartMask;
	unsigned short	length;
	unsigned char  Command;
	unsigned char  Parameter;
	//상태체크
	short temper;					// 컨트롤러 내부 온도 * 10
	short ext_temp;					// 외부 써미스터 온도체크
	short int_temp;					// 내부 써미스터 온도체크
	short dummy1;
	
	unsigned short ext_bat_voltage;	// 입력 베터리 전압*10
	unsigned short dip_vlu;			// 전원 설정 DIP SW값
	
	unsigned char motor_com_err;	// 통신포트Error시 Set
	unsigned char sbc_com_err;		// 통신포트Error시 Set
	unsigned char v5_i;				// 5~12V입력포트,접점포트 체크
	unsigned char i_sel;			// i_sel의 입력체크0~2값  입력체크2 | 입력체크1 | 입력체크0
	unsigned char op_mode;			// 동작모드 설정
	unsigned char pcu_status;			// PCU 충전 상태 "0000" | PROG | ICL | CHG | ACP , 각 상태가 활성화시 SET
	unsigned char dummy3;
	unsigned char dummy4;
	
	//설정
	unsigned int relay_on_time;		// 릴레이 ON시 동작시간
	
	//제어
	unsigned short voltage;			// voltage5 | voltage12
	unsigned char  sg_shdn;			// 메인전원 입력 Enable --PCU_NSG버전에서는 사용안함
	unsigned char  ltc4012_shdn;	// 충전Enable 1: 충전, 0: 충전안함
	unsigned char  relay;			// Relay 동작시 Set 1:Relay동작준비
	unsigned char  io_out_vlu;		// IO출력값
	unsigned char  com_sel;			// COM2, COM4간 점유 선택, 0 :COM2, COM4 바이패스, 1: SCU점유
	unsigned char  fan_control;      // 0 : fan 수동 조절, 2:fan동작 자동

	unsigned char CheckSum0;
	unsigned char CheckSum1;
	unsigned char EndMask;
} S_PRIMUSStatus;
#pragma pack()

#pragma pack(1)
typedef struct{
	unsigned char   StartMask;
	unsigned short	length;
	unsigned char  Command;
	unsigned char  Parameter;
	//Parameter
	short main_pwr_warning;        // Main Power 경고 범위설정, 단위 0.1V
	short main_pwr_alarm;          // Main Power 알람 범위설정, 단위 0.1V    
	
	short sub_bat_temp_max_warning;// Sub Battery 온도 경고 최대값 , 단위 1℃
	short sub_bat_temp_max_alarm;  // Sub Battery 온도 알람 최대값 , 단위 1℃
	
	short sub_bat_temp_min_warning;// Sub Battery 온도 경고 최소값 , 단위 1℃
	short sub_bat_temp_min_alarm;  // Sub Battery 온도 알람 최소값 , 단위 1℃
	
	short primus_temp_max_warning; // PRIMUS 온도 경고 최대값      , 단위 0.1℃
	short primus_temp_max_alarm;   // PRIMUS 온도 알람 최대값      , 단위 0.1℃
	
	short primus_temp_min_warning; // PRIMUS 온도 경고 최소값      , 단위 0.1℃
	short primus_temp_min_alarm;   // PRIMUS 온도 알람 최소값      , 단위 0.1℃
	
	unsigned char sbc_cpu_warning; // SBC CPU 사용량 경고         
	unsigned char sbc_cpu_alarm;   // SBC CPU 사용량 알람         
	unsigned char sbc_hdd_warning; // SBC HDD 사용량 경고         
	unsigned char sbc_hdd_alarm;   // SBC HDD 사용량 알람         
	
	unsigned char sbc_mem_warning; // SBC 메모리 사용량 경고      
	unsigned char sbc_mem_alarm;   // SBC 메모리 사용량 알람      
	unsigned char power_off_op;    // 메인전원차단시 동작설정     
	unsigned char fan_temp;        // 팬 동작 온도                     

	unsigned int run_timeout;      // 통신 timeout 값(RUN모드)    , 단위 1ms
	unsigned int ready_timeout;    // 통신 timeout 값(Ready모드)  , 단위 1ms
	unsigned int reset_timeout;    // 통신 timeout 값(Reset모드)  , 단위 1ms
	unsigned int boot_timeout;     // 통신 timeout 값(boot모드)   , 단위 1ms
	unsigned int sensor_time;      // 온도센서 전원 체크시간 간격 , 단위 1ms
	unsigned int input_vlu_time;   // 입력포트 체크 시간 간격     , 단위 1ms
	unsigned int relay_on_time;    // 릴레이포트 동작 시간        , 단위 1ms

	unsigned char out_op[12];      // 출력포트 설정               
	unsigned char power_op[22];    // 전원출력 동작 설정          
	unsigned char dummy1;
	unsigned char dummy2;

	unsigned char CheckSum0;
	unsigned char CheckSum1;
	unsigned char EndMask;
} S_PRIMUSParameter;
#pragma pack()

#pragma pack(1)
typedef struct{
	unsigned char   StartMask;
	unsigned short	length;
	unsigned char  Command;
	unsigned char  Parameter;
	unsigned char sbc_cpu_usage; // SBC CPU 사용량
	unsigned char sbc_hdd_usage; // SBC HDD 사용량
	unsigned char sbc_memory_usage; // SBC Memory 사용량
	unsigned char CheckSum0;
	unsigned char CheckSum1;
	unsigned char EndMask;
} S_SBCStatus;
#pragma pack()

#pragma pack(1)
typedef struct{
	unsigned char   StartMask;
	unsigned short	length;
	unsigned char  Command;
	unsigned char  Parameter;
	unsigned char MainVolt; 
	unsigned char SubBatteryVolt; 
	unsigned char PRIMUSTemp;
	unsigned char sbccpuusage; 
	unsigned char sbchddusage; 
	unsigned char sbcmemoryusage;
	unsigned char CheckSum0;
	unsigned char CheckSum1;
	unsigned char EndMask;
} S_ERRORStatus;
#pragma pack()

#endif