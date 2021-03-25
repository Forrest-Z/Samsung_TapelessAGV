
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
	//����üũ
	short temper;					// ��Ʈ�ѷ� ���� �µ� * 10
	short ext_temp;					// �ܺ� ��̽��� �µ�üũ
	short int_temp;					// ���� ��̽��� �µ�üũ
	short dummy1;
	
	unsigned short ext_bat_voltage;	// �Է� ���͸� ����*10
	unsigned short dip_vlu;			// ���� ���� DIP SW��
	
	unsigned char motor_com_err;	// �����ƮError�� Set
	unsigned char sbc_com_err;		// �����ƮError�� Set
	unsigned char v5_i;				// 5~12V�Է���Ʈ,������Ʈ üũ
	unsigned char i_sel;			// i_sel�� �Է�üũ0~2��  �Է�üũ2 | �Է�üũ1 | �Է�üũ0
	unsigned char op_mode;			// ���۸�� ����
	unsigned char pcu_status;			// PCU ���� ���� "0000" | PROG | ICL | CHG | ACP , �� ���°� Ȱ��ȭ�� SET
	unsigned char dummy3;
	unsigned char dummy4;
	
	//����
	unsigned int relay_on_time;		// ������ ON�� ���۽ð�
	
	//����
	unsigned short voltage;			// voltage5 | voltage12
	unsigned char  sg_shdn;			// �������� �Է� Enable --PCU_NSG���������� ������
	unsigned char  ltc4012_shdn;	// ����Enable 1: ����, 0: ��������
	unsigned char  relay;			// Relay ���۽� Set 1:Relay�����غ�
	unsigned char  io_out_vlu;		// IO��°�
	unsigned char  com_sel;			// COM2, COM4�� ���� ����, 0 :COM2, COM4 �����н�, 1: SCU����
	unsigned char  fan_control;      // 0 : fan ���� ����, 2:fan���� �ڵ�

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
	short main_pwr_warning;        // Main Power ��� ��������, ���� 0.1V
	short main_pwr_alarm;          // Main Power �˶� ��������, ���� 0.1V    
	
	short sub_bat_temp_max_warning;// Sub Battery �µ� ��� �ִ밪 , ���� 1��
	short sub_bat_temp_max_alarm;  // Sub Battery �µ� �˶� �ִ밪 , ���� 1��
	
	short sub_bat_temp_min_warning;// Sub Battery �µ� ��� �ּҰ� , ���� 1��
	short sub_bat_temp_min_alarm;  // Sub Battery �µ� �˶� �ּҰ� , ���� 1��
	
	short primus_temp_max_warning; // PRIMUS �µ� ��� �ִ밪      , ���� 0.1��
	short primus_temp_max_alarm;   // PRIMUS �µ� �˶� �ִ밪      , ���� 0.1��
	
	short primus_temp_min_warning; // PRIMUS �µ� ��� �ּҰ�      , ���� 0.1��
	short primus_temp_min_alarm;   // PRIMUS �µ� �˶� �ּҰ�      , ���� 0.1��
	
	unsigned char sbc_cpu_warning; // SBC CPU ��뷮 ���         
	unsigned char sbc_cpu_alarm;   // SBC CPU ��뷮 �˶�         
	unsigned char sbc_hdd_warning; // SBC HDD ��뷮 ���         
	unsigned char sbc_hdd_alarm;   // SBC HDD ��뷮 �˶�         
	
	unsigned char sbc_mem_warning; // SBC �޸� ��뷮 ���      
	unsigned char sbc_mem_alarm;   // SBC �޸� ��뷮 �˶�      
	unsigned char power_off_op;    // �����������ܽ� ���ۼ���     
	unsigned char fan_temp;        // �� ���� �µ�                     

	unsigned int run_timeout;      // ��� timeout ��(RUN���)    , ���� 1ms
	unsigned int ready_timeout;    // ��� timeout ��(Ready���)  , ���� 1ms
	unsigned int reset_timeout;    // ��� timeout ��(Reset���)  , ���� 1ms
	unsigned int boot_timeout;     // ��� timeout ��(boot���)   , ���� 1ms
	unsigned int sensor_time;      // �µ����� ���� üũ�ð� ���� , ���� 1ms
	unsigned int input_vlu_time;   // �Է���Ʈ üũ �ð� ����     , ���� 1ms
	unsigned int relay_on_time;    // ��������Ʈ ���� �ð�        , ���� 1ms

	unsigned char out_op[12];      // �����Ʈ ����               
	unsigned char power_op[22];    // ������� ���� ����          
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
	unsigned char sbc_cpu_usage; // SBC CPU ��뷮
	unsigned char sbc_hdd_usage; // SBC HDD ��뷮
	unsigned char sbc_memory_usage; // SBC Memory ��뷮
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