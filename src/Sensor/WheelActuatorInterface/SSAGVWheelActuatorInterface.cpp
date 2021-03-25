#include "stdafx.h"
#include "SSAGVWheelActuatorInterface.h"

/*************************************************************
* FUNCTION NAME : on()
* DISCRIPTION : servo를 on 시킨다.
* ARGUMENT :
* RETURN : void
* AUTHOR Joongtea,park
* REVISION 2007/09
*************************************************************/
SSAGVWheelActuatorInterface::SSAGVWheelActuatorInterface()
{
	m_bISConnected = false;

	//엔코더 정보를 받아오는 프로토콜 셋팅-----------------
	memset(m_getEncoderProtocol,0,sizeof(m_getEncoderProtocol));
	m_getEncoderProtocol[0] = 0x02; // STX
	m_getEncoderProtocol[1] = 0x45; // E
	m_getEncoderProtocol[2] = 0x03; // ETX
	//==================================================

	memset(m_RecvEncoderProtocol,0,sizeof(m_RecvEncoderProtocol));

	m_dTranslationVelocity=0;
	m_dRotationVelocity=0;


	m_nDistBetweenWheel = 560; //로봇 양바퀴사이의 거리 unit--> mm
	m_nGearRatio = 25; // 로봇의 기어비. 
	m_nEncoderResolution = 12; //로봇의 엔코도 resolution.
	m_dWheelRadius = 101.5; // 로봇의 바퀴 반지름. unit mm

	//------------------------------------------------------
	m_nLeftEncoderData = m_nRightEncoderData =0;

	//이런식으로 설정해야 로봇기준으로는 전진한다.
	m_bLeftMortorRotateSign = true; //정방향
	m_bRightMortorRotateSign = false; //역방향 
	//======================================================

	//-----------------엔코더 초기화------------------------//
	m_nRefLeftPulseCnt = m_nRefRightPulseCnt =0;


	m_dReferenceWheelEncoderCount[0] = 0;
	m_dReferenceWheelEncoderCount[1] = 0;	
	//------------------엔코더 초기화 끝-------------------//

	m_dReferenceX = 0;
	m_dReferenceY = 0;
	m_dReferenceT = 0;

	m_nRefLeftVelmm = 0;
	m_nRefRightVelmm = 0; 
}

SSAGVWheelActuatorInterface::~SSAGVWheelActuatorInterface()
{

}

void SSAGVWheelActuatorInterface::setRobotPos(KuPose RobotPos)
{
	m_RobotPos = RobotPos;
	m_dReferenceX = RobotPos.getX();
	m_dReferenceY = RobotPos.getY();
	m_dReferenceT = RobotPos.getThetaRad();
}

KuPose SSAGVWheelActuatorInterface::getRobotPos()
{
	getDelEncoderData();
	m_RobotPos.setX( m_dReferenceX );
	m_RobotPos.setY( m_dReferenceY );
	m_RobotPos.setThetaRad( m_dReferenceT );

	return m_RobotPos;
}

/**
 @brief Korean: 로봇에 연결하는 작업을 수행.
 @brief English: 
*/
bool SSAGVWheelActuatorInterface::connect(string strSerialPort)
{
	m_nPort= atoi(&strSerialPort.c_str()[3]);
	m_KuSerialComm.SetComport(m_nPort, 9600, 8, '1', 0);		//port, baudrate, databit, stopbit, paritybit
	m_KuSerialComm.CreateCommInfo();
	m_bISConnected = m_KuSerialComm.OpenComport();
	m_KuSerialComm.m_bStartFlag = m_bISConnected;
		
	return m_bISConnected;
}

/**
@brief Korean: v, w값을 입력으로 wheel을 구동하는 함수
@brief English: 
*/
// void SSAGVWheelActuatorInterface::moveByTRVelocity(int nTranslationVelocity, int nRotationalVelocity)
// {
// 	printf("nTranslationVelocity=%d\n",nTranslationVelocity);
// 	printf("nRotationalVelocity=%d\n",nRotationalVelocity);
// 
// 	m_dTranslationVelocity=nTranslationVelocity;
// 	m_dRotationVelocity=nRotationalVelocity;
// 
// 	if(nTranslationVelocity==0 && nRotationalVelocity==0){
// 		stop(); return;
// 	}
// 
// 
// 	char cVelData[9]={0}; // Sending Buffer수 선언 : 총 9 byte사용
// 	cVelData[0] = 0x02; // STX 
// 	cVelData[1] = 0x56; // V
// 
// 	//cVelData[2] = 0x2B; // +
// 	cVelData[3] = 0x30; // 0
// 	cVelData[4] = 0x30; // 0
// 	//cVelData[5] = 0x2D; // -
// 	cVelData[6] = 0x30; // 0
// 	cVelData[7] = 0x30; // 0
// 	//cVelData[8] = 0x03; // ETX
// 
// 
// 	//v,w를 왼쪽바퀴 속도와 오른쪽 바퀴로 바꿔주는 과정-----------------------------------------------------------
// 
// 	//각속도를 알때 오른쪽바퀴속도를 구하는식:
// 	//1:VR=wB+VL 2: VR=2V-VL 기본적인 이동로봇 수식임 책에 나와있음. 
// 	//1번 2번 연립하면 알 수 있음.
// 	double dRotVelRad = nRotationalVelocity*D2R;
// 	int nLeftVelmm = (int)( nTranslationVelocity - ( m_nDistBetweenWheel * dRotVelRad )/2. );
// 	int nRightVelmm = (int)( nTranslationVelocity + ( m_nDistBetweenWheel * dRotVelRad )/2. );
// 	//=========================================================================================================
// 
// 
// 	//mm/sec로 나온 왼쪽/오른쪽 바퀴 속도를 m/min으로 변환하는 과정-----------------------------------------------
// 	int nLeftVelM = ((double)nLeftVelmm * MM2M) * SEC2MIN; 
// 	int nRightVelM = ((double)nRightVelmm * MM2M) * SEC2MIN; 
// 	if(nLeftVelM>=0){
// 		m_bLeftMortorRotateSign = true; //정방향		
// 	}else{
// 		m_bLeftMortorRotateSign = false; //역방향		
// 	}
// 	if(nRightVelM>=0){
// 		m_bRightMortorRotateSign = false; //역방향 
// 	}else{
// 		m_bRightMortorRotateSign = true; //정방향		
// 	}
// 	//=========================================================================================================
// 
// 	char cLeftVel[4]={0};
// 	char cRightVel[4]={0};
// 
// 	if(abs(nLeftVelM)<10){
// 		sprintf(&cLeftVel[0],"%d",0);
// 		sprintf(&cLeftVel[1],"%d",abs(nLeftVelM));
// 	}
// 	else{
// 		sprintf(cLeftVel,"%d",abs(nLeftVelM));
// 	}
// 
// 	if(abs(nRightVelM)<10){
// 		sprintf(&cRightVel[0],"%d",0);
// 		sprintf(&cRightVel[1],"%d",abs(nRightVelM));
// 	}
// 	else{
// 		sprintf(cRightVel,"%d",abs(nRightVelM));
// 	}
// 
// 
// 	//sprintf(cRightVel,"%d",nRightVelM);
// 
// 	//왼쪽 바퀴 속도--->프로토콜화-----------------------------------------------------------
// 	if(m_bLeftMortorRotateSign == true){ //양수
// 		cVelData[LEFT_VEL_SIGN] = 0x2B; // +
// 	}else{
// 		cVelData[LEFT_VEL_SIGN] = 0x2D; // -
// 	}
// 	cVelData[3] = cLeftVel[0];
// 	cVelData[4] = cLeftVel[1];
// 
// 
// 	//=======================================================================================
// 
// 	//오른쪽 바퀴 속도---> 프로토콜화----------------------------------------------------------
// 	//오른쪽 바퀴는 반대로 해줘야 로봇이 정방향 전후진을 할 수 있다.
// 	if(m_bRightMortorRotateSign == false){ //양수
// 		cVelData[RIGHT_VEL_SING] = 0x2D; // -
// 	}else{		
// 		cVelData[RIGHT_VEL_SING] = 0x2B; // +
// 	}
// 
// 	cVelData[6] = cRightVel[0];
// 	cVelData[7] = cRightVel[1];
// 
// 	//=======================================================================================
// 
// 
// 	cVelData[8] = 0x03; // ETX
// 
// 	m_KuSerialComm.sendData(cVelData,VEL_PROTOCOL_SIZE);
// }


void SSAGVWheelActuatorInterface::moveByTRVelocity(int nTranslationVelocity, int nRotationalVelocity)
{
	//Ready();

	m_dTranslationVelocity =  nTranslationVelocity;
	m_dRotationVelocity =  nRotationalVelocity;
	
	if(m_dRotationVelocity>22) m_dRotationVelocity=22;
	else if(m_dRotationVelocity<-22) m_dRotationVelocity=-22;
	
	if(m_dTranslationVelocity<0.0)m_dTranslationVelocity=0.0;

// 	printf("m_dTranslationVelocity=%d\n",nTranslationVelocity);
// 	printf("m_dRotationVelocity=%d\n",nRotationalVelocity); 
// 	if(nTranslationVelocity==0 && nRotationalVelocity==0){
// 		stop(); return;
// 	}
// 	
	char cVelData[13]={0}; // Sending Buffer수 선언 : 총 13 byte사용
	cVelData[0] = 0x02; // STX 
	cVelData[1] = 0x56; // V

	//cVelData[2] = 0x2B; // +
	cVelData[3] = 0x30; // 0
	cVelData[4] = 0x30; // 0
	cVelData[5] = 0x30; // 0
	cVelData[6] = 0x30; // 0

	//cVelData[7] = 0x2D; // -
	cVelData[8] = 0x30; // 0
	cVelData[9] = 0x30; // 0
	cVelData[10] = 0x30; // 0
	cVelData[11] = 0x30; // 0
	//cVelData[12] = 0x03; // ETX


	//v,w를 왼쪽바퀴 속도와 오른쪽 바퀴로 바꿔주는 과정-----------------------------------------------------------

	//각속도를 알때 오른쪽바퀴속도를 구하는식:
	//1:VR=wB+VL 2: VR=2V-VL 기본적인 이동로봇 수식임 책에 나와있음. 
	//1번 2번 연립하면 알 수 있음.
	double dRotVelRad = m_dRotationVelocity*D2R;
	int nLeftVelmm = (int)( m_dTranslationVelocity - ( m_nDistBetweenWheel * dRotVelRad )/2. );
	int nRightVelmm = (int)( m_dTranslationVelocity + ( m_nDistBetweenWheel * dRotVelRad )/2. );
// 	printf("nLeftVelmm=%d\n",nLeftVelmm);
// 	printf("nRightVelmm=%d\n",nRightVelmm);

	if(nLeftVelmm > 1000) nLeftVelmm = 1000;
	if(nRightVelmm > 1000) nRightVelmm = 1000;
	if(nLeftVelmm < -1000) nLeftVelmm = -1000;
	if(nRightVelmm < -1000) nRightVelmm = -1000;
	m_nRefLeftVelmm = nLeftVelmm;
	m_nRefRightVelmm = nRightVelmm; 

	//=========================================================================================================

	//속도를 이용하여 정/역 방향 플레그 설정과정------------------------------------------------------------------
	if(nLeftVelmm>=0){
		m_bLeftMortorRotateSign = true; //정방향		
	}else{
		m_bLeftMortorRotateSign = false; //역방향		
	}
	if(nRightVelmm>=0){
		m_bRightMortorRotateSign = false; //역방향 
	}else{
		m_bRightMortorRotateSign = true; //정방향		
	}
	//=========================================================================================================

	char cLeftVelmm[8]={0};
	char cRightVelmm[8]={0};

	//계산된 mm단위 왼쪽 바퀴 속도의 자리수를 감안하여 프로토콜화해 가는 과정-------------------------------------
	if(abs(nLeftVelmm)<10){
		sprintf(&cLeftVelmm[0],"%d",0);
		sprintf(&cLeftVelmm[1],"%d",0);
		sprintf(&cLeftVelmm[2],"%d",0);
		sprintf(&cLeftVelmm[3],"%d",abs(nLeftVelmm));
	}
	else if(abs(nLeftVelmm)<100){
		sprintf(&cLeftVelmm[0],"%d",0);
		sprintf(&cLeftVelmm[1],"%d",0);
		sprintf(&cLeftVelmm[2],"%d",abs(nLeftVelmm));
	}
	else if(abs(nLeftVelmm)<1000){
		sprintf(&cLeftVelmm[0],"%d",0);
		sprintf(&cLeftVelmm[1],"%d",abs(nLeftVelmm));
	}
	else{
		sprintf(cLeftVelmm,"%d",abs(nLeftVelmm));
	}
	//===========================================================================================================

	if(abs(nRightVelmm)<10){
		sprintf(&cRightVelmm[0],"%d",0);
		sprintf(&cRightVelmm[1],"%d",0);
		sprintf(&cRightVelmm[2],"%d",0);
		sprintf(&cRightVelmm[3],"%d",abs(nRightVelmm));
	}
	else if(abs(nRightVelmm)<100){
		sprintf(&cRightVelmm[0],"%d",0);
		sprintf(&cRightVelmm[1],"%d",0);
		sprintf(&cRightVelmm[2],"%d",abs(nRightVelmm));
	}
	else if(abs(nRightVelmm)<1000){
		sprintf(&cRightVelmm[0],"%d",0);
		sprintf(&cRightVelmm[1],"%d",abs(nRightVelmm));
	}
	else{
		sprintf(cRightVelmm,"%d",abs(nRightVelmm));
	}

		
	//왼쪽 바퀴 속도--->프로토콜화-----------------------------------------------------------
	if(m_bLeftMortorRotateSign == true){ //양수
		cVelData[LEFT_VEL_SIGN] = 0x2B; // +
	}else{
		cVelData[LEFT_VEL_SIGN] = 0x2D; // -
	}
	cVelData[3] = cLeftVelmm[0]; 
	cVelData[4] = cLeftVelmm[1];
	cVelData[5] = cLeftVelmm[2]; 
	cVelData[6] = cLeftVelmm[3];
	//=======================================================================================

	//오른쪽 바퀴 속도---> 프로토콜화----------------------------------------------------------
	//오른쪽 바퀴는 반대로 해줘야 로봇이 정방향 전후진을 할 수 있다.
	if(m_bRightMortorRotateSign == false){ //양수
		cVelData[RIGHT_VEL_SING] = 0x2D; // -
	}else{		
		cVelData[RIGHT_VEL_SING] = 0x2B; // +
	}

	cVelData[8] = cRightVelmm[0]; 
	cVelData[9] = cRightVelmm[1]; 
	cVelData[10] = cRightVelmm[2]; 
	cVelData[11] = cRightVelmm[3];
	//=======================================================================================

	cVelData[12] = 0x03; // ETX
	
	m_KuSerialComm.sendData(cVelData,VEL_PROTOCOL_SIZE);
	
	
}


double SSAGVWheelActuatorInterface::getTVel()
{
	return m_dTranslationVelocity;
}
double SSAGVWheelActuatorInterface::getRVel()
{
	return m_dRotationVelocity;
}
void SSAGVWheelActuatorInterface::stop()
{
	m_dRotationVelocity=0;
	m_dTranslationVelocity=0;
	char cVelData[VEL_PROTOCOL_SIZE]={0}; // Sending Buffer수 선언 : 총 13 byte사용
	cVelData[0] = 0x02; // STX 
	cVelData[1] = 0x56; // V

	if(m_bLeftMortorRotateSign==true){
		cVelData[LEFT_VEL_SIGN] = 0x2B; // +
	}else{
		cVelData[LEFT_VEL_SIGN] = 0x2D; // -
	}
	cVelData[3] = 0x30; // 0
	cVelData[4] = 0x30; // 0
	cVelData[5] = 0x30; // 0
	cVelData[6] = 0x30; // 0

	if(m_bRightMortorRotateSign==true){
		cVelData[RIGHT_VEL_SING] = 0x2B; // +
	}else{
		cVelData[RIGHT_VEL_SING] = 0x2D; // -
	}
	cVelData[8] = 0x30; // 0
	cVelData[9] = 0x30; // 0
	cVelData[10] = 0x30; // 0
	cVelData[11] = 0x30; // 0
	cVelData[12] = 0x03; // ETX
	m_KuSerialComm.sendData(cVelData,VEL_PROTOCOL_SIZE);
	Sleep(10);
	m_KuSerialComm.readData(m_RecvEncoderProtocol, RECV_ENCODER_PROTOCOL_SIZE);

}
void SSAGVWheelActuatorInterface::convertPulseCnt2EncoderCnt(int nCurLeftPulseCnt, int nCurRightPulseCnt, int* nLeftEncoderCnt, int* nRightEncoderCnt)
{
	//왼쪽 모터관련-----------------------------------------------------------------------------------
	int nLeftPulseCnt = 0;
	if(true == m_bLeftMortorRotateSign){ //즉 로봇 기준으로 전진 방향으로 회전할 경우
		if(m_nRefLeftPulseCnt > nCurLeftPulseCnt){ 
			//왼쪽 모터가 정방향으로 회전할때, 모터 펄스 수가 16진수 FF(255)
			//를 넘어간 상태이므로 이를 고려하여 증가된 펄스 카운트를 계산해야 한다. 
			//예를 들어, 기존의 펄스 카운트는 253이였고, 현재 받은 펄스 카운트는 15인 경우이다. 
			nLeftPulseCnt = (255 - m_nRefLeftPulseCnt) + nCurLeftPulseCnt;
		}
		else{
			nLeftPulseCnt = nCurLeftPulseCnt - m_nRefLeftPulseCnt;
		}
	}else{ //왼쪽 모터가 역방향 할때, 즉 로봇 기준으로 후진방향으로 회전할 경우
		if(nCurLeftPulseCnt > m_nRefLeftPulseCnt ){
			//왼쪽 모터가 역방향으로 회전할때, 모터 펄스 수가 16진수 00(00)
			//를 넘어간 상태이므로 이를 고려하여 펄스 카운트를 계산해야 한다. 
			//예를 들어, 기존의 펄스 카운트는 50이였고, 현재 받은 펄스 카운트는 250인 경우이다. 
			nLeftPulseCnt =  (255 - nCurLeftPulseCnt ) + m_nRefLeftPulseCnt;
			nLeftPulseCnt = nLeftPulseCnt*-1;
		}else{
			nLeftPulseCnt = nCurLeftPulseCnt - m_nRefLeftPulseCnt; 
			//nLeftPulseCnt = nLeftPulseCnt*-1;
		}
	}
	//============================================================================================

	//오른쪽 모터관련-----------------------------------------------------------------------------------
	int nRightPulseCnt = 0;	
	if(true == m_bRightMortorRotateSign){ //왼쪽 모터와 반대인 경우이다. 로봇 기준으로 후진 방향으로 회전할 경우
		if(m_nRefRightPulseCnt > nCurRightPulseCnt){ 
			nRightPulseCnt = (255 - m_nRefRightPulseCnt) + nCurRightPulseCnt;
			nRightPulseCnt = nRightPulseCnt*-1;
		}
		else{
			nRightPulseCnt = nCurRightPulseCnt - m_nRefRightPulseCnt;
			nRightPulseCnt = nRightPulseCnt*-1;
			
		}
		
	}else{ //오른쪽 모터가 역방향 할때, 즉 로봇 기준으로 전진방향으로 회전할 경우
		if(nCurRightPulseCnt > m_nRefRightPulseCnt ){
			nRightPulseCnt = (255 - nCurRightPulseCnt ) + m_nRefRightPulseCnt;						
		}else{
			nRightPulseCnt = m_nRefRightPulseCnt - nCurRightPulseCnt; 
			//nRightPulseCnt = nRightPulseCnt*-1;
			
		}
	}
	//============================================================================================



	*nLeftEncoderCnt +=  (nLeftPulseCnt /** m_nGearRatio*/);
	*nRightEncoderCnt += (nRightPulseCnt /** m_nGearRatio*/);

	
/*
	cout<<"nLeftPulseCnt="<<nLeftPulseCnt<<endl;
	cout<<"nRightPulseCnt="<<nRightPulseCnt<<endl;
	cout<<"---------------------------------------------"<<endl;
	cout<<"*nLeftEncoderCnt="<<*nLeftEncoderCnt<<endl;
	cout<<"*nRightEncoderCnt="<<*nRightEncoderCnt<<endl;
	cout<<"**********************************************"<<endl<<endl;
*/
	

	m_nRefLeftPulseCnt = nCurLeftPulseCnt;
	m_nRefRightPulseCnt = nCurRightPulseCnt;
}

bool SSAGVWheelActuatorInterface::Ready()
{
	m_nDistBetweenWheel = 560; //로봇 양바퀴사이의 거리 unit--> mm
	m_nGearRatio = 25; // 로봇의 기어비. 
	m_nEncoderResolution = 12; //로봇의 엔코도 resolution.
	m_dWheelRadius = 101.5; // 로봇의 바퀴 반지름. unit mm

	return m_bISConnected;
}
/**
@brief Korean: 다른 클래스에서 X,Y,Theta의 변화량의 값을 가져간다.
@brief English: 
*/
KuPose SSAGVWheelActuatorInterface::getDelEncoderData()
{	
	KuPose DelEncoderPos;


	m_KuSerialComm.sendData(m_getEncoderProtocol, GET_ENCODER_PROTOCOL_SIZE);
	Sleep(20);
	//int nLen=m_KuSerialComm.readData(m_RecvEncoderProtocol, RECV_ENCODER_PROTOCOL_SIZE);
	int nLength = m_KuSerialComm.readData(m_RecvEncoderProtocol,MAXBLOCK);

	char cSStart[10]={0};
	char cSEnd[10]={0};
	char cSLeft[10]={0};
	char cSRight[10]={0};
	int nSelBit=-1;
	
	if(nLength<RECV_ENCODER_PROTOCOL_SIZE)
	{
		if(nLength>0)	printf("Cannot read encoder data: packet length = %d\n", nLength);
		
		return DelEncoderPos;
	}

	for(int nBit=0; nBit<nLength-RECV_ENCODER_PROTOCOL_SIZE+1; nBit++)
	{
		char cLeft[10]={0};
		char cRight[10]={0};
		char cStart[10]={0};
		char cEnd[10]={0};


		sprintf(cStart,"%X",m_RecvEncoderProtocol[0+nBit]);
		sprintf(cLeft,"%X",m_RecvEncoderProtocol[1+nBit]);
		sprintf(cRight,"%X",m_RecvEncoderProtocol[2+nBit]);
		sprintf(cEnd,"%X",m_RecvEncoderProtocol[3+nBit]);

		// 	printf("%X\n",m_RecvEncoderProtocol[0]);
		// 	printf("%X\n",m_RecvEncoderProtocol[1]);
		// 	printf("%X\n",m_RecvEncoderProtocol[2]);
		// 	printf("%X\n",m_RecvEncoderProtocol[3]);
		if(cStart[0]=='2'&&cEnd[0]=='3')
		{
			nSelBit=nBit;
			break;
		}
	}

	
		m_nDistBetweenWheel = 560; //로봇 양바퀴사이의 거리 unit--> mm
		m_nGearRatio = 25; // 로봇의 기어비. 
		m_nEncoderResolution = 12; //로봇의 엔코도 resolution.
		m_dWheelRadius = 101.5; // 로봇의 바퀴 반지름. unit mm



	// 	cout<<"cLeft2 ="<<cLeft<<endl;
	// 	cout<<"cRight2 ="<<cRight<<endl;
	if(nSelBit==-1)
	{
		printf("Cannot read encoder data: packet length = %d\n",nLength);
		return DelEncoderPos;
	}

	sprintf(cSStart,"%X",m_RecvEncoderProtocol[0+nSelBit]);
	sprintf(cSLeft,"%X",m_RecvEncoderProtocol[1+nSelBit]);
	sprintf(cSRight,"%X",m_RecvEncoderProtocol[2+nSelBit]);
	sprintf(cSEnd,"%X",m_RecvEncoderProtocol[3+nSelBit]);


	int nLeftMotorPulseCnt = strtol(cSLeft, NULL, 16); 
	int nRightMotorPulseCnt = strtol(cSRight, NULL, 16); 

	convertPulseCnt2EncoderCnt(nLeftMotorPulseCnt, nRightMotorPulseCnt, &m_nLeftEncoderData, &m_nRightEncoderData);

	DelEncoderPos = calcDelEncoderPos(m_nLeftEncoderData, m_nRightEncoderData);

	return DelEncoderPos;
}


KuPose SSAGVWheelActuatorInterface::calcDelEncoderPos(int nLeftWheelEncCnt, int nRightWheelEncCnt)
{
//	double dDelXYT[3]; //[0]-->x, [1]-->y, [2]-->theta
	double dDeltaX, dDeltaY, dDeltaTRad;

	m_dWheelEncoderCount[0] = (double)nLeftWheelEncCnt;
	m_dWheelEncoderCount[1] = (double)nRightWheelEncCnt;

	m_dWheelDistance[0] = ( (double)(m_dWheelEncoderCount[0] - m_dReferenceWheelEncoderCount[0])
							/ (double)(m_nGearRatio * m_nEncoderResolution)
						  ) * 2. * M_PI * (double)m_dWheelRadius;

	m_dWheelDistance[1] = ( (double)(m_dWheelEncoderCount[1] - m_dReferenceWheelEncoderCount[1] )
							/ (double)(m_nGearRatio * m_nEncoderResolution)
						  ) * 2. * M_PI * (double)m_dWheelRadius;


	m_dReferenceWheelEncoderCount[0] = m_dWheelEncoderCount[0];
	m_dReferenceWheelEncoderCount[1] = m_dWheelEncoderCount[1];

	m_dAverageWheelDistance = (m_dWheelDistance[0] + m_dWheelDistance[1])/2;

	dDeltaTRad = (m_dWheelDistance[1] - m_dWheelDistance[0]) / m_nDistBetweenWheel;

	if(fabs(dDeltaTRad) >= 0.0017f ) {
		m_dDistance2RobotCenter = m_dAverageWheelDistance / dDeltaTRad;
		dDeltaY = m_dDistance2RobotCenter - ( m_dDistance2RobotCenter * cos(dDeltaTRad) );
		dDeltaX = m_dDistance2RobotCenter * sin(dDeltaTRad);
	}       
	else {
		m_dDistance2RobotCenter = 0.0f;
		dDeltaY = 0;
		dDeltaX = m_dAverageWheelDistance;
	}

	//상대좌표를 절대좌표로 변환하는것.
	m_dReferenceX +=  dDeltaX * cos( m_dReferenceT ) + dDeltaY * sin( -m_dReferenceT);
	m_dReferenceY +=  dDeltaX * sin( m_dReferenceT ) + dDeltaY * cos( m_dReferenceT );
	m_dReferenceT =  m_dReferenceT + dDeltaTRad;
	
	if(m_dReferenceT > M_PI){ m_dReferenceT -= 2*M_PI; }
	else if(m_dReferenceT < -M_PI){ m_dReferenceT += 2*M_PI; }

	KuPose DelEncPos;
	DelEncPos.setX(dDeltaX);
	DelEncPos.setY(dDeltaY);
	DelEncPos.setThetaRad(dDeltaTRad);

	return DelEncPos;
	
}

/**
 @brief Korean: 로봇과의 연결을 끝는다.
 @brief English: 
*/
bool SSAGVWheelActuatorInterface::disConnect()
{
	m_KuSerialComm.CloseConnection();
	return true;
}
