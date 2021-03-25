#include "stdafx.h"
#include "MultiRobotActuatorInterface.h"

MultiRobotActuatorInterface::MultiRobotActuatorInterface()
{
	m_bISConnected = false;
	for(int i=0; i<AGV_NUM; i++)
	{	m_nTransVel[i]=0;
	m_nRotVel[i]=0;
	}
}

MultiRobotActuatorInterface::~MultiRobotActuatorInterface()
{

}

/**
@brief Korean: 로봇에 연결하는 작업을 수행.
@brief English: 
*/
bool MultiRobotActuatorInterface::connect(string strSerialPort)
{
	int nPort = atoi(&strSerialPort.c_str()[3]);
	m_KuSerialComm.SetComport(nPort, 9600, 8, '1', 0);		//port, baudrate, databit, stopbit, paritybit
	m_KuSerialComm.CreateCommInfo();
	m_bISConnected = m_KuSerialComm.OpenComport();
	m_KuSerialComm.m_bStartFlag = m_bISConnected;
	if(m_bISConnected){
		initialize();
		m_SendThread.start(doSendThread,this,50, "MultiRobotActuatorInterface::connect()_send"); //메인 스레드 시작	
		m_ReadThread.start(doReadThread,this,50, "MultiRobotActuatorInterface::connect()_read"); //메인 스레드 시작	
	}
	return m_bISConnected;
}

void MultiRobotActuatorInterface::initialize()
{
	for(int i=0; i<AGV_NUM;i++)
	{
		m_RobotPos[i].init();
	}
	m_nData=0;
}

void MultiRobotActuatorInterface::doSendThread(void* arg)
{
	MultiRobotActuatorInterface* pCI = (MultiRobotActuatorInterface*)arg;

	if(!pCI->m_bISConnected){return;}

	pCI->sendData();

}

void MultiRobotActuatorInterface::sendData()
{
	int  nID=KuRobotParameter::getInstance()->getRobotID()-1;
	m_RobotPos[nID]=KuDrawingInfo::getInstance()->getRobotPos();

	if(m_nData>=AGV_DATA_NUM)m_nData=0;

	setCommandMsg(nID,m_nData);	

	m_nData++;
}

void MultiRobotActuatorInterface::setCommandMsg(int nID,int nDataType)
{
	int nTransVel = MultiRobotActuatorInterface::getInstance()->getTransVel(nID+1);
	int nRotVel = MultiRobotActuatorInterface::getInstance()->getRotVel(nID+1);

	char charSendProtocol[FULL_SIZE];//demand data 

	charSendProtocol[0] = 0x02; // STX 

	// ID--------------------------------------
	charSendProtocol[1] = nID+31; // AGV1 ID
	// ID======================================

	//command-----------------------------------------------
	charSendProtocol[2] = 0x67; // 보내다

	if(nDataType==DATA_X){
		charSendProtocol[3] = 0x58; // X
	}
	else if(nDataType==DATA_Y){
		charSendProtocol[3] = 0x59; // Y
	}
	else if(nDataType==DATA_T){
		charSendProtocol[3] = 0x5A; // T
	}
	else if(nDataType==DATA_VT){
		charSendProtocol[3] = 0x5B; // VT
	}
	else if(nDataType==DATA_VR){
		charSendProtocol[3] = 0x5C; // VR
	}
	//command==============================================

	//DATA size-----------------------------------------------
	if(nDataType==DATA_X){
		int nTemp1 = (int)(m_RobotPos[nID].getX()/128/128);
		charSendProtocol[4] =nTemp1;
		int nTemp2 = (int)((m_RobotPos[nID].getX() - nTemp1*128*128)/128);
		charSendProtocol[5] =nTemp2;
		int nTemp3 = (int)((m_RobotPos[nID].getX() - nTemp1*128*128) - (nTemp2*128));
		charSendProtocol[6] =nTemp3;
	}
	else if(nDataType==DATA_Y){
		int nTemp1 = (int)(m_RobotPos[nID].getY()/128/128);
		charSendProtocol[4] =nTemp1;
		int nTemp2 = (int)((m_RobotPos[nID].getY() - nTemp1*128*128)/128);
		charSendProtocol[5] =nTemp2;
		int nTemp3 = (int)((m_RobotPos[nID].getY() - nTemp1*128*128) - (nTemp2*128));
		charSendProtocol[6] =nTemp3;
	}
	else if(nDataType==DATA_T){
		int nTemp1 = (int)(m_RobotPos[nID].getThetaDeg()/128/128);
		charSendProtocol[4] =nTemp1;
		int nTemp2 = (int)((m_RobotPos[nID].getThetaDeg() - nTemp1*128*128)/128);
		charSendProtocol[5] =nTemp2;
		int nTemp3 = (int)((m_RobotPos[nID].getThetaDeg() - nTemp1*128*128) - (nTemp2*128));
		charSendProtocol[6] =nTemp3;
	}
	else if(nDataType==DATA_VT){
		int nTemp1 = (int)( nTransVel/128/128);
		charSendProtocol[4] =nTemp1;
		int nTemp2 = (int)((nTransVel - nTemp1*128*128)/128);
		charSendProtocol[5] =nTemp2;
		int nTemp3 = (int)((nTransVel - nTemp1*128*128) - (nTemp2*128));
		charSendProtocol[6] =nTemp3;
	}
	else if(nDataType==DATA_VR){
		int nTemp1 = (int)( nRotVel/128);
		charSendProtocol[4] =nTemp1;
		int nTemp2 = (int)((nRotVel - nTemp1*128*128)/128);
		charSendProtocol[5] =nTemp2;
		int nTemp3 = (int)((nRotVel - nTemp1*128*128) - (nTemp2*128));
		charSendProtocol[6] =nTemp3;
	}

	//DATA size==========================================================
	int nCheckSum=0;
	for(int i=0; i<7; i++){
		nCheckSum+=(int)charSendProtocol[i];
	}// 오류방지
	charSendProtocol[7]=nCheckSum;
	charSendProtocol[8] = 0x03; // ETX


	m_KuSerialComm.sendData(charSendProtocol,FULL_SIZE);// 보냄
}

void MultiRobotActuatorInterface::doReadThread(void* arg)
{
	MultiRobotActuatorInterface* pCI = (MultiRobotActuatorInterface*)arg;

	if(!pCI->m_bISConnected){return;}

	pCI->readData();
}

bool MultiRobotActuatorInterface::readData()
{
	unsigned char ucharReadProtocol[MAXBLOCK+1];//demand data 
	int nLength = m_KuSerialComm.readData(ucharReadProtocol,MAXBLOCK);

	//항상 9개를 보내고 받는다.
	for(int nBit=0; nBit<nLength; nBit++)
	{
		char cFirst[2]={0};
		char cSecond[2]={0};
		char cThird[2]={0};
		char cFourth[2]={0};
		char cFifth[2]={0};
		char cSixth[2]={0};
		char cSeventh[2]={0};
		char cEighth[2]={0};
		char cNinth[2]={0};
		int nSelID=-1;
		int nSelDataType=-1;

		sprintf(cFirst,"%c",ucharReadProtocol[nBit]);
		sprintf(cSecond, "%c", ucharReadProtocol[nBit+1]);
		sprintf(cThird, "%c", ucharReadProtocol[nBit+2]);
		sprintf(cFourth, "%c", ucharReadProtocol[nBit+3]);
		sprintf(cFifth, "%c", ucharReadProtocol[nBit+4]);
		sprintf(cSixth, "%c", ucharReadProtocol[nBit+5]);
		sprintf(cSeventh, "%c", ucharReadProtocol[nBit+6]);
		sprintf(cEighth, "%c", ucharReadProtocol[nBit+7]);
		sprintf(cNinth, "%c", ucharReadProtocol[nBit+8]);

		//시작 검사
		if(cFirst[0]!=2){continue;}

		//ID 검사
		bool bcheckID=false;

		for(int i=0;i<AGV_NUM;i++)
		{
			if(cSecond[0]==31+i)
			{
				bcheckID=true;
				nSelID=i;
			}
		}
		if(!bcheckID) continue;

		//Command 검사
		if(cThird[0]!=COMMAND_SEND){continue;}

		//Data 검사
		bool bcheckData=false;

		for(int i=0;i<AGV_DATA_NUM;i++)
		{
			if(cFourth[0]==88+i)
			{
				bcheckData=true;
				nSelDataType=i;
			}
		}
		if(!bcheckData) continue;

		//--오류방지----------------------------------------------
		int CheckSum=0;
		char chSum[2];
		for (int i=0;i<7;i++)
		{
			CheckSum+= (int)ucharReadProtocol[nBit+i];
		}
		chSum[0]=CheckSum;
		//int OriCheckSum = atoi(cEighth);  
		if(chSum[0]!=cEighth[0]) continue;//추가
		//--오류방지==========================================

		if(nSelID!=-1)
		{
			if(nSelDataType==DATA_X){
				m_RobotPos[nSelID].setX(ucharReadProtocol[nBit+4]*128*128+ucharReadProtocol[nBit+5]*128+ucharReadProtocol[nBit+6]);
				printf("\n8 Robot %d Pose : X:%f\n",nSelID,m_RobotPos[nSelID].getX());
			}
			else if(nSelDataType==DATA_Y){
				m_RobotPos[nSelID].setY(ucharReadProtocol[nBit+4]*128*128+ucharReadProtocol[nBit+5]*128+ucharReadProtocol[nBit+6]);
				printf("\n9 Robot %d Pose :Y:%f",nSelID,m_RobotPos[nSelID].getY());
			}
			else if(nSelDataType==DATA_T){
				m_RobotPos[nSelID].setThetaDeg(ucharReadProtocol[nBit+4]*128*128+ucharReadProtocol[nBit+5]*128+ucharReadProtocol[nBit+6]);
				printf("\n10 Robot %d Pose :T:%f\n",nSelID,m_RobotPos[nSelID].getThetaDeg());
			}
			else if(nSelDataType==DATA_VT){
				setTransVel(nSelID,ucharReadProtocol[nBit+4]*128*128+ucharReadProtocol[nBit+5]*128+ucharReadProtocol[nBit+6]);
				printf("\n10 Robot %d Velocity :VT:%d\n",nSelID,m_nTransVel[nSelID]);
			}
			else if(nSelDataType==DATA_VR){
				setRotVel(nSelID,ucharReadProtocol[nBit+4]*128*128+ucharReadProtocol[nBit+5]*128+ucharReadProtocol[nBit+6]);
				printf("\n10 Robot %d Velocity :VR:%d\n",nSelID,m_nRotVel[nSelID]);
			}
		}
		int nID=KuRobotParameter::getInstance()->getRobotID()-1;
		if(nID!=nSelID)
			KuDrawingInfo::getInstance()->setRobotPos2(m_RobotPos[nSelID]);

		return true;
	}

	return false;
}

KuPose* MultiRobotActuatorInterface::getRobotPos()
{	
	return m_RobotPos;
}

void MultiRobotActuatorInterface::setRobotPos(KuPose RobotPos)
{
	int  nID=KuRobotParameter::getInstance()->getRobotID()-1;
	m_RobotPos[nID]=RobotPos;
}
////////////////////////////////1017
void MultiRobotActuatorInterface::setTransVel(int nID,int nTransVel)
{
	m_nTransVel[nID]=nTransVel;
}
void MultiRobotActuatorInterface::setRotVel(int nID,int nRotVel)
{	
	m_nRotVel[nID]=nRotVel;
}
int MultiRobotActuatorInterface::getTransVel(int nID)
{
	return m_nTransVel[nID];
}
int MultiRobotActuatorInterface::getRotVel(int nID)
{
	return m_nRotVel[nID];
}