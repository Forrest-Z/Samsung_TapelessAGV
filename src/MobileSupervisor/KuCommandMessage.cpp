#include "stdafx.h"
#include "KuCommandMessage.h"

KuCommandMessage::KuCommandMessage()
{
	m_nBehaviorName =-1; //실행할 behavior name가 할당되어 있지 않은 경우
	m_nCommandName = -1; //실행할 명령어가 할당되어 있지 않은 경우
	m_nBehaviorPeriod = 0;
	m_nMapSizeX = 0; 
	m_nMapSizeY = 0;
	m_bAvoidMode=false;
}

KuCommandMessage::~KuCommandMessage()
{

}
/**
@brief Korean: Behavior의 이름을 저장하는  함수.
@brief English: 
*/
void KuCommandMessage::setBehaviorName(int nName)
{
	m_nBehaviorName = nName;
}
/**
@brief Korean: Behavior에 내리는 명령을저장하는  함수.
@brief English: 
*/
void KuCommandMessage::setCommandName(int nCommandName)
{
	m_nCommandName = nCommandName;
}

/**
@brief Korean: Behavior을 실행시킬 주기를  저장하는  함수.
@brief English: 
*/
void KuCommandMessage::setBehaviorPeriod(int nPeriod)
{
	m_nBehaviorPeriod = nPeriod;
}
/**
@brief Korean:로봇의 위치를 저장하는  함수.
@brief English: 
*/
void KuCommandMessage::setRobotPos(KuPose RobotPos)
{
	m_RobotPos = RobotPos;
}
/**
@brief Korean:목적지의 위치를 저장하는  함수.
@brief English: 
*/
void KuCommandMessage::setGoalPos(KuPose GoalPos)
{
	m_GoalPos = GoalPos;
}
/**
@brief Korean:지도의 크기를  저장하는  함수.
@brief English: 
*/
void KuCommandMessage::setMapSizeXmYm(int nX, int nY)
{
	m_nMapSizeX = nX; 
	m_nMapSizeY = nY;
}
/**
@brief Korean: Behavior의 이름을 가져가는  함수.
@brief English: 
*/
int KuCommandMessage::getBehaviorName()
{
	return m_nBehaviorName;
}
/**
@brief Korean: Behavior에 내리는 명령을 가져가는  함수.
@brief English: 
*/
int KuCommandMessage::getCommandName()
{
	return m_nCommandName;
}
/**
@brief Korean: Behavior를 실행 시킬 주기를 가져가는  함수.
@brief English: 
*/
int KuCommandMessage::getBehaviorPeriod()
{
	return m_nBehaviorPeriod;
}
/**
@brief Korean: 목적지의 위치를 가져가는  함수.
@brief English: 
*/
KuPose KuCommandMessage::getGoalPos()
{
	return m_GoalPos;
}
/**
@brief Korean: 로봇의  위치를 가져가는  함수.
@brief English: 
*/
KuPose KuCommandMessage::getRobotPos()
{
	return m_RobotPos;
}
/**
@brief Korean: 지도의 크기를 가져가는  함수.
@brief English: 
*/
void KuCommandMessage::getMapSizeXmYm(int* nX, int* nY)
{
	*nX = m_nMapSizeX;
	*nY = m_nMapSizeY;
}
/**
@brief Korean: 
@brief English: 
*/
void KuCommandMessage::setAvoidMode(bool bAvoidMode)
{
	m_bAvoidMode = bAvoidMode;
}
/**
@brief Korean: 
@brief English: 
*/
bool KuCommandMessage::getAvoidMode()
{
	return m_bAvoidMode;
}