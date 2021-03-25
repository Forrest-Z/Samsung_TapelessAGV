#include "stdafx.h"
#include "KuCommandMessage.h"

KuCommandMessage::KuCommandMessage()
{
	m_nBehaviorName =-1; //������ behavior name�� �Ҵ�Ǿ� ���� ���� ���
	m_nCommandName = -1; //������ ��ɾ �Ҵ�Ǿ� ���� ���� ���
	m_nBehaviorPeriod = 0;
	m_nMapSizeX = 0; 
	m_nMapSizeY = 0;
	m_bAvoidMode=false;
}

KuCommandMessage::~KuCommandMessage()
{

}
/**
@brief Korean: Behavior�� �̸��� �����ϴ�  �Լ�.
@brief English: 
*/
void KuCommandMessage::setBehaviorName(int nName)
{
	m_nBehaviorName = nName;
}
/**
@brief Korean: Behavior�� ������ ����������ϴ�  �Լ�.
@brief English: 
*/
void KuCommandMessage::setCommandName(int nCommandName)
{
	m_nCommandName = nCommandName;
}

/**
@brief Korean: Behavior�� �����ų �ֱ⸦  �����ϴ�  �Լ�.
@brief English: 
*/
void KuCommandMessage::setBehaviorPeriod(int nPeriod)
{
	m_nBehaviorPeriod = nPeriod;
}
/**
@brief Korean:�κ��� ��ġ�� �����ϴ�  �Լ�.
@brief English: 
*/
void KuCommandMessage::setRobotPos(KuPose RobotPos)
{
	m_RobotPos = RobotPos;
}
/**
@brief Korean:�������� ��ġ�� �����ϴ�  �Լ�.
@brief English: 
*/
void KuCommandMessage::setGoalPos(KuPose GoalPos)
{
	m_GoalPos = GoalPos;
}
/**
@brief Korean:������ ũ�⸦  �����ϴ�  �Լ�.
@brief English: 
*/
void KuCommandMessage::setMapSizeXmYm(int nX, int nY)
{
	m_nMapSizeX = nX; 
	m_nMapSizeY = nY;
}
/**
@brief Korean: Behavior�� �̸��� ��������  �Լ�.
@brief English: 
*/
int KuCommandMessage::getBehaviorName()
{
	return m_nBehaviorName;
}
/**
@brief Korean: Behavior�� ������ ����� ��������  �Լ�.
@brief English: 
*/
int KuCommandMessage::getCommandName()
{
	return m_nCommandName;
}
/**
@brief Korean: Behavior�� ���� ��ų �ֱ⸦ ��������  �Լ�.
@brief English: 
*/
int KuCommandMessage::getBehaviorPeriod()
{
	return m_nBehaviorPeriod;
}
/**
@brief Korean: �������� ��ġ�� ��������  �Լ�.
@brief English: 
*/
KuPose KuCommandMessage::getGoalPos()
{
	return m_GoalPos;
}
/**
@brief Korean: �κ���  ��ġ�� ��������  �Լ�.
@brief English: 
*/
KuPose KuCommandMessage::getRobotPos()
{
	return m_RobotPos;
}
/**
@brief Korean: ������ ũ�⸦ ��������  �Լ�.
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