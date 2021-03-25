#ifndef KUNS_COMMAND_MESSAGE_H
#define KUNS_COMMAND_MESSAGE_H

#include <iostream>
#include "../KUNSPose/KuPose.h"
#include <list>
using namespace std;

class KuCommandMessage
{
public:
	
	static const int START_THREAD = 0;
	static const int TERMINATE_THREAD = 1;
	static const int SUSPEND_THREAD = 2;
	static const int RESUME_THREAD = 3;
	static const int HYBRID_MAP_BUILDING_BH = 4;
	static const int GOTOGOAL_BH = 5;
	static const int PATH_TEACHING_BH= 6;
	static const int AUTONOMOUS_GOTOGOAL = 7;
	static const int GLOBAL_LOCALIZATION_BH = 8;
	static const int MULTI_GOTOGOAL_BH = 9;
	static const int GLOBAL_MAP_BUILDING_BH = 10;

	
private:
	int m_nBehaviorName;
	int m_nCommandName;
	int m_nBehaviorPeriod;
	int m_nMapSizeX, m_nMapSizeY;
	KuPose m_GoalPos, m_RobotPos;
	list<KuPose>m_GoalPosList;
	bool m_bAvoidMode;
public:
	void setBehaviorName(int nName);
	void setCommandName(int nCommandName);
	void setBehaviorPeriod(int nPeriod);
	void setRobotPos(KuPose RobotPos);
	void setGoalPos(KuPose GoalPos);
	void setMapSizeXmYm(int nX, int nY);
	void setAvoidMode(bool bAvoidMode);
	
	int getBehaviorName();
	int getCommandName();
	int getBehaviorPeriod();
	KuPose getGoalPos();
	KuPose getRobotPos();
	void getMapSizeXmYm(int* nX, int* nY);
	bool getAvoidMode();
	

	KuCommandMessage();
	virtual ~KuCommandMessage();
	
};

#endif