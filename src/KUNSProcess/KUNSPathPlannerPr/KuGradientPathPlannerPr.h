/*______________________________________________________________________________________________
PROJECT : Intelligent Robotics Lab. Department of Mecanical Engineering Korea University                                    
(c) Copyright 2007 KUME Intelligent Robotics Lab.                                             
All rights reserved                                                                         
$Directed by Jae-Bok Song                                                                     
$Designed by Joong-Tae Park                                                                  
$Description : ��ΰ�ȹ�� �����ϴ� �����ϴ� ���μ���
$Created on: 2012. 6. 12.                                                                        
$Author: Joong-Tae Park      
$e-mail: jtpark1114@gmail.com                                                                     
______________________________________________________________________________________________*/

#ifndef KUNS_GRADIENT_PATHPLANNER_PROCESS_H
#define KUNS_GRADIENT_PATHPLANNER_PROCESS_H

#include <list>
#include <cmath>
#include <iostream>

#include "../../KUNSPose/KuPose.h"
#include "../../KUNSMap/KuMap.h"
#include "../../KUNSMath/KuMath.h"
#include "../../KUNSUtil/KuUtil.h"
#include "../../KUNSUtil/KUNSSmartPointer/KuSmartPointer.h"

using namespace std;
class KuGradientPathPlannerPr 
{

	static const int CORRECT_PATH_PLANNED = 0;
	static const int ALL_PATH_BLOCKED = 1;
	static const int ROBOT_NEAR_OBSTACLE = 2;
	static const int GOAL_NEAR_OBSTACLE	= 3;

	static const int INFINITY_VALUE = 100000;
	static const int CSPACE_OBSTACLE_INFINITY=2;//CSpace�� ���Ѵ� ����( ���ڴ���)
	static const int CSPACE_OBSTACLE=CSPACE_OBSTACLE_INFINITY+4;//CSpace ��ü ����(���ڴ���)
	static const int GRAVITY_REGION=CSPACE_OBSTACLE;//Gravity path �� CSpace
	static const int CSPACE_OBSTACLE_ASSISTANCE=50;// CSpace�ǹ��Ѵ� ������ ������ �κ��� ��� 
	static const int PATH_MAXSIZE=50000;

private:
	KuSmartPointer<KuMap> m_smtpMap; //���������� ������ ����.
	KuMath m_math;
	KuUtil m_KuUtil;

public:
	void setMap(KuMap* pMap);
	int generatePath(KuPose GoalPos, KuPose RobotPos);
	list<KuPose> getPath();   // ��ǥ���������� ��� vector�� ����.
	void setRadiusofRobotp(int nRadiusofRobot);
	void initializeMapSize(int nMapSizeX, int nMapSizeY);
	void initIntCost(int nRange);
	
	KuGradientPathPlannerPr();
	~KuGradientPathPlannerPr();


private:
	list<KuPose> m_listPath; //��������� �����ϴ� vector.
	

	int** m_nMap;
	float** m_fIntCost;
	float** m_fAdjCost;
	float** m_fNavCost;
	
	float**m_finitIntCost;

	int m_nMapSizeX;
	int m_nMapSizeY;
	int m_nCSpaceObstacle;


	double m_dRadiusofRobot;
	int m_nCSpaceInfinity;

private:
	void initialize();
	void clear();
	void initMap();

	void calAdjCost(int nGoalPosition[2]);
	bool calAdjCostUp(int nI, int nJ, float fValue);
	bool calAdjCostDown(int nI, int nJ, float fValue);
	bool calAdjCostLeft(int nI, int nJ, float fValue);
	bool calAdjCostRight(int nI, int nJ, float fValue);
	bool calAdjCostUpRight(int nI, int nJ, float fValue);
	bool calAdjCostDownLeft(int nI, int nJ, float fValue);
	bool calAdjCostLeftUp(int nI, int nJ, float fValue);
	bool calAdjCostRightDown(int nI, int nJ, float fValue);

	void calNavCost();
	bool checkUPIsMin(int nI, int nJ);
	bool checkDownIsMin(int nI, int nJ);
	bool checkLeftIsMin(int nI, int nJ);
	bool checkRightIsMin(int nI, int nJ);
	bool checkUpRightIsMin(int nI, int nJ);
	bool checkDownLeftIsMin(int nI, int nJ);
	bool checkLeftUpIsMin(int nI, int nJ);
	bool checkRightDownIsMin(int nI, int nJ);
	void mapdataOutput(); //����׿� ���߿� ����ϰ� ������ public���� ��ȯ�ؼ�.

	void calCastIntrinCost(int nRange);
	bool extractGradientPath(int nStartPosition[2],int nGoalPosition[2]);
	int calGradient(int nX,int nY,int nGradientnum);


};

#endif /*KUNS_GRADIENT_PATHPLANNER_PROCESS_H*/
