#include "stdafx.h"
#include "KuTeachingPathPlannerPr.h"

KuTeachingPathPlannerPr::KuTeachingPathPlannerPr()
{
	clear();
}
KuTeachingPathPlannerPr::~KuTeachingPathPlannerPr()
{

}
/**
@brief Korean: ��θ� ��������  �Լ�.
@brief English: 
*/
list<KuPose> KuTeachingPathPlannerPr::getPathList()
{
	return m_PathList;
}
/**
@brief Korean: ��θ� ��������  �Լ�.
@brief English: 
*/
list<KuPose> KuTeachingPathPlannerPr::getWayPointList()
{
	return m_WayPointList;
}

/**
@brief Korean: ��θ� �ʱ�ȭ ���ִ� �Լ�
@brief English: 
*/
void KuTeachingPathPlannerPr::clear()
{
	//������ ��� �� ������ ������ �����Ѵ�.
	m_PathList.clear();
	m_WayPointList.clear();
}
/**
@brief Korean: ��θ� �ҷ����� �Լ�
@brief English: 
*/
list<KuPose> KuTeachingPathPlannerPr::loadPath( string  strDataPath)
{
	ifstream DataLog;
	m_PathList.clear();
	DataLog.open(strDataPath);
	if( !DataLog.is_open() )
	{
		return m_PathList;
	}

	double dPathX,dPathY,dPathThetaDeg;
	KuPose PathPos;
	while(!DataLog.eof()){
		DataLog >> dPathX >> dPathY>>dPathThetaDeg;
		
		if(dPathX<0||dPathY<0){return m_PathList;}

		PathPos.setX(dPathX);
		PathPos.setY(dPathY);
		PathPos.setThetaDeg(dPathThetaDeg);
		m_PathList.push_back(PathPos);
	}	
	DataLog.close();
	
	if(m_PathList.size()>1)
		m_PathList.pop_back();
	
	return m_PathList;
}

/**
@brief Korean: �н��� ��θ� �����ϴ� �Լ�.
@brief English: 
*/
void  KuTeachingPathPlannerPr::savePath(string strDataPath )
{

	ofstream DataLog;
	DataLog.open(strDataPath);

	list<KuPose>::iterator it;

	//�������
	for(it=m_PathList.begin(); it!=m_PathList.end(); it++){
		KuPose Path;
		Path.setX(it->getX());
		Path.setY(it->getY());
		Path.setThetaDeg(it->getThetaDeg());
		DataLog<<Path.getX()<<" "<<Path.getY()<<" "<<Path.getThetaDeg()<<endl;
	}

	DataLog.close();
}

/**
@brief Korean: ��θ� ����� �Լ�
@brief English: 
*/
list<KuPose> KuTeachingPathPlannerPr::execute(KuPose RobotPos, bool *bsetWayPointflag)
{
	double dDist = sqrt(pow((m_RobotPos.getX() - RobotPos.getX() ),2)+pow((m_RobotPos.getY() - RobotPos.getY()),2));
	double 	dDegDiff =getAngleDiff(m_RobotPos.getThetaRad(),RobotPos.getThetaRad() )*R2D;

	if(true==(*bsetWayPointflag)&&(m_WayPoint.getX()!=RobotPos.getX()||m_WayPoint.getY()!=RobotPos.getY()))
	{
		m_PathList.push_back(RobotPos);
		m_WayPointList.push_back(RobotPos);
		(*bsetWayPointflag)=false;
		m_WayPoint=RobotPos;
	}

	if(dDist > 200){
		m_PathList.push_back(RobotPos);		
		m_RobotPos = RobotPos;
	}
	else if(dDegDiff>45)
	{
		m_PathList.push_back(RobotPos);		
		m_RobotPos = RobotPos;
	}
	return m_PathList;
}

/**
@brief Korean: ���� �κ���ġ�� ���� �κ��� ��ġ������ ���� ���̸� �����ִ� �Լ�
@brief English: 
*/
float KuTeachingPathPlannerPr::getAngleDiff(const float& fAngle1, const float& fAngle2)
{
	float fDiff;

	fDiff = fAngle1 - fAngle2;

	if(fabs(fDiff) > M_PI)
	{
		if(fDiff > 0)
			fDiff = fDiff - (float)6.283184;
		else
			fDiff = (float)6.283184 + fDiff;
	}

	return fDiff;
}
/**
@brief Korean: �н��� ��θ� �����ϴ� �Լ�.
@brief English: 
*/
void  KuTeachingPathPlannerPr::saveWayPointList(string strDataWaypoint)
{
	ofstream DataLog;
	DataLog.open(strDataWaypoint);

	list<KuPose>::iterator it;

	//�������
	for(it=m_WayPointList.begin(); it!=m_WayPointList.end(); it++){
		KuPose WayPoint;
		WayPoint.setX(it->getX());
		WayPoint.setY(it->getY());
		WayPoint.setThetaDeg(it->getThetaDeg());
		WayPoint.setPro(1);
		WayPoint.setDist(1);
		DataLog<<WayPoint.getX()<<" "<<WayPoint.getY()<<" "<<WayPoint.getThetaDeg()<<" "<<WayPoint.getPro()<<" "<<WayPoint.getDist()<<endl;
	}

	DataLog.close();
}

/**
@brief Korean: ��θ� �ҷ����� �Լ�
@brief English: 
*/
list<KuPose> KuTeachingPathPlannerPr::loadWayPointList(string  strDataWaypoint  )
{
	
	ifstream DataLog;
	m_WayPointList.clear();

	DataLog.open(strDataWaypoint);

	if( !DataLog.is_open() )
	{
		return m_WayPointList;
	}

	double dPathX,dPathY,dPathThetaDeg,dPro;
	double dDist;
	KuPose WayPoint;
	while(!DataLog.eof()){
		DataLog >> dPathX >> dPathY>>dPathThetaDeg>>dPro>>dDist;
		
		if(dPathX<0||dPathY<0){return m_WayPointList;}
	
		WayPoint.setX(dPathX);
		WayPoint.setY(dPathY);
		WayPoint.setThetaDeg(dPathThetaDeg);
		WayPoint.setPro(dPro);
		WayPoint.setDist(dDist);
		m_WayPointList.push_back(WayPoint);
	}	
	DataLog.close();

	if(m_WayPointList.size()>1)
		m_WayPointList.pop_back();

	return m_WayPointList;
}