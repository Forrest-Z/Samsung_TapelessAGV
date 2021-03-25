#include "stdafx.h"
#include "KuPathBlockPr.h"

KuPathBlockPr::KuPathBlockPr()
{
	m_dBlockSizeX = KuRobotParameter::getInstance()->getBlockSizeX();
	m_dBlockSizeY = KuRobotParameter::getInstance()->getBlockSizeY();
}

KuPathBlockPr::~KuPathBlockPr()
{

}

PBlock KuPathBlockPr::Path1()
{
	PBlock PB;
	PB.x=0;
	PB.y=0;
	PB.id=-1;
	PB.velocity= 0;
	PB.task=-1;
	PB.state=0;
	PB.nRouteOrder=0;
	PB.waypoint=0;
	PB.pathdata=1;
	PB.sizex=m_dBlockSizeX;
	PB.sizey=m_dBlockSizeY;
	PB.check=false;
	PB.overlap=false;
	LinePoint Lpoint;
	Lpoint.start_x=-PB.sizex/2.0;
	Lpoint.start_y=0.0;
	Lpoint.end_x=PB.sizex/2.0;
	Lpoint.end_y=0.0;
	PB.dist1=Lpoint;
	Lpoint.start_x=0.0;
	Lpoint.start_y=0.0;
	Lpoint.end_x=0.0;
	Lpoint.end_y=0.0;
	PB.dist2=Lpoint;
	PB.detectObstacle=true;
	PB.TaskList.clear();
	return PB;
}
PBlock KuPathBlockPr::Path2()
{
	PBlock PB;
	PB.x=0;
	PB.y=0;
	PB.id=-1;
	PB.velocity= 0;
	PB.task=-1;
	PB.state=0;
	PB.nRouteOrder=0;
	PB.waypoint=0;
	PB.pathdata=2;
	PB.sizex=m_dBlockSizeX;
	PB.sizey=m_dBlockSizeY;
	PB.check=false;
	PB.overlap=false;
	LinePoint Lpoint;
	Lpoint.start_x=0.0;
	Lpoint.start_y=-PB.sizey/2;
	Lpoint.end_x=0.0;
	Lpoint.end_y=PB.sizey/2;
	PB.dist1=Lpoint;
	Lpoint.start_x=0.0;
	Lpoint.start_y=0.0;
	Lpoint.end_x=0.0;
	Lpoint.end_y=0.0;
	PB.dist2=Lpoint;
	PB.detectObstacle=true;
	PB.TaskList.clear();


	return PB;
}

PBlock KuPathBlockPr::Path3()
{
	PBlock PB;
	PB.x=0;
	PB.y=0;
	PB.id=-1;
	PB.velocity= 0;
	PB.task=-1;
	PB.state=0;
	PB.nRouteOrder=0;
	PB.waypoint=0;
	PB.pathdata=3;
	PB.sizex=m_dBlockSizeX;
	PB.sizey=m_dBlockSizeY;
	PB.check=false;
	PB.overlap=false;
	LinePoint Lpoint;
	Lpoint.start_x=-PB.sizex/2;
	Lpoint.start_y=-PB.sizey/2;
	Lpoint.end_x=PB.sizex/2;
	Lpoint.end_y=PB.sizey/2;
	PB.dist1=Lpoint;
	Lpoint.start_x=0.0;
	Lpoint.start_y=0.0;
	Lpoint.end_x=0.0;
	Lpoint.end_y=0.0;
	PB.dist2=Lpoint;
	PB.detectObstacle=true;
	PB.TaskList.clear();

	return PB;
}

PBlock KuPathBlockPr::Path4()
{
	PBlock PB;
	PB.x=0;
	PB.y=0;
	PB.id=-1;
	PB.velocity= 0;
	PB.task=-1;
	PB.state=0;
	PB.nRouteOrder=0;
	PB.waypoint=0;
	PB.pathdata=4;
	PB.sizex=m_dBlockSizeX;
	PB.sizey=m_dBlockSizeY;
	PB.check=false;
	PB.overlap=false;
	LinePoint Lpoint;
	Lpoint.start_x=-PB.sizex/2;
	Lpoint.start_y=PB.sizey/2;
	Lpoint.end_x=PB.sizex/2;
	Lpoint.end_y=-PB.sizey/2;
	PB.dist1=Lpoint;
	Lpoint.start_x=0.0;
	Lpoint.start_y=0.0;
	Lpoint.end_x=0.0;
	Lpoint.end_y=0.0;
	PB.dist2=Lpoint;
	PB.detectObstacle=true;
	PB.TaskList.clear();

	return PB;
}



PBlock KuPathBlockPr::Path5()
{
	PBlock PB;
	PB.x=0;
	PB.y=0;
	PB.id=-1;
	PB.velocity= 0;
	PB.task=-1;
	PB.state=0;
	PB.nRouteOrder=0;
	PB.waypoint=0;
	PB.pathdata=5;
	PB.sizex=m_dBlockSizeX;
	PB.sizey=m_dBlockSizeY;
	PB.check=false;
	PB.overlap=false;
	LinePoint Lpoint;
	Lpoint.start_x=-PB.sizex/2;
	Lpoint.start_y=-PB.sizey/4;
	Lpoint.end_x=PB.sizex/2;
	Lpoint.end_y=PB.sizey/4;
	PB.dist1=Lpoint;
	Lpoint.start_x=0.0;
	Lpoint.start_y=0.0;
	Lpoint.end_x=0.0;
	Lpoint.end_y=0.0;
	PB.dist2=Lpoint;
	PB.detectObstacle=true;
	PB.TaskList.clear();

	return PB;
}

PBlock KuPathBlockPr::Path6()
{
	PBlock PB;
	PB.x=0;
	PB.y=0;
	PB.id=-1;
	PB.velocity= 0;
	PB.task=-1;
	PB.state=0;
	PB.nRouteOrder=0;
	PB.waypoint=0;
	PB.pathdata=6;
	PB.sizex=m_dBlockSizeX;
	PB.sizey=m_dBlockSizeY;
	PB.check=false;
	PB.overlap=false;
	LinePoint Lpoint;
	Lpoint.start_x=-PB.sizex/2;
	Lpoint.start_y=PB.sizey/4;
	Lpoint.end_x=PB.sizex/2;
	Lpoint.end_y=-PB.sizey/4;
	PB.dist1=Lpoint;
	Lpoint.start_x=0.0;
	Lpoint.start_y=0.0;
	Lpoint.end_x=0.0;
	Lpoint.end_y=0.0;
	PB.dist2=Lpoint;
	PB.detectObstacle=true;
	PB.TaskList.clear();

	return PB;
}

PBlock KuPathBlockPr::Path7()
{
	PBlock PB;
	PB.x=0;
	PB.y=0;
	PB.id=-1;
	PB.velocity= 0;
	PB.task=-1;
	PB.state=0;
	PB.nRouteOrder=0;
	PB.waypoint=0;
	PB.pathdata=7;
	PB.sizex=m_dBlockSizeX;
	PB.sizey=m_dBlockSizeY;
	PB.check=false;
	PB.overlap=false;
	LinePoint Lpoint;
	Lpoint.start_x=-PB.sizex/4;
	Lpoint.start_y=PB.sizey/2;
	Lpoint.end_x=PB.sizex/4;
	Lpoint.end_y=-PB.sizey/2;
	PB.dist1=Lpoint;
	Lpoint.start_x=0.0;
	Lpoint.start_y=0.0;
	Lpoint.end_x=0.0;
	Lpoint.end_y=0.0;
	PB.dist2=Lpoint;
	PB.detectObstacle=true;
	PB.TaskList.clear();

	return PB;
}


PBlock KuPathBlockPr::Path8()
{
	PBlock PB;
	PB.x=0;
	PB.y=0;
	PB.id=-1;
	PB.velocity= 0;
	PB.task=-1;
	PB.state=0;
	PB.nRouteOrder=0;
	PB.waypoint=0;
	PB.pathdata=8;
	PB.sizex=m_dBlockSizeX;
	PB.sizey=m_dBlockSizeY;
	PB.check=false;
	PB.overlap=false;
	LinePoint Lpoint;
	Lpoint.start_x=PB.sizex/4;
	Lpoint.start_y=PB.sizey/2;
	Lpoint.end_x=-PB.sizex/4;
	Lpoint.end_y=-PB.sizey/2;
	PB.dist1=Lpoint;
	Lpoint.start_x=0.0;
	Lpoint.start_y=0.0;
	Lpoint.end_x=0.0;
	Lpoint.end_y=0.0;
	PB.dist2=Lpoint;
	PB.detectObstacle=true;
	PB.TaskList.clear();

	return PB;
}

PBlock KuPathBlockPr::Path10()
{
	PBlock PB;
	PB.x=0;
	PB.y=0;
	PB.id=-1;
	PB.velocity= 0;
	PB.task=-1;
	PB.state=1;
	PB.nRouteOrder=0;
	PB.waypoint=0;
	PB.pathdata=10;
	PB.sizex=m_dBlockSizeX;
	PB.sizey=m_dBlockSizeY;
	PB.check=false;
	PB.overlap=false;
	LinePoint Lpoint;
	Lpoint.start_x=-PB.sizex/2.0;
	Lpoint.start_y=0.0;
	Lpoint.end_x=0.0;
	Lpoint.end_y=PB.sizey/2.0;
	PB.dist1=Lpoint;
	Lpoint.start_x=0.0;
	Lpoint.start_y=0.0;
	Lpoint.end_x=0.0;
	Lpoint.end_y=0.0;
	PB.dist2=Lpoint;
	PB.detectObstacle=true;
	PB.TaskList.clear();

	return PB;
}

PBlock KuPathBlockPr::Path11()
{
	PBlock PB;
	PB.x=0;
	PB.y=0;
	PB.id=-1;
	PB.velocity= 0;
	PB.task=-1;
	PB.state=1;
	PB.nRouteOrder=0;
	PB.waypoint=0;
	PB.pathdata=11;
	PB.sizex=m_dBlockSizeX;
	PB.sizey=m_dBlockSizeY;
	PB.check=false;
	PB.overlap=false;
	LinePoint Lpoint;
	Lpoint.start_x=PB.sizex/2.0;
	Lpoint.start_y=0.0;
	Lpoint.end_x=0.0;
	Lpoint.end_y=PB.sizey/2.0;
	PB.dist1=Lpoint;
	Lpoint.start_x=0.0;
	Lpoint.start_y=0.0;
	Lpoint.end_x=0.0;
	Lpoint.end_y=0.0;
	PB.dist2=Lpoint;
	PB.detectObstacle=true;
	PB.TaskList.clear();

	return PB;
}


PBlock KuPathBlockPr::Path12()
{
	PBlock PB;
	PB.x=0;
	PB.y=0;
	PB.id=-1;
	PB.velocity= 0;
	PB.task=-1;
	PB.state=1;
	PB.nRouteOrder=0;
	PB.waypoint=0;
	PB.pathdata=12;
	PB.sizex=m_dBlockSizeX;
	PB.sizey=m_dBlockSizeY;
	PB.check=false;
	PB.overlap=false;
	LinePoint Lpoint;
	Lpoint.start_x=PB.sizex/2.0;
	Lpoint.start_y=0.0;
	Lpoint.end_x=0.0;
	Lpoint.end_y=-PB.sizey/2.0;
	PB.dist1=Lpoint;
	Lpoint.start_x=0.0;
	Lpoint.start_y=0.0;
	Lpoint.end_x=0.0;
	Lpoint.end_y=0.0;
	PB.dist2=Lpoint;
	PB.detectObstacle=true;
	PB.TaskList.clear();

	return PB;
}

PBlock KuPathBlockPr::Path13()
{
	PBlock PB;
	PB.x=0;
	PB.y=0;
	PB.id=-1;
	PB.velocity= 0;
	PB.task=-1;
	PB.state=1;
	PB.nRouteOrder=0;
	PB.waypoint=0;
	PB.pathdata=13;
	PB.sizex=m_dBlockSizeX;
	PB.sizey=m_dBlockSizeY;
	PB.check=false;
	PB.overlap=false;
	LinePoint Lpoint;
	Lpoint.start_x=-PB.sizex/2.0;
	Lpoint.start_y=0.0;
	Lpoint.end_x=0.0;
	Lpoint.end_y=-PB.sizey/2.0;
	PB.dist1=Lpoint;
	Lpoint.start_x=0.0;
	Lpoint.start_y=0.0;
	Lpoint.end_x=0.0;
	Lpoint.end_y=0.0;
	PB.dist2=Lpoint;
	PB.detectObstacle=true;
	PB.TaskList.clear();

	return PB;
}


PBlock KuPathBlockPr::Path20()
{
	PBlock PB;
	PB.x=0;
	PB.y=0;
	PB.id=-1;
	PB.velocity= 0;
	PB.task=-1;
	PB.state=2;
	PB.nRouteOrder=0;
	PB.waypoint=0;
	PB.pathdata=20;
	PB.sizex=m_dBlockSizeX;
	PB.sizey=m_dBlockSizeY;
	PB.check=false;
	PB.overlap=true;
	LinePoint Lpoint;
	Lpoint.start_x=-PB.sizex/2.0;
	Lpoint.start_y=0.0;
	Lpoint.end_x=PB.sizex/2.0;
	Lpoint.end_y=0.0;
	PB.dist1=Lpoint;
	Lpoint.start_x=0.0;
	Lpoint.start_y=-PB.sizey/2;
	Lpoint.end_x=0.0;
	Lpoint.end_y=PB.sizey/2;
	PB.dist2=Lpoint;
	PB.detectObstacle=true;
	PB.TaskList.clear();

	return PB;
}

PBlock KuPathBlockPr::Path21()
{
	PBlock PB;
	PB.x=0;
	PB.y=0;
	PB.id=-1;
	PB.velocity= 0;
	PB.task=-1;
	PB.state=2;
	PB.nRouteOrder=0;
	PB.waypoint=0;
	PB.pathdata=21;
	PB.sizex=m_dBlockSizeX;
	PB.sizey=m_dBlockSizeY;
	PB.check=false;
	PB.overlap=true;
	LinePoint Lpoint;
	Lpoint.start_x=-PB.sizex/2.0;
	Lpoint.start_y=0.0;
	Lpoint.end_x=PB.sizex/2.0;
	Lpoint.end_y=0.0;
	PB.dist1=Lpoint;
	Lpoint.start_x=PB.sizex/4;
	Lpoint.start_y=PB.sizey/2;
	Lpoint.end_x=-PB.sizex/4;
	Lpoint.end_y=-PB.sizey/2;
	PB.dist2=Lpoint;
	PB.detectObstacle=true;
	PB.TaskList.clear();

	return PB;
}

PBlock KuPathBlockPr::Path22()
{
	PBlock PB;
	PB.x=0;
	PB.y=0;
	PB.id=-1;
	PB.velocity= 0;
	PB.task=-1;
	PB.state=2;
	PB.nRouteOrder=0;
	PB.waypoint=0;
	PB.pathdata=22;
	PB.sizex=m_dBlockSizeX;
	PB.sizey=m_dBlockSizeY;
	PB.check=false;
	PB.overlap=true;
	LinePoint Lpoint;
	Lpoint.start_x=-PB.sizex/2.0;
	Lpoint.start_y=0.0;
	Lpoint.end_x=PB.sizex/2.0;
	Lpoint.end_y=0.0;
	PB.dist1=Lpoint;
	Lpoint.start_x=-PB.sizex/4;
	Lpoint.start_y=PB.sizey/2;
	Lpoint.end_x=PB.sizex/4;
	Lpoint.end_y=-PB.sizey/2;
	PB.dist2=Lpoint;	
	PB.detectObstacle=true;
	PB.TaskList.clear();

	return PB;
}

PBlock KuPathBlockPr::Path23()
{
	PBlock PB;
	PB.x=0;
	PB.y=0;
	PB.id=-1;
	PB.velocity= 0;
	PB.task=-1;
	PB.state=2;
	PB.nRouteOrder=0;
	PB.waypoint=0;
	PB.pathdata=23;
	PB.sizex=m_dBlockSizeX;
	PB.sizey=m_dBlockSizeY;
	PB.check=false;
	PB.overlap=true;
	LinePoint Lpoint;
	Lpoint.start_x=0.0;
	Lpoint.start_y=-PB.sizey/2;
	Lpoint.end_x=0.0;
	Lpoint.end_y=PB.sizey/2;
	PB.dist1=Lpoint;
	Lpoint.start_x=-PB.sizex/2;
	Lpoint.start_y=-PB.sizey/4;
	Lpoint.end_x=PB.sizex/2;
	Lpoint.end_y=PB.sizey/4;
	PB.dist2=Lpoint;	
	PB.detectObstacle=true;
	PB.TaskList.clear();

	return PB;
}


PBlock KuPathBlockPr::Path24()
{
	PBlock PB;
	PB.x=0;
	PB.y=0;
	PB.id=-1;
	PB.velocity= 0;
	PB.task=-1;
	PB.state=2;
	PB.nRouteOrder=0;
	PB.waypoint=0;
	PB.pathdata=24;
	PB.sizex=m_dBlockSizeX;
	PB.sizey=m_dBlockSizeY;
	PB.check=false;
	PB.overlap=true;
	LinePoint Lpoint;
	Lpoint.start_x=0.0;
	Lpoint.start_y=-PB.sizey/2;
	Lpoint.end_x=0.0;
	Lpoint.end_y=PB.sizey/2;
	PB.dist1=Lpoint;
	Lpoint.start_x=-PB.sizex/2;
	Lpoint.start_y=PB.sizey/4;
	Lpoint.end_x=PB.sizex/2;
	Lpoint.end_y=-PB.sizey/4;
	PB.dist2=Lpoint;
	PB.detectObstacle=true;
	PB.TaskList.clear();

	return PB;
}


PBlock KuPathBlockPr::Path30()
{
	PBlock PB;
	PB.x=0;
	PB.y=0;
	PB.id=-1;
	PB.velocity= 0;
	PB.task=-1;
	PB.state=3;
	PB.nRouteOrder=0;
	PB.waypoint=0;
	PB.pathdata=30;
	PB.sizex=m_dBlockSizeX;
	PB.sizey=m_dBlockSizeY;
	PB.check=false;
	PB.overlap=true;
	LinePoint Lpoint;
	Lpoint.start_x=-PB.sizex/2.0;
	Lpoint.start_y=0.0;
	Lpoint.end_x=PB.sizex/2.0;
	Lpoint.end_y=0.0;
	PB.dist1=Lpoint;
	Lpoint.start_x=0.0;
	Lpoint.start_y=0.0;
	Lpoint.end_x=0.0;
	Lpoint.end_y=PB.sizey/2;
	PB.dist2=Lpoint;
	PB.detectObstacle=true;
	PB.TaskList.clear();

	return PB;
}


PBlock KuPathBlockPr::Path31()
{
	PBlock PB;
	PB.x=0;
	PB.y=0;
	PB.id=-1;
	PB.velocity= 0;
	PB.task=-1;
	PB.state=3;
	PB.nRouteOrder=0;
	PB.waypoint=0;
	PB.pathdata=31;
	PB.sizex=m_dBlockSizeX;
	PB.sizey=m_dBlockSizeY;
	PB.check=false;
	PB.overlap=true;
	LinePoint Lpoint;
	Lpoint.start_x=0.0;
	Lpoint.start_y=PB.sizey/2;
	Lpoint.end_x=0.0;
	Lpoint.end_y=-PB.sizey/2;
	PB.dist1=Lpoint;
	Lpoint.start_x=0.0;
	Lpoint.start_y=0.0;
	Lpoint.end_x=PB.sizex/2.0;
	Lpoint.end_y=0.0;
	PB.dist2=Lpoint;	
	PB.detectObstacle=true;
	PB.TaskList.clear();

	return PB;
}
PBlock KuPathBlockPr::Path32()
{
	PBlock PB;
	PB.x=0;
	PB.y=0;
	PB.id=-1;
	PB.velocity= 0;
	PB.task=-1;
	PB.state=3;
	PB.nRouteOrder=0;
	PB.waypoint=0;
	PB.pathdata=32;
	PB.sizex=m_dBlockSizeX;
	PB.sizey=m_dBlockSizeY;
	PB.check=false;
	PB.overlap=true;
	LinePoint Lpoint;
	Lpoint.start_x=PB.sizex/2.0;
	Lpoint.start_y=0.0;
	Lpoint.end_x=-PB.sizex/2.0;
	Lpoint.end_y=0.0;
	PB.dist1=Lpoint;
	Lpoint.start_x=0.0;
	Lpoint.start_y=0.0;
	Lpoint.end_x=0.0;
	Lpoint.end_y=-PB.sizey/2;
	PB.dist2=Lpoint;	
	PB.detectObstacle=true;
	PB.TaskList.clear();

	return PB;
}


PBlock KuPathBlockPr::Path33()
{
	PBlock PB;
	PB.x=0;
	PB.y=0;
	PB.id=-1;
	PB.velocity= 0;
	PB.task=-1;
	PB.state=3;
	PB.nRouteOrder=0;
	PB.waypoint=0;
	PB.pathdata=33;
	PB.sizex=m_dBlockSizeX;
	PB.sizey=m_dBlockSizeY;
	PB.check=false;
	PB.overlap=true;
	LinePoint Lpoint;
	Lpoint.start_x=0.0;
	Lpoint.start_y=-PB.sizey/2;
	Lpoint.end_x=0.0;
	Lpoint.end_y=PB.sizey/2;
	PB.dist1=Lpoint;
	Lpoint.start_x=0.0;
	Lpoint.start_y=0.0;
	Lpoint.end_x=-PB.sizex/2.0;
	Lpoint.end_y=0.0;
	PB.dist2=Lpoint;	
	PB.detectObstacle=true;
	PB.TaskList.clear();

	return PB;
}



PBlock KuPathBlockPr::Path40()
{
	PBlock PB;
	PB.x=0;
	PB.y=0;
	PB.id=-1;
	PB.velocity= 0;
	PB.task=-1;
	PB.state=4;
	PB.nRouteOrder=0;
	PB.waypoint=0;
	PB.pathdata=40;
	PB.sizex=m_dBlockSizeX;
	PB.sizey=m_dBlockSizeY;
	PB.check=false;
	PB.overlap=false;
	LinePoint Lpoint;
	Lpoint.start_x=0.0;
	Lpoint.start_y=0.0;
	Lpoint.end_x=-PB.sizex/2.0;
	Lpoint.end_y=0.0;
	PB.dist1=Lpoint;
	Lpoint.start_x=0.0;
	Lpoint.start_y=0.0;
	Lpoint.end_x=0.0;
	Lpoint.end_y=0.0;
	PB.dist2=Lpoint;	
	PB.detectObstacle=true;
	PB.TaskList.clear();

	return PB;
}

PBlock KuPathBlockPr::Path41()
{
	PBlock PB;
	PB.x=0;
	PB.y=0;
	PB.id=-1;
	PB.velocity= 0;
	PB.task=-1;
	PB.state=4;
	PB.nRouteOrder=0;
	PB.waypoint=0;
	PB.pathdata=41;
	PB.sizex=m_dBlockSizeX;
	PB.sizey=m_dBlockSizeY;
	PB.check=false;
	PB.overlap=false;
	LinePoint Lpoint;
	Lpoint.start_x=0.0;
	Lpoint.start_y=0.0;
	Lpoint.end_x=0.0;
	Lpoint.end_y=-PB.sizey/2;
	PB.dist1=Lpoint;
	Lpoint.start_x=0.0;
	Lpoint.start_y=0.0;
	Lpoint.end_x=0.0;
	Lpoint.end_y=0.0;
	PB.dist2=Lpoint;	
	PB.detectObstacle=true;
	PB.TaskList.clear();

	return PB;
}

PBlock KuPathBlockPr::Path42()
{
	PBlock PB;
	PB.x=0;
	PB.y=0;
	PB.id=-1;
	PB.velocity= 0;
	PB.task=-1;
	PB.state=4;
	PB.nRouteOrder=0;
	PB.waypoint=0;
	PB.pathdata=42;
	PB.sizex=m_dBlockSizeX;
	PB.sizey=m_dBlockSizeY;
	PB.check=false;
	PB.overlap=false;
	LinePoint Lpoint;
	Lpoint.start_x=0.0;
	Lpoint.start_y=0.0;
	Lpoint.end_x=PB.sizex/2;
	Lpoint.end_y=0.0;
	PB.dist1=Lpoint;
	Lpoint.start_x=0.0;
	Lpoint.start_y=0.0;
	Lpoint.end_x=0.0;
	Lpoint.end_y=0.0;
	PB.dist2=Lpoint;	
	PB.detectObstacle=true;
	PB.TaskList.clear();

	return PB;
}


PBlock KuPathBlockPr::Path43()
{
	PBlock PB;
	PB.x=0;
	PB.y=0;
	PB.id=-1;
	PB.velocity= 0;
	PB.task=-1;
	PB.waypoint=0;
	PB.state=4;
	PB.nRouteOrder=0;
	PB.pathdata=43;
	PB.sizex=m_dBlockSizeX;
	PB.sizey=m_dBlockSizeY;
	PB.check=false;
	PB.overlap=false;
	LinePoint Lpoint;
	Lpoint.start_x=0.0;
	Lpoint.start_y=0.0;
	Lpoint.end_x=0.0;
	Lpoint.end_y=PB.sizex/2;
	PB.dist1=Lpoint;
	Lpoint.start_x=0.0;
	Lpoint.start_y=0.0;
	Lpoint.end_x=0.0;
	Lpoint.end_y=0.0;
	PB.dist2=Lpoint;	
	PB.detectObstacle=true;
	PB.TaskList.clear();

	return PB;
}

PBlock KuPathBlockPr::getPathBlock(int nPathID, bool bInit /*= true*/)
{
	PBlock PB;

	if(bInit)
	{
		PB.x=0;
		PB.y=0;
		PB.id=-1;
		PB.velocity= 0;
		PB.task=-1;
		PB.waypoint=0;
		PB.state=0;
		PB.nRouteOrder=0;
		PB.pathdata=-1;
 		PB.sizex=1;
 		PB.sizey=1;//초기화..
		PB.check=false;
		PB.overlap=false;
		LinePoint Lpoint;
		Lpoint.start_x=0.0;
		Lpoint.start_y=0.0;
		Lpoint.end_x=0.0;
		Lpoint.end_y=0.0;
		PB.dist1=Lpoint;
		Lpoint.start_x=0.0;
		Lpoint.start_y=0.0;
		Lpoint.end_x=0.0;
		Lpoint.end_y=0.0;
		PB.dist2=Lpoint;
		PB.detectObstacle=true;
		PB.TaskList.clear();
	}

	switch(nPathID)
	{	
	case 1:
		PB = Path1();
		break;
	case 2:
		PB = Path2();
		break;
	case 3:
		PB = Path3();
		break;
	case 4:
		PB = Path4();
		break;
	case 5:
		PB = Path5();
		break;
	case 6:
		PB = Path6();
		break;
	case 7:
		PB = Path7();
		break;
	case 8:
		PB = Path8();
		break;
	case 10:
		PB = Path10();
		break;
	case 11:
		PB = Path11();
		break;
	case 12:
		PB = Path12();
		break;
	case 13:
		PB = Path13();
		break;
	case 20:
		PB = Path20();
		break;
	case 21:
		PB = Path21();
		break;
	case 22:
		PB = Path22();
		break;
	case 23:
		PB = Path23();
		break;
	case 24:
		PB = Path24();
		break;
	case 30:
		PB = Path30();
		break;
	case 31:
		PB = Path31();
		break;
	case 32:
		PB = Path32();
		break;
	case 33:
		PB = Path33();
		break;
	case 40:
		PB = Path40();
		break;
	case 41:
		PB = Path41();
		break;
	case 42:
		PB = Path42();
		break;
	case 43:
		PB = Path43();
		break;
	default:break;
	}
	return PB;
}
bool KuPathBlockPr::checkPathBlock(KuPose PathPose1,KuPose PathPose2,int Pathdata)
{
	// 	if(fabs(PathPose1.getXm()-PathPose2.getXm())<0.1&&fabs(PathPose1.getYm()-PathPose2.getYm())<0.1)
	// 	{		
	// 		if(Pathdata<20&&Pathdata>=10)
	// 		{
	// 			KuPose Pose;
	// 			Pose.setXm(m_vecPathBlockPos[i].x);	Pose.setYm(m_vecPathBlockPos[i].y);
	// 			m_Pathlist.push_back(Pose);
	// 		}
	// 		m_Pathlist.push_back(PathPose2);
	// 
	// 		m_vecPathBlockPos[i].check=true;
	// 		i=0;continue;
	// 	}	
	return true;
}

void  KuPathBlockPr::setPathBlockPos(vector<PBlock> vecPathBlockPos)
{
	//global path generation (전체 경로) for KUNS mode
	m_vecPathBlockPos.clear();
	for(int i=0; i<vecPathBlockPos.size();i++)
	{
		m_vecPathBlockPos.push_back(vecPathBlockPos[i]);
		m_vecPathBlockPos[i].id=i;
	}

	//start point 선택
	int nStartPosNum=0;
	vector<int> vectnStarPoint;

	//way point 선택
	int nWayPosNum=0;

	//via point 선택
	int nVialPosNum=0;
	vector<int> vecnViaPoint;

	//end point 선택
	int nEndPosNum=0;
	vector<int> vectnEndPoint;

	for(int i=0; i<m_vecPathBlockPos.size();i++)
	{
		if(m_vecPathBlockPos[i].waypoint==2)
		{
			nStartPosNum++;
			vectnStarPoint.push_back(i);
		}

		if(m_vecPathBlockPos[i].waypoint==3)
		{
			nEndPosNum++;
			vectnEndPoint.push_back(i);
		}

		if(!(m_vecPathBlockPos[i].nRouteOrder==0))
		{
			nVialPosNum++;
		}

		if(m_vecPathBlockPos[i].waypoint==1)
		{
			nWayPosNum++;
		}
	}

	//index순서대로 viapoint 저장
	for(int i=1; i<nVialPosNum+1; i++)
	{
		for(int j=0; j<m_vecPathBlockPos.size(); j++)
		{
			if(m_vecPathBlockPos[j].nRouteOrder==i)
			{
				vecnViaPoint.push_back(j);
			}
		}
	}

	//start, vias, end point 선택
	vector<int> vecnStartViasEndPoint;
	if(!vectnStarPoint.size()==0)	vecnStartViasEndPoint.push_back(vectnStarPoint[0]);
	for(int i=0; i<vecnViaPoint.size(); i++)
	{
		vecnStartViasEndPoint.push_back(vecnViaPoint[i]);
	}
	if(!vectnEndPoint.size()==0)	vecnStartViasEndPoint.push_back(vectnEndPoint[0]);

	if(vecnStartViasEndPoint.size()>1)
	{
		//전체 경로 인데스 추출
		vector<int> vecnGlobalPath;
		getPathIdx(vecPathBlockPos,vecnStartViasEndPoint,vecnGlobalPath);

		//경로
		vector<PBlock> vecPathBlock;
		m_vecPathBlockGroup.clear();
		if(!vectnStarPoint.size()==0) vecPathBlock.push_back(m_vecPathBlockPos[vectnStarPoint[0]]);
		for(int i=0; i<vecnGlobalPath.size(); i++)
		{
			int nPathIdx = vecnGlobalPath[i];
			vecPathBlock.push_back(m_vecPathBlockPos[nPathIdx]);
			if(m_vecPathBlockPos[nPathIdx].waypoint==1)
			{
				m_vecPathBlockGroup.push_back(vecPathBlock);
				vecPathBlock.clear();
				//vecPathBlock.push_back(m_vecPathBlockPos[nPathIdx]);
			}
		}
		m_vecPathBlockGroup.push_back(vecPathBlock);

		//vector->list
		m_vecPathlist.clear();
		for(int i=0; i<m_vecPathBlockGroup.size();i++)
		{
			list<KuPose> pathlist=generatePathList(m_vecPathBlockGroup[i]);
			m_vecPathlist.push_back(pathlist);
		}
	}	
}

void  KuPathBlockPr::setPathBlockPosForISSAC(vector<PBlock> vecPathBlockPos)
{
	//global path generation (전체 경로) for ISSAC mode
	m_vecPathBlockPos.clear();
	for(int i=0; i<vecPathBlockPos.size();i++)
	{
		m_vecPathBlockPos.push_back(vecPathBlockPos[i]);
		m_vecPathBlockPos[i].id=i;
	}

	//경로
	vector<PBlock> vecPathBlock;
	m_vecPathBlockGroup.clear();
	for(int i=0; i<vecPathBlockPos.size(); i++)
	{
		vecPathBlock.push_back(m_vecPathBlockPos[i]);
		if(m_vecPathBlockPos[i].waypoint==1)//여기가 way point냐?
		{
			m_vecPathBlockGroup.push_back(vecPathBlock);
			vecPathBlock.clear();
		}
	}

	vecPathBlock.push_back(m_vecPathBlockPos[0]); // 시작점으로 이동하는 path

	m_vecPathBlockGroup.push_back(vecPathBlock);
	vecPathBlock.clear();

	// 시작점으로 이동하는 path
//	PBlock blockLast = m_vecPathBlockPos[m_vecPathBlockPos.size() - 1];
//	vecPathBlock.push_back(blockLast);
//	PBlock blockStart = m_vecPathBlockPos[0];
//	vecPathBlock.push_back(blockStart);
//	m_vecPathBlockGroup.push_back(vecPathBlock);

	//vector->list
	m_vecPathlist.clear();
	for(int i=0; i<m_vecPathBlockGroup.size();i++)
	{
		list<KuPose> pathlist=generatePathList(m_vecPathBlockGroup[i]);
		m_vecPathlist.push_back(pathlist);
	}
}

bool KuPathBlockPr::generatePathBlockConnection(vector<PBlock> vecTotalPathBlock,int nStartPoint,vector<PBlock> *vecPathBlock)
{
	if(KuRobotParameter::getInstance()->getUsingISSAC() == 1) // ISSAC을 사용하는 경우
	{
		setPathBlockPosForISSAC(vecTotalPathBlock);
	}
	else	
	{
		setPathBlockPos(vecTotalPathBlock);
	}

	//경로에 시작점이 있는지 검사하고 시작점부터 저장
	bool bConnectPath = false;
	if(!(m_vecPathBlockGroup[0].size()>0)) return false;
	for(int i=0; i<m_vecPathBlockGroup.size(); i++)
	{
		for(int j=0; j<m_vecPathBlockGroup[i].size(); j++)
		{
			int nIdx = (m_vecPathBlockGroup[i])[j].id;
			if(nIdx==nStartPoint)
			{
				bConnectPath=true;
			}
			if(bConnectPath)
			{
				(*vecPathBlock).push_back((m_vecPathBlockGroup[i])[j]);
			}
		}
	}
	if(bConnectPath) return true;
	else return false;
}

list<KuPose> KuPathBlockPr::generatePathList(vector<PBlock> vecPathBlockPos)
{
	list<KuPose> Pathlist;

	KuPose PathPose;

	for(int i=0; i<vecPathBlockPos.size();i++)
	{
		PathPose.setXm(vecPathBlockPos[i].x);
		PathPose.setYm(vecPathBlockPos[i].y);
		Pathlist.push_back(PathPose);
		if(i+1<vecPathBlockPos.size()){
			PathPose.setXm((vecPathBlockPos[i].x+vecPathBlockPos[i+1].x)/2.0);
			PathPose.setYm((vecPathBlockPos[i].y+vecPathBlockPos[i+1].y)/2.0);
			Pathlist.push_back(PathPose);
		}
	}

	return Pathlist;
}

vector<list<KuPose>> KuPathBlockPr::getPathlist()
{
	return m_vecPathlist;
}

vector<PBlock>  KuPathBlockPr::loadPathBlock(string strDataPath )
{
	vector<PBlock> vecPathBlockPos;
	PBlock PB;

	double x, y;
	int id;
	int pathdata;
	double sizex,sizey;
	int velocity;
	int task;
	int state;
	bool check;
	bool overlap;
	LinePoint dist1;
	LinePoint dist2;
	int waypoint;
	bool detectObstacle;
	int TaskDataNum; 
	int TaskData; 

	ifstream DataLog;
	DataLog.open(strDataPath);

	if( !DataLog.is_open() )
	{
		return vecPathBlockPos;
	}

	while(!DataLog.eof()){
		DataLog >> PB.id >> PB.block_id_for_ISSAC >> PB.pathdata >> PB.x >> PB.y;
		DataLog >> PB.sizex >> PB.sizey >> PB.velocity >> PB.task;
		DataLog >> PB.state >> PB.nRouteOrder >> PB.check >> PB.overlap >> PB.waypoint;
		DataLog >> PB.dist1.start_x >> PB.dist1.start_y >> PB.dist1.end_x >> PB.dist1.end_y;
		DataLog >> PB.dist2.start_x >> PB.dist2.start_y >> PB.dist2.end_x >> PB.dist2.end_y;
		DataLog >> PB.detectObstacle;
		DataLog >> TaskDataNum;
		PB.TaskList.clear();
		for(int i=0; i<TaskDataNum;i++)
		{
			DataLog >>TaskData;
			PB.TaskList.push_back(TaskData);
		}
		vecPathBlockPos.push_back(PB);
	}

	if(vecPathBlockPos.size()>1)
		vecPathBlockPos.pop_back();
	DataLog.close();
/*
	if(KuRobotParameter::getInstance()->getUsingISSAC() == 1) // 0: ISSAC 미사용, 1: ISSAC 사용
	{
		vecPathBlockPos.push_back(vecPathBlockPos[0]); // Starting point로 귀환하기 위하여 처음 block을 마지막에 또 추가해줌
	}
*/
	return vecPathBlockPos;
}


void  KuPathBlockPr::savePathBlock(vector<PBlock> vecPathBlockPos,string strDataPath )
{
	ofstream DataLog;
	DataLog.open(strDataPath, ios::out);

	for(int i=0; i<vecPathBlockPos.size() ; i++){
		vecPathBlockPos[i].id=i;
		DataLog<<vecPathBlockPos[i].id<<" ";

		if(vecPathBlockPos[i].block_id_for_ISSAC != "")
		{
			DataLog<<vecPathBlockPos[i].block_id_for_ISSAC<<" ";
		}
		else
		{
			DataLog<< 0 << " ";
		}
		
		DataLog<<vecPathBlockPos[i].pathdata<<" ";
		DataLog<<vecPathBlockPos[i].x<<" ";
		DataLog<<vecPathBlockPos[i].y<<" ";
		DataLog<<vecPathBlockPos[i].sizex<<" ";
		DataLog<<vecPathBlockPos[i].sizey<<" ";
		DataLog<<vecPathBlockPos[i].velocity<<" ";
		DataLog<<vecPathBlockPos[i].task<<" ";
		DataLog<<vecPathBlockPos[i].state<<" ";
		DataLog<<vecPathBlockPos[i].nRouteOrder<<" ";
		DataLog<<(vecPathBlockPos[i].check ? 1:0)<<" ";
		DataLog<<(vecPathBlockPos[i].overlap ? 1:0)<<" ";
		DataLog<<vecPathBlockPos[i].waypoint<<" ";
		DataLog<<vecPathBlockPos[i].dist1.start_x<<" ";
		DataLog<<vecPathBlockPos[i].dist1.start_y<<" ";
		DataLog<<vecPathBlockPos[i].dist1.end_x<<" ";
		DataLog<<vecPathBlockPos[i].dist1.end_y<<" ";
		DataLog<<vecPathBlockPos[i].dist2.start_x<<" ";
		DataLog<<vecPathBlockPos[i].dist2.start_y<<" ";
		DataLog<<vecPathBlockPos[i].dist2.end_x<<" ";
		DataLog<<vecPathBlockPos[i].dist2.end_y<<" ";
		DataLog<<(vecPathBlockPos[i].detectObstacle ? 1:0)<<" ";
		DataLog<<vecPathBlockPos[i].TaskList.size()<<" ";

		for(int j=0; j<vecPathBlockPos[i].TaskList.size();j++)
		{
			DataLog<<vecPathBlockPos[i].TaskList[j]<<" ";
		}
		DataLog<<endl;
	}

	DataLog.close();	
}

void KuPathBlockPr::loadGraph(vector<PBlock> vecTotalPathBlock, double** dGraph)
{
	//초기화 : 연결되지 않은 노드의 가중치는 무한대
	for (int i=0; i<vecTotalPathBlock.size(); ++i) {
		for (int j=0; j<vecTotalPathBlock.size(); ++j) { 
			dGraph[i][j] = DBL_MAX/2;
		}
	}

	//가중치 부여

	for (int i=0; i<vecTotalPathBlock.size(); ++i) {


		for (int j=0; j<vecTotalPathBlock.size(); ++j) { 

			if(checkBlocksNear(vecTotalPathBlock[i],vecTotalPathBlock[j]))//두 블락이 인접해 있으면
			{
				double dDistX = vecTotalPathBlock[i].x-vecTotalPathBlock[j].x;
				double dDIstY = vecTotalPathBlock[i].y-vecTotalPathBlock[j].y;
				double dDist = sqrt(dDistX*dDistX+dDIstY*dDIstY);
				dGraph[i][j] = dDist;//두 블락의 거리를 가중치로
			}
		}
	}
}

bool KuPathBlockPr::checkBlocksNear(PBlock Block1, PBlock Block2)
{
	bool bNear=false;

	//Block1
	KuPose EdgePose;
	vector<KuPose> vecEdgePose;

	EdgePose.setXm(Block1.x+Block1.dist1.start_x);
	EdgePose.setYm(Block1.y+Block1.dist1.start_y);
	vecEdgePose.push_back(EdgePose);

	EdgePose.setXm(Block1.x+Block1.dist1.end_x);
	EdgePose.setYm(Block1.y+Block1.dist1.end_y);
	vecEdgePose.push_back(EdgePose);

	EdgePose.setXm(Block1.x+Block1.dist2.start_x);
	EdgePose.setYm(Block1.y+Block1.dist2.start_y);
	vecEdgePose.push_back(EdgePose);

	EdgePose.setXm(Block1.x+Block1.dist2.end_x);
	EdgePose.setYm(Block1.y+Block1.dist2.end_y);
	vecEdgePose.push_back(EdgePose);

	//Block2
	KuPose EdgePose2;
	vector<KuPose> vecEdgePose2;

	EdgePose2.setXm(Block2.x+Block2.dist1.start_x);
	EdgePose2.setYm(Block2.y+Block2.dist1.start_y);
	vecEdgePose2.push_back(EdgePose2);

	EdgePose2.setXm(Block2.x+Block2.dist1.end_x);
	EdgePose2.setYm(Block2.y+Block2.dist1.end_y);
	vecEdgePose2.push_back(EdgePose2);

	EdgePose2.setXm(Block2.x+Block2.dist2.start_x);
	EdgePose2.setYm(Block2.y+Block2.dist2.start_y);
	vecEdgePose2.push_back(EdgePose2);

	EdgePose2.setXm(Block2.x+Block2.dist2.end_x);
	EdgePose2.setYm(Block2.y+Block2.dist2.end_y);
	vecEdgePose2.push_back(EdgePose2);

	for(int i=0; i<vecEdgePose.size(); i++)
	{
		for(int j=0; j<vecEdgePose2.size(); j++)
		{
			if(fabs(vecEdgePose[i].getXm()-vecEdgePose2[j].getXm())<0.1&&fabs(vecEdgePose[i].getYm()-vecEdgePose2[j].getYm())<0.1)
			{
				bNear = true;
				return true;
			}
		}
	}
	return bNear;
}

void KuPathBlockPr::findShortestPath(int v, double** dGraph, double* dDist, bool* bFound, int* nPreVertex, int n)
{
	// init
	for (int i=0; i<n; ++i) {
		dDist[i] = dGraph[v][i];
		bFound[i] = false;
		if ( dDist[i] > 0 && dDist[i] < DBL_MAX/2) nPreVertex[i] = v;
		else nPreVertex[i] = -1;
	}

	//자기 자신은 제외
	dDist[v] = 0;
	bFound[v] = true;

	for (int i=0; i<n-1; ++i) {
		int u = choose(dDist, bFound, n);	//가장 가까운 정점을 찾는다.
		bFound[u] = true;


		for (int w=0; w<n; ++w) {		// 최단 경로상의 정점들로부터 인접정점들까지의 모든 distance를 계산한다.
			if (bFound[w]) continue;
			if (dDist[u] + dGraph[u][w] >= dDist[w]) continue;


			dDist[w] = dDist[u] + dGraph[u][w];
			nPreVertex[w] = u;
		}
	}
}

int KuPathBlockPr::choose(double* dDist, bool* bFound, int n)
{ 
	int i;
	double dMin = DBL_MAX/2;
	int minpos = -1;


	for (i=0; i<n; ++i) { 
		if (bFound[i]) continue;
		if (dDist[i] >=  dMin) continue;

		dMin = dDist[i];
		minpos = i;
	}
	return minpos;
}

void KuPathBlockPr::getPathIdx(vector<PBlock> vecPathBlockPos, vector<int> vecnStartViasEndPoint, vector<int>& vecnGlobalPath)
{
	//start -> via, via->via, via->end point 최단경로
	int nGraphNum = vecPathBlockPos.size();
	double** dGraph;
	double* dDist;
	bool* bFound;
	int* nPreVertex;
	int v;

	dGraph = new double*[nGraphNum];
	for(int i=0; i<nGraphNum; i++)
	{
		dGraph[i] = new double[nGraphNum];
	}

	dDist = new double[nGraphNum];
	bFound = new bool[nGraphNum];
	nPreVertex = new int[nGraphNum];

	loadGraph(vecPathBlockPos, dGraph);//graph 정보 저장


	for(int i=0; i<vecnStartViasEndPoint.size()-1; i++)
	{
		//국소 최단경로 탐색
		findShortestPath(vecnStartViasEndPoint[i], dGraph, dDist, bFound, nPreVertex, nGraphNum);//error
		int nPreBlockIdx = vecnStartViasEndPoint[i+1];//국소 end point
		vector<int> nReversePathOrder, nPathOrder;
		nReversePathOrder.push_back(nPreBlockIdx);// 국소 끝점
		int nCnt=0;
		while(!(nPreBlockIdx==vecnStartViasEndPoint[i]))//국소 시작점까지 안온경우
		{
			nPreBlockIdx = nPreVertex[nPreBlockIdx];
			nCnt++;
			if(nCnt>100000)
			{
				break;//무한루프에 빠진경우 강제종료
			}
			if(nPreBlockIdx==vecnStartViasEndPoint[i])
			{
				break;//start포인트는 저장 안함 : 겹치는 경우를 막기 위해
			}
			nReversePathOrder.push_back(nPreBlockIdx);//국소 경로(역방향)
		}
		for(int j=0; j<nReversePathOrder.size(); j++)
		{
			vecnGlobalPath.push_back(nReversePathOrder[nReversePathOrder.size()-1-j]);//국소 경로(정방향)
		}
	}
	vecnGlobalPath.push_back(vecnStartViasEndPoint[0]);//시작블락은 추가

	for(int i=0; i<nGraphNum; i++)
	{
		delete[] dGraph[i];
	}
	delete[] dGraph;
	delete[] dDist;
	delete[] bFound;
	delete[] nPreVertex;
}

void KuPathBlockPr::setBlockSize(double dSizeX, double dSizeY)
{
	m_dBlockSizeX = dSizeX;
	m_dBlockSizeY = dSizeY;
}

void KuPathBlockPr::getBlockSize(double& dSizeX, double& dSizeY)
{
	dSizeX = m_dBlockSizeX;
	dSizeY = m_dBlockSizeY;
}
