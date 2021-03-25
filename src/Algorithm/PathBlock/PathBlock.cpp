#include "stdafx.h"
#include "PathBlock.h"

PathBlock::PathBlock()
{

}

PathBlock::~PathBlock()
{

}
void PathBlock::init()
{
	PBlock PB;
	PB.x=0;
	PB.y=0;
	PB.id=-1;
	PB.block_id_for_ISSAC = "0";
	PB.velocity= 0;
	PB.task=-1;
	PB.waypoint=0;
	PB.state=0;
	PB.nRouteOrder=0;
	PB.pathdata=-1;
	PB.sizex=0.68;;
	PB.sizey=0.75;
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

	// ISSAC
	PB.block_id_for_ISSAC.clear();
}
