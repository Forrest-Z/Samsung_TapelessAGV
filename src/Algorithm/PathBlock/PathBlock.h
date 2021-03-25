#ifndef PATHBLOCK_H
#define PATHBLOCK_H

#include <vector>
#define  PATHBLOCKNUM 25

using namespace std;

static const int PathBlockIDNum[] =
{
	1,2,3,4,5,6,7,8,10,11,12,13,20,21,22,23,24,30,31,32,33,40,41,42,43
};

typedef struct _tagLinePoint
{
	double start_x;
	double start_y;
	double end_x;
	double end_y;
}LinePoint;

typedef struct _tagMotionData
{
	bool Pause;
	bool Resume;
	int Rotaion;

}MotionData;

typedef struct _tagBehaviorData
{
	int Sound;
	int Device;
	bool ReadytoSignal;
	bool TowerLamp;

} BehaviorData;

typedef struct _tagPathBlock
{
	double x, y;
	int id;
	int pathdata;
	double sizex,sizey;
	int velocity;
	int task;
	int state;
	int nRouteOrder;
	bool check;
	bool overlap;
	LinePoint dist1;
	LinePoint dist2;
	int waypoint;
	bool detectObstacle;
	std::vector<int> TaskList;
	// 	MotionData Motion;
	// 	BehaviorData Behavior;
	string block_id_for_ISSAC; // ISSAC에서 사용하는 block id

} PBlock;



class PathBlock
{
public:
	static const int WAYPOINT = 1;
	static const int START = 2;
	static const int GOAL  = 3;


	static const int MOTION_PAUSE =  3;
	static const int MOTION_RESUME = 4;

	static const int MOTION_ROTATION_LEFT90  = 11;
	static const int MOTION_ROTATION_LEFT180  = 12;
	static const int MOTION_ROTATION_RIGHT90  = 13;
	static const int MOTION_ROTATION_RIGHT180  = 14;

	static const int SOUND1  = 56;
	static const int SOUND2  = 57;
	static const int SOUND3  = 58;
	static const int SOUND4  = 59;

	static const int DEVICE1  = 61;
	static const int DEVICE2  = 62;
	static const int DEVICE3  = 63;
	static const int DEVICE4  = 64;
	static const int DEVICE5  = 65;

	static const int READYTOSIGNAL  = 5;
	static const int TOWERLAMP  = 51;

/*
	static const int MOTION_PAUSE = 0;
	static const int MOTION_RESUME =1;

	static const int MOTION_ROTATION_LEFT90  = 2;
	static const int MOTION_ROTATION_LEFT180  = 3;
	static const int MOTION_ROTATION_RIGHT90  = 4;
	static const int MOTION_ROTATION_RIGHT180  = 5;

	static const int SOUND1  = 6;
	static const int SOUND2  = 7;
	static const int SOUND3  = 8;
	static const int SOUND4  = 9;

	static const int DEVICE1  = 10;
	static const int DEVICE2  = 11;
	static const int DEVICE3  = 12;
	static const int DEVICE4  = 13;

	static const int READYTOSIGNAL  = 14;
	static const int TOWERLAMP  = 15;
*/


public:
	void init();
	PathBlock();
	~PathBlock();
};

#endif /*PathBlock*/

