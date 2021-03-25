#ifndef SENSOR_H
#define SENSOR_H

#include <iostream>
using namespace std;

class Sensor
{
public:
	static const int URG04LX_DATA_NUM181 = 181;
	static const int KINECT_SENSOR_FOV = 56;
	static const int  KINECT_IMAGE_WIDTH= 640;
	static const int  KINECT_IMAGE_HEIGHT= 480;
	static const int  IMAGE_WIDTH =320; // Kinect
	static const int  IMAGE_HEIGHT= 240; // Kinect
	static const int  CEILING_IMAGE_WIDTH= 640;//320;
	static const int  CEILING_IMAGE_HEIGHT= 480;//240;
	static const int  CELLSIZE= 10;
	static const int SONAR_NUM = 4;
	
	static const int SAFE = 0;
	static const int WARNING = 1; //60CM
	static const int DANGER = 2; //30CM	


	Sensor();
	~Sensor();
};

#endif /*SENSOR_H*/

