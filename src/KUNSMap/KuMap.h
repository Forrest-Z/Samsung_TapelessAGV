/*______________________________________________________________________________________________
PROJECT : Intelligent Robotics Lab. Department of Mecanical Engineering Korea University                                    
(c) Copyright 2007 KUME Intelligent Robotics Lab.                                             
All rights reserved                                                                         
$Directed by Jae-Bok Song                                                                     
$Designed by Joong-Tae Park                                                                  
$Description : 모든 주행 behavior들을 담당하는 최고관리 component.
$Data: 2007/09                                                                           
$Author: Joong-Tae Park                                                                      
______________________________________________________________________________________________*/

#ifndef KuMap_H
#define KuMap_H



#include <iostream>
#include <cstring>
#include <cstdlib>

using namespace std; 
class KuMap
{
public:
	static const int EMPTY_AREA = 0;
	static const int OCCUPIED_AREA = 1;
	static const int UNKNOWN_AREA = 2;
	static const int COBSTACLE_AREA = 3;
	static const int GIVEN_PATH_AREA = 4;
	static const int VIRTUAL_WALL_AREA = 5;
	static const int GRAVITATION_AREA =6;
	static const int BOTTLENECK_WARNING_AREA = 7;
	static const int BOTTLENECK_AREA = 8;
	static const int CORNER_AREA = 9;
	static const int WARNING_AREA = 10;


	static const int NOMALVELOCITY = 100;
	static const int FIRDECEL= 101;
	static const int SECDECEL = 102;
	static const int THIDECEL = 103;
	static const int FOUDECEL = 104;
	static const int FIFDECEL = 105;

	static const int FIXED_CAD_AREA = 200;
	static const int TEMP_CAD_AREA = 201;
	static const int DYNAMIC_ENVIRONMENT_AREA = 202;
	static const int LUGGAGE_AREA = 203;

private:
	int m_nSizeX;
	int m_nSizeY;   
	int **m_nMap; 
	double **m_dProbabilityMap;



public:

	void setX(int nX);
	void setY(int nY);
	int getX();
	int getY();
	int** getMap();
	double** getProbMap();
	void setMap(int ** nMap);
	void setProbMap(double** pProbMap);

	KuMap(int nX, int nY);
	~KuMap();



};

#endif
