#ifndef C_MAP_BUILDER_PARAMETER_H
#define C_MAP_BUILDER_PARAMETER_H

#include <iostream>
#include "../../KUNSPose/KuPose.h"
#include "../../KUNSMap/KuMap.h"
#include "../../KUNSMath/KuMath.h"
#include "../../KUNSUtil/KuUtil.h"

using namespace std;
class KuMapBuilderParameter
{

private:
	KuPose m_RobotPos, m_DelRobotPos;
	int m_nLaserScanIdx;
	int_1DArray m_nLaserData;
	int m_nMinDistofSensorData, m_nMaxDistofSensorData;
	int m_nMapSizeXm, m_nMapSizeYm;
	double m_dLaserOffset;
	bool m_bMapBuildingspeedflag;
	int m_nCellSize;
	double m_dThicknessofWall;
	double m_dRadiusofRobot;
	double m_dSigma;
	double m_dXOffset;

public:
	void setDelRobotPos(KuPose DelRobotPos);
	KuPose getDelRobotPos();

	void setRobotPos(KuPose RobotPos);
	KuPose getRobotPos();

	void setLaserData(int_1DArray nLaserData);
	int_1DArray getLaserData();
	
	void setLaserScanIdx(int nLaserScanIdx);
	int getLaserScanIdx();
	
	void setLaserXOffset(double dOffset);
	double getLaserXOffset();
	
	void setMinDistofSensorData(int nDist);
	void setMaxDistofSensorData(int nDist);


	int getMinDistofSensorData();
	int getMaxDistofSensorData();
	
	void setMapSizeXmYm(int nMapSizeXm, int nMapSizeYm);
	void getMapSizeXmYm(int* nMapSizeXm, int* nMapSizeYm);

	void setLaserUpdateSpeedflag(bool bMapBuildingspeedflag);
	bool getLaserUpdateSpeedflag();
	
	void setCellSize(int nCellSize);
	int getCellSize();

	void setThicknessofWall(double dThicknessofWall);
	double getThicknessofWall();

	void setRadiusofRobot(double dRadiusofRobot);
	double getRadiusofRobot();

	void setSigma(double dSigma);
	double getSigma();
		

	void setXOffset(double dXOffset);
	double getXOffset();



	KuMapBuilderParameter();
	virtual ~KuMapBuilderParameter();

};

#endif
