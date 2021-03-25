#ifndef C_LASER_MAPBUIDER_H
#define C_LASER_MAPBUIDER_H

#include <iostream> 
#include <list>
#include "../../KUNSPose/KuPose.h"
#include "../../KUNSMap/KuMap.h"
#include "../../KUNSMath/KuMath.h"
#include "../../KUNSUtil/KuUtil.h"
#include "KuMapBuilderParameter.h"
#include "../../Sensor/Sensor.h"

using namespace std;
class KuLaserMapBuilderPr 
{
private:
	KuUtil m_KuUtil;

private:
	int m_nScanIdX;
	int m_nLaserMinDist, m_nLaserMaXDist;
	int_1DArray m_nLaserData;
	KuPose* m_LaserscannerConfigurationFront;
	KuPose* m_LaserscannerConfigurationRear;
	int m_nMapSizeX, m_nMapSizeY;
	
	int m_nCellSize;	//1cell 100mm;
	double MIN_PROBABILITY;// ���� ���� ���� �ּ� Ȯ�� ��
	double MAX_PROBABILITY;// ���� ���� ���� �ִ� Ȯ�� ��
	double INITIAL_PROBABILITY;// �ʱ� Ȯ��(unknown region) : 0.5
	double GAUSSIAN_SD;//���� ���� ����þ� ǥ������ ��, ���� ������ ���� ����
	double m_dThicknessofWall;// ���� �β� mm
	double m_dRadiusofRobot;//�κ������� 400(mm)

	double** m_dProMap; // Ȯ�� ���� ����
	int** m_nMap;
	int** m_nRefMap;

	KuMap* m_pMap; // ���������ۼ��� ���� ������ ������ �����ϱ� ���� ����.
	KuMath m_Math;

private:
	void buildGridmapByBayesUpdate(KuPose RobotPos, int_1DArray nLaserData, int nLaserIdx, KuPose* pSensorConfig, bool bupdateSpeedflag);//������꿡 ���� Ȯ���� ������Ʈ �Լ�

public:
	void initialize(KuMapBuilderParameter InputParamFront, KuMapBuilderParameter InputParamRear);
	void buildMapFront(KuMapBuilderParameter InputParam);
	void buildMapRear(KuMapBuilderParameter InputParam);
	double** getProMap(); //Ȯ�� �ʵ����͸� �������� �Լ�
	int** getMap(); //�ʵ����͸� �������� �Լ�
	void setSigma(double dSigma);//���� ���� ����þ� ǥ������ ���� �����ϴ� �Լ�
	KuMap* getpMap();
	void initMap();
	void setReferenceCADMap(int** nMap );
	
	KuLaserMapBuilderPr(void);
	~KuLaserMapBuilderPr(void);
		
};
#endif