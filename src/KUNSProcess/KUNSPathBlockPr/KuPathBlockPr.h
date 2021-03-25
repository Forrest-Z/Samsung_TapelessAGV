#ifndef KUNS_PATHBLOCK_PROC_H
#define KUNS_PATHBLOCK_PROC_H

#include <cmath>
#include <vector>
#include <list>
#include <fstream>
#include "../../KUNSPose/KuPose.h"
#include "../../Algorithm/PathBlock/PathBlock.h"
#include "../../MobileSupervisor/KuRobotParameter.h"


using namespace std;

class KuPathBlockPr
{
private:
	vector<PBlock>m_vecPathBlockPos;
	vector<list<KuPose>> m_vecPathlist;
	vector<vector<PBlock>> m_vecPathBlockGroup;

private:
	double m_dBlockSizeX;
	double m_dBlockSizeY;

private:
	PBlock Path1();
	PBlock Path2();
	PBlock Path3();
	PBlock Path4();
	PBlock Path5();
	PBlock Path6();
	PBlock Path7();
	PBlock Path8();
	PBlock Path10();
	PBlock Path11();
	PBlock Path12();
	PBlock Path13();
	PBlock Path20();
	PBlock Path21();
	PBlock Path22();
	PBlock Path23();
	PBlock Path24();
	PBlock Path30();
	PBlock Path31();
	PBlock Path32();
	PBlock Path33();
	PBlock Path40();
	PBlock Path41();
	PBlock Path42();
	PBlock Path43();


public:
	PBlock getPathBlock(int nPathID, bool bInit = true);
	void  setPathBlockPos(vector<PBlock> vecPathBlockPos);
	void  setPathBlockPosForISSAC(vector<PBlock> vecPathBlockPos);
	vector<list<KuPose>> getPathlist();
	void  savePathBlock(vector<PBlock> vecPathBlockPos,string strDataPath );
	vector<PBlock> loadPathBlock(string strFilePath);
	bool checkPathBlock(KuPose PathPose1,KuPose PathPose2,int Pathdata);
	bool generatePathBlockConnection(vector<PBlock> vecTotalPathBlock,int nStartPoint,vector<PBlock> *vecPathBlock);
	list<KuPose> generatePathList(vector<PBlock> vecPathBlockPos);

	void getPathIdx(vector<PBlock> vecPathBlockPos, vector<int> vecnStartViasEndPoint, vector<int>& vecnGlobalPath);
	void loadGraph(vector<PBlock> vecTotalPathBlock, double** dGraph);
	void findShortestPath(int v, double** dGraph, double* dDist, bool* bFound, int* nPreVertex, int n); 
	int choose(double* distance, bool* found, int n);
	bool checkBlocksNear(PBlock Block1, PBlock Block2);
	void setBlockSize(double dSizeX, double dSizeY);
	void getBlockSize(double& dSizeX, double& dSizeY);

	KuPathBlockPr();
	~KuPathBlockPr();
};

#endif 

