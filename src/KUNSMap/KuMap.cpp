#include "stdafx.h"
#include "KuMap.h"

KuMap::KuMap(int nX, int nY)
{
	cout<<"[KuMap] : KuMap Instance is created!!!"<<endl;
	m_nSizeX = nX;
	m_nSizeY = nY;

	//============================================
	// 메모리 공간 할당
	//============================================
	m_nMap = new int*[nX];

	if(m_nMap){
		for(int i = 0 ; i < nX ; i++){
			m_nMap[i] = new int[nY];

		}
	}

	m_dProbabilityMap = new double*[nX];

	if(m_dProbabilityMap){
		for(int i=0; i <nX; i++){
			m_dProbabilityMap[i] = new double[nY];

		}
	}

	for(int i=0;i<m_nSizeX; i++){
		for(int j=0; j<m_nSizeY; j++){
			m_nMap[i][j] = 0;
			m_dProbabilityMap[i][j]=0;
		}
	}

	cout<<"[KuMap] : KuMap Instance creation is completed!!!"<<endl;	     
}

KuMap::~KuMap()
{
	if(m_nMap){
		for(int i = 0 ; i < m_nSizeX ; i++){
			delete[] m_nMap[i];
			m_nMap[i] = NULL;


		}
		delete[] m_nMap;
	}

	if(m_dProbabilityMap){
		for(int i = 0 ; i < m_nSizeX ; i++){
			delete[] m_dProbabilityMap[i];
			m_dProbabilityMap[i] = NULL;


		}
		delete[] m_dProbabilityMap;
	}

	cout<<"[KuMap] : Instance is destroyed."<<endl;
}

void KuMap::setX(int nX)
{ 
	m_nSizeX = nX; 
}
void KuMap::setY(int nY)
{ 
	m_nSizeY = nY; 
}
int KuMap::getX()
{ 
	return m_nSizeX; 
}
int KuMap::getY()
{ 
	return m_nSizeY; 
}

int** KuMap::getMap()
{
	return m_nMap; 
}

double** KuMap::getProbMap()
{
	return m_dProbabilityMap; 
}

void KuMap::setMap(int ** nMap)
{
	m_nMap=nMap;
}

void KuMap::setProbMap(double** pProbMap)
{
	// Release
/*
	if(m_dProbabilityMap)
	{
		for(int i = 0 ; i < m_nSizeX ; i++)
		{
			delete [] m_dProbabilityMap[i];
			m_dProbabilityMap[i] = NULL;
		}

		delete [] m_dProbabilityMap;
	}
*/

	m_dProbabilityMap = pProbMap;
}
