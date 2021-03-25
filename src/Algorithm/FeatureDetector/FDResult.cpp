#include "StdAfx.h"
#include "FDResult.h"


FDResult::FDResult(void)
: totalnum(0)
	, data(0)
	, data_group(0)
	, m_bInitialized(false)
	, m_fTimeElapsed(0)
	, totalnum_group(0)
	, m_nMaxDBNumGroup(0)
{
}


FDResult::~FDResult(void)
{
	if(!data)
		delete [] data;

	if(!data_group)
	{
		delete [] data_group;
	}
}


void FDResult::reset(void)
{
	for(int i = 0; i < totalnum; i++)
	{

		data[i].img_boundary = false;
		data[i].associated = false;
		data[i].dead = false;
		data[i].unique = false;
		data[i].grouped = false;
		data[i].node_num = 0;
		data[i].pi = 0;
		data[i].axis1 = 0;
		data[i].axis2 = 0;
		data[i].ori_exist = false;
		data[i].ori_strength = 0;
		data[i].uniqueness = 1;
	}

	totalnum = 0;
}

void FDResult::init(int nDBNum, int nDBNumGroup, int nMaxElements, int nSimilarityRange)
{
	if(!m_bInitialized)
	{
		int i, j;

		// raw features
		data = new FeatureData [nDBNum];

		for(i = 0; i < nDBNum; i++)
		{
			data[i].idx = 0;
			data[i].img_boundary = false;
			data[i].node_num = 0;
			data[i].node = 0; 
			data[i].areanode = 0; 
			data[i].linenode = 0; 
			data[i].associated = false;
			data[i].dead = false;
		}

		// feature groups
		if(nDBNumGroup)
		{
			data_group = new FDataGroup [nDBNumGroup];

			for(i = 0; i < nDBNumGroup; i++)
			{
				data_group[i].element.reserve(nMaxElements);
				data_group[i].element.clear();
				data_group[i].unique = false;
				data_group[i].similarity_prob_init = false;
				data_group[i].uniqueness = 1;
			}

			m_nMaxDBNumGroup = nDBNumGroup;
		}

		// set initialization flag as true
		m_bInitialized = true;
	}
}

void FDResult::resetGroups(void)
{
	for(int i = 0; i < totalnum_group; i++)
	{
		data_group[i].element.clear();
		data_group[i].unique = false;
		data_group[i].similarity_prob_init = false;
		data_group[i].uniqueness = 1;

	}

	totalnum_group = 0;
}

int FDResult::factorial(int nVal)
{
	int nRes(1);

	for(int i = 2; i <= nVal; i++)
	{
		nRes *= i;
	}

	return nRes;
}
