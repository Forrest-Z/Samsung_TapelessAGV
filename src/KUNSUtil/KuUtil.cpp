#include "stdafx.h"
#include "KuUtil.h"

KuUtil::KuUtil()
{

}

KuUtil::~KuUtil()
{

}

bool_1DArray KuUtil::generateBoolType1DArray(int nSize, bool bInitVal)
{
	vector<bool> bArray(nSize,bInitVal);
	return bArray;
}

bool_2DArray KuUtil::generateBoolType2DArray(int nRow, int nCol, bool bInitVal)
{
	vector<bool> vec(nCol,bInitVal);
	bool_2DArray bArray(nRow, vec);
	return bArray;
}

int_1DArray KuUtil::generateIntType1DArray(int nSize, int nInitVal)
{
	vector<int> nArray(nSize,nInitVal);
	return nArray;

}

int_2DArray KuUtil::generateIntType2DArray(int nRow, int nCol, int nInitVal)
{
	vector<int> vec(nCol,nInitVal);
	int_2DArray nArray(nRow, vec);
	return nArray;
}

void KuUtil::releaseIntType1DArray(int_2DArray n2DArray)
{
	int_2DArray nVoidArray;
	nVoidArray.swap(n2DArray);
}

float_1DArray KuUtil::generateFloatType1DArray(int nSize, float fInitVal)
{
	vector<float> fArray(nSize,fInitVal);
	return fArray;
}

float_2DArray KuUtil::generateFloatType2DArray(int nRow, int nCol, float fInitVal)
{
	vector<float> vec(nCol,fInitVal);
	float_2DArray fArray(nRow, vec);
	return fArray;
}

double_1DArray KuUtil::generateDoubleType1DArray(int nSize, double dInitVal)
{
	vector<double> dArray(nSize,dInitVal);
	return dArray;
}

double_2DArray KuUtil::generateDoubleType2DArray(int nRow, int nCol, double dInitVal)
{
	vector<double> vec(nCol,dInitVal);
	double_2DArray dArray(nRow, vec);
	return dArray;
}


