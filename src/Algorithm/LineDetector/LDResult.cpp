#include "StdAfx.h"
#include "LDResult.h"


CLDResult::CLDResult(void)
{
}


CLDResult::~CLDResult(void)
{
	data.clear();
}


void CLDResult::init(int nDBNum)
{
	data.reserve(nDBNum);
	data.clear();
}


void CLDResult::reset(void)
{
	data.clear();
}
