#pragma once

#include <vector>

using namespace std;

class AFX_CLASS_EXPORT CLDResult
{
public:
	/* Structures */
	typedef struct _tagLineData
	{
		float u1, v1; // first point of a detected line
		float u2, v2; // second point of a detected line
		float radian;
		bool associated;
		bool add;
		bool dead;
	} LineData;

	/* Functions */
	CLDResult(void);
	~CLDResult(void);
	void init(int nDBNum);
	void reset(void);

	/* Variables */
	vector<LineData> data;
};

