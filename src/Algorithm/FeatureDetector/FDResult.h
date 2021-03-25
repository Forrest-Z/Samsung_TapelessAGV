#pragma once
#include <vector>

using namespace std;

class AFX_CLASS_EXPORT FDResult
{
public:

	typedef struct _tagFeaturePoint
	{
		int u, v;
		float r, th, area; // polar coordinate
		bool match;
	} FPoint;

	typedef struct _tagFLineData
	{
		float u1, v1; // first point of a detected line
		float u2, v2; // second point of a detected line
		float radian;
		bool associated;
		bool add;
		bool dead;
	} FLineData;

	/* Structures */
	typedef struct _tagFeatureData
	{
		int u, v; // center points
		int area; // area of blob
		float r, th; // polar coordinate
		int idx;
		int idx_fdresult;
		FPoint* node; // coordinates of nodes
		FPoint* areanode; // coordinates of nodes
		FLineData* linenode; // coordinates of nodes
		FPoint top_left; // area of ROI
		FPoint bottom_right; // area of ROI
		int node_num; // number of nodes
		int intensity; // average intensity
		bool img_boundary;
		bool associated;
		bool dead;
		bool add;
		bool unique;
		bool grouped;
		bool ori_exist;
		float associated_val;
		float pi;
		float axis1;
		float axis2;
		float ori_strength;
		float uniqueness;
	} FeatureData;

// 	typedef struct _tagRegionData
// 	{
// 		int u, v; // center points
// 		int area; // area of blob
// 		float th; // polar coordinate
// 		int width, height; // center points
// 		int idx;
// 		
// 	} FRegionData;


	typedef struct _tagFDataGroup
	{
		vector<FeatureData*> element;
		bool unique;
		bool similarity_prob_init;
		float uniqueness;
	} FDataGroup;

	int totalnum; // total number of raw features
	int totalnum_group; // total number of unique features and feature groups

	FeatureData* data;
//	FRegionData* RegionData;
	FDataGroup* data_group;
	bool m_bInitialized;
	float m_fTimeElapsed;
	int m_nMaxDBNumGroup;

	/* Functions */
	FDResult(void);
	~FDResult(void);
	void reset(void);
	void resetGroups(void);
	void init(int nDBNum, int nDBNumGroup = 0, int nMaxElements = 0, int nSimilarityRange = 0);
	int factorial(int nVal);


};

