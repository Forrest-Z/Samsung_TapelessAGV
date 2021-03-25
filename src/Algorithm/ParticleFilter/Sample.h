#ifndef C_SAMPLE_H
#define C_SAMPLE_H
// 샘플의 위치 및 확률값 저장

struct Sample{
	double x;
	double y;
	double t;
	double dPro;
	double dRangeUpdatePro;
	double dVisionUpdataPro;
	double dLineInfoUpdatePro;
	double dMatchingError;	    
};

struct Accumulation{	// 샘플 재분포를 위한 확률값 누적에 사용되는 변수
	double dSum;
	double x;
	double y;
	double t;
};

#endif