#ifndef C_SAMPLE_H
#define C_SAMPLE_H
// ������ ��ġ �� Ȯ���� ����

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

struct Accumulation{	// ���� ������� ���� Ȯ���� ������ ���Ǵ� ����
	double dSum;
	double x;
	double y;
	double t;
};

#endif