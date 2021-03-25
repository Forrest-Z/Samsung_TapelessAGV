/*______________________________________________________________________________________________
PROJECT : Intelligent Robotics Lab. Department of Mechanical Engineering Korea University                                    
(c) Copyright 2012 KUME Intelligent Robotics Lab.                                             
All rights reserved                                                                         
$Directed by Jae-Bok Song                                                                     
$Designed by Joong-Tae Park                                                                  
$Description : smart arrary등 각종 프로그램밍에 유리한 기능들을 제공해준다. 
$Created on: 2012. 5. 1.                                                                        
$Author: Joong-Tae Park      
$e-mail: jtpark1114@gmail.com
______________________________________________________________________________________________*/
#ifndef KUNS_UTIL_H_
#define KUNS_UTIL_H_

#include <iostream>
#include <vector>
using namespace std;

typedef vector <bool> bool_1DArray;
typedef vector < vector <bool> > bool_2DArray;

typedef vector <int> int_1DArray;
typedef vector < vector <int> > int_2DArray;

typedef vector <float> float_1DArray;
typedef vector < vector <float> > float_2DArray;

typedef vector <double> double_1DArray;
typedef vector < vector <double> > double_2DArray;


class KuUtil
{
private:

public:
	KuUtil();
	~KuUtil();

	bool_1DArray generateBoolType1DArray(int nSize, bool bInitVal=true);
	bool_2DArray generateBoolType2DArray(int nRow, int nCol, bool bInitVal=true);

	int_1DArray generateIntType1DArray(int nSize, int nInitVal=0);
	int_2DArray generateIntType2DArray(int nRow, int nCol, int nInitVal=0);

	void releaseIntType1DArray(int_2DArray n2DArray);

	float_1DArray generateFloatType1DArray(int nSize, float fInitVal=0.);
	float_2DArray generateFloatType2DArray(int nRow, int nCol, float fInitVal=0.);

	double_1DArray generateDoubleType1DArray(int nSize, double dInitVal=0.);
	double_2DArray generateDoubleType2DArray(int nRow, int nCol, double dInitVal=0.);


};

#endif /* KUNS_UTIL_H_ */
