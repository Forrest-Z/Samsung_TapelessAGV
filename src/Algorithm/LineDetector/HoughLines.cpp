#include "StdAfx.h"
#include "HoughLines.h"
#include <opencv/cv.h>

using namespace cv;

CHoughLines::CHoughLines(void)
{
	m_context.db_max_num =1000;
	m_ldresult.init(m_context.db_max_num);
}


CHoughLines::~CHoughLines(void)
{
}


// Reads EKF parameters from ini file
// void CHoughLines::readINI(std::string path)
// {
// //	KuINIReadWriter ini(path);
// 
// 	// read parameters from ini file
// // 	m_context.db_max_num = ini.getIntValue("Line Detector", "db_max_num", 0);
// // 
// // 	// check
// // 	if(m_context.db_max_num == 0)
// // 	{
// // 		AfxMessageBox(_T("[CHoughLines] Failed to load the ini file!"));
// // 	}
// }


CLDResult* CHoughLines::detect(Mat matImgSrc)
{
	// detect ===============================================
	//Mat matImgSrc(img.height, img.width, CV_8UC1, img.imageData);
	Mat matImgDst;

	Canny(matImgSrc, matImgDst, 150, 300, 3);

	vector<Vec4i> lines;
	
	HoughLinesP(matImgDst, lines, 1, CV_PI/180, 80, 80, 15); // 60, 30, 10

	// add to line detection result ================================
	CLDResult::LineData data_temp;

	m_ldresult.reset(); // delete previous results

	for(int i = 0; i < lines.size(); i++)
	{
		data_temp.u1 = lines[i][0];
		data_temp.v1 = lines[i][1];
		data_temp.u2 = lines[i][2];
		data_temp.v2 = lines[i][3];
		data_temp.associated = false;
		data_temp.add = true;
		data_temp.dead = false;

		m_ldresult.data.push_back(data_temp);
	}
	 
	// for display
// 	Mat color_dst;
// 
// 	cvtColor(matImgDst, color_dst, CV_GRAY2BGR);
// 
// 	for(size_t i = 0; i < lines.size(); i++)
// 	{
// 		line(color_dst, Point(lines[i][0], lines[i][1]), Point(lines[i][2], lines[i][3]), Scalar(0, 0, 255), 1, 8);
// 	}
// 
// 	namedWindow("Detected Lines", 1);
// 	imshow("Detected Lines", color_dst);
// 
// 	waitKey(10);


	return &m_ldresult;
}


CLDResult* CHoughLines::getDetectionResult(void)
{
	return &m_ldresult;
}
