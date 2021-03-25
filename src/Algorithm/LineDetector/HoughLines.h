#pragma once

//#include "SImage.h"
#include "LDResult.h"
//#include "KuINIReadWriter.h"
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/legacy/legacy.hpp>

using namespace cv;
class AFX_CLASS_EXPORT CHoughLines
{
public:
	/* Structures */
	typedef struct _tagLineContext
	{
		int db_max_num;
	} LineContext;

	/* Functions */
	CHoughLines(void);
	~CHoughLines(void);

	//void readINI(std::string path);
	//CLDResult* detect(const CSImage& img);
	CLDResult* detect(Mat matImgSrc);

	/* Variables */

private:
	/* Functions */

	/* Variables */
	CLDResult m_ldresult;
	LineContext m_context;
public:
	CLDResult* getDetectionResult(void);
};

