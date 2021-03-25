#include "StdAfx.h"
#include "FeatureDetector.h"

FREAK extractor(true, true, 45, 4, std::vector<int>());//45(40도)//72(90도)
FastFeatureDetector detector(30);

CFeatureDetector::CFeatureDetector(void)
{
	m_cvGrayMatImage.create(480,640,CV_8UC1);//Result image initialization
	m_cvMatImage.create(480,640,CV_8UC3);//Result image initialization
	m_IplCeilingImage= cvCreateImage(cvSize(640,480),8,3);
	m_storage = cvCreateMemStorage(0); 
	m_nDBNum=0;
	m_dWidth=100;
	m_dHeight=50;
}


CFeatureDetector::~CFeatureDetector(void)
{

	cvReleaseMemStorage(&m_storage);
	if(m_IplCeilingImage!=NULL)
		cvReleaseImage(&m_IplCeilingImage);
	m_cvMatImage.release();
	m_cvGrayMatImage.release();
}

void CFeatureDetector::init()
{
	m_fRegionresult.clear();
	m_vecFTData.clear();

}
void CFeatureDetector::setTampletData(vector<CFREAKData> FTData)
{
	m_vecFTData.clear();
	for(int i=0; i<FTData.size();i++)
	{
		if(FTData[i].add==true)
			m_vecFTData.push_back(FTData[i]);
	}

}

void CFeatureDetector::getEllipseData(double *dWidth,double *dHeight)
{
	(*dWidth)=m_dWidth ;
	(*dHeight)=m_dHeight ;
}
void CFeatureDetector::setEllipseData(double dWidth,double dHeight)
{
	m_dWidth= dWidth;
	m_dHeight= dHeight;
}

void CFeatureDetector::setRegion(vector<CLAMPData>  fRegionresult)
{
	m_fRegionresult.clear();
	int nMaxmatch=0;
	for(int i=0; i<fRegionresult.size();i++)
	{
		if(fRegionresult[i].add==true)
		{	
			m_fRegionresult.push_back(fRegionresult[i]);

			if(fRegionresult[i].nmatchnum>nMaxmatch)
			{
				nMaxmatch=fRegionresult[i].nmatchnum;
				m_dWidth=fRegionresult[i].width;
				m_dHeight=fRegionresult[i].height;

			}
		}
	}
	double dtemp;
	if(m_dHeight>m_dWidth)
	{
		dtemp=m_dHeight;
		m_dHeight=m_dWidth;
		m_dWidth=dtemp;
	}

}

void CFeatureDetector::trainingRegion(Mat matImgSrc)
{
	int nPixX=0;
	int nPixY=0;
	Mat matImgDst;

	for(int i = 0; i < 640* 480; i++)
	{
		m_cvGrayMatImage.data[i]=255;
		if(matImgSrc.data[i]<225){
			m_cvGrayMatImage.data[i]=0;
		}	
	}

	for(int i = 0; i < 640 * 480; i++)
	{
		m_IplCeilingImage->imageData[3*i]=m_cvGrayMatImage.data[i];
		m_IplCeilingImage->imageData[3*i+1]=m_cvGrayMatImage.data[i];
		m_IplCeilingImage->imageData[3*i+2]=m_cvGrayMatImage.data[i];

		m_cvMatImage.data[3*i]=255;
		m_cvMatImage.data[3*i+1]=255;
		m_cvMatImage.data[3*i+2]=255;
	}

	Mat  img, yuv;

	cvtColor(m_cvGrayMatImage, img, CV_GRAY2BGR);	
	cvtColor(img, yuv, COLOR_BGR2YCrCb);

	vector<vector<Point> > contours;
	MSER(5, 20, 14400,
		0.25, 0.2,
		1000, 1.002,
		0.001, 5 )(yuv, contours);

	CvBox2D box; //fitting된 타원 정보가 들어감
	CvPoint center;
	CvSize size;
	double dMaxWidth=0;

	for( int i = 0; i < (int)contours.size(); i++ )
	{
		const vector<Point>& r = contours[i];


		if(r.size()>3500){continue;}

		CvSeq* seq = cvCreateSeq(CV_32FC2, sizeof(CvSeq), sizeof(CvPoint2D32f), m_storage); 

		for ( int j = 0; j < (int)r.size(); j++ )
		{
			Point pt = r[j];
			//			img.at<Vec3b>(pt) = bcolor[i%9];
			m_cvMatImage.at<Vec3b>(pt) = bcolor[i%9];

			CvPoint2D32f p;     
			p.x = (float)pt.x;     
			p.y = (float)pt.y;     
			cvSeqPush(seq, &p); 
		}

		box = cvFitEllipse2(seq); 
		cvClearSeq(seq);
		center = cvPointFrom32f(box.center);
		size.width = cvRound(box.size.width*0.5);
		size.height = cvRound(box.size.height*0.5);	

		if(box.size.height<box.size.width&&box.size.height*1.5>box.size.width){	continue;}
		else if(box.size.width<box.size.height&&box.size.width*1.5>box.size.height){continue;}

		if(size.width/size.height>dMaxWidth||size.height/size.width>dMaxWidth)
		{
			if(m_dWidth/m_dHeight>dMaxWidth)dMaxWidth=m_dWidth/m_dHeight;
			if(m_dHeight/m_dWidth>dMaxWidth)dMaxWidth=m_dHeight/m_dWidth;
			m_dWidth=size.width ;
			m_dHeight=size.height;
		}

		//cvEllipse(m_IplCeilingImage, center, size, box.angle, 0, 360, CV_RGB(255,0,0), 2, CV_AA, 0); 	
	}

// 	cvShowImage( "original", m_IplCeilingImage );
// 	imshow( "response", img );
// 	imshow( "m_cvMatImage", m_cvMatImage );
// 	cvWaitKey(10);

	double dtemp;
	if(m_dHeight>m_dWidth)
	{
		dtemp=m_dHeight;
		m_dHeight=m_dWidth;
		m_dWidth=dtemp;
	}
}

vector<CLAMPData> CFeatureDetector::detectRegion(Mat matImgSrc,bool bmapping)
{
	m_brecData=false;	

	int nPixX=0;
	int nPixY=0;
	Mat matImgDst;

	for(int i = 0; i < 640* 480; i++)
	{
		m_IplCeilingImage->imageData[3*i]=matImgSrc.data[i];
		m_IplCeilingImage->imageData[3*i+1]=matImgSrc.data[i];
		m_IplCeilingImage->imageData[3*i+2]=matImgSrc.data[i];

	}
	Mat  img, yuv;

	for(int i = 0; i < 640* 480*3; i++)
	{
		m_cvMatImage.data[i]=255;
	}

	cvtColor(matImgSrc, img, CV_GRAY2BGR);	
	cvtColor(img, yuv, COLOR_BGR2YCrCb);


	vector<vector<Point> > contours;
	MSER(5, 20, 14400,
		0.25, 0.2,
		1000, 1.002,
		0.001, 5 )(yuv, contours);


	//! the full constructor
	// 	explicit MSER( int _delta=5, int _min_area=60, int _max_area=14400,
	// 		double _max_variation=0.25, double _min_diversity=.2,
	// 		int _max_evolution=200, double _area_threshold=1.01,
	// 		double _min_margin=0.003, int _edge_blur_size=5 );

	CvBox2D box; //fitting된 타원 정보가 들어감
	CvPoint center;
	CvSize size;

	CLAMPData RegionData;

	for( int i = 0; i < (int)contours.size(); i++ )
	{
		const vector<Point>& r = contours[i];

		if(r.size()>3500){continue;}

		CvSeq* seq = cvCreateSeq(CV_32FC2, sizeof(CvSeq), sizeof(CvPoint2D32f), m_storage); 

		for ( int j = 0; j < (int)r.size(); j++ )
		{
			Point pt = r[j];
			//			img.at<Vec3b>(pt) = bcolor[i%9];
			m_cvMatImage.at<Vec3b>(pt) = bcolor[i%9];

			CvPoint2D32f p;     
			p.x = (float)pt.x;     
			p.y = (float)pt.y;     
			cvSeqPush(seq, &p); 
		}

		box = cvFitEllipse2(seq); 
		cvClearSeq(seq);
		center = cvPointFrom32f(box.center);
		size.width = cvRound(box.size.width*0.5);
		size.height = cvRound(box.size.height*0.5);

		//if(bmapping)
		{
			if(box.size.height<box.size.width&&box.size.height*1.5>box.size.width){	RegionData.ntype=1;}
			else if(box.size.width<box.size.height&&box.size.width*1.5>box.size.height){RegionData.ntype=1;}
			else{RegionData.ntype=0;}
		}

		if(matImgSrc.data[center.x+center.y*640]==0) continue;

		RegionData.width=box.size.width*0.5;
		RegionData.height=box.size.height*0.5;
		RegionData.u=box.center.x;
		RegionData.v=box.center.y;
		if(RegionData.ntype==0){RegionData.angle= box.angle;}
		else if(RegionData.ntype==1) RegionData.angle= 0;
		RegionData.add= false;
		RegionData.area=(int)r.size();
		RegionData.detect=false;
		//cvEllipse(m_IplCeilingImage, center, size, box.angle, 0, 360, CV_RGB(255,0,0), 2, CV_AA, 0); 
		m_fRegionresult.push_back(RegionData);
	}

// 	imshow( "original", img );
// 	cvShowImage( "response", m_IplCeilingImage );
// 	imshow( "m_cvMatImage", m_cvMatImage );
// 	cvWaitKey(10);
	
	return m_fRegionresult;
}

/**
@brief Korean: 소요 시간을 측정하기 위해서 초기화하는 함수
@brief English: Initializes to count the duration
*/
void CFeatureDetector::startTimeCheck(LARGE_INTEGER& nStart)
{
	QueryPerformanceCounter(&nStart);
}
/**
@brief Korean: 측정된 소요 시간을 받아오는 함수
@brief English: Gets the estimated duration
*/
float CFeatureDetector::finishTimeCheck(const LARGE_INTEGER nStart)
{
	LARGE_INTEGER nFinish, freq;

	QueryPerformanceFrequency(&freq);

	QueryPerformanceCounter(&nFinish);

	return (float)(1000.0 * (nFinish.QuadPart - nStart.QuadPart) / freq.QuadPart);
}

vector<CFREAKData>  CFeatureDetector::checktempletRegion(Mat matImgSrc,bool bmapping)
{
	Mat PImage;	 
	cvtColor(matImgSrc, PImage, CV_GRAY2BGR);	

	vector<KeyPoint> keypointsA,keypointsB;
	vector<KeyPoint> keypoints;
	KeyPoint kp;
	Mat descriptorsA,descriptorsB;
	Mat matImgRevSrc;
	vector<DMatch> matches;

	detector.detect( matImgSrc, keypointsA );

	bool* bflag;
	int nSize=keypointsA.size();
	bflag = new bool [nSize];

	if(keypointsA.size()>0)
	{
		for(int k=0; k<keypointsA.size();k++)
		{
			bflag[k]=true;

			//if(bmapping==true)
			{
				for(int l=k; l<keypointsA.size();l++)
				{
					double dDist = _hypot((double)(keypointsA[k].pt.x-keypointsA[l].pt.x),(double)(keypointsA[k].pt.y-keypointsA[l].pt.y));
					if(dDist<15&&k!=l)
					{
						bflag[k]=false;
						bflag[l]=false;
					}
				}	
			}
		}	
	}

	for(int k=0; k<keypointsA.size();k++)
	{
		if(bflag[k]==true)
		{
			keypoints.push_back(keypointsA[k]);
		}
	}

	extractor.compute( matImgSrc, keypoints, descriptorsA );

	m_vecFTData.clear();	
	

	if(keypoints.size()>0)
	{
		CFREAKData FTData;

		for(int k=0; k<keypoints.size();k++)
		{
			//if(bflag[k]==true)
			{
				FTData.sourceImage=descriptorsA.row(k);
				FTData.u=keypoints[k].pt.x;
				FTData.v=keypoints[k].pt.y;
				FTData.angle=keypoints[k].angle;
				FTData.response=keypoints[k].response;
				FTData.imageSizeX=1;
				FTData.imageSizeY=descriptorsA.cols;
				FTData.add=false;
				FTData.nmatchnum=0;
				FTData.associate=false;
				m_vecFTData.push_back(FTData);
			}
		}	

// 		drawKeypoints(matImgSrc,keypoints,PImage,Scalar(255,0,0),DrawMatchesFlags::DRAW_RICH_KEYPOINTS);	 
// 		imshow("FREAK Features",PImage);
// 		cvWaitKey(10);
	}
	
	delete [] bflag;

	return m_vecFTData;

}


int CFeatureDetector::templateMatching(IplImage *sourceImage,IplImage *templateImage,double dthreshold,double dtheta, CvPoint* cvResultsMinPoint,CvPoint* cvResultsMaxPoint,double* dminVal)
{
	int i, j, x, y, key;

	int nState=-1;

	CvPoint minLoc;
	CvPoint tempLoc;
	bool bcheck= false;
	double dminValue;

	if (sourceImage == NULL) 
	{
		printf( "소스 영상이 발견되지 않습니다\n");
		return -1;
	}
	if(templateImage == NULL)
	{
		printf( "템플릿 영상이 발견되지 않습니다\n");
		return -1;
	}

	IplImage *graySourceImage = cvCreateImage(cvGetSize(sourceImage), IPL_DEPTH_8U, 1); 
	IplImage *grayTemplateImage = cvCreateImage(cvGetSize(templateImage), IPL_DEPTH_8U, 1);
	
	IplImage *binarySourceImage = cvCreateImage(cvGetSize(sourceImage), IPL_DEPTH_8U, 1); 
	IplImage *binaryTemplateImage = cvCreateImage(cvGetSize(templateImage), IPL_DEPTH_8U, 1); 
	
	cvCopy(sourceImage, graySourceImage);
	cvCopy(templateImage, grayTemplateImage);

	cvThreshold(graySourceImage, binarySourceImage, 100, 255, CV_THRESH_BINARY);
	cvThreshold(grayTemplateImage, binaryTemplateImage, 100, 255, CV_THRESH_BINARY);

// 	cvCanny(graySourceImage, binarySourceImage, 200, 200);
// 	cvCanny(grayTemplateImage, binaryTemplateImage, 200, 200);
// 	cvFlip(binarySourceImage,binarySourceImage,0);
// 	cvFlip(binaryTemplateImage,binaryTemplateImage,0);

	//cvResize(binaryTemplateImage, tempBinaryTemplateImage, CV_INTER_LINEAR);


	// 템플릿 매칭 수행부분
	int templateHeight = templateImage->height;
	int templateWidth = templateImage->width;
// 	int templateHeight = (int)(templateWidth );
// 	int templateWidth = (int)(templateHeight );

	IplImage *rotateBinaryTemplateImage = cvCreateImage(cvSize(templateWidth,templateHeight), IPL_DEPTH_8U, 1);
	IplImage *result = cvCreateImage(cvSize(sourceImage->width-templateWidth+1,sourceImage->height-templateHeight+ 1), IPL_DEPTH_32F, 1);

	//float degree = 20.0f;
	//for(j = 0; j <= 0; j++) // 회전(0도부터 180도까지 20도 간격으로 회전) => j * 20 = degree ,j 및 degree 값 수정하여 조정 가능
	{
		float radian = dtheta;

		double dCos=cos(radian);
		double dSin=sin(radian);


		// 템플릿 회전
		for(y = 0; y < templateHeight; y++)
		{
			for(x = 0; x < templateWidth; x++)
			{
				rotateBinaryTemplateImage->imageData[y * rotateBinaryTemplateImage->width + x] = 255;

				int scale = y * templateWidth + x;
				int rotateY = - dSin * ((float)x - (float)templateWidth / 2.0f) + dCos* ((float)y - (float)templateHeight / 2.0f) + templateHeight/2.0;
				int rotateX = dCos * ((float)x - (float)templateWidth / 2.0f) + dSin * ((float)y - (float)templateHeight / 2.0f) + templateWidth /2.0;

				if(rotateY > templateHeight-1 || rotateX > templateWidth-1 || rotateY < 1 || rotateX  < 1) continue;

				rotateBinaryTemplateImage->imageData[scale] = 	binaryTemplateImage->imageData[rotateY * templateWidth + rotateX];
			}
		}


		cvMatchTemplate(binarySourceImage, rotateBinaryTemplateImage, result, CV_TM_SQDIFF_NORMED); 
		cvMinMaxLoc(result, &dminValue, NULL, cvResultsMinPoint, cvResultsMaxPoint, NULL);
		
// 		cvRectangle(sourceImage, cvPoint((*cvResultsMinPoint).x, (*cvResultsMinPoint).y), cvPoint((*cvResultsMinPoint).x+tempTemplateWidth,  (*cvResultsMinPoint).y+tempTemplateHeight ), CV_RGB(255, 0, 0));
// 		cvShowImage("Source", sourceImage);
// 		 cvWaitKey(10);
		double dDegree = dtheta*57.3;
		double dSimm = (1 -  (dminValue)) * 100;
		printf("템플릿 회전각도 %f 의 최대 유사도 : %f\n",  dDegree, dSimm );    // 최대 유사도값 로그 출력

		//if(minVal < 0.065) // 1 - 0.065 = 0.935 : 93.5% 이상 유사한 경우 탐지하였다고 가정
		if( dminValue< dthreshold)
		{
			(*dminVal)=dminValue;
			nState=1;//matching success
		
			//break;
			cvRectangle(binarySourceImage, cvPoint((*cvResultsMinPoint).x, (*cvResultsMinPoint).y), cvPoint((*cvResultsMinPoint).x+templateWidth,  (*cvResultsMinPoint).y+templateHeight ), CV_RGB(255, 0, 0));
			cvShowImage("Source", binarySourceImage);
			cvShowImage("rotateBinaryTemplateImage", rotateBinaryTemplateImage);

			cvWaitKey(10);
		}
	}

	// 템플릿 매칭 수행부분 끝



	// 메모리를 해방한다
	cvReleaseImage(&graySourceImage);
	cvReleaseImage(&grayTemplateImage);
	cvReleaseImage(&binarySourceImage);
	cvReleaseImage(&binaryTemplateImage);
	cvReleaseImage(&result);
	cvReleaseImage(&rotateBinaryTemplateImage);

	return nState;
	
}
