#include "stdafx.h"
#include "CeilingImagebasedParticleFilter.h"

CeilingImageBasedParticleFilter::CeilingImageBasedParticleFilter()
{
	m_nMaxSampleNum =2000;		// 샘플의 갯수의 최대값. set2DMode()에서도 바꿔줘야 함.
	m_nMinSampleNum =400;			// 샘플의 갯수의 최소값. set2DMode()에서도 바꿔줘야 함.
	m_nSampleNum = m_nMaxSampleNum;
	m_nMaxIteration=2000;
	m_nMinIteration=100;
	m_dSDThreshold= KuRobotParameter::getInstance()->getSDValue();

	m_bgenerateSample=false;
	m_bImageModel=false;

	m_stSample = new Sample[m_nMaxSampleNum];

	srand((unsigned)time(NULL)); //랜덤 씨드를 항상 바꿔주기 위해서 설정해야한다.

	m_nMinSampleStandardDeviation= 100;
	m_nSampleDensity=400;
	m_dSampleSD = m_nMinSampleStandardDeviation;
	m_dSampleSDX = m_nMinSampleStandardDeviation;
	m_dSampleSDY = m_nMinSampleStandardDeviation; 

	m_dfx = KuRobotParameter::getInstance()->getCeilingCameraPrameterFx();
	m_dfy = KuRobotParameter::getInstance()->getCeilingCameraPrameterFy();
	m_dcx = KuRobotParameter::getInstance()->getCeilingCameraPrameterCx();
	m_dcy = /*Sensor::CEILING_IMAGE_HEIGHT-*/KuRobotParameter::getInstance()->getCeilingCameraPrameterCy();

	m_nXMargin=640;
	m_nYMargin=480;
}

CeilingImageBasedParticleFilter::~CeilingImageBasedParticleFilter()
{
	m_bgenerateSample=false;
	m_bImageModel=false;

	m_bResetSamples = false;

	delete [] m_stSample;

}
/**
@brief Korean: 레이저 스캐너의 측정 거리에 따른 오차확률을 계산하는 함수
@brief English: Function calculating the probabilities of error according to the range of laser
*/
void CeilingImageBasedParticleFilter::initProbability()
{
	m_nMaxSampleNum =2000;		// 샘플의 갯수의 최대값. set2DMode()에서도 바꿔줘야 함.
	m_nMinSampleNum =400;			// 샘플의 갯수의 최소값. set2DMode()에서도 바꿔줘야 함.
	m_nSampleNum = m_nMaxSampleNum;

	m_bgenerateSample=false;
	m_bImageModel=false;

	m_nMinSampleStandardDeviation= 100;
	m_nSampleDensity=400;
	m_dSampleSD = m_nMinSampleStandardDeviation;
	m_dSampleSDX = m_nMinSampleStandardDeviation;
	m_dSampleSDY = m_nMinSampleStandardDeviation; 

	m_bImageModel=false;
}
/**
@brief Korean: 파티클 필터 기반 이미지 정합
@brief English: Function calculating the probabilities of error according to the range of laser
*/
double CeilingImageBasedParticleFilter::doImagebasedParticlefilter(Point2i ptGlobalPos, Mat matRetinexImg, Mat matGlobalImg, Point2i& ptCeilingCtrPoint, int nx, int ny)
{
	m_dGlobalMaxWidth=(double)matGlobalImg.cols;
	m_dGlobalMaxHeigh=(double)matGlobalImg.rows;
	
	Mat matRetinexInputImg;
	matRetinexInputImg.create(matRetinexImg.rows,matRetinexImg.cols,CV_8UC1);
	for(int i=0; i<matRetinexImg.cols*matRetinexImg.rows; i++)
	{
		matRetinexInputImg.data[i]=matRetinexImg.data[i];
	}

	Point2i ptPrePose;
	ptPrePose=ptGlobalPos;

	Point2i ptCeilingPoint, ptTempPoint;
	initProbability();
	setSamplesNearRobot(ptPrePose.x,ptPrePose.y,50,matGlobalImg);

	/////////////////
	double dDelTheta=0.0;
	vector<Mat> vec_matCeilingImg;
	for(double dDelta=-4.; dDelta<=4.; dDelta+=0.25)
	{
		Point2d ptPreImageCenter = Point2d(m_dcx,m_dcy);
		Mat center = getRotationMatrix2D(Point2f((float)ptPreImageCenter.x,(float)ptPreImageCenter.y),dDelta,1);
		Mat matTempImg;
		matTempImg.create(matRetinexInputImg.rows,matRetinexInputImg.cols,CV_8UC1);
		warpAffine(matRetinexInputImg,matTempImg,center,matRetinexInputImg.size());//천장영상
		vec_matCeilingImg.push_back(matTempImg);
	}
	/////////////////
	int nCt=0;
	int nTempX=0, nTempY=0;
	double dMinSD=DBL_MAX;
	while(nCt<m_nMaxIteration)
	{
		nCt++;
		if(doParticleFiltering(matGlobalImg,vec_matCeilingImg, nx, ny))
		{
			if(dMinSD>m_dSampleSD)
			{
				dMinSD=m_dSampleSD;
				ptTempPoint=ptCeilingPoint;
			}
		}
		else
		{
			ptTempPoint=ptCeilingPoint;
			if(nCt>m_nMinIteration)
			{
				nCt=m_nMaxIteration;
			}
		}
	}

	double dMaxPro=0;
	int nMaxIdx=-1;
	for(int i=0; i<m_nSampleNum; i++)
	{
		if(m_stSample[i].dPro>dMaxPro) 
		{
			dMaxPro = m_stSample[i].dPro;
			nMaxIdx = i;
		}
	}
	if(nMaxIdx>=0)
	{
		ptCeilingCtrPoint.x = m_stSample[nMaxIdx].x;
		ptCeilingCtrPoint.y = m_stSample[nMaxIdx].y;printf("u: %f, v: %f\n",ptCeilingCtrPoint.x,ptCeilingCtrPoint.y);
		dDelTheta = (m_stSample[nMaxIdx].t*0.25)-4.0;
	}

	return dDelTheta;
}


/**
@brief Korean: MCL을 수행하는 함수
@brief English: Function performing MCL
*/
bool CeilingImageBasedParticleFilter::doParticleFiltering(Mat matGlobalImg, vector<Mat> vec_matCeilingImg, int nx, int ny)
{
	bool bModelResult;	// Motion Model을 수행하여 샘플이 지도 위에 남아있는지를 확인하는 변수
	//-----------------------------------------------------------------------------------//
	// 모션모델 적용
	// Motion Model
	if(m_bgenerateSample&&m_bImageModel)
	{
		bModelResult = ImageModel(nx, ny);			// 영상 정보를 이용하여 각 샘플에 모션모델 적용.
	}
	//-----------------------------------------------------------------------------------//
	// image 정보를 사용한 업데이트.(image 비교)
	// Sensor Model Update
	vector<int> vecnError;
	for(int i = 0; i<m_nSampleNum; i++){
		Point2i ptCeilingCtrPoint;
		ptCeilingCtrPoint.x = (int)(m_stSample[i].x+0.5);
		ptCeilingCtrPoint.y = (int)(m_stSample[i].y+0.5);
		m_stSample[i].dRangeUpdatePro=0;
		int nError=0.0;
		for(int nHeigh=-ny;nHeigh<ny+1;nHeigh++)
		{
			for(int nWidth=-nx;nWidth<nx+1;nWidth++)
			{
				unsigned int nGlobalIdx = (ptCeilingCtrPoint.y+nHeigh)*matGlobalImg.cols+ptCeilingCtrPoint.x+nWidth;
				unsigned int nCeilingIdx = ((int)(m_dcy)+nHeigh)*vec_matCeilingImg[((int)m_stSample[i].t)].cols+(int)(m_dcx)+nWidth;
				int nTemp = matGlobalImg.data[nGlobalIdx]-vec_matCeilingImg[((int)m_stSample[i].t)].data[nCeilingIdx];
				if(nTemp<0) nTemp=-nTemp;
				nError+=nTemp*nTemp;
			}
		}
		vecnError.push_back(nError+1);
	}
	int nMaxError=0.0;
	for(int i=0; i<m_nSampleNum; i++)
	{
		if(vecnError[i]>nMaxError) nMaxError=vecnError[i];
	}
	for(int i=0; i<m_nSampleNum; i++)
	{
		m_stSample[i].dPro = m_stSample[i].dPro*((double)nMaxError/(double)vecnError[i]);
	}

	//-----------------------------------------------------------------------------------//
	// 확률 정규화
	// Normalizing the probabilities
	Normalizing();
	//-----------------------------------------------------------------------------------//

	Resampling();
	m_bImageModel=true;

	// 	Mat matTest;
	// 	matTest.create(matGlobalImg.rows,matGlobalImg.cols,CV_8UC1);
	// 	for(int i=0; i<matTest.cols*matTest.rows; i++)
	// 	{
	// 		matTest.data[i]=0;
	// 	}
	// 	for(int i=0; i<m_nSampleNum; i++)
	// 	{
	// 		matTest.data[(int)(m_stSample[i].y)*matTest.cols+(int)(m_stSample[i].x)]=255;
	// 	}
	//	imwrite("1SampleTest.bmp",matTest);

	//if (m_nSampleNum<=m_nMinSampleNum && m_nLocalizationState==10) m_nLocalizationState= 0;
	// 전역위치추정 시 샘플이 수렴하면 반복 종료
	if (m_dSampleSD<m_dSDThreshold){
		return false;
	}
	else
		return true;
}


/**
@brief Korean:  샘플들을 이동시키는 함수
@brief English: Function making every samples moving
*/
bool CeilingImageBasedParticleFilter::ImageModel(int nx, int ny)
{
	double dx = (double)nx+10;
	double dy = (double)ny+10;
	int nCnt = 0;

	for (int i=0; i<m_nSampleNum; i++) {
		// --------------------------------------- 제거할 샘플을 제거하는 부분 --------------------- //
		double dDeltaX = m_stSample[nCnt].x;
		double dDeltaY = m_stSample[nCnt].y;

		double dVariance = (double)((float)rand()/(float)RAND_MAX)-0.5;//-0.5~0.5 의 임의값(정규분포확률)
		m_stSample[nCnt].x = dVariance*m_dSampleSDX+m_stSample[nCnt].x;
		double dDelX=dVariance*m_dSampleSDX;
		if(m_stSample[nCnt].x<=dx) m_stSample[nCnt].x=dx;
		if(m_stSample[nCnt].x>=(m_dGlobalMaxWidth-dx)) m_stSample[nCnt].x=m_dGlobalMaxWidth-dx;

		dVariance = (double)((float)rand()/(float)RAND_MAX)-0.5;//-0.5~0.5 의 임의값(정규분포확률)
		m_stSample[nCnt].y = dVariance*m_dSampleSDY+m_stSample[nCnt].y;
		double dDelY=dVariance*m_dSampleSDY;
		if(m_stSample[nCnt].y<=dy) m_stSample[nCnt].y=dy;
		if(m_stSample[nCnt].y>=(m_dGlobalMaxHeigh-dy)) m_stSample[nCnt].y=(m_dGlobalMaxHeigh-dy);

		m_stSample[nCnt].t=m_stSample[nCnt].t+2*((double)((float)rand()/(float)RAND_MAX)-0.5);
		if(m_stSample[nCnt].t>32) m_stSample[nCnt].t=32;
		else if(m_stSample[nCnt].t<0) m_stSample[nCnt].t=0;

		double dMaxProMotion=0.8;
		double dMinProMotion=0.2;
		double d1=0.3;
		double dTransdistance = sqrt(pow(dDelX,2)+pow(dDelY,2));			// 직선이동거리
		double dNoiseTrans = GetRandValue(dTransdistance*d1);			// 직선운동에 따른 오차. 이동거리에 비례하여 증가한다.
		double dMaxProTrans = GetGaussianValue(dTransdistance*d1, 0.0);
		double dProTrans = GetGaussianValue(dTransdistance*d1, dNoiseTrans);
		dProTrans=(dMaxProMotion-dMinProMotion)*dProTrans/dMaxProTrans+dMinProMotion;

		m_stSample[i].dPro=dProTrans;

		nCnt++;
		// ----------------------------------------------------------------------------------------- //
	}
	if (nCnt==0){
		return false;	// 살아남은 샘플이 없을 때 // case of no samples in the map
	}
	else{
		return true;
	}
}


/**
@brief Korean: 확률값을 이용하여 샘플 복사 및 확률값 정규화
@brief English: Function redistributing samples according to prior probabilities of samples
*/
void CeilingImageBasedParticleFilter::Resampling()
{
	Accumulation *pAccumulation;

	double dSum = 0.0;
	int nNoOfAccumulated;
	int nCnt;
	double dRandPro;
	int i;
	double dSumPro = 0.0;

	pAccumulation = NULL;
	pAccumulation = (Accumulation*)calloc(m_nSampleNum,sizeof(Accumulation));

	// 확률에 따른 resampling을 위한 확률누적.
	// Accumulation of the probabilities of samples for normalizing
	nNoOfAccumulated=0;
	for (i =0; i<m_nSampleNum; i++) {
		if (m_stSample[i].dPro==0.0) continue;
		dSum += m_stSample[i].dPro;
		pAccumulation[nNoOfAccumulated].dSum = dSum;
		pAccumulation[nNoOfAccumulated].x = m_stSample[i].x;
		pAccumulation[nNoOfAccumulated].y = m_stSample[i].y;
		pAccumulation[nNoOfAccumulated].t = m_stSample[i].t;
		nNoOfAccumulated++;
	}

	// 샘플의 위치의 표준편차를 이용하여 다음 단계 샘플의 갯수를 계산.
	// Calculation of the number of samples by using deviation of the sample pose
	CalculateNoOfSamples();

	// 샘플 갯수에 따른 정규화된 확률값.
	m_dBasePro = 1.0/(double)m_nSampleNum;
	int nTemp=0;

	if(dSum == 0.0){
		for (i =0; i<m_nSampleNum; i++) {
			m_stSample[nTemp].x = m_stSample[i].x;
			m_stSample[nTemp].y = m_stSample[i].y;
			m_stSample[nTemp].t = m_stSample[i].t;
			m_stSample[nTemp].dPro = m_dBasePro;
			nTemp++;
		}
	}

	// 확률값이 높은 샘플은 많이 복사하고, 확률값이 낮은 샘플은 적게 복사하는 부분.
	// 지정한 갯수(m_nSampleNum)만큼 새로운 샘플 복사.

	//샘플의 개수에 상관없이 추출.
	while(nTemp<m_nSampleNum && dSum != 0.0){
		nCnt = 0;
		dRandPro = (double)rand()/(double)RAND_MAX;
		while (pAccumulation[nCnt].dSum<dRandPro && nCnt<nNoOfAccumulated-1){
			nCnt++;
		}
		m_stSample[nTemp].x = pAccumulation[nCnt].x;
		m_stSample[nTemp].y = pAccumulation[nCnt].y;
		m_stSample[nTemp].t = pAccumulation[nCnt].t;
		m_stSample[nTemp].dPro = m_dBasePro;
		nTemp++;
	}

	// 작업용 동적배열 삭제.
	free (pAccumulation);
	pAccumulation = NULL;
}


/**
@brief Korean: 일정한 로봇 주변 반경으로 샘플들을 뿌려주는 함수
@brief English: Function distributing samples near constant region around the robot.
*/
bool CeilingImageBasedParticleFilter::setSamplesNearRobot(int dx,int dy,int dSize, Mat& matGlobalImage)
{
	////TRACE("MCL: before reset samples\n");
	int nCnt = 0;
	int nSampleRegion[4];
	
	nSampleRegion[0] = dx-dSize;
	nSampleRegion[1] = dx+dSize;
	nSampleRegion[2] = dy-dSize;
	nSampleRegion[3] = dy+dSize;
	//---------------------------------예외처리--------------------------------------
	if(nSampleRegion[0]<0) nSampleRegion[0]=0;
	if(nSampleRegion[1]>=matGlobalImage.cols) nSampleRegion[1]=matGlobalImage.cols-1;
	if(nSampleRegion[2]<0) nSampleRegion[2]=0;
	if(nSampleRegion[3]>=matGlobalImage.rows) nSampleRegion[3]=matGlobalImage.rows-1;
	//---------------------------------예외처리--------------------------------------

	// 샘플이 뿌려질 영역 및 SAMPLE_DENSITY값에 따른 샘플 갯수 계산.
	m_nSampleNum = m_nMinSampleNum;

	// 정규화된 샘플의 확률값.
	m_dBasePro = 1.0/(double)m_nSampleNum;

	while(nCnt<m_nSampleNum)
	{
		// 샘플이 뿌려지는 영역을 고려하여 임의의 x, y, theta 값 입력.
		m_stSample[nCnt].x = (double)nSampleRegion[0] + (double)(((float)rand()/(float)RAND_MAX)*(nSampleRegion[1]-nSampleRegion[0]));
		m_stSample[nCnt].y = (double)nSampleRegion[2] + (double)(((float)rand()/(float)RAND_MAX)*(nSampleRegion[3]-nSampleRegion[2]));
		m_stSample[nCnt].t = (rand()%33);

		// 확률값 초기화.
		m_stSample[nCnt].dPro = m_dBasePro;
		nCnt++;
	}
	m_bgenerateSample=true;
	
	return true;
}


/**
@brief Korean:  샘플의 위치 분산을 이용하여 다음 단계에서 추출할 샘플의 갯수를 계산.
@brief English: calculate number of samples which will be extracted based on standard deviation of sample positions.
*/
void CeilingImageBasedParticleFilter::CalculateNoOfSamples()
{
	double dAvgPose[3], dSD[3];		// 샘플의 평균 및 표준편차를 구하기 위한 변수.

	// 확률을 고려하여 분산과 다음 단계 샘플의 갯수를 계산함.
	dAvgPose[0] = 0.0;
	dAvgPose[1] = 0.0;
	dAvgPose[2] = 0.0;
	dSD[0] = 0.0;
	dSD[1] = 0.0;
	dSD[2] = 0.0;

	// 샘플 위치의 평균 계산
	for(int i = 0; i<m_nSampleNum; i++) {
		dAvgPose[0] += m_stSample[i].x*m_stSample[i].dPro;	
		dAvgPose[1] += m_stSample[i].y*m_stSample[i].dPro;
		dAvgPose[2] += m_stSample[i].t*m_stSample[i].dPro;
	}

	// 평균에 따른 표준편차 계산
	for(int i = 0; i<m_nSampleNum; i++) {
		dSD[0] += (m_stSample[i].x-dAvgPose[0])*(m_stSample[i].x-dAvgPose[0])*m_stSample[i].dPro;
		dSD[1] += (m_stSample[i].y-dAvgPose[1])*(m_stSample[i].y-dAvgPose[1])*m_stSample[i].dPro;
		dSD[2] += (m_stSample[i].t-dAvgPose[2])*(m_stSample[i].t-dAvgPose[2])*m_stSample[i].dPro;
	}

	// m_dSampleSD는 위치에 따른 표준편차, m_dSampleSDForRotation은 회전량에 따른 표준편차
	m_dSampleSD = sqrt(dSD[0] + dSD[1]);
	m_dSampleSDX = sqrt(dSD[0]);
	m_dSampleSDY = sqrt(dSD[1]);
	m_dSampleSDForRotation = sqrt(dSD[2]);

	dSD[0] = sqrt(dSD[0]);
	dSD[1] = sqrt(dSD[1]);

	// 샘플의 표준편차를 이용하여 다음 단계에 추출될 샘플의 갯수를 계산.
	m_nSampleNum = (int)( 1.0*(dSD[0]*dSD[1]/1000.0/1000.0)*m_nSampleDensity );

	// 최대값 이상, 최소값 이하의 샘플 갯수가 계산되면 최대값, 최소값으로 대치.
	if (m_nSampleNum > m_nMaxSampleNum) m_nSampleNum = m_nMaxSampleNum;
	if (m_nSampleNum < m_nMinSampleNum) m_nSampleNum = m_nMinSampleNum;

}


/**
@brief Korean:  샘플들의 확률을 정규화하는 함수.
@brief English: Function normalizing probabilities of samples
*/
void CeilingImageBasedParticleFilter::Normalizing()
{
	double dSum = 0.0;
	int nCnt = 0;
	double dMax=0.0;

	for(int i=0; i<m_nSampleNum; i++)
		dSum += m_stSample[i].dPro;

	if(dSum !=0.0){
		for(int i=0; i<m_nSampleNum; i++)
			m_stSample[i].dPro = m_stSample[i].dPro / dSum;
	}
}

/**
@brief Korean: 모션에 대한 불확실성 정보를 받아오는 함수
@brief English: Function setting uncertainty of wheel motion
*/
void CeilingImageBasedParticleFilter::setDeviation(double dDeivationforTrans, double dDeviationforRotate,double dDeviationforTransRotate )
{
	m_dDeviationforTransConverged = dDeivationforTrans;
	m_dDeviationforRotateConverged = dDeviationforRotate;
	m_dDeviationforTransRotateConverged= M_PI/180.0*dDeviationforTransRotate;
}

/**
@brief Korean: 샘플의 최대,최소 개수를 받아오는 함수
@brief English: Function setting maximum and minimum sample numbers
*/
void CeilingImageBasedParticleFilter::setSampleNum(int nMaxSample,int nMinSample)
{
	m_nMaxSampleNum =nMaxSample;		
	m_nMinSampleNum =nMinSample;			
	m_nSampleNum = m_nMaxSampleNum;

	if(m_stSample!=NULL)	delete [] m_stSample;
	m_stSample = new Sample[m_nMaxSampleNum];
}

/**
@brief Korean: 2*sigma ~ 2*sigma 범위 내에서 임의의 값을 리턴하는 함수. (신뢰도 95% 이내)
@brief English: return random value from -2*sigma to 2*sigma
*/
double CeilingImageBasedParticleFilter::GetRandValue(double sigma)
{
	return sigma*2.0*( (double)rand()/(double)RAND_MAX - 0.5);
}

/**
@brief Korean: 평균 0, sigma인 가우시안 분포를 따르는 값을 출력하는 함수
@brief English: Function returning values according to Gaussian distribution with sigma and x
*/
double CeilingImageBasedParticleFilter::GetGaussianValue(double sigma, double x)
{
	if (sigma==0) return 0.1;

	return 1/sqrt(2*M_PI*sigma*sigma)*exp(-x*x/2.0/sigma/sigma);
}

/**
@brief Korean: 평균 0, sigma인 가우시안 분포를 따르는 값을 출력하는 함수
@brief English: Function returning values according to Gaussian distribution with sigma and x
*/
double CeilingImageBasedParticleFilter::GetGaussianValue(double sigma, double x, double y)
{
	if (sigma==0) return 0.1;
	double dGaussian = (1/sqrt(2*M_PI*sigma*sigma))*exp(-(x*x+y*y)/(2*sigma*sigma));
	return dGaussian;
}

/**
@brief Korean: 샘플들의 정보를 리턴해주는 함수
@brief English: Function getting samples' information
*/
vector<Sample> CeilingImageBasedParticleFilter::getParticle()
{
	vector<Sample> vecDummy;
	Sample stSample;
	vecDummy.clear();

	for (int i=0; i<m_nSampleNum; i++) {
		stSample.x = m_stSample[i].x;
		stSample.y = m_stSample[i].y;
		stSample.t = m_stSample[i].t;

		stSample.dPro = m_stSample[i].dPro;
		stSample.dRangeUpdatePro = m_stSample[i].dRangeUpdatePro;
		vecDummy.push_back(stSample);
	}

	return vecDummy;
}

/**
@brief Korean: MCL 중단 이전의 샘플 모음을 다시 불러오는 함수
@brief English: Function call old samples before deactivating MCL
*/
vector<Sample> CeilingImageBasedParticleFilter::GetOldParticle()
{
	return m_vecOldSampleCopy;
}

