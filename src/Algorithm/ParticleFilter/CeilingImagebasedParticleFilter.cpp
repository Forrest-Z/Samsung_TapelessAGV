#include "stdafx.h"
#include "CeilingImagebasedParticleFilter.h"

CeilingImageBasedParticleFilter::CeilingImageBasedParticleFilter()
{
	m_nMaxSampleNum =2000;		// ������ ������ �ִ밪. set2DMode()������ �ٲ���� ��.
	m_nMinSampleNum =400;			// ������ ������ �ּҰ�. set2DMode()������ �ٲ���� ��.
	m_nSampleNum = m_nMaxSampleNum;
	m_nMaxIteration=2000;
	m_nMinIteration=100;
	m_dSDThreshold= KuRobotParameter::getInstance()->getSDValue();

	m_bgenerateSample=false;
	m_bImageModel=false;

	m_stSample = new Sample[m_nMaxSampleNum];

	srand((unsigned)time(NULL)); //���� ���带 �׻� �ٲ��ֱ� ���ؼ� �����ؾ��Ѵ�.

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
@brief Korean: ������ ��ĳ���� ���� �Ÿ��� ���� ����Ȯ���� ����ϴ� �Լ�
@brief English: Function calculating the probabilities of error according to the range of laser
*/
void CeilingImageBasedParticleFilter::initProbability()
{
	m_nMaxSampleNum =2000;		// ������ ������ �ִ밪. set2DMode()������ �ٲ���� ��.
	m_nMinSampleNum =400;			// ������ ������ �ּҰ�. set2DMode()������ �ٲ���� ��.
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
@brief Korean: ��ƼŬ ���� ��� �̹��� ����
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
		warpAffine(matRetinexInputImg,matTempImg,center,matRetinexInputImg.size());//õ�念��
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
@brief Korean: MCL�� �����ϴ� �Լ�
@brief English: Function performing MCL
*/
bool CeilingImageBasedParticleFilter::doParticleFiltering(Mat matGlobalImg, vector<Mat> vec_matCeilingImg, int nx, int ny)
{
	bool bModelResult;	// Motion Model�� �����Ͽ� ������ ���� ���� �����ִ����� Ȯ���ϴ� ����
	//-----------------------------------------------------------------------------------//
	// ��Ǹ� ����
	// Motion Model
	if(m_bgenerateSample&&m_bImageModel)
	{
		bModelResult = ImageModel(nx, ny);			// ���� ������ �̿��Ͽ� �� ���ÿ� ��Ǹ� ����.
	}
	//-----------------------------------------------------------------------------------//
	// image ������ ����� ������Ʈ.(image ��)
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
	// Ȯ�� ����ȭ
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
	// ������ġ���� �� ������ �����ϸ� �ݺ� ����
	if (m_dSampleSD<m_dSDThreshold){
		return false;
	}
	else
		return true;
}


/**
@brief Korean:  ���õ��� �̵���Ű�� �Լ�
@brief English: Function making every samples moving
*/
bool CeilingImageBasedParticleFilter::ImageModel(int nx, int ny)
{
	double dx = (double)nx+10;
	double dy = (double)ny+10;
	int nCnt = 0;

	for (int i=0; i<m_nSampleNum; i++) {
		// --------------------------------------- ������ ������ �����ϴ� �κ� --------------------- //
		double dDeltaX = m_stSample[nCnt].x;
		double dDeltaY = m_stSample[nCnt].y;

		double dVariance = (double)((float)rand()/(float)RAND_MAX)-0.5;//-0.5~0.5 �� ���ǰ�(���Ժ���Ȯ��)
		m_stSample[nCnt].x = dVariance*m_dSampleSDX+m_stSample[nCnt].x;
		double dDelX=dVariance*m_dSampleSDX;
		if(m_stSample[nCnt].x<=dx) m_stSample[nCnt].x=dx;
		if(m_stSample[nCnt].x>=(m_dGlobalMaxWidth-dx)) m_stSample[nCnt].x=m_dGlobalMaxWidth-dx;

		dVariance = (double)((float)rand()/(float)RAND_MAX)-0.5;//-0.5~0.5 �� ���ǰ�(���Ժ���Ȯ��)
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
		double dTransdistance = sqrt(pow(dDelX,2)+pow(dDelY,2));			// �����̵��Ÿ�
		double dNoiseTrans = GetRandValue(dTransdistance*d1);			// ������� ���� ����. �̵��Ÿ��� ����Ͽ� �����Ѵ�.
		double dMaxProTrans = GetGaussianValue(dTransdistance*d1, 0.0);
		double dProTrans = GetGaussianValue(dTransdistance*d1, dNoiseTrans);
		dProTrans=(dMaxProMotion-dMinProMotion)*dProTrans/dMaxProTrans+dMinProMotion;

		m_stSample[i].dPro=dProTrans;

		nCnt++;
		// ----------------------------------------------------------------------------------------- //
	}
	if (nCnt==0){
		return false;	// ��Ƴ��� ������ ���� �� // case of no samples in the map
	}
	else{
		return true;
	}
}


/**
@brief Korean: Ȯ������ �̿��Ͽ� ���� ���� �� Ȯ���� ����ȭ
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

	// Ȯ���� ���� resampling�� ���� Ȯ������.
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

	// ������ ��ġ�� ǥ�������� �̿��Ͽ� ���� �ܰ� ������ ������ ���.
	// Calculation of the number of samples by using deviation of the sample pose
	CalculateNoOfSamples();

	// ���� ������ ���� ����ȭ�� Ȯ����.
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

	// Ȯ������ ���� ������ ���� �����ϰ�, Ȯ������ ���� ������ ���� �����ϴ� �κ�.
	// ������ ����(m_nSampleNum)��ŭ ���ο� ���� ����.

	//������ ������ ������� ����.
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

	// �۾��� �����迭 ����.
	free (pAccumulation);
	pAccumulation = NULL;
}


/**
@brief Korean: ������ �κ� �ֺ� �ݰ����� ���õ��� �ѷ��ִ� �Լ�
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
	//---------------------------------����ó��--------------------------------------
	if(nSampleRegion[0]<0) nSampleRegion[0]=0;
	if(nSampleRegion[1]>=matGlobalImage.cols) nSampleRegion[1]=matGlobalImage.cols-1;
	if(nSampleRegion[2]<0) nSampleRegion[2]=0;
	if(nSampleRegion[3]>=matGlobalImage.rows) nSampleRegion[3]=matGlobalImage.rows-1;
	//---------------------------------����ó��--------------------------------------

	// ������ �ѷ��� ���� �� SAMPLE_DENSITY���� ���� ���� ���� ���.
	m_nSampleNum = m_nMinSampleNum;

	// ����ȭ�� ������ Ȯ����.
	m_dBasePro = 1.0/(double)m_nSampleNum;

	while(nCnt<m_nSampleNum)
	{
		// ������ �ѷ����� ������ ����Ͽ� ������ x, y, theta �� �Է�.
		m_stSample[nCnt].x = (double)nSampleRegion[0] + (double)(((float)rand()/(float)RAND_MAX)*(nSampleRegion[1]-nSampleRegion[0]));
		m_stSample[nCnt].y = (double)nSampleRegion[2] + (double)(((float)rand()/(float)RAND_MAX)*(nSampleRegion[3]-nSampleRegion[2]));
		m_stSample[nCnt].t = (rand()%33);

		// Ȯ���� �ʱ�ȭ.
		m_stSample[nCnt].dPro = m_dBasePro;
		nCnt++;
	}
	m_bgenerateSample=true;
	
	return true;
}


/**
@brief Korean:  ������ ��ġ �л��� �̿��Ͽ� ���� �ܰ迡�� ������ ������ ������ ���.
@brief English: calculate number of samples which will be extracted based on standard deviation of sample positions.
*/
void CeilingImageBasedParticleFilter::CalculateNoOfSamples()
{
	double dAvgPose[3], dSD[3];		// ������ ��� �� ǥ�������� ���ϱ� ���� ����.

	// Ȯ���� ����Ͽ� �л�� ���� �ܰ� ������ ������ �����.
	dAvgPose[0] = 0.0;
	dAvgPose[1] = 0.0;
	dAvgPose[2] = 0.0;
	dSD[0] = 0.0;
	dSD[1] = 0.0;
	dSD[2] = 0.0;

	// ���� ��ġ�� ��� ���
	for(int i = 0; i<m_nSampleNum; i++) {
		dAvgPose[0] += m_stSample[i].x*m_stSample[i].dPro;	
		dAvgPose[1] += m_stSample[i].y*m_stSample[i].dPro;
		dAvgPose[2] += m_stSample[i].t*m_stSample[i].dPro;
	}

	// ��տ� ���� ǥ������ ���
	for(int i = 0; i<m_nSampleNum; i++) {
		dSD[0] += (m_stSample[i].x-dAvgPose[0])*(m_stSample[i].x-dAvgPose[0])*m_stSample[i].dPro;
		dSD[1] += (m_stSample[i].y-dAvgPose[1])*(m_stSample[i].y-dAvgPose[1])*m_stSample[i].dPro;
		dSD[2] += (m_stSample[i].t-dAvgPose[2])*(m_stSample[i].t-dAvgPose[2])*m_stSample[i].dPro;
	}

	// m_dSampleSD�� ��ġ�� ���� ǥ������, m_dSampleSDForRotation�� ȸ������ ���� ǥ������
	m_dSampleSD = sqrt(dSD[0] + dSD[1]);
	m_dSampleSDX = sqrt(dSD[0]);
	m_dSampleSDY = sqrt(dSD[1]);
	m_dSampleSDForRotation = sqrt(dSD[2]);

	dSD[0] = sqrt(dSD[0]);
	dSD[1] = sqrt(dSD[1]);

	// ������ ǥ�������� �̿��Ͽ� ���� �ܰ迡 ����� ������ ������ ���.
	m_nSampleNum = (int)( 1.0*(dSD[0]*dSD[1]/1000.0/1000.0)*m_nSampleDensity );

	// �ִ밪 �̻�, �ּҰ� ������ ���� ������ ���Ǹ� �ִ밪, �ּҰ����� ��ġ.
	if (m_nSampleNum > m_nMaxSampleNum) m_nSampleNum = m_nMaxSampleNum;
	if (m_nSampleNum < m_nMinSampleNum) m_nSampleNum = m_nMinSampleNum;

}


/**
@brief Korean:  ���õ��� Ȯ���� ����ȭ�ϴ� �Լ�.
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
@brief Korean: ��ǿ� ���� ��Ȯ�Ǽ� ������ �޾ƿ��� �Լ�
@brief English: Function setting uncertainty of wheel motion
*/
void CeilingImageBasedParticleFilter::setDeviation(double dDeivationforTrans, double dDeviationforRotate,double dDeviationforTransRotate )
{
	m_dDeviationforTransConverged = dDeivationforTrans;
	m_dDeviationforRotateConverged = dDeviationforRotate;
	m_dDeviationforTransRotateConverged= M_PI/180.0*dDeviationforTransRotate;
}

/**
@brief Korean: ������ �ִ�,�ּ� ������ �޾ƿ��� �Լ�
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
@brief Korean: 2*sigma ~ 2*sigma ���� ������ ������ ���� �����ϴ� �Լ�. (�ŷڵ� 95% �̳�)
@brief English: return random value from -2*sigma to 2*sigma
*/
double CeilingImageBasedParticleFilter::GetRandValue(double sigma)
{
	return sigma*2.0*( (double)rand()/(double)RAND_MAX - 0.5);
}

/**
@brief Korean: ��� 0, sigma�� ����þ� ������ ������ ���� ����ϴ� �Լ�
@brief English: Function returning values according to Gaussian distribution with sigma and x
*/
double CeilingImageBasedParticleFilter::GetGaussianValue(double sigma, double x)
{
	if (sigma==0) return 0.1;

	return 1/sqrt(2*M_PI*sigma*sigma)*exp(-x*x/2.0/sigma/sigma);
}

/**
@brief Korean: ��� 0, sigma�� ����þ� ������ ������ ���� ����ϴ� �Լ�
@brief English: Function returning values according to Gaussian distribution with sigma and x
*/
double CeilingImageBasedParticleFilter::GetGaussianValue(double sigma, double x, double y)
{
	if (sigma==0) return 0.1;
	double dGaussian = (1/sqrt(2*M_PI*sigma*sigma))*exp(-(x*x+y*y)/(2*sigma*sigma));
	return dGaussian;
}

/**
@brief Korean: ���õ��� ������ �������ִ� �Լ�
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
@brief Korean: MCL �ߴ� ������ ���� ������ �ٽ� �ҷ����� �Լ�
@brief English: Function call old samples before deactivating MCL
*/
vector<Sample> CeilingImageBasedParticleFilter::GetOldParticle()
{
	return m_vecOldSampleCopy;
}

