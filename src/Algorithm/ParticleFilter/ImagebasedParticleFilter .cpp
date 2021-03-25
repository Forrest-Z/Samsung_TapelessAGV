#include "stdafx.h"
#include "ImagebasedParticleFilter .h"

/**
@사용법.
@ 1. SetSampleRegion() 함수를 이용하여 샘플이 뿌려질 영역 설정. 
@ 2. ResetSamples() 함수를 이용하여 샘플을 초기화시킴.
@ 1,2는 생성자에서 초기설정값으로 실행. 위치추정 실패나 키드냅 등으로 인하여 다시 전역위치추정을 수행할 때 1,2 순서로 호출하면 OK.
@ 3. SetRangeSensorParameter() 함수를 이용하여 거리센서 파라미터를 설정.
@ 4. SetMapData() 함수를 이용하여 지도정보 전달. 중간에 지도를 수정하는 것은 안만들었음.
@ 5. MCLStart(), MCLStop() 함수를 이용하여 MCL 수행여부를 설정. 실행 중에 MCLStop()을 하면 센서를 이용한 업데이트는 수행 안하고 모션모델만 적용.
@ 6. SetEncoderData() 함수를 이용하여 엔코더의 변화량 입력.
@ 7. SetRangeData() 함수를 이용하여 거리센서값 입력. 거리센서값은 최대값 이상의 값은 최대값으로 변경하여 저장되어 있어야 함.
@ 8. GetSamplePos() 함수를 이용하여 결과값 리턴. 입력 파라미터는 1. 추정된 위치, 2. 추정된 위치예서 예상되는 거리센서값 리턴을 위한 변수.
@ 9. 6,7,8을 반복하면 됨.
@ 다른 센서를 사용할 경우, 센서값을 입력받고, 센서모델을 이용한 확률 갱신을 추가하면 됨.
@ m_nMaxSampleNum은 전역위치추정 시 뿌릴 샘플의 수, m_nMinSampleNum은 위치트래킹시 뿌릴 최소한의 샘플 수.
@ m_nMaxSampleNum은 필요하다면 아무때나 변경하면 됨.
@ m_nMinSampleNum은 100이상으로 추천함.
*/

ImagebasedParticleFilter::ImagebasedParticleFilter()
{

	m_nMaxSampleNum =8000;		// 샘플의 갯수의 최대값. set2DMode()에서도 바꿔줘야 함.
	m_nMinSampleNum =500;			// 샘플의 갯수의 최소값. set2DMode()에서도 바꿔줘야 함.
	m_nSampleNum = m_nMaxSampleNum;
	m_dRangeSensorOffset = 0.0;

	m_bDoRangeUpdate = false;
	m_bDoVisionUpdate= false;
	m_bResetSamples = false;
	m_bMCLWorkingNow = false;
	m_bMCLWork = true;
	m_bgenerateSample=false;

	m_nMCLReturnValue = 1;
		
	m_dMaxProForUpdate = 0.8;
	m_dMinProForUpdate = 0.2;

	m_dAccumulatedRotation = 0.0;
	m_dAccumulatedDistance = 0.0;

	m_stSample = new Sample[m_nMaxSampleNum];


	m_nMapSizeX = 0;
	m_nMapSizeY = 0;
	srand((unsigned)time(NULL)); //랜덤 씨드를 항상 바꿔주기 위해서 설정해야한다.

	m_fRangeMCLPro = NULL;
	m_nGridMap=NULL;

	m_dDeviationforTransConverged = 0.20;
	m_dDeviationforRotateConverged = 0.10;
	m_dDeviationforTransRotateConverged= M_PI/180.0*0.02;
	m_nMinSampleStandardDeviation= 100;
	m_nSampleDensity=400;
	m_dSampleSD = m_nMinSampleStandardDeviation;

	m_cvMatImage.create(Sensor::CEILING_IMAGE_HEIGHT,Sensor::CEILING_IMAGE_WIDTH,CV_8UC1);//Result image initialization
	m_cvMatImageBright.create(Sensor::CEILING_IMAGE_HEIGHT,Sensor::CEILING_IMAGE_WIDTH,CV_8UC1);//Result image initialization
		
	m_dAlphaV=0.5;
	m_dAlphaR=0.5;
	m_dCam_f=7.962e-004;
	
	m_dCovPose=0.5;//30cm
	m_dCovImage= 30;//20pixel
	m_dCovTheta = 5;//7degree
	m_dCovArea= 80; //50 pixel
	m_dCovRub=40;//10
	m_dCovResp=30;//10


	initStateforMap();
}

ImagebasedParticleFilter::~ImagebasedParticleFilter()
{
	m_bgenerateSample=false;
	m_bDoRangeUpdate = false;
	m_bDoVisionUpdate= false;
	m_bResetSamples = false;
	m_bMCLWorkingNow = false;
	m_bMCLWork = true;

	m_cvMatImage.release();
	m_cvMatImageBright.release();

	delete [] m_stSample;
	if(m_fRangeMCLPro!=NULL){
		for(int i = 0; i < (int)(m_dMaxDistance/10.0)+1; ++i) {
			delete [] m_fRangeMCLPro[i];
		}
		delete [] m_fRangeMCLPro;
	}
	if(m_nGridMap!=NULL){
		for(int i = 0 ; i < m_nMapSizeX ; i++){
			delete[] m_nGridMap[i];
			m_nGridMap[i] = NULL;
		}
		delete[] m_nGridMap;
	}
}
/**
@brief Korean: 레이저 스캐너의 측정 거리에 따른 오차확률을 계산하는 함수
@brief English: Function calculating the probabilities of error according to the range of laser
*/
void ImagebasedParticleFilter::initProbability()
{
	// 모션모델에서 계산되는 확률의 범위. 0~1.0사이. 센서모델에서 계산되는 범위를 정할 때 이 범위를 고려해야 함.
	m_dMaxProForUpdate = 0.8;
	m_dMinProForUpdate = 0.2;

	// range-based MCL에서 거리차에 따른 확률을 미리 계산
	if(m_fRangeMCLPro==NULL){
		m_fRangeMCLPro = new float*[(int)(m_dMaxDistance/10.0)+1];

		for(int i = 0; i < (int)(m_dMaxDistance/10.0)+1; ++i) {
			m_fRangeMCLPro[i] = new float[(int)(m_dMaxDistance/10.0)+1];
			memset(m_fRangeMCLPro[i], 0, sizeof(float)*(int)(m_dMaxDistance/10.0)+1);   // 메모리 공간을 0으로 초기화
		}
	}
	float sigma;
	for(int i=0;i<(int)(m_dMaxDistance/10.0)+1;i++) {	// expected distance
		sigma = 0.001*(float)i + 0.150;
		for(int j=0;j<(int)(m_dMaxDistance/10.0)+1;j++) {	// measured distance
			m_fRangeMCLPro[i][j] = 0.05*exp(-((float)i*0.01-(float)j*0.01)*((float)i*0.01-(float)j*0.01)/2/sigma/sigma)
					+ 0.005*((float)m_dMaxDistance-(float)j*10.0)/(float)m_dMaxDistance;
// 			m_fRangeMCLPro[i][j] = (m_dMaxProForUpdate-m_dMinProForUpdate)/(sqrt(2*M_PI*sigma))*exp(-((float)i*0.01 - (float)j*0.01)*((float)i*0.01 - (float)j*0.01)/2/sigma/sigma)
// 				+ 0.005*((float)m_dMaxDistance-(float)j*10.0)/(float)m_dMaxDistance+ m_dMinProForUpdate;	

			if((float)j*10.0>(float)m_dMaxDistance*0.99)m_fRangeMCLPro[i][j] = (float)m_dMinProForUpdate;

			// 모션모델의 확률범위를 고려하여 범위를 맞춰주는 부분. (0~1.0사이 조절)*(가중치)
			//m_fRangeMCLPro[i][j] *= (100.0);
			m_fRangeMCLPro[i][j] *= (100.0* 1.5);
			//m_fRangeMCLPro[i][j] *= (100.0);
		}
	}

}

void ImagebasedParticleFilter::initStateforMap()
{
	m_EmptyArea=KuMap::EMPTY_AREA;
	m_OccupiedArea=KuMap::OCCUPIED_AREA;
	m_UnkownArea=KuMap::UNKNOWN_AREA;
	m_FixedCADArea=KuMap::FIXED_CAD_AREA;
	m_DynamicEnvArea=KuMap::DYNAMIC_ENVIRONMENT_AREA;
	m_LuggageArea=KuMap::LUGGAGE_AREA;
}

void ImagebasedParticleFilter::setRegion(vector<CLAMPData>  fRegionresult)
{
	m_FRD.clear();

	for(int i = 0; i < fRegionresult.size(); i++)
	{
		m_FRD.push_back(fRegionresult[i]);
	}
}

void ImagebasedParticleFilter::setTemplate(vector<CFREAKData> fRegionresult)
{
	m_FTD.clear();

	for(int i = 0; i < fRegionresult.size(); i++)
	{
		m_FTD.push_back(fRegionresult[i]);
	}
}

void ImagebasedParticleFilter::copyCeilingImage(Mat cvMatImage)
{
	for(int i = 0; i < Sensor::CEILING_IMAGE_WIDTH * Sensor::CEILING_IMAGE_HEIGHT; i++)
	{
		m_cvMatImage.data[i]= cvMatImage.data[i];

		m_cvMatImageBright.data[i]=0;
		if(cvMatImage.data[i]>225){
			m_cvMatImageBright.data[i]=255;
		}
	}

	m_bDoVisionUpdate= true;
}

/**
@brief Korean: 소요 시간을 측정하기 위해서 초기화하는 함수
@brief English: Initializes to count the duration
*/
void ImagebasedParticleFilter::startTimeCheck(LARGE_INTEGER& nStart)
{
	QueryPerformanceCounter(&nStart);
}
/**
@brief Korean: 측정된 소요 시간을 받아오는 함수
@brief English: Gets the estimated duration
*/
float ImagebasedParticleFilter::finishTimeCheck(const LARGE_INTEGER nStart)
{
	LARGE_INTEGER nFinish, freq;

	QueryPerformanceFrequency(&freq);

	QueryPerformanceCounter(&nFinish);

	return (float)(1000.0 * (nFinish.QuadPart - nStart.QuadPart) / freq.QuadPart);
}



/**
@brief Korean: 샘플의 지도 상의 (x,y,t)위치에서 예상되는 거리센서 값을 구하는 함수
@brief English:  Function predicting range values of the samples on (x,y,t) based on map data
*/
inline double ImagebasedParticleFilter::predictCeilingData(double x, double y, double t, int nGap)
{
	double dPro=0;

	if(m_observedFRD.size()<1&&m_observedFTD.size()<1){

		double dDistforMotion=hypot(m_dSamplePos[0]-x, m_dSamplePos[1]-y);
		double dDegreeforMotion=sqrt((fabs)(m_dSamplePos[2]-t))/M_PI;

		double dDistProR=0;
		if(dDistforMotion<300) { dDistProR=1.0;}
		else{dDistProR=300/dDistforMotion;}
	
		double dProthetaT=0;
		if(dDegreeforMotion<5.0*D2R) { dProthetaT=1.0;}
		else{dProthetaT=5.0*D2R/dDegreeforMotion;}
		
		dPro=dDistProR*dProthetaT;

		return dPro;
	}

	double dProDist=0;
	double dProTheta=0;
	double dProArea=0;
	double dProWidth=0;
	double dProimage=0;
	double dProDir=0;
	double dProTotal=0;
	double dmaRNM=0.0;
	double dProResp=0;
	double dProRub=0;

	double dCos=cos(t-M_PI/2.0);
	double dSin=sin(t-M_PI/2.0);
	
	CamContext* CamCt=KuSiriusCameraInterface::getInstance()->getCamContext();

	for(int i=0; i<m_observedFRD.size();i++)
	{
		m_observedFRD[i].th=(180-m_observedFRD[i].angle)+t*R2D;

		if (m_observedFRD[i].th>180) m_observedFRD[i].th -= 2.0*180;
		else if (m_observedFRD[i].th<-180) m_observedFRD[i].th += 2.0*180;
		if (m_observedFRD[i].th<0) m_observedFRD[i].th += 180;

		double dDistforRange=0.0;
		double dimageDistProR =0.0;
		double dPredictimageDistProR =0.0;
		double dDistforMinRange=100000000.0;
		double dimageDistProMinR =100000000.0;
		double dTheta =0.0;
		double dDirDataAngle =0.0;
		int nSelIDX=-1;	

		for(int j=0; j<m_FRD.size();j++)
		{
			if(fabs(m_FRD[j].x-x/1000.0)<3.0&&fabs(m_FRD[j].y-y/1000.0)<3.0
				&&m_FRD[j].ntype==m_observedFRD[i].ntype)
			{
				float fDFx, fDFy, fDFz, fDFpi; // feature pose in global coordinate

				trans_img2global((float)x/1000.0, (float)y/1000.0, (float)t, m_observedFRD[i].u, m_observedFRD[i].v,
					fDFx, fDFy, fDFz,CamCt,m_FRD[j].ceilingheight);

				dDistforRange=_hypot(m_FRD[j].x-fDFx,m_FRD[j].y-fDFy);

				int mv, mu;
				trans_global2img((float)x/1000.0, (float)y/1000.0, (float)t, m_FRD[j].x,m_FRD[j].y, m_FRD[j].ceilingheight, 
					mu,mv, CamCt);

				dimageDistProR=_hypot(m_observedFRD[i].u-mu, m_observedFRD[i].v-mv);	

				if(dDistforRange<dDistforMinRange&&dimageDistProR<dimageDistProMinR&&dDistforRange<0.75)
				{
					dDistforMinRange=dDistforRange;
					dimageDistProMinR=dimageDistProR;
					nSelIDX=j;
				}
			}
		}


		if(nSelIDX!=-1)
		{

			float fDFx, fDFy, fDFz, fDFpi; // feature pose in global coordinate

			//trans_img2global((float)m_FRD[nSelIDX].Rx, (float)m_FRD[nSelIDX].Ry, (float)m_FRD[nSelIDX].Rt, m_observedFRD[i].u, m_observedFRD[i].v, fDFx, fDFy, fDFz,KuSiriusCameraInterface::getInstance()->getCamContext(),m_FRD[nSelIDX].ceilingheight);
			trans_img2global((float)x/1000.0, (float)y/1000.0, (float)t, m_observedFRD[i].u, m_observedFRD[i].v,
				fDFx, fDFy, fDFz,CamCt,m_FRD[nSelIDX].ceilingheight);

			dDistforRange=_hypot(m_FRD[nSelIDX].x-fDFx,m_FRD[nSelIDX].y-fDFy);

			int mv, mu;
			
			trans_global2img((float)x/1000.0, (float)y/1000.0, (float)t, m_FRD[nSelIDX].x,m_FRD[nSelIDX].y, m_FRD[nSelIDX].ceilingheight, 
				mu,mv, CamCt);

			dimageDistProR=_hypot(m_observedFRD[i].u-mu, m_observedFRD[i].v-mv);	

			double dObsTheta=m_observedFRD[i].th;

			if(m_FRD[nSelIDX].th>150&&dObsTheta<30)
			{
				dObsTheta+=180;
			}
			else if(m_FRD[nSelIDX].th<30&&dObsTheta>150)
			{
				dObsTheta-=180;
			}

			dTheta=fabs(dObsTheta-m_FRD[nSelIDX].th);

			if(dDistforRange<1.5
				&&dTheta<30.0
				&&dimageDistProR<100)
			{
				dmaRNM++;

				m_FRD[nSelIDX].detect=true;
				double  dMaxProTrans = GetGaussianValue(m_dCovPose , 0.0);//500
				double  dProTrans = GetGaussianValue(m_dCovPose ,dDistforRange);
				dProDist=((0.6)*dProTrans/dMaxProTrans+0.2);				
	
				dMaxProTrans = GetGaussianValue(m_dCovImage,0);//5
				dProTrans = GetGaussianValue(m_dCovImage, dimageDistProR);
				dProimage=((0.6)*dProTrans/dMaxProTrans+0.2);				
	
				if(m_observedFRD[i].ntype==0)
				{
					dMaxProTrans = GetGaussianValue(m_dCovTheta, 0);//20
					dProTrans = GetGaussianValue(m_dCovTheta, dTheta);
					dProTheta=((0.6)*dProTrans/dMaxProTrans+0.2);
					dProTheta=dProTheta;
				}
				else  dProTheta=0.8;

				dMaxProTrans = GetGaussianValue(m_dCovArea, 0);//20
				dProTrans = GetGaussianValue(m_dCovArea, (double)m_observedFRD[i].area-(double)m_FRD[nSelIDX].area);
				dProArea=((0.6)*dProTrans/dMaxProTrans+0.2);
	
				dProTotal+=1.47*(dProDist*dProTheta*dProArea*dProimage)+0.2;
				
			}
		}		
	}

	if(m_observedFRD.size()>0)
		dProTotal=dProTotal/(double)m_observedFRD.size();

	double dTempl=0.0;
	double dmatNM=0.0;


	for(int i=0; i<m_observedFTD.size();i++)
	{

 		int nSelIDX=m_observedFTD[i].MatchNum;

		float fDFx, fDFy, fDFz, fDFpi; // feature pose in global coordinate

		trans_img2global((float)x/1000.0, (float)y/1000.0, (float)t, m_observedFTD[i].u, m_observedFTD[i].v, 
			fDFx, fDFy, fDFz,CamCt,m_FTD[nSelIDX].ceilingheight);
				
		double  dDistT =hypot(m_FTD[nSelIDX].x-fDFx,m_FTD[nSelIDX].y-fDFy);

		 int mv, mu;
		 trans_global2img((float)x/1000.0, (float)y/1000.0, (float)t, m_FTD[nSelIDX].x,m_FTD[nSelIDX].y, 
			 m_FTD[nSelIDX].ceilingheight, mu,mv, CamCt);

		double  dimageDistProT=hypot(m_observedFTD[i].u-mu, m_observedFTD[i].v-mv);	

		if(dDistT<1.5&&dimageDistProT<100)
		{
			m_FTD[nSelIDX].detect=true;

			double  dMaxProTrans = GetGaussianValue(m_dCovPose , 0.0);//500
			double  dProTrans = GetGaussianValue(m_dCovPose ,dDistT);
			dProDist=((0.6)*dProTrans/dMaxProTrans+0.2);	

			dMaxProTrans = GetGaussianValue(m_dCovImage,0);//5
			dProTrans = GetGaussianValue(m_dCovImage, dimageDistProT);
			dProimage=((0.6)*dProTrans/dMaxProTrans+0.2);			


			dMaxProTrans = GetGaussianValue(m_dCovResp,0);//5
			dProTrans = GetGaussianValue(m_dCovResp, m_observedFTD[i].response);
			dProResp=((0.6)*dProTrans/dMaxProTrans+0.2);				
				
			dMaxProTrans = GetGaussianValue(m_dCovRub,0);//5
			dProTrans = GetGaussianValue(m_dCovRub, m_FTD[nSelIDX].nmatchnum);
			dProRub=((0.6)*dProTrans/dMaxProTrans+0.2);	
			
			//dTempl+=(100.0/m_observedFTD[i].response)*m_FTD[nSelIDX].nmatchnum/100.0*dProDist*dProimage;
			
			dTempl+=1.47*(dProDist*dProResp*dProRub*dProimage)+0.2;
		
			dmatNM++;
		}
	}

	if(m_observedFTD.size()>0)
	{
		dTempl=dTempl*dmatNM/(double)(m_observedFTD.size());
	}
	

	if(dmatNM<1&&dmaRNM<1){

		double dDistforMotion=hypot(m_dSamplePos[0]-x, m_dSamplePos[1]-y);
		double dDegreeforMotion=sqrt((fabs)(m_dSamplePos[2]-t))/M_PI;

		double dDistProR=0;
		if(dDistforMotion<300) { dDistProR=1.0;}
		else{dDistProR=300/dDistforMotion;}

		double dProthetaT=0;
		if(dDegreeforMotion<5.0*D2R) { dProthetaT=1.0;}
		else{dProthetaT=5.0*D2R/dDegreeforMotion;}

		dPro=dDistProR*dProthetaT;
		
		return dPro;
	}


	dPro=(dProTotal*7+dTempl*3);
	
	if(dPro==0) dPro=0.000002;

	return dPro;

}


inline int ImagebasedParticleFilter::max(int a, int b)
{
	return a > b ? a : b; 
}

inline int ImagebasedParticleFilter::min(int a, int b)
{
	return a < b ? a : b; 
}

/**
@brief Korean: 지도정보를 저장하는 함수
@brief English: Function saving the map information
*/
void ImagebasedParticleFilter::setMap(int sizeX, int sizeY, int** nGridMap)
{
	if(m_nGridMap==NULL)
	{
		m_nGridMap = new int*[sizeX];
		if(m_nGridMap){
			for(int i = 0 ; i < sizeX ; i++){
				m_nGridMap[i] = new int[sizeY];
			}
		}
	}
	else if((m_nMapSizeX != sizeX||m_nMapSizeY != sizeY))
	{
		for(int i = 0 ; i < m_nMapSizeX ; i++){
			delete[] m_nGridMap[i];
			m_nGridMap[i] = NULL;
		}
		delete[] m_nGridMap;

		m_nGridMap = new int*[sizeX];
		if(m_nGridMap){
			for(int i = 0 ; i < sizeX ; i++){
				m_nGridMap[i] = new int[sizeY];
			}
		}	
	}

	for(int i = 0; i<sizeX; i++){
		for(int j = 0; j< sizeY;j++){
			if(nGridMap[i][j]==m_FixedCADArea  )
				m_nGridMap[i][j] = m_OccupiedArea  ;
			else if(nGridMap[i][j]==m_LuggageArea)
				m_nGridMap[i][j] = m_OccupiedArea  ;
			else if(nGridMap[i][j]==m_DynamicEnvArea)
				m_nGridMap[i][j] = m_EmptyArea ;
			else if(nGridMap[i][j]!=m_UnkownArea)
				m_nGridMap[i][j] = nGridMap[i][j];
		}
	}

	m_nMapSizeX = sizeX;
	m_nMapSizeY = sizeY;

	// 샘플이 뿌려지는 영역에 대한 좌표 초기화. 지도 전체에 뿌림. 전역위치추정.
	// distributing the samples on whole map for global localization when every samples are out of the reference map
	SetSampleRegion(100, m_nMapSizeX*100-100, 100, m_nMapSizeY*100-100);
}

/**
@brief Korean:  갱신 된 지도정보를 저장하는 함수
@brief English:  Function update the map after getting new map information
*/
void ImagebasedParticleFilter::updateMapData(int sizeX, int sizeY, int **nGridMap)
{

	if(m_nMapSizeX<sizeX||m_nMapSizeY<sizeY) return;

	for(int i = 0; i<sizeX; i++){
		for(int j = 0; j< sizeY;j++){
			if(nGridMap[i][j]==m_FixedCADArea  )
				m_nGridMap[i][j] = m_OccupiedArea  ;
			else if(nGridMap[i][j]==m_LuggageArea)
				m_nGridMap[i][j] = m_OccupiedArea  ;
			else if(nGridMap[i][j]==m_DynamicEnvArea)
				m_nGridMap[i][j] = m_EmptyArea ;
			else if(nGridMap[i][j]!=m_UnkownArea)
				m_nGridMap[i][j] = nGridMap[i][j];
		}
	}
}

/**
@brief Korean: MCL을 종료시키는 함수
@brief English: Function deactivating MCL
*/
void ImagebasedParticleFilter::MCLStop()
{
	// 동적배열 사용하는 부분이 다 끝나면 MCL을 끝내도록.... 메인문의 exit()직전에 수행.
	m_nSampleNum = 0;
	m_bMCLWork = false;
	m_bCalculationStop = true;

}

/**
@brief Korean: 각 샘플의 위치를 확률에 따라 평균하여 로봇의 위치를 추정하는 함수
@brief English: Function estimating robot pose by averaging every pose of samples according to the probabilities
*/
int ImagebasedParticleFilter::getSamplePos(double dPos[3], double dEstimatedRangeData[181])
{
	if (m_bMCLWork==false) return NO_WORK;
	m_bMCLWorkingNow = true;

	double dSum = 0.0;
	if(false==doParticleFiltering() && m_nMCLReturnValue == MCL_ERROR_ALLSAMPLES_DISAPPEARED){ // MCL 수행.
		// 샘플이 지도밖으로 모두 사라진 경우
		// 샘플이 뿌려지는 영역에 대한 좌표 초기화. 지도 전체에 뿌림. 전역위치추정.
		// distributing the samples on whole map for global localization when every samples are out of the reference map
		SetSampleRegion(STARTPOS_MARGIN, m_math.AI2MM(m_nMapSizeX)-ENDPOS_MARGIN,
			STARTPOS_MARGIN, m_math.AI2MM(m_nMapSizeY)-ENDPOS_MARGIN);
		setSamples();
		m_bMCLWorkingNow = false;
		return m_nLocalizationState=INITIAL_STATE;
	}

	// 180도 지점에서 각도가 179에서 -179로 변하는 문제를 처리하기 위해.
	// 	if (m_dSamplePos[2]>120*D2R || m_dSamplePos[2]<-120*D2R) bAngleCalculationMode = true;
	// 	else bAngleCalculationMode = false;
	// 새로운 위치를 계산하기 위해 변수 초기화.
	m_dSamplePos[0] = 0.0;
	m_dSamplePos[1] = 0.0;
	m_dSamplePos[2] = 0.0;

	// 확률값과 위치값을 이용한 평균.
	// calculating robot pose by averaging poses of samples
	double dx, dy;
	dx = 0; dy = 0;
	for(int i=0; i<m_nSampleNum; i++)  {
		dSum += m_stSample[i].dPro;
		m_dSamplePos[0] += m_stSample[i].x*m_stSample[i].dPro;
		m_dSamplePos[1] += m_stSample[i].y*m_stSample[i].dPro;
		//if(bAngleCalculationMode && m_stSample[i].t<0) m_dSamplePos[2] += (m_stSample[i].t+2.0*M_PI)*m_stSample[i].dPro;
		//else m_dSamplePos[2] += m_stSample[i].t*m_stSample[i].dPro;
		dx += cos(m_stSample[i].t)*m_stSample[i].dPro;
		dy += sin(m_stSample[i].t)*m_stSample[i].dPro;
	}

	if (dSum!=0.0) {
		m_dSamplePos[0] = m_dSamplePos[0]/dSum;
		m_dSamplePos[1] = m_dSamplePos[1]/dSum;
		m_dSamplePos[2] = atan2(dy/dSum, dx/dSum);
		if (m_dSamplePos[2]>M_PI) m_dSamplePos[2] -= 2.0*M_PI;
		else if (m_dSamplePos[2]<-M_PI) m_dSamplePos[2] += 2.0*M_PI;
	}

	// 값 전달.
	dPos[0] = m_dSamplePos[0];
	dPos[1] = m_dSamplePos[1];
	dPos[2] = m_dSamplePos[2];


	// 추정한 위치에서 예상되는 거리센서값 저장.
	// saving the range data predicted in the position estimated
//	predictRangeData(dPos[0], dPos[1], dPos[2], m_dRangeSensorAngleInterval);
	for(int i=0; i<m_FTD.size();i++) m_FTD[i].detect=false;
	for(int i=0; i<m_FRD.size();i++) m_FRD[i].detect=false;

	predictCeilingData(dPos[0], dPos[1], dPos[2], m_dRangeSensorAngleInterval);

	//memcpy(dEstimatedRangeData, m_dEstimatedRangeData, sizeof(m_dEstimatedRangeData));

	m_bMCLWorkingNow = false;
	return m_nLocalizationState;

}
void ImagebasedParticleFilter::getFeature(vector<CLAMPData>* FRD,vector<CFREAKData>* FTD)
{
	for(int i=0; i<m_FTD.size();i++) {(*FTD).push_back(m_FTD[i]);}
	for(int i=0; i<m_FRD.size();i++) {(*FRD).push_back(m_FRD[i]);}

}
/**
@brief Korean: 샘플들의 표준편차를 통하여 전역 위치추정 여부를 판단하는 함수
@brief English: Function checking whether global localization should be activated or not
*/
void ImagebasedParticleFilter::CheckLocalizationState()
{
	//	cout << "SD = "<<m_dSampleSD <<" " <<"MME = "<<m_dMinMatchingError<<endl;

	if(fabs(m_dSampleSD) > 350) //원래 250;
		m_nLocalizationState = GLOBAL_LOCALIZATION;

	//	else if(m_dMinMatchingError > 0.60)
	//		m_nLocalizationState = 2;
	else
		m_nLocalizationState = LOCAL_TRACKING;

}

/**
@brief Korean: MCL을 수행하는 함수
@brief English: Function performing MCL
*/
bool ImagebasedParticleFilter::doParticleFiltering()
{
	if(m_bgenerateSample==false) return true;

	bool bMotionModelResult;	// Motion Model을 수행하여 샘플이 지도 위에 남아있는지를 확인하는 변수

	m_bCalculationStop = false;

	//-----------------------------------------------------------------------------------//
	// 모션모델 적용
	// Motion Model
	bMotionModelResult = MotionModel();			// 엔코더 정보를 이용하여 각 샘플에 모션모델 적용.

	// 모션모델 적용 후 샘플이 모두 사라졌다면(장애물로 돌진) 에라처리.
	// After applying motion model, if every samples disappear, deactivate MCL and mark the error
	if (bMotionModelResult==false) {
		m_nMCLReturnValue = MCL_ERROR_ALLSAMPLES_DISAPPEARED;
		return false;
	}
	//-----------------------------------------------------------------------------------//

	//-----------------------------------------------------------------------------------//
	// 로봇이 일정거리 이상 움직인 경우만 센서 모델을 업데이트 한다.
	// Update only in the case that the robot moves over certain distance.
	if ( m_nLocalizationState==INITIAL_STATE) {
		if (CheckMovementForUpdateOrNot(GLOBAL_LOCALIZATION)==false) {
			return true;
		}
	}
	else {
		if (CheckMovementForUpdateOrNot(LOCAL_TRACKING)==false) {
			return true;
		}
	}

	//-----------------------------------------------------------------------------------//
	// 거리센서를 사용한 업데이트.
	// Sensor Model Update
	if(!SensorModel())
		return false;
	
	//-----------------------------------------------------------------------------------//
	// 확률 정규화
	// Normalizing the probabilities
	Normalizing();
	//-----------------------------------------------------------------------------------//

	m_nMCLReturnValue = MCL_DIVERGING_NOW;
	if ( m_bDoRangeUpdate||m_bDoVisionUpdate  ){
		Resampling();
	}

	//added CYK
	//check standard deviation of the samples and determine the localization state
	if(m_nLocalizationState != LOCAL_TRACKING)
	{
		CheckLocalizationState();
	}
	//if (m_nSampleNum<=m_nMinSampleNum && m_nLocalizationState==10) m_nLocalizationState= 0;
	// 전역위치추정 시 샘플이 수렴하면 local tracking 모드로 변환
	// 위치 표준편차가 MINSAMPLESTANDARDDEVIATION의 3배보다 작고, 각도 표준편차가 10도 보다 작으면 수렴했다고 판단.
	// 수렴을 제대로 했는지 판단하는 상태로 전환
	// Global localization -> if deviation of samples' position < 3 * MINSAMPLESTANDARDDEVIATION, and deviation of samples' angle < 10, local tracking mode is applied.
	if ( m_nLocalizationState==INITIAL_STATE && m_nSampleNum<=m_nMinSampleNum
		&& fabs(m_dSampleSD) < m_nMinSampleStandardDeviation*3.0
		&& fabs(m_dSampleSDForRotation) < 10.*M_PI/180.){
			m_nLocalizationState = LOCAL_TRACKING;
	}

	m_bDoRangeUpdate = false;
	m_bDoVisionUpdate= false;

	return true;
}
void ImagebasedParticleFilter::getSampleSD(double *dSampleSD,double *dSampleSDForRotation,double *dVisionWeight)
{
	(*dSampleSD)=m_dSampleSD;
	(*dSampleSDForRotation)=m_dSampleSDForRotation;
	(*dVisionWeight)=m_dVisionWeight;
}

/**
@brief Korean:  모션 모델을 기반으로 샘플들을 이동시키는 함수
@brief English: Function making every samples moving based on motion model
*/
bool ImagebasedParticleFilter::MotionModel()
{
	double dEncoderDelta2[3];
	//double dDeltaDistance, dDeltaTheta;
	int nCnt = 0;
	double s1,c1;
	double dPro;
	double dProTrans, dProTransRotate, dProRotate;
	double dMaxProTrans, dMaxProTransRotate, dMaxProRotate;
	double d1, d2, d3;
	double dNoiseTrans, dNoisexTrans, dNoiseyTrans,dNoiseTransRotate, dNoiseRotate;
	double dTransdistance;
	double dMaxTransdistance = 100;

	d1 = (double)m_dDeviationforTransConverged;					// 직선이동에 의한 위치오차의 표준편차
	d2 = (double)m_dDeviationforTransRotateConverged;			// 직선이동에 의한 각도오차의 표준편차
	d3 = (double)m_dDeviationforRotateConverged;				// 회전에 의한 각도오차의 표준편차
//  	double dMaxProMotion= 1.0/m_dMaxProForUpdate;
//  	double dMinProMotion= m_dMinProForUpdate*dMaxProMotion/m_dMaxProForUpdate;
	double dMaxProMotion= m_dMaxProForUpdate;
	double dMinProMotion= m_dMinProForUpdate;
	// Case that there are no encoder data, deactivating this function.
	if (m_dEncoderData[0]==0 && m_dEncoderData[1]==0 && m_dEncoderData[2]==0) return true;

	// 노이즈 추가. Konolige가 유도한 모션모델 사용.
	// Motion model induced by Konolige
	for (int i=0; i<m_nSampleNum; i++) {
		dTransdistance = sqrt(pow(m_dEncoderData[0],2)+pow(m_dEncoderData[1],2));			// 직선이동거리
		dNoiseTrans = GetRandValue(dTransdistance*d1);			// 직선운동에 따른 오차. 이동거리에 비례하여 증가한다.
		dNoiseTransRotate = GetRandValue(dTransdistance*d2);	// 직선운동에 따른 각도오차. 이동거리에 비례하여 증가한다.
		dNoiseRotate = GetRandValue(m_dEncoderData[2]*d3);
		//dDeltaDistance = dTransdistance + dNoiseTrans;

		dEncoderDelta2[0] = m_dEncoderData[0] + dNoiseTrans;
		dEncoderDelta2[1] = m_dEncoderData[1];
		dEncoderDelta2[2] = m_dEncoderData[2] // 각도 오차는,
		+dNoiseTransRotate // 직선이동에 의한 각도오차와
			+dNoiseRotate ; // 회전에 의한 각도오차를 통해 구해진다.
		 
		s1 = sin(m_stSample[i].t);	// yaw
		c1 = cos(m_stSample[i].t);

		// 오차가 포함된 엔코더 정보를 이용하여 샘플의 절대좌표계 상에서의 이동을 계산.
		// Calculation of the movement with encoder data and errors in the the absoulte coordinate

		m_stSample[i].x += (c1*dEncoderDelta2[0] + (-s1)*dEncoderDelta2[1]);
		m_stSample[i].y += (s1*dEncoderDelta2[0] + (c1)*dEncoderDelta2[1]);
		m_stSample[i].t += dEncoderDelta2[2];
		if (m_stSample[i].t>M_PI) m_stSample[i].t -=2*M_PI;
		else if (m_stSample[i].t<-M_PI) m_stSample[i].t +=2*M_PI;

		// 이동량에 따른 확률 갱신.
		// 엔코더로 예측한 값이 정확하다면 엔코더 변화량과 노이즈가 추가된 변화량이 일치해야 하므로,
		// 엔코더 변화량과 노이즈가 추가된 변화량 사이의 차이가 작을수록 높은 확률값 부여.

		dMaxProTrans = GetGaussianValue(dTransdistance*d1, 0.0);
		dProTrans = GetGaussianValue(dTransdistance*d1, dNoiseTrans);
		dProTrans=(dMaxProMotion-dMinProMotion)*dProTrans/dMaxProTrans+dMinProMotion;

		dProTransRotate = GetGaussianValue(dTransdistance*d2, dNoiseTransRotate);
		dMaxProTransRotate = GetGaussianValue(dTransdistance*d2, 0.0);
		dProTransRotate=(dMaxProMotion-dMinProMotion)*dProTransRotate/dMaxProTransRotate+dMinProMotion;

		dProRotate = GetGaussianValue(m_dEncoderData[2]*d3, dNoiseRotate);
		dMaxProRotate = GetGaussianValue(m_dEncoderData[2]*d3, 0.0);
		dProRotate=(dMaxProMotion-dMinProMotion)*dProRotate/dMaxProRotate+dMinProMotion;


		m_stSample[i].dPro *= (dProTrans+dProTransRotate+dProRotate)/3.0;//*dProTransRotate/+dProRotate)/2.0;

		// --------------------------------------- 제거할 샘플을 제거하는 부분 --------------------- //
		// 지도 범위 바깥에 추출된 샘플은 무시.
		// Reducing the samples extracted out of the reference map
		if (m_stSample[i].x<STARTPOS_MARGIN*3 || m_stSample[i].x >= m_math.AI2MM(m_nMapSizeX-3)
			|| m_stSample[i].y< STARTPOS_MARGIN*3  || m_stSample[i].y >= m_math.AI2MM(m_nMapSizeY-3)){

				m_stSample[i].dPro = 0.0;
		}

		//else if ( m_nGridMap[(int)(0.5+m_stSample[i].x/100.0)][(int)(0.5+m_stSample[i].y/100.0)] != 0)
		else if ( m_nGridMap[(int)(0.5+m_stSample[i].x/100.0)][(int)(0.5+m_stSample[i].y/100.0)] != 0
			&&m_nGridMap[(int)(0.5+m_stSample[i].x/100.0)][(int)(0.5+m_stSample[i].y/100.0)] != 1)// 장애
			m_stSample[i].dPro = 0.0;
		// 온전한 것이 몇개인지 셈.
		// Count of extracted samples in the reference map
		else nCnt++;
		// ----------------------------------------------------------------------------------------- //
	}

	if (nCnt==0){
		return false;	// 살아남은 샘플이 없을 때 // case of no samples in the map
	}
	else{
		return true;
	}
}
bool ImagebasedParticleFilter::SensorModel()
{
	double dRangeUpdatePro =0.0;
	double dVisionUpdatePro =0.0;	

	if (m_bDoRangeUpdate){
		if (!UpdateUsingRangeSensor()) {
			m_bDoRangeUpdate = false;
			return false;
		}
		for(int i = 0; i<m_nSampleNum; i++){
			dRangeUpdatePro+=m_stSample[i].dRangeUpdatePro;
		}
		dRangeUpdatePro=dRangeUpdatePro/(double)(m_nSampleNum);
	}
	 if (m_bDoVisionUpdate) {
		if (!UpdateUsingCeilingSensor()) {
			m_bDoVisionUpdate = false;
			return false;
		}
		for(int i = 0; i<m_nSampleNum; i++){
			dVisionUpdatePro+=m_stSample[i].dVisionUpdataPro;
		}
		dVisionUpdatePro=dVisionUpdatePro/(double)(m_nSampleNum);
		// Update the probability of each sample		
	}

	 for(int i = 0; i<m_nSampleNum; i++){
		 if (m_bCalculationStop) return false;	
		 
		 if(dRangeUpdatePro>6.0&&dRangeUpdatePro>dVisionUpdatePro)
			m_stSample[i].dPro = m_stSample[i].dPro *( m_stSample[i].dRangeUpdatePro);
		else
		 if(dVisionUpdatePro>1.0)
			 m_stSample[i].dPro = m_stSample[i].dPro *( m_stSample[i].dVisionUpdataPro);
		 else 
		 {
				 double dDistforMotion=hypot(m_dSamplePos[0]-m_stSample[i].x, m_dSamplePos[1]-m_stSample[i].y);
				 double dDegreeforMotion=sqrt((fabs)(m_dSamplePos[2]-m_stSample[i].t))/M_PI;

				 double dDistProR=0;
				 if(dDistforMotion<300) { dDistProR=1.0;}
				 else{dDistProR=300/dDistforMotion;}

				 double dProthetaT=0;
				 if(dDegreeforMotion<5.0*D2R) { dProthetaT=1.0;}
				 else{dProthetaT=5.0*D2R/dDegreeforMotion;}

				 m_stSample[i].dPro = m_stSample[i].dPro *dDistProR*dProthetaT;
		
	
		 }

	 }

	printf("dRangeUpdatePro=%f, dVisionUpdatePro=%f\n",dRangeUpdatePro,dVisionUpdatePro);
	return true;

}

/**
@brief Korean: 이중 matching된 결과 제거
@brief English: 
*/
void ImagebasedParticleFilter::symmetryTest(vector<DMatch>& matches1, vector<DMatch>& matches2, vector<DMatch>& symMatches)
{
	// for all matches image#1 -> image#2
	for(int i=0; i<matches1.size(); i++)
	{
		// ignore deleted matches
		if(matches1.size()<2)
			continue;

		// for all matches image#2 -> image#1
		for(int j=0; j<matches2.size(); j++)
		{
			// ignore deleted matches
			if(matches2.size() < 2)
				continue;

			// Match symmetry test
			if( ( matches1[i].queryIdx == matches2[j].trainIdx) && ( matches2[j].queryIdx == matches1[i].trainIdx))
			{	
				// add symmetrical match
				symMatches.push_back(DMatch( matches1[i].queryIdx, matches1[i].trainIdx, matches1[i].distance));
				break;		// next match in image#1 -> image#2
			}

		}

	}

}
bool ImagebasedParticleFilter::UpdateUsingCeilingSensor()
{

	LARGE_INTEGER t;
	startTimeCheck(t);

	m_FDetectorAlg.init();
	m_observedFRD = m_FDetectorAlg.detectRegion(m_cvMatImageBright,false);		

	m_observedFTD.clear();
	vector<KeyPoint> keypointsA, keypointsB;
	KeyPoint keypoints;
	Mat descriptorsA, descriptorsB;
	vector<DMatch> matches;
	vector<DMatch> matchesA,matchesB;
	BruteForceMatcher<Hamming> matcher;

	vector<CFREAKData> FTData = m_FDetectorAlg.checktempletRegion(m_cvMatImage,false);
	int nIDX[200]={0};
	int ntempIDX=0;

	for(int i=0; i<m_FTD.size();i++)
	{
		m_FTD[i].detect =false;
		
		if(hypot(m_dSamplePos[0]/1000.0-m_FTD[i].x,m_dSamplePos[1]/1000.0-m_FTD[i].y)<3.0) 
		{
			keypoints.angle = m_FTD[i].angle;
			keypoints.response = m_FTD[i].response;
			keypoints.pt.x = m_FTD[i].u;
			keypoints.pt.y = m_FTD[i].v;
			keypointsA.push_back(keypoints);
			descriptorsA.push_back(m_FTD[i].sourceImage);	
			nIDX[ntempIDX]=i;
			ntempIDX++;
		}		
	}

	if(FTData.size()>0&&keypointsA.size()>0)
	{
		for(int i=0; i<FTData.size();i++)
		{
			keypoints.angle = FTData[i].angle;
			keypoints.response = FTData[i].response;
			keypoints.pt.x = FTData[i].u;
			keypoints.pt.y = FTData[i].v;

			keypointsB.push_back(keypoints);
			descriptorsB.push_back(FTData[i].sourceImage);
		}

		matcher.match(descriptorsA, descriptorsB, matchesA);
		matcher.match(descriptorsB, descriptorsA, matchesB);
		symmetryTest(matchesB, matchesA,  matches);	
	}


	if(matches.size()>0)
	{
		for(int m=0;matches.size()>m;m++)
		{
			if(matches[m].distance<100)//&&matches[m].distance!=-1)
			{
				FTData[matches[m].queryIdx].MatchNum=nIDX[matches[m].trainIdx];
				if(matches[m].distance<1) FTData[matches[m].queryIdx].response=1.0;
				else FTData[matches[m].queryIdx].response=matches[m].distance;
				m_observedFTD.push_back(FTData[matches[m].queryIdx]);
			}

		}
	}
	

		for (int i=0; i<m_nSampleNum; i++) {

			if (m_bCalculationStop){return false;}

			m_stSample[i].dVisionUpdataPro = predictCeilingData(m_stSample[i].x, m_stSample[i].y , m_stSample[i].t, m_dRangeSensorAngleInterval);

		}



	double dtimeRegion = finishTimeCheck(t);
	//printf("dtimeRegion=%f\n",dtimeRegion);

	return true;
}

/**
@brief Korean: 센서의 거리정보를 기반으로 하여 샘플들 각각의 사후확률을 갱신하는 함수
@brief English: Function updating probabilities of samples by range data of sensor
*/
bool ImagebasedParticleFilter::UpdateUsingRangeSensor()
{

	int nCnt;	// the number of samples
	double dAlpha=0.0;
	nCnt = 0;
	double dMaxPro=0.0;
	double dMinPro=1000000.0;
	double dError=0.0;

	for (int j=0;j<m_nNoOfRangeSensorPoint;j=j+m_dRangeSensorAngleInterval) {
		if (m_dRangeData[j] >= m_dMaxDistance) {
			m_dRangeData[j] = m_dMaxDistance;
		}
		else if (m_dRangeData[j] == 0){
			m_dRangeData[j] = m_dMaxDistance;
		}
	}

	for (int i=0; i<m_nSampleNum; i++) {
		if (m_bCalculationStop){
			return false;
		}

		predictRangeData(m_stSample[i].x, m_stSample[i].y , m_stSample[i].t, m_dRangeSensorAngleInterval);

		nCnt = 0;
		m_stSample[i].dRangeUpdatePro = 0.0;	// Sample에 할당되는 확률
		//m_stSample[i].dMatchingError = 0.0;		

		for (int j=0;j<m_nNoOfRangeSensorPoint;j=j+m_dRangeSensorAngleInterval) {
		
			if(m_dRangeData[j] >= m_dMaxDistance||m_dRangeData[j]<50){ // 감지가능 거리 이상의 거리가 감지됫을 경우에는 확률을 계산하지 않음 // Pass probability calculation of each sample when range is sensed over the range of the sensor
				continue;
			}

			if (m_dEstimatedRangeData[j]==0||m_dEstimatedRangeData[j] >= m_dMaxDistance){
				m_dEstimatedRangeData[j] = m_dMaxDistance;
			}
				
			m_stSample[i].dRangeUpdatePro += (double)m_fRangeMCLPro[(int)(0.5 + m_dEstimatedRangeData[j]/10.0)][(int)(0.5 + m_dRangeData[j]/10.0)];
			nCnt++;
		}

		if (nCnt>0){
			m_stSample[i].dRangeUpdatePro /= (double)nCnt;
		}
	}

	return true;

}

/**
@brief Korean: 확률값을 이용하여 샘플 복사 및 확률값 정규화
@brief English: Function redistributing samples according to prior probabilities of samples
*/
void ImagebasedParticleFilter::Resampling()
{
	Accumulation *pAccumulation;

	double dSum = 0.0;
	int nNoOfAccumulated;
	int nCnt;
	double dRandPro;
	int i;
	double dSumPro = 0.0;
	double dSampleMeanX=0.0;
	double dSampleMeanY=0.0;
	double dSampleMeanTheta=0.0;

	pAccumulation = NULL;
	pAccumulation = (Accumulation*)calloc(m_nSampleNum,sizeof(Accumulation));

	//TRACE("SD: %.3f\n", m_dSampleSD);
	if (m_dSampleSD<m_nMinSampleStandardDeviation&&m_dSampleSDForRotation<10/180*M_PI)
	{
		CalculateNoOfSamples(false);
		//cout<<"[CLaserBasedParticleFilter] : No sensor update\n"<<endl;
	}
	else
	{
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


		// 모든 샘플이 벽속으로 들어가 버리면 resampling 단계에서 복사할 수 있는 샘플이 없음.
// 		if (nNoOfAccumulated==0) {
// 			m_nMCLReturnValue = MCL_ERROR_ALLSAMPLES_DISAPPEARED;
// 
// 		}

		// 샘플의 위치의 표준편차를 이용하여 다음 단계 샘플의 갯수를 계산.
		// Calculation of the number of samples by using deviation of the sample pose
		CalculateNoOfSamples(true);

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

		//방법 1, 샘플의 개수에 상관없이 추출하는 방법.
		while(nTemp<m_nSampleNum && dSum != 0.0){
			nCnt = 0;
			dRandPro = dSum*(double)rand()/(double)RAND_MAX;
			while (pAccumulation[nCnt].dSum<dRandPro && nCnt<nNoOfAccumulated-1){
				nCnt++;
			}
			m_stSample[nTemp].x = pAccumulation[nCnt].x;
			m_stSample[nTemp].y = pAccumulation[nCnt].y;
			m_stSample[nTemp].t = pAccumulation[nCnt].t;
			m_stSample[nTemp].dPro = m_dBasePro;
			nTemp++;
		
		}
	}

	// 작업용 동적배열 삭제.
	free (pAccumulation);
	pAccumulation = NULL;
}

/**
@brief Korean: 지정된 영역에 샘플을 골고루 뿌리고 확률값을 초기화시키는 함수
@brief English: Function distributing samples uniformly on predefined region with normalized probabilities.
*/
void ImagebasedParticleFilter::setSamples()
{
	////TRACE("MCL: before reset samples\n");
	int nCnt = 0;

	m_bResetSamples = false;
	m_nLocalizationState = INITIAL_STATE;
	m_dSampleSD = m_nMinSampleStandardDeviation;

	// MCL 결과 저장 변수 초기화.
	m_nMCLReturnValue = MCL_CONVERGING_YET;

	// 샘플이 뿌려질 영역 및 SAMPLE_DENSITY값에 따른 샘플 갯수 계산.
	m_nSampleNum = m_nMaxSampleNum;

	// 최대값 이상, 최소값 이하의 샘플 갯수가 계산되면 최대값, 최소값으로 대치.
	if (m_nSampleNum > m_nMaxSampleNum) m_nSampleNum = m_nMaxSampleNum;
	if (m_nSampleNum < m_nMinSampleNum) m_nSampleNum = m_nMinSampleNum;

	// 정규화된 샘플의 확률값.
	m_dBasePro = 1.0/(double)m_nSampleNum;

	while(nCnt<m_nSampleNum)
	{
		// 샘플이 뿌려지는 영역을 고려하여 임의의 x, y, theta 값 입력.
		m_stSample[nCnt].x = (double)m_nSampleRegion[0] + (float)rand()/(float)RAND_MAX*(double)(m_nSampleRegion[1]-m_nSampleRegion[0]);
		m_stSample[nCnt].y = (double)m_nSampleRegion[2] + (float)rand()/(float)RAND_MAX*(double)(m_nSampleRegion[3]-m_nSampleRegion[2]);
		m_stSample[nCnt].t = GetRandValue(M_PI/2);

		// 지도 범위 바깥에 추출된 샘플은 무시.
		if ((int)(0.5+m_stSample[nCnt].x/100.)<0 || (int)(0.5+m_stSample[nCnt].x/100.) >= m_nMapSizeX
			|| (int)(0.5+m_stSample[nCnt].y/100.) < 0 || (int)(0.5+m_stSample[nCnt].y/100.) >= m_nMapSizeY ){
				continue;
		}

		if ( (m_nGridMap[(int)(0.5+m_stSample[nCnt].x/100.0)][(int)(0.5+m_stSample[nCnt].y/100.0)] == m_UnkownArea )
			|| (m_nGridMap[(int)(0.5+m_stSample[nCnt].x/100.0)][(int)(0.5+m_stSample[nCnt].y/100.0)] == m_OccupiedArea ) ){
				continue;
		}

		// 확률값 초기화.
		m_stSample[nCnt].dPro = m_dBasePro;
		nCnt++;
	}


	// 그림그리기 위해 샘플정보 한 번 넘겨주기
	m_nOldSampleNum = m_nSampleNum;
	m_vecOldSample.clear();
	Sample stSample;
	for (int i=0; i<m_nSampleNum; i++) {
		if (m_stSample[i].dPro==0.0) continue;
		stSample.x = m_stSample[i].x;
		stSample.y = m_stSample[i].y;
		stSample.t = m_stSample[i].t;
		stSample.dPro = m_stSample[i].dPro;
		stSample.dRangeUpdatePro = m_stSample[i].dRangeUpdatePro;
		m_vecOldSample.push_back(stSample);
	}
	m_vecOldSampleCopy = m_vecOldSample;

	m_bgenerateSample=true;

}

/**
@brief Korean: 일정한 로봇 주변 반경으로 샘플들을 뿌려주는 함수
@brief English: Function distributing samples near constant region around the robot.
*/
void ImagebasedParticleFilter::setSamplesNearRobot(double dx,double dy,double dt,double dSize)
{
	////TRACE("MCL: before reset samples\n");
	int nCnt = 0;
	int nSampleRegion[4];

	nSampleRegion[0] = (int)(dx-(dSize*1000));
	nSampleRegion[1] = (int)(dx+(dSize*1000));
	nSampleRegion[2] = (int)(dy-(dSize*1000));
	nSampleRegion[3] = (int)(dy+(dSize*1000));

	m_bResetSamples = false;
	m_nLocalizationState = INITIAL_STATE;
	m_dSampleSD = m_nMinSampleStandardDeviation;

	// MCL 결과 저장 변수 초기화.
	m_nMCLReturnValue = MCL_CONVERGING_YET;

	// 샘플이 뿌려질 영역 및 SAMPLE_DENSITY값에 따른 샘플 갯수 계산.
	m_nSampleNum = m_nMinSampleNum;

	// 정규화된 샘플의 확률값.
	m_dBasePro = 1.0/(double)m_nSampleNum;

	//TRACE("%d, %d, %d, %d\n", nSampleRegion[0], nSampleRegion[1], nSampleRegion[2], nSampleRegion[3]);
	while(nCnt<m_nSampleNum)
	{
		// 샘플이 뿌려지는 영역을 고려하여 임의의 x, y, theta 값 입력.
		m_stSample[nCnt].x = (double)nSampleRegion[0] + (float)rand()/(float)RAND_MAX*(double)(nSampleRegion[1]-nSampleRegion[0]);
		m_stSample[nCnt].y = (double)nSampleRegion[2] + (float)rand()/(float)RAND_MAX*(double)(nSampleRegion[3]-nSampleRegion[2]);
		m_stSample[nCnt].t = dt;// + 2.0 * (0.5 - (float)rand()/(float)RAND_MAX) * 10.0/180.0 * M_PI;

		// 지도 범위 바깥에 추출된 샘플은 무시.
		if ((int)(0.5+m_stSample[nCnt].x/100.)<0 || (int)(0.5+m_stSample[nCnt].x/100.) >= m_nMapSizeX
			|| (int)(0.5+m_stSample[nCnt].y/100.)<0 || (int)(0.5+m_stSample[nCnt].y/100.) >= m_nMapSizeY)
			continue;

		// 빈 공간이 아닌 곳에 추출된 샘플은 무시.
		if ( (m_nGridMap[(int)(0.5+m_stSample[nCnt].x/100.0)][(int)(0.5+m_stSample[nCnt].y/100.0)] == m_UnkownArea)
			|| (m_nGridMap[(int)(0.5+m_stSample[nCnt].x/100.0)][(int)(0.5+m_stSample[nCnt].y/100.0)] == m_OccupiedArea) )
			continue;

		// 확률값 초기화.
		m_stSample[nCnt].dPro = m_dBasePro;
		nCnt++;
	}
	////TRACE("MCL: after reset samples\n");
	// 그림그리기 위해 샘플정보 한 번 넘겨주기
	m_nOldSampleNum = m_nSampleNum;
	m_vecOldSample.clear();
	Sample stSample;
	for (int i=0; i<m_nSampleNum; i++) {
		if (m_stSample[i].dPro==0.0) continue;
		stSample.x = m_stSample[i].x;
		stSample.y = m_stSample[i].y;
		stSample.t = m_stSample[i].t;

		stSample.dPro = m_stSample[i].dPro;
		stSample.dRangeUpdatePro = m_stSample[i].dRangeUpdatePro;
		m_vecOldSample.push_back(stSample);
	}
	m_vecOldSampleCopy = m_vecOldSample;

	m_bgenerateSample=true;


}


/**
@brief Korean:  샘플의 위치 분산을 이용하여 다음 단계에서 추출할 샘플의 갯수를 계산.
@brief English: calculate number of samples which will be extracted based on standard deviation of sample positions.
*/
void ImagebasedParticleFilter::CalculateNoOfSamples(bool bCalSampleCnt)
{
	double dAvgPose[3], dSD[3];		// 샘플의 평균 및 표준편차를 구하기 위한 변수.

	if (bCalSampleCnt) {
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
		m_dSampleSDForRotation = sqrt(dSD[2]);

		dSD[0] = sqrt(dSD[0]);
		dSD[1] = sqrt(dSD[1]);

		// 샘플의 표준편차를 이용하여 다음 단계에 추출될 샘플의 갯수를 계산.
		m_nSampleNum = (int)( 1.0*(dSD[0]*dSD[1]/1000.0/1000.0)*m_nSampleDensity );

		// 최대값 이상, 최소값 이하의 샘플 갯수가 계산되면 최대값, 최소값으로 대치.
		if (m_nSampleNum > m_nMaxSampleNum) m_nSampleNum = m_nMaxSampleNum;
		if (m_nSampleNum < m_nMinSampleNum) m_nSampleNum = m_nMinSampleNum;
	}
	else {
		// 확률을 고려하지 않고 샘플 각각의 위치만 고려하여 분산을 구함.
		dAvgPose[0] = 0.0;
		dAvgPose[1] = 0.0;
		dAvgPose[2] = 0.0;
		dSD[0] = 0.0;
		dSD[1] = 0.0;
		dSD[2] = 0.0;

		for(int i = 0; i<m_nSampleNum; i++) {
			dAvgPose[0] += m_stSample[i].x;
			dAvgPose[1] += m_stSample[i].y;
		}
		dAvgPose[0] /= (double)m_nSampleNum;
		dAvgPose[1] /= (double)m_nSampleNum;
		for(int i = 0; i<m_nSampleNum; i++) {
			dSD[0] += (m_stSample[i].x-dAvgPose[0])*(m_stSample[i].x-dAvgPose[0]);
			dSD[1] += (m_stSample[i].y-dAvgPose[1])*(m_stSample[i].y-dAvgPose[1]);
		}
		m_dSampleSD = sqrt( (dSD[0]+dSD[1])/(double)m_nSampleNum);
		m_dSampleSDForRotation=sqrt(dSD[2]/(double)m_nSampleNum);
	}
}


/**
@brief Korean:  샘플들의 확률을 정규화하는 함수.
@brief English: Function normalizing probabilities of samples
*/
void ImagebasedParticleFilter::Normalizing()
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
@brief Korean:  거리 정보를 설정하는 함수
@brief English: Function inserting range data to member variable m_dRangeData
*/
void ImagebasedParticleFilter::setRangeData(int nRangeData[Sensor::URG04LX_DATA_NUM181])
{
	//memcpy(m_dRangeData, nRangeData, sizeof(m_dRangeData));
	for(int i=0; i<181; i++){
		m_dRangeData[i] = (double)(nRangeData[i]);
	}
	m_bDoRangeUpdate = true;

}


/**
@brief Korean:  Encoder 정보를 멤버 변수에 입력하는 함수
@brief English: Function inserting encoder data to member variable m_dAccumulatedDistance, and m_dAccumulatedRotation
*/
void ImagebasedParticleFilter::setEncoderData(double dEncoderDelta[3])
{
	memcpy(m_dEncoderData, dEncoderDelta, sizeof(m_dEncoderData));

	m_dAccumulatedDistance += sqrt(m_dEncoderData[0]*m_dEncoderData[0]+m_dEncoderData[1]*m_dEncoderData[1]);
	m_dAccumulatedRotation += fabs(m_dEncoderData[2]);
}

/**
@brief Korean: 거리센서의 파라미터 정보를 입력하는 함수
@brief English: Function setting parameters of range sensor
*/
void ImagebasedParticleFilter::setRangeSensorParameter(int nNo, int nMinAngle, double dInterval, double dMaxDistance,double dRangeSensorOffset)
{
	m_nNoOfRangeSensorPoint = nNo;
	m_dRangeSensorAngleInterval = dInterval;
	m_nRangeSensorMinAngle = nMinAngle;
	m_dMaxDistance = dMaxDistance;
	m_dRangeSensorOffset = dRangeSensorOffset;
	initProbability();
}
/**
@brief Korean: 
@brief English: 
*/
void ImagebasedParticleFilter::setCameraParameter( double dCam_f,double dHeight,double dWidth)
{
	m_dCam_f=dCam_f;
	m_FDetectorAlg.setEllipseData(dWidth,dHeight);
}

/**
@brief Korean: 모션에 대한 불확실성 정보를 받아오는 함수
@brief English: Function setting uncertainty of wheel motion
*/
void ImagebasedParticleFilter::setDeviation(double dDeivationforTrans, double dDeviationforRotate,double dDeviationforTransRotate )
{
	m_dDeviationforTransConverged = dDeivationforTrans;
	m_dDeviationforRotateConverged = dDeviationforRotate;
	m_dDeviationforTransRotateConverged= M_PI/180.0*dDeviationforTransRotate;
}

/**
@brief Korean: 샘플의 최대,최소 개수를 받아오는 함수
@brief English: Function setting maximum and minimum sample numbers
*/
void ImagebasedParticleFilter::setSampleNum(int nMaxSample,int nMinSample)
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
double ImagebasedParticleFilter::GetRandValue(double sigma)
{
	return sigma*2.0*( (double)rand()/(double)RAND_MAX - 0.5);
}

/**
@brief Korean: 평균 0, sigma인 가우시안 분포를 따르는 값을 출력하는 함수
@brief English: Function returning values according to Gaussian distribution with sigma and x
*/
double ImagebasedParticleFilter::GetGaussianValue(double sigma, double x)
{
	if (sigma==0) return 0.1;

	return 1/sqrt(2*M_PI*sigma*sigma)*exp(-x*x/2.0/sigma/sigma);
}

/**
@brief Korean: 샘플들의 정보를 리턴해주는 함수
@brief English: Function getting samples' information
*/
vector<Sample> ImagebasedParticleFilter::getParticle()
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
vector<Sample> ImagebasedParticleFilter::GetOldParticle()
{
	return m_vecOldSampleCopy;
}

/**
@brief Korean: 샘플이 뿌려질 영역을 설정하는 함수
@brief English: Function setting region where sample is distributed
*/
void ImagebasedParticleFilter::SetSampleRegion(int minX, int maxX, int minY, int maxY)
{
	// 거리단위 : mm, minimum = 100mm, maximum = size of a reference map
	if (minX>100)
		m_nSampleRegion[0] = minX;
	else m_nSampleRegion[0] = 100;

	if (maxX<m_nMapSizeX*100-100)
		m_nSampleRegion[1] = maxX;
	else m_nSampleRegion[1] = m_nMapSizeX*100-100;

	if (minY>100)
		m_nSampleRegion[2] = minY;
	else m_nSampleRegion[2] = 100;

	if (maxY<m_nMapSizeY*100-100)
		m_nSampleRegion[3] = maxY;
	else m_nSampleRegion[3] = m_nMapSizeY*100-100;

}

/**
@brief Korean: 샘플들의 위치를 업데이트 하기 위해 일정 이상 로봇이 움직였는지 확인하는 함수
@brief English: Function checking the movement of the robot for updating position of the samples
*/
bool ImagebasedParticleFilter::CheckMovementForUpdateOrNot(int nMode)
{
	// for global localization: nMode = 0
	// for local tracking: nMode = 1
	////TRACE("%.3f\n", m_dAccumulatedDistance);
	if (nMode == GLOBAL_LOCALIZATION) {
		if (m_dAccumulatedDistance > 300.0 || m_dAccumulatedRotation > 10.0*M_PI/180.0) {
			m_dAccumulatedDistance = 0.0;
			m_dAccumulatedRotation = 0.0;
			return true;
		}
		return false;
	}

	if (nMode == LOCAL_TRACKING) {
		if (m_dAccumulatedDistance > 100.0 || m_dAccumulatedRotation > 3.0*M_PI/180.0) {
			m_dAccumulatedDistance = 0.0;
			m_dAccumulatedRotation = 0.0;
			return true;
		}
		return false;
	}

	return false;
}

/**
@brief Korean: 샘플의 지도 상의 (x,y,t)위치에서 예상되는 거리센서 값을 구하는 함수
@brief English:  Function predicting range values of the samples on (x,y,t) based on map data
*/
inline void ImagebasedParticleFilter::predictRangeData(double x, double y, double t, int nGap)
{
	int nPosX1=0,nPosY1=0;
	int nPosX2=0,nPosY2=0;
	int nHitX=0, nHitY=0;	// Ray-tracing을 통해 얻은 지도에서의 샘플 각각의 거리정보

	double dAngle=0.;
	double dPosTh=0.;

	// 100으로 나눠주는 이유는 10cm격자에 맞춰서 방향을 이동시키기 위함이다.
	nPosX1 = (int)((x + m_dRangeSensorOffset * cos(t)) / 100.0);		// 샘플의 위치에서 레이저 센서 방향에 따른 X방향 방향벡터
	nPosY1 = (int)((y + m_dRangeSensorOffset * sin(t))/ 100.0);			// 샘플의 위치에서 레이저 센서 방향에 따른 Y방향 방향벡터
	dPosTh = t*R2D + (float)+m_nRangeSensorMinAngle;					// 샘플의 위치에서 레이저 센서 방향을 나타내는 변수
	dPosTh = dPosTh*D2R;

	for (int i=0; i<m_nNoOfRangeSensorPoint; i=i+nGap, dAngle+=m_dRangeSensorAngleInterval){

		nPosX2 = nPosX1 + (int)(m_dMaxDistance*cos(dPosTh)/100.);	// 센서 감지 거리에 따른 X방향의 최대 감지 범위 설정.	
		nPosY2 = nPosY1 + (int)(m_dMaxDistance*sin(dPosTh)/100.);	// 센서 감지 거리에 따른 Y방향의 최대 감지 범위 설정.
		//getRayTracingData(nPosX1, nPosX2, nPosY1, nPosY2, &nHitX, &nHitY); // Ray-tracing 수행
		doRayTracing(nPosX1, nPosX2, nPosY1, nPosY2, &nHitX, &nHitY); // Ray-tracing 수행
		// 지도의 경계 근처에 있는 경우를 따지는 부분
		if (nHitX<=10 || nHitX >= (m_nMapSizeX-10) || nHitY<=10 || nHitY >=(m_nMapSizeY-10)) {
			m_dEstimatedRangeData[i]=0.;
		}
		else{
			m_dEstimatedRangeData[i] = sqrt((double)(nHitX*100-nPosX1*100)*(nHitX*100-nPosX1*100)+
				(double)(nHitY*100-nPosY1*100)*(nHitY*100-nPosY1*100)
				);
		}
		dPosTh= dPosTh + (1*D2R);
	}

}
/**
@brief Korean: 샘플의 지도 상의 (x,y,t)위치에서 예상되는 거리센서 값을 구하는 함수
@brief English:  Function predicting range values of the samples on (x,y,t) based on map data
*/
inline void  ImagebasedParticleFilter::getRayTracingData(int x_1, int x_2, int y_1, int y_2, int *hit_x, int *hit_y)
{
	int nX=0;
	int nY=0;

	double dGradientX = (double)(x_2 -x_1)/100;		
	double dGradientY =(double)(y_2- y_1)/100;		

	for(int nDistance = 0; nDistance< m_dMaxDistance/100; nDistance += 5) // 10 cm step
	{
		double dRayOfX =(double) nDistance*dGradientX;		//레이저의 X방향으로 간격만큼 증가 시킴
		double dRayOfY =(double) nDistance*dGradientY;		//레이저의 Y방향으로 간격만큼 증가 시킴

		int nX = (int)( x_1+dRayOfX );	//nX를 확률지도상의 센서데이터의 X좌표 인덱스로사용
		int nY = (int)( y_1+dRayOfY );	//nY를 확률지도상의 센서데이터의 Y좌표 인덱스로사용
		if(nX < 1 || nX >= m_nMapSizeX || nY < 1 || nY >= m_nMapSizeY) continue;

		if(m_nGridMap[nX][nY] == m_UnkownArea || m_nGridMap[nX][nY] == m_OccupiedArea) 
		{		
			*hit_x =(int)nX;
			*hit_y = (int)nY;
			break;

		}
	}
}

/**
@brief Korean: 로봇 위치로부터 측정된 거리 위치까지의 차이를 격자 값으로 나타내는 함수
@brief English: 
*/
inline void ImagebasedParticleFilter::doRayTracing(int x_1, int x_2, int y_1, int y_2, int *hit_x, int *hit_y)
{

	int eps = 0;
	int x_thres=x_2, y_thres=y_2;

	if(x_1 < 0) x_1 = 0;
	else if(x_1 >= m_nMapSizeX) x_1 = m_nMapSizeX-1;
	if(y_1 < 0) y_1 = 0;
	else if(y_1 >= m_nMapSizeY) y_1 = m_nMapSizeY-1;

	if(x_thres < 0) x_thres = 0;
	else if(x_thres >= m_nMapSizeX) x_thres = m_nMapSizeX-1;
	if(y_thres < 0) y_thres = 0;
	else if(y_thres >= m_nMapSizeY) y_thres = m_nMapSizeY-1;

	int x = x_1,y = y_1;

	int delx = x_thres - x_1;
	int dely = y_thres - y_1;


	if(delx > 0) {
		if(dely >=0) {
			if(delx > dely) {
				for(int x=x_1; x<=x_thres; x++) {
					if(m_nGridMap[x][y] == m_UnkownArea || m_nGridMap[x][y] == m_OccupiedArea) {
						// 거리값을 넣는다.
						*hit_x = x;
						*hit_y = y;
						return;
					}

					eps += dely;
					if((eps<<1) >= delx) {
						y++;
						eps -= delx;
					}
				}

			}
			else { //delx <= dely인 경우

				for(y=y_1; y<=y_thres; y++) {
					if(m_nGridMap[x][y] == m_UnkownArea || m_nGridMap[x][y] == m_OccupiedArea) {
						// 거리값을 넣는다.
						*hit_x = x;
						*hit_y = y;
						return;
					}

					eps += delx;
					if((eps<<1) >= dely) {
						x++;
						eps -= dely;
					}
				}

			}

		}
		else { // dely < 0인경유
			if(delx > -dely) {
				for(x=x_1; x<=x_thres; x++) {
					if(m_nGridMap[x][y] == m_UnkownArea || m_nGridMap[x][y] == m_OccupiedArea) {
						// 거리값을 넣는다.
						*hit_x = x;
						*hit_y = y;
						return;
					}

					eps += dely;
					if((eps<<1) <= -delx) {
						y--;
						eps += delx;
					}
				}

			}
			else { //delx <= -dely인 경우

				for(y=y_1; y>=y_thres; y--) {
					if(m_nGridMap[x][y] == m_UnkownArea || m_nGridMap[x][y] == m_OccupiedArea) {
						// 거리값을 넣는다.
						*hit_x = x;
						*hit_y = y;
						return;
					}

					eps += delx;
					if((eps<<1) >= -dely) {
						x++;
						eps += dely;
					}
				}

			}
		}
	}

	else { //delx <=0인경우
		if(dely >= 0) {
			if(-delx > dely) {

				for(x=x_1; x>=x_thres; x--) {
					if(m_nGridMap[x][y] == m_UnkownArea || m_nGridMap[x][y] == m_OccupiedArea) {
						// 거리값을 넣는다.
						*hit_x = x;
						*hit_y = y;
						return;
					}

					eps += dely;
					if((eps<<1) >= -delx) {
						y++;
						eps += delx;
					}
				}

			}
			else { //-delx <= dely인경우

				for(y=y_1; y<=y_thres; y++) {
					if(m_nGridMap[x][y] == m_UnkownArea || m_nGridMap[x][y] == m_OccupiedArea) {
						// 거리값을 넣는다.
						*hit_x = x;
						*hit_y = y;
						return;
					}

					eps += delx;
					if((eps<<1) <= -dely) {
						x--;
						eps += dely;
					}
				}

			}
		}
		else { //dely < 0인경우
			if(-delx > -dely) {
				for(x=x_1; x>=x_thres; x--) {
					if(m_nGridMap[x][y] == m_UnkownArea || m_nGridMap[x][y] == m_OccupiedArea) {
						// 거리값을 넣는다.
						*hit_x = x;
						*hit_y = y;
						return;
					}

					eps -= dely;
					if((eps<<1) > -delx) {
						y--;
						eps += delx;
					}
				}

			}
			else { //-delx <= -dely인경우
				for(y=y_1; y>=y_thres; y--) {
					if(m_nGridMap[x][y] == m_UnkownArea || m_nGridMap[x][y] == m_OccupiedArea) {
						// 거리값을 넣는다.
						*hit_x = x;
						*hit_y = y;
						return;
					}

					eps -= delx;
					if((eps<<1) >= -dely) {
						x--;
						eps += dely;
					}
				}

			}
		}
	}

}




inline void ImagebasedParticleFilter::trans_global2img(const float& fRx, const float& fRy, const float& fRth, const float& fFx, const float& fFy, const float& fFz, int& nu, int& nv, const CamContext* pContext)
{
	double fCRth=cos(fRth);
	double fSRth=sin(fRth);
	double a11=fCRth;
	double a12=-fSRth;
	double a14=fRx + pContext->cam_offset_x * fCRth;
	double a21=fSRth;
	double a22=fCRth;
	double a24=fRy+ pContext->cam_offset_x * fSRth;
	// 	float a33=1.0;
	// 	float a44=1.0;

	double b11=a11;
	double b12=-a12;
	double b13=0.0;
	double b14=a12*a24-a14*a22;

	double b21=-a21;
	double b22=a11;
	double b23=0.0;
	double b24=a14*a21-a11*a24;

	double b31=0.0;
	double b32=0.0;
	double b33=a11*a22-a12*a21;
	double b34=0.0;

	double b41=0.0;
	double b42=0.0;
	double b43=0.0;
	double b44=a11*a22-a12*a21;	

	double d=b31*fFx+b32*fFy+b33*fFz;
	double Det=1;//a11*a22-a12*a21;

	nv = 1.0/(fFz*Det)*(pContext->cam_fy*(b11*fFx+b12*fFy+b13*fFz+b14)+pContext->img_center_v*d);
	nu =1.0/(fFz*Det)*(pContext->cam_fx *(b21*fFx+b22*fFy+b23*fFz+b24)+pContext->img_center_u*d);
}

inline void ImagebasedParticleFilter::trans_img2global(const float& fRx, const float& fRy, const float& fRth, const int& nu, const int& nv, float& fFx, float& fFy, float& fFz, const CamContext* pContext,double dHeight)
{
	double fCRth=cos(fRth);
	double fSRth=sin(fRth);
	double a11=fCRth;
	double a12=-fSRth;
	double a14=fRx + pContext->cam_offset_x * fCRth;
	double a21=fSRth;
	double a22=fCRth;
	double a24=fRy+ pContext->cam_offset_x * fSRth;
	double a33=1.0;
	double a44=1.0;

	double b11=pContext->cam_fx;
	double b13=-pContext->img_center_v*pContext->cam_fx ;
	double b22=pContext->cam_fy;
	double b23= -pContext->cam_fy*pContext->img_center_u;
	double b33=pContext->cam_fx*pContext->cam_fy;

	double Det=pContext->cam_fx*pContext->cam_fy;

	double A1=dHeight/Det*(b11*nv+b13);
	double A2=dHeight/Det*(b22*nu+b23);
	double A3=dHeight/Det*b33;

	fFx = a11*A1+a12*A2+a14;
	fFy = a21*A1+a22*A2+a24;
	fFz = a33*A3;

}
