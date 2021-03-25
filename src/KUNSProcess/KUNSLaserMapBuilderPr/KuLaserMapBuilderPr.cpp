#include "stdafx.h"
#include "KuLaserMapBuilderPr.h"

KuLaserMapBuilderPr::KuLaserMapBuilderPr(void)
{

	m_nCellSize = 100;				// 1cell 100mm;
	MIN_PROBABILITY = 0.2;		// 센서 모델이 갖는 최소 확률 값
	MAX_PROBABILITY = 0.8;		// 센서 모델이 갖는 최대 확률 값
	INITIAL_PROBABILITY = 0.5;	// 초기 확률: 0.5 (unknown region)

	m_dThicknessofWall= 50;  // 벽의 두께 (50mm)

	m_dRadiusofRobot = 400; //로봇반지름 400(mm)

	GAUSSIAN_SD = 50;			// 50mm; 센서 모델의 가우시안 표준편차 값, 점유 영역의 폭을 조절//0.4

	m_pMap =  NULL;
	m_nRefMap=NULL;
	m_LaserscannerConfigurationFront = NULL; //Laser위치 정보
	m_LaserscannerConfigurationRear = NULL;

	m_nScanIdX = -1;
	m_nLaserMinDist = -1;
	m_nLaserMaXDist = -1; 
	m_nMapSizeX=0, m_nMapSizeY=0;

	m_nLaserData = m_KuUtil.generateIntType1DArray(Sensor::URG04LX_DATA_NUM181,0);
}

KuLaserMapBuilderPr::~KuLaserMapBuilderPr(void)
{
	if(NULL!=m_pMap){
		delete m_pMap;		
	}
	if(NULL!=m_nRefMap){
		delete []m_nRefMap;		
	}
	if(NULL!=m_LaserscannerConfigurationFront){		
		delete [] m_LaserscannerConfigurationFront;
	}
	if(m_LaserscannerConfigurationRear != NULL)
	{
		delete [] m_LaserscannerConfigurationRear;
	}
}

/**
@brief Korean: 초기화 함수
@brief English: 
*/
void KuLaserMapBuilderPr::initialize(KuMapBuilderParameter InputParamFront, KuMapBuilderParameter InputParamRear)
{
	int nMapSizeXm=0, nMapSizeYm=0;
	int i;

	m_dRadiusofRobot = InputParamFront.getRadiusofRobot();
	m_dThicknessofWall =InputParamFront.getThicknessofWall();
	m_nCellSize= InputParamFront.getCellSize();
	m_nScanIdX = InputParamFront.getLaserScanIdx();	//LRF 데이터 개수
	m_nLaserMinDist = InputParamFront.getMinDistofSensorData();//센서 측정가능 최소거리
	m_nLaserMaXDist = InputParamFront.getMaxDistofSensorData();//센서 측정가능 최대거리
	GAUSSIAN_SD = InputParamFront.getSigma();//센서 측정가능 최대거
	double dCellSize=m_nCellSize;

	InputParamFront.getMapSizeXmYm(&nMapSizeXm, &nMapSizeYm);
	m_nMapSizeX = nMapSizeXm * M2MM * 1/dCellSize; //m -> mm 변환하고 mm-> 한격자가 100mm인 배열 형태로 변환
	m_nMapSizeY = nMapSizeYm * M2MM * 1/dCellSize; //m -> mm 변환하고 mm-> 한격자가 100mm인 배열 형태로 변환

	if(m_pMap==NULL)
	{
		m_pMap = new KuMap(m_nMapSizeX, m_nMapSizeY); 
		m_dProMap = m_pMap->getProbMap();
		m_nMap = m_pMap->getMap();
		
		if(NULL!=m_nRefMap){delete []m_nRefMap;}
		m_nRefMap = new int*[m_nMapSizeX];
		if(m_nRefMap){
			for(int i = 0 ; i < m_nMapSizeX ; i++){
				m_nRefMap[i] = new int[m_nMapSizeY];
			}
		}
	}
	else
	{
		if(NULL!=m_pMap&&(m_pMap->getX()!=m_nMapSizeX||m_pMap->getY()!=m_nMapSizeY)){
			delete m_pMap;
			m_pMap = new KuMap(m_nMapSizeX, m_nMapSizeY); 
		}

		m_dProMap = m_pMap->getProbMap();
		m_nMap = m_pMap->getMap();
	
// 		if(m_nRefMap!=NULL)
// 		{
// 			for(int i = 0 ; i < m_nMapSizeX ; i++){
// 				delete[] m_nRefMap[i];
// 				m_nRefMap[i] = NULL;
// 			}
// 			delete[] m_nRefMap;
// 
// 			m_nRefMap = new int*[m_nMapSizeX];
// 
// 			if(m_nRefMap){
// 				for(int i = 0 ; i < m_nMapSizeX ; i++){
// 					m_nRefMap[i] = new int[m_nMapSizeY];
// 				}
// 			}
// 		}	
	}

	// 확률 맵 초기화
	for(int i = 0; i<m_nMapSizeX; i++){
		for(int j = 0; j< m_nMapSizeY;j++){
			m_dProMap[i][j] = INITIAL_PROBABILITY;
			m_nMap[i][j] = KuMap::UNKNOWN_AREA;
		}
	}
	

	if(m_LaserscannerConfigurationFront==NULL)
	{
		//실제 레이저의 위치를 반영하기 위해서 값을 넣어 준다.------------------------------------
		m_LaserscannerConfigurationFront = new KuPose[m_nScanIdX];
	}
// 	else
// 	{
// 		if(NULL!=m_LaserscannerConfiguration)
// 		{	
// 			delete [] m_LaserscannerConfiguration;	
// 		}		
// 		//실제 레이저의 위치를 반영하기 위해서 값을 넣어 준다.------------------------------------
// 		m_LaserscannerConfiguration = new KuPose[m_nScanIdX];
// 	}

	for(i=0; i<m_nScanIdX; i++)
	{
		double dAngleDeg = (double)(i - (double)m_nScanIdX/2.0);
		m_LaserscannerConfigurationFront[i].setX(InputParamFront.getLaserXOffset());
		m_LaserscannerConfigurationFront[i].setY(0.0);
		m_LaserscannerConfigurationFront[i].setThetaDeg(dAngleDeg);
	}

	if(m_LaserscannerConfigurationRear == NULL)
	{
		m_LaserscannerConfigurationRear = new KuPose[m_nScanIdX];
	}

	for(i = 0; i < m_nScanIdX; i++)
	{
		double dAngleDeg = (double)(i - (double)m_nScanIdX/2.0 - 180);
		m_LaserscannerConfigurationRear[i].setX(InputParamRear.getLaserXOffset());
		m_LaserscannerConfigurationRear[i].setY(0.0);
		m_LaserscannerConfigurationRear[i].setThetaDeg(dAngleDeg);
	}
}
/**
@brief Korean: 모든 지도정보를 초기화 해주는 함수
@brief English: 
*/
void KuLaserMapBuilderPr::initMap()
{
	// 확률 맵 초기화
	for(int i = 0; i<m_nMapSizeX; i++){
		for(int j = 0; j< m_nMapSizeY;j++){
			m_dProMap[i][j] = INITIAL_PROBABILITY;
			m_nMap[i][j] = KuMap::UNKNOWN_AREA;
		}
	}
}
/**
@brief Korean: KuLaserMapBuilderPr의 시작 함수
@brief English: 
*/
void KuLaserMapBuilderPr::buildMapFront(KuMapBuilderParameter InputParam)
{
	KuPose RobotPos = InputParam.getRobotPos();//로봇 위치
	KuPose DelRobotPos = InputParam.getDelRobotPos();//엔코더 데이터의 로봇위치

	int_1DArray nData = InputParam.getLaserData();	//레이저 데이터 받아옴

	for(int i=0; i<m_nScanIdX; i++)
	{
		if(InputParam.getLaserUpdateSpeedflag())
		{
			if(nData[i]==-1)
				m_nLaserData[i] = m_nLaserMaXDist;
			else
				m_nLaserData[i] = nData[i];
		}
		else
		{
			m_nLaserData[i] = nData[i];
		}
	}

	buildGridmapByBayesUpdate(RobotPos,m_nLaserData, m_nScanIdX, m_LaserscannerConfigurationFront, InputParam.getLaserUpdateSpeedflag());//베이즈룰에 따른 확률값 업데이트 함수 호출
}

/**
@brief Korean: KuLaserMapBuilderPr의 시작 함수
@brief English: 
*/
void KuLaserMapBuilderPr::buildMapRear(KuMapBuilderParameter InputParam)
{
	KuPose RobotPos = InputParam.getRobotPos();//로봇 위치
	KuPose DelRobotPos = InputParam.getDelRobotPos();//엔코더 데이터의 로봇위치

	int_1DArray nData = InputParam.getLaserData();	//레이저 데이터 받아옴

	for(int i=0; i<m_nScanIdX; i++)
	{
		if(InputParam.getLaserUpdateSpeedflag())
		{
			if(nData[i]==-1)
				m_nLaserData[i] = m_nLaserMaXDist;
			else
				m_nLaserData[i] = nData[i];
		}
		else
		{
			m_nLaserData[i] = nData[i];
		}
	}

	buildGridmapByBayesUpdate(RobotPos,m_nLaserData, m_nScanIdX, m_LaserscannerConfigurationRear, InputParam.getLaserUpdateSpeedflag());//베이즈룰에 따른 확률값 업데이트 함수 호출
}

/**
@brief Korean: 베이즈룰에 따른 확률값 업데이트 함수
@brief English: 
*/
void KuLaserMapBuilderPr::buildGridmapByBayesUpdate(KuPose RobotPos, int_1DArray nLaserData, int nLaserIdx, KuPose* pSensorConfig, bool bupdateSpeedflag)
{
	double dCellSize = m_nCellSize;			// 격자지도의 격자 사이즈(10cm X 10cm)
	double dPro;							// 가우시안분포를 갖는 확률밀도함수의계산값
	double dInitPro = INITIAL_PROBABILITY;	// 초기 확률: 0.5 (unknown region)
	double dMinPro = MIN_PROBABILITY;		// 센서 모델이 갖는 최소 확률 값
	double dMaxPro = MAX_PROBABILITY;		// 센서 모델이 갖는 최대 확률 값
	double dSigma = GAUSSIAN_SD;			// 50mm; 가우시안 모델의 표준편차 값, 점유 영역의 폭을 조절할 수 있음//0.4
	double dThicknessofWall = m_dThicknessofWall;     // 지도의 두께를 나타내는 변수 mm
	double dRadiusofRobot_cell =m_dRadiusofRobot/dCellSize;
	double dLold, dLnew, dLsensor;							// Log odds form 값
	double dLo = (double)log(dInitPro/(1.0-dInitPro));		// 초기 log값
	int nIntervalDistance=dCellSize/10.0;

	for (int nSensorNO=0; nSensorNO<nLaserIdx; nSensorNO++) // Scan data 개수
	{
		double dData = (double)nLaserData[nSensorNO];

		if (dData > m_nLaserMaXDist) {dData = m_nLaserMaXDist;}//LRF최대측정가능거리보다 큰데이터는 최대 측정거리로 맞춰줌
		if (dData < m_nLaserMinDist) {continue;}//LRF최소측정가능거리보다 작은데이터는 사용하지 않음

		// 센서의 위치
		KuPose dSensorPose = m_Math.getTransformedPose(0.0, pSensorConfig[nSensorNO], RobotPos);	//실제 센서위치
		KuPose dDetectPose = m_Math.getTransformedPose(dData, pSensorConfig[nSensorNO], RobotPos);	//레이저데이터가 검출된 곳의 위치	

		double dGradientX = (dDetectPose.getX() - dSensorPose.getX())/(dCellSize*1000);		//센서로부터 레이저데이터가 검출된 위치까지 X거리
		double dGradientY = (dDetectPose.getY() - dSensorPose.getY())/(dCellSize*1000);		//센서로부터 레이저데이터가 검출된 위치까지 Y거리

		int nRobotX=RobotPos.getX()/dCellSize;
		int nRobotY=RobotPos.getY()/dCellSize;

		for(int i=-dRadiusofRobot_cell; i<dRadiusofRobot_cell;i++)
		{
			for(int j=-dRadiusofRobot_cell; j<dRadiusofRobot_cell;j++)
			{
				if (nRobotX+i<0 || nRobotY+j<0 || nRobotX+i>=m_pMap->getX() || nRobotY+j>=m_pMap->getY()) {continue;}	// 맵사이즈를 벗어날 경우 예외 처리
				m_dProMap[nRobotX+i][nRobotY+j] =0.01;//로봇위치 주변 반경 4x4맵을 비점유영역으로 설정
				m_nMap[nRobotX+i][nRobotY+j] = KuMap::EMPTY_AREA;	
			}
		}

		double dRay = dData;
		int nCheckX=0, nCheckY=0;
		double dGridDist = 0;
		for (int nDistance=0; dGridDist<dData+dThicknessofWall; nDistance+=dCellSize) //ndistance는 증가시킬 간격
		{
			double dRayOfX = nDistance*dGradientX;		//레이저의 X방향으로 간격만큼 증가 시킴
			double dRayOfY = nDistance*dGradientY;		//레이저의 Y방향으로 간격만큼 증가 시킴
			dGridDist=hypot(dRayOfX, dRayOfY);
			dRay = dData-dGridDist;		//레이저 데이터로부터 내부영역 확률 계산을위해 센서로부터 레이저데이터가 검출된 위치까지 거리에 nDistance수치를 곱한 값을 뺀거리

			if(dGridDist >= m_nLaserMaXDist*0.95) {continue;}	//받아들인 레이저가 최대값이상 일때//0.8

			int nX = (int)( 0.5 + (dSensorPose.getX()+dRayOfX)/dCellSize );	//nX를 확률지도상의 센서데이터의 X좌표 인덱스로사용(0.5는 반올림)
			int nY = (int)( 0.5 + (dSensorPose.getY()+dRayOfY)/dCellSize );	//nY를 확률지도상의 센서데이터의 Y좌표 인덱스로사용(0.5는 반올림)
			if (nX<0 || nY<0 || nX>=m_pMap->getX() || nY>=m_pMap->getY()) {continue;}	// 맵사이즈를 벗어날 경우 예외 처리
			if (nX==nCheckX && nY==nCheckY) {continue;}									// 중복 격자 예외 처리

			if (bupdateSpeedflag) //로컬 맵빌딩시 즉시 점유 시키기위해 레이저의 끝에 높은 확률값을줌
			{
				int nEndGridX = (int)( 0.5 + dDetectPose.getX()/dCellSize );
				int nEndGridY = (int)( 0.5 + dDetectPose.getY()/dCellSize );

				if (nX==nEndGridX && nY==nEndGridY) {
					m_dProMap[nX][nY] = 0.95;
				}
				else {
					m_dProMap[nX][nY] = 0.05;
				}
			}
			else 
			{
				// Bayesian update formula===========================================================
				dPro = 1.6*dSigma * (exp( -pow( (double)((dRay)/dSigma) , 2) / 2)) / (sqrt(2.0*M_PI)*dSigma) + dMinPro;
				if(dData<dGridDist)dPro=0.9;
				dLsensor = (double)log(dPro/(1.0-dPro));						//inverse sensor model
				dLold = (double)log(m_dProMap[nX][nY]/(1.0-m_dProMap[nX][nY]));	//이전 맵확률의 log odds form				                                                                
				dLnew = dLold + dLsensor - dLo;//점유 확률을 log odds 형태로 계산

				m_dProMap[nX][nY] = (double)(1.0 - 1.0/ (1.0 + exp(dLnew)));//log odds 형태를 확률값으로 변환

				if (m_dProMap[nX][nY]>0.99) //최대확률 값을 벗어날때 0.99로 제한
				{ 
					m_dProMap[nX][nY]=0.99; 
				}
				else if (m_dProMap[nX][nY]<0.01) //최소확률 값을 벗어날때 0.01로 제한
				{ 
					m_dProMap[nX][nY]=0.01;
				}
				//===================================================================================
			}			

			if(m_nRefMap[nX][nY] == KuMap::FIXED_CAD_AREA) continue;
		
			//격자의 점유상태를 int 형 지도에 저장-----------------------------------------------
			if (m_dProMap[nX][nY] > MAX_PROBABILITY){ 
				m_nMap[nX][nY] = KuMap::OCCUPIED_AREA; }		//최대확률보다 크다면 점유 영역
			else if (m_dProMap[nX][nY] < MIN_PROBABILITY){ 
				m_nMap[nX][nY] = KuMap::EMPTY_AREA;	}		//최소확률보다 작다면 비점유 영역
			else{ 
				m_nMap[nX][nY] = KuMap::UNKNOWN_AREA;}		// 미지 영역
			//------------------------------------------------------------------------------------

			//중복격자인지 검사하기 위해
			nCheckX = nX;
			nCheckY = nY;
		} // for문 끝		
	} // for문 끝

}

/**
@brief Korean: 확률 맵데이터를 가져가는 함수
@brief English: 
*/
double** KuLaserMapBuilderPr::getProMap()
{
	return m_dProMap;
}

/**
@brief Korean: 맵데이터를 가져가는 함수
@brief English: 
*/
int** KuLaserMapBuilderPr::getMap()
{
	return m_nMap;
}
/**
@brief Korean: 맵데이터를 가져가는 함수
@brief English: 
*/
KuMap* KuLaserMapBuilderPr::getpMap()
{
	return m_pMap;
}

/**
@brief Korean: 센서 모델의 가우시안 표준편차값을 설정하는 함수
@brief English: 
*/
void KuLaserMapBuilderPr::setSigma(double dSigma)
{
	GAUSSIAN_SD = dSigma;	
}

void KuLaserMapBuilderPr::setReferenceCADMap(int** nMap )
{

	// 확률 맵 초기화
	for(int i = 0; i<m_nMapSizeX; i++){
		for(int j = 0; j< m_nMapSizeY;j++){
			m_nRefMap[i][j] = nMap[i][j];
		}
	}
}