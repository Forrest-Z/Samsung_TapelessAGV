#include "stdafx.h"
#include "KuICPLocalizerPr.h"


KuICPLocalizerPr::KuICPLocalizerPr()
{
	initialize(); //각종 변수 초기화 함수 실행.
	cout<<"[KuICPLocalizerPr]: Instance is created!!!"<<endl;

}
KuICPLocalizerPr::~KuICPLocalizerPr()
{
	cout<<"[KuICPLocalizerPr]: Instance is destroyed!!!"<<endl;
}


/**
@brief Korean: 각종 변수등을 초기화 하는 함수
@brief English : 
*/
void KuICPLocalizerPr::initialize()
{
	m_RobotPos.init();	
	m_DeltaPosForICP.init();
	m_LastRobotPosForICP.init();


	for(int i=0; i< 181; i++) {
		m_CartesianCoordiNewRangeData[i].setXm(0.);
		m_CartesianCoordiNewRangeData[i].setYm(0.);
		m_CartesianCoordiLastRangeData[i].setXm(0.);
		m_CartesianCoordiLastRangeData[i].setYm(0.);
	}
	
	m_dAccumulatedDeltaMovementForICP = 0.0;	
	m_dLaserSensorOffsetmm = KuRobotParameter::getInstance()->getFrontLaserXOffset(); //레이저 센서와 로봇 중심간의 거리, unit mm
	m_dMaxSensorRangeMM = 1/3.0*KuRobotParameter::getInstance()->getURG04LXLaserMaxDist(); //사용할 센서데이터의 최대 거리, unit mm
	m_dMinSensorRangeMM= KuRobotParameter::getInstance()->getURG04LXLaserMinDist(); //사용할 센서데이터의 최대 거리, unit mm

	// ICP가 수행될 간격. 이 거리마다 ICP가 수행. 
	// 너무 작으면 오차 발생. 최소 0.3~0.5m를 권장
	// 너무 크면 오차 발생. 특히 좁은 구역에서 작으면 매칭되는 포인트가 적어서 각도가 확~ 틀어짐.
	m_dDistanceForICPExecution = 0.35;
}

/**
@brief Korean: 절대좌표상의 x(mm)좌표를 다른 클래스에서 가져감
@brief English : 
*/
double KuICPLocalizerPr::getRobotPosX()
{
	return m_RobotPos.getX();	
}

/**
@brief Korean: 절대좌표상의 y(mm)좌표를 다른 클래스에서 가져감
@brief English : 
*/
double KuICPLocalizerPr::getRobotPosY()
{
	return m_RobotPos.getY();	
}

/**
@brief Korean: 절대좌표상의 각도(deg)를 다른 클래스에서 가져감
@brief English : 
*/
double KuICPLocalizerPr::getRobotPosDeg()
{
	return m_RobotPos.getThetaDeg();	

}

/**
@brief Korean: 절대 좌표상의 X(mm)좌표를 저장함 
@brief English : 
*/
void KuICPLocalizerPr::setRobotPosX(double dRobotPosX)
{
	m_RobotPos.setX(dRobotPosX);	
	m_LastRobotPosForICP.setX(dRobotPosX); 
}

/**
@brief Korean: 절대 좌표상의 Y(mm)좌표를 저장함 
@brief English : 
*/
void KuICPLocalizerPr::setRobotPosY(double dRobotPosY)
{
	m_RobotPos.setY( dRobotPosY ); 	
	m_LastRobotPosForICP.setY( dRobotPosY );	
}

/**
@brief Korean: 절대 좌표상의 각도를 저장함 
@brief English : 
*/
void KuICPLocalizerPr::setRobotPosDeg(double dRobotPosThetaDeg)
{
	m_RobotPos.setThetaDeg( dRobotPosThetaDeg );
	m_LastRobotPosForICP.setThetaDeg(dRobotPosThetaDeg);	
}

/**
@brief Korean: 절대 좌표상의 로봇 위치를 저장함. 
@brief English : 
*/
void KuICPLocalizerPr::setRobotPos(KuPose RobotPos)
{
	m_RobotPos = RobotPos;
	m_LastRobotPosForICP = RobotPos;	
	m_dAccumulatedDeltaMovementForICP = 0.0;
}

/**
@brief Korean: 절대 좌표상의 로봇 위치를 넘겨준다.
@brief English : 
*/
KuPose KuICPLocalizerPr::getRobotPos()
{
	return m_RobotPos;
}


/**
@brief Korean: 절대 좌표상의 초기 로봇 위치를 저장함. 
@brief English : 
*/
void KuICPLocalizerPr::setInitRobotPos(KuPose RobotPos)
{
	m_RobotPos = RobotPos;
	m_LastRobotPosForICP = RobotPos;
}


/**
@brief Korean: 거리센서 값을 저장함. 
			   poloar coordinate기준의 거리 센서 값을, 
			   cartesian coordinate기준의 값을 변경한다.
@brief English : 
*/
void KuICPLocalizerPr::copyRangeData(int_1DArray  nData)
{
	KuCartesianCoordinate2D CartesianCoordiData;
	KuCartesianCoordinate2D PreCartesianCoordiData;

	for(int i=0; i<LASER_SCAN_IDX; i++){

		if (nData[i]< m_dMaxSensorRangeMM &&
			nData[i]>m_dMinSensorRangeMM)
		{	// 레이저스캐너의 유효거리를 벗어나면 처리 안함.

			CartesianCoordiData=m_math.transfromPolar2CartesianMM( nData[i], (i-90), m_dLaserSensorOffsetmm );

			if(0.05<hypot(PreCartesianCoordiData.getXm()-CartesianCoordiData.getXm(),PreCartesianCoordiData.getYm()-CartesianCoordiData.getYm()))
			{			
				m_CartesianCoordiNewRangeData[i] = CartesianCoordiData;
				PreCartesianCoordiData = CartesianCoordiData;
			}
		}	
		else
		{
			m_CartesianCoordiNewRangeData[i].setXm(0.);
			m_CartesianCoordiNewRangeData[i].setYm(0.);
		}
	}
}

/**
@brief Korean:ICP 알고리듬을 이용하여 로봇의 위치를 추정하는 함수
@brief English: 
*/
KuPose KuICPLocalizerPr::estimateRobotPos(int_1DArray Data, KuPose DeltaEncoderData)
{
	static bool bfirst= true;
	copyRangeData(Data);

	if(bfirst)
	{
		memcpy(m_CartesianCoordiLastRangeData, m_CartesianCoordiNewRangeData, sizeof(m_CartesianCoordiLastRangeData));
		bfirst=false;
		return m_RobotPos;
	}
	computeAccumulatedDeltaMovementForICP(DeltaEncoderData); //ICP수행을 위해 엔코더에서 얻은 delta x,y,theta. 변화량을 누적한다.

	bool bICPExecution = isAccDeltaMovementForICPOver(m_dDistanceForICPExecution); //ICP를 수행할지를 정하는 함수.

	if(bICPExecution)
	{	
		float ftransX=0.0;
		float ftransY=0.0;
		float ftransTheta=0.0;

		double dErrorNewtoLast = calDeltaPosebyRangeData(m_CartesianCoordiLastRangeData,m_CartesianCoordiNewRangeData,
			m_DeltaPosForICP,&ftransX,&ftransY,&ftransTheta);
		
		m_RobotPos = computeRobotPoseByICPFor2DfromRobotPos((double)ftransX, (double) ftransY, (double) ftransTheta,
			DeltaEncoderData,m_RobotPos);

		m_DeltaPosForICP.init();
	
		memcpy(m_CartesianCoordiLastRangeData, m_CartesianCoordiNewRangeData, sizeof(m_CartesianCoordiLastRangeData));
	}
	else
	{
		calculateRobotPosByEncoderData(DeltaEncoderData);
	}
	//setDeltaPose(DeltaEncoderData);

	return m_RobotPos;
}		


KuPose KuICPLocalizerPr::computeRobotPoseByICPFor2DfromRobotPos(double dX, double dY, double dTheta,KuPose DeltaPos,KuPose PreRobotPos)
{

	KuPose CurRobotPos;

	double s1=0.,c1=0.;
	double dXm =0.0;
	double dYm =0.0;


	// ------------------------- movement by encoder data ------------------------- //
	// yaw 계산에 반복되는 싸인,코싸인값 미리 계산
	s1 = sin(PreRobotPos.getThetaRad() );	// yaw
	c1 = cos(PreRobotPos.getThetaRad() );


	// localizer를 수행하여 보정하는 위치
	dXm = (c1 * DeltaPos.getXm() -s1 * DeltaPos.getYm());
	dYm = (s1 * DeltaPos.getXm() + c1 * DeltaPos.getYm());

	PreRobotPos.setXm( PreRobotPos.getXm() + dXm);
	PreRobotPos.setYm( PreRobotPos.getYm() + dYm);
	PreRobotPos.setThetaRad( PreRobotPos.getThetaRad() + DeltaPos.getThetaRad() );


	// ------------------------- motion correction by ICP result ------------------------- //
	// 반복되는 싸인,코싸인값 미리 계산
	s1 = sin(PreRobotPos.getThetaRad() );	// yaw
	c1 = cos(PreRobotPos.getThetaRad());


	// localizer를 수행하여 보정하는 위치
	dXm = (c1*dX -s1*dY);
	dYm = (s1*dX + c1*dY);

	CurRobotPos.setXm( PreRobotPos.getXm() + dXm);
	CurRobotPos.setYm( PreRobotPos.getYm() + dYm);
	CurRobotPos.setThetaRad( PreRobotPos.getThetaRad() + dTheta);

	return CurRobotPos;
}
double KuICPLocalizerPr::calDeltaPosebyRangeData(KuCartesianCoordinate2D* CartesianCoordiLastRangeData, 
	KuCartesianCoordinate2D* CartesianCoordiNewRangeData,KuPose DeltaPos,float *fx,float *fy,float *ft)
{
	int NoNewData, NoPreData;

	// -------------------------------------------------------- ICP ---------------------------------------------------------------- //			
	double PreData[362], NewData[362]; //--> 181*2 181은 레이저 데이터 개수, 2는 x,y
	double Tmatrix[3], Rmatrix[3][3];
	double RMSError=-1;

	// 형식에 맞춰서 센서값 입력
	NoPreData = 0;
	for (int i=0; i<LASER_SCAN_IDX; i++) {
		double dDIst= sqrt(CartesianCoordiLastRangeData[i].getXm() * CartesianCoordiLastRangeData[i].getXm() + 
			CartesianCoordiLastRangeData[i].getYm() * CartesianCoordiLastRangeData[i].getYm() );
		if (  dDIst<= m_dMaxSensorRangeMM*MM2M&&dDIst> m_dMinSensorRangeMM*MM2M) {
			PreData[NoPreData*2+0] = CartesianCoordiLastRangeData[i].getXm();
			PreData[NoPreData*2+1] = CartesianCoordiLastRangeData[i].getYm();
			NoPreData++;
		}
	}
	NoNewData = 0;

	for (int i=0; i<LASER_SCAN_IDX; i++) {
		double dDIst= sqrt(m_CartesianCoordiNewRangeData[i].getXm() * m_CartesianCoordiNewRangeData[i].getXm() + 
			m_CartesianCoordiNewRangeData[i].getYm() * m_CartesianCoordiNewRangeData[i].getYm() );
		if (  dDIst<= m_dMaxSensorRangeMM*MM2M&&dDIst> m_dMinSensorRangeMM*MM2M)  {
			NewData[NoNewData*2+0] = DeltaPos.getXm() + CartesianCoordiNewRangeData[i].getXm() * cos(DeltaPos.getThetaRad() ) - 
				CartesianCoordiNewRangeData[i].getYm() * sin(DeltaPos.getThetaRad() );
			NewData[NoNewData*2+1] = DeltaPos.getYm() + CartesianCoordiNewRangeData[i].getXm() * sin(DeltaPos.getThetaRad() ) + 
				CartesianCoordiNewRangeData[i].getYm() * cos( DeltaPos.getThetaRad()  );
			NoNewData++;
		}
	}
	double dthreshold=0.01;
	//printf("NoPreData=%d, NoNewData=%d\n",NoPreData,NoNewData);

	// ICP 수행
	if(NoPreData>100&&NoNewData>100)
		RMSError = m_pICP.icp(Rmatrix, Tmatrix, PreData, NoPreData, NewData, NoNewData, 30, 100, 0.9, dthreshold, 5.0, DeltaPos.getXm(), DeltaPos.getYm(),DeltaPos.getThetaRad());

	//printf("(%.3f, %.3f, %.3f) - (%.3f, %.3f, %.3f)\n", Rmatrix[0][0], Rmatrix[1][1], Rmatrix[2][2],Tmatrix[0],Tmatrix[1],Tmatrix[2]);		
	// ----------------------------------------------------------------------------------------------------------------------------- //

	// motion update by ICP
	if (RMSError!=-1 && RMSError<1.0){
		(*fx)=Tmatrix[0];
		(*fy)=Tmatrix[1];
		(*ft)=Rmatrix[0][0];
	}
	else{
		(*fx)=0.0;
		(*fy)=0.0;
		(*ft)=0.0;
		RMSError=1.0;
	}

	return RMSError;
}


bool KuICPLocalizerPr::setDeltaPose(KuPose DeltaPos)
{
	bool bReturn = false;
	int NoNewData, NoPreData;
		
	m_DeltaPos = DeltaPos; // 엔코더에서 얻은 delta x,y,theta. 변화량. 절대값 아님.
	bool bICPExecution = isAccDeltaMovementForICPOver(m_dDistanceForICPExecution); //ICP를 수행할지를 정하는 함수.
	if (bICPExecution) {
		// -------------------------------------------------------- ICP ---------------------------------------------------------------- //			
		double PreData[362], NewData[362]; //--> 181*2 181은 레이저 데이터 개수, 2는 x,y
		double Tmatrix[3], Rmatrix[3][3];
		double RMSError;

		// 형식에 맞춰서 센서값 입력
		NoPreData = 0;
		for (int i=0; i<LASER_SCAN_IDX; i++) {
			if ( sqrt(m_CartesianCoordiLastRangeData[i].getXm() * m_CartesianCoordiLastRangeData[i].getXm() + 
						  m_CartesianCoordiLastRangeData[i].getYm() * m_CartesianCoordiLastRangeData[i].getYm() ) <= m_dMaxSensorRangeMM*MM2M) {
					PreData[NoPreData*2+0] = m_CartesianCoordiLastRangeData[i].getXm();
					PreData[NoPreData*2+1] = m_CartesianCoordiLastRangeData[i].getYm();
					NoPreData++;
			}
		}
		NoNewData = 0;

		for (int i=0; i<LASER_SCAN_IDX; i++) {
			if ( sqrt( m_CartesianCoordiNewRangeData[i].getXm() * m_CartesianCoordiNewRangeData[i].getXm() + 
				       m_CartesianCoordiNewRangeData[i].getYm() * m_CartesianCoordiNewRangeData[i].getYm() ) <= m_dMaxSensorRangeMM*MM2M) {
				NewData[NoNewData*2+0] = m_DeltaPosForICP.getXm() + m_CartesianCoordiNewRangeData[i].getXm() * cos(m_DeltaPosForICP.getThetaRad() ) - 
																	m_CartesianCoordiNewRangeData[i].getYm() * sin(m_DeltaPosForICP.getThetaRad() );

				NewData[NoNewData*2+1] = m_DeltaPosForICP.getYm() + m_CartesianCoordiNewRangeData[i].getXm() * sin(m_DeltaPosForICP.getThetaRad() ) + 
																	m_CartesianCoordiNewRangeData[i].getYm() * cos( m_DeltaPosForICP.getThetaRad()  );
				NoNewData++;
			}
		}

		// ICP 수행
		RMSError = m_pICP.icp(Rmatrix, Tmatrix, PreData, NoPreData, NewData, NoNewData, 30, 100, 0.2, 0.01, 3.0);
		//printf("(%.3f, %.3f, %.3f) - (%.3f, %.3f, %.3f)\n", Rmatrix[0][0], Rmatrix[1][1], Rmatrix[2][2],Tmatrix[0],Tmatrix[1],Tmatrix[2]);		
		// ----------------------------------------------------------------------------------------------------------------------------- //

		// motion update by ICP
		memcpy(m_CartesianCoordiLastRangeData, m_CartesianCoordiNewRangeData, sizeof(m_CartesianCoordiLastRangeData));
		if (RMSError!=-1 && RMSError<1.0){
			computeRobotPoseByICPFor2D(Tmatrix[0], Tmatrix[1], Rmatrix[0][0]);
		}
		else{
			computeRobotPoseByICPFor2D(0,0,0);
		}
			bReturn = true;
	} 
	else{ //ICP를 수행하지 않는 상황에서는 엔코더 데이터 만으로 로봇의 위치를 계산한다.
		calculateRobotPosByEncoderData();
	}
		
	computeAccumulatedDeltaMovementForICP(DeltaPos); //ICP수행을 위해 엔코더에서 얻은 delta x,y,theta. 변화량을 누적한다.
	return bReturn;
}

void KuICPLocalizerPr::computeRobotPoseByICPFor2D(double dX, double dY, double dTheta)
{
	double s1=0.,c1=0.;
	m_RobotPos = m_LastRobotPosForICP;


	// ------------------------- movement by encoder data ------------------------- //
	// yaw 계산에 반복되는 싸인,코싸인값 미리 계산
	s1 = sin(m_RobotPos.getThetaRad() );	// yaw
	c1 = cos(m_RobotPos.getThetaRad() );


	// localizer를 수행하여 보정하는 위치
	double dXm = (c1 * m_DeltaPosForICP.getXm()  -s1 * m_DeltaPosForICP.getYm());
	double dYm = (s1 * m_DeltaPosForICP.getXm() + c1 * m_DeltaPosForICP.getYm());

	m_RobotPos.setXm( m_RobotPos.getXm() + dXm);
	m_RobotPos.setYm( m_RobotPos.getYm() + dYm);
	m_RobotPos.setThetaRad( m_RobotPos.getThetaRad() + m_DeltaPosForICP.getThetaRad() );

	m_DeltaPosForICP.init();
	

	// ------------------------- motion correction by ICP result ------------------------- //
	// 반복되는 싸인,코싸인값 미리 계산
	s1 = sin(m_RobotPos.getThetaRad());	// yaw
	c1 = cos(m_RobotPos.getThetaRad());

	
	// localizer를 수행하여 보정하는 위치
	dXm = (c1*dX + -s1*dY);
	dYm = (s1*dX + c1*dY);

	m_RobotPos.setXm( m_RobotPos.getXm() + dXm);
	m_RobotPos.setYm( m_RobotPos.getYm() + dYm);
	m_RobotPos.setThetaRad( m_RobotPos.getThetaRad() + dTheta);

	m_LastRobotPosForICP = m_RobotPos;
}
/**
@brief Korean: 엔코더 데이터만을 이용한 로봇위치 계산 함수
@brief English: 
*/
void KuICPLocalizerPr::calculateRobotPosByEncoderData(KuPose DeltaPos)
{
	double s1=0.,c1=0.;		

	// roll-pitch-yaw 계산에 반복되는 싸인,코싸인값 미리 계산
	s1 = sin(m_RobotPos.getThetaRad());	// yaw
	c1 = cos(m_RobotPos.getThetaRad() );


	// localizer를 수행하여 보정하는 위치
	double dXm = ( c1* DeltaPos.getXm() + -s1 * DeltaPos.getYm() );
	double dYm = (s1 * DeltaPos.getXm() + c1*DeltaPos.getYm() );

	m_RobotPos.setXm( m_RobotPos.getXm() + dXm );
	m_RobotPos.setYm( m_RobotPos.getYm() + dYm );
	m_RobotPos.setThetaRad( m_RobotPos.getThetaRad() + DeltaPos.getThetaRad() );
}
	
/**
@brief Korean: 엔코더 데이터만을 이용한 로봇위치 계산 함수
@brief English: 
*/
void KuICPLocalizerPr::calculateRobotPosByEncoderData()
{
	double s1=0.,c1=0.;
	
	s1 = sin(m_RobotPos.getThetaRad());	// yaw
	c1 = cos(m_RobotPos.getThetaRad() );


	// localizer를 수행하여 보정하는 위치
	double dXm = (c1* m_DeltaPos.getXm()  -s1 * m_DeltaPos.getYm() );
	double dYm = (s1 * m_DeltaPos.getXm() + c1*m_DeltaPos.getYm() );
	m_RobotPos.setXm( m_RobotPos.getXm() + dXm );
	m_RobotPos.setYm( m_RobotPos.getYm() + dYm );
	m_RobotPos.setThetaRad( m_RobotPos.getThetaRad() + m_DeltaPos.getThetaRad() );
}

/**
@brief Korean: //ICP수행을 위해 엔코더에서 얻은 delta x,y,theta. 변화량을 누적하는 함수.
@brief English: 
*/
void KuICPLocalizerPr::computeAccumulatedDeltaMovementForICP(KuPose DeltaPos)
{
	m_dAccumulatedDeltaMovementForICP += fabs(DeltaPos.getXm()) + fabs(DeltaPos.getYm()) + fabs(DeltaPos.getThetaRad())*20.0/m_math.getPI();	// 각도는 180도 이동을 20m 이동한 것으로 취급
	
	double dXm = DeltaPos.getXm() * cos(m_DeltaPosForICP.getThetaRad() ) - DeltaPos.getYm() * sin(m_DeltaPosForICP.getThetaRad() );
	double dYm = DeltaPos.getXm() *sin(m_DeltaPosForICP.getThetaRad() ) + DeltaPos.getYm() * cos(m_DeltaPosForICP.getThetaRad() );

	m_DeltaPosForICP.setXm( m_DeltaPosForICP.getXm() + dXm );
	m_DeltaPosForICP.setYm( m_DeltaPosForICP.getYm() + dYm );
	m_DeltaPosForICP.setThetaRad( m_DeltaPosForICP.getThetaRad() + DeltaPos.getThetaRad() );
	
		
}

/**
@brief Korean: ICP를 수행할지를 정하는 함수. 				
@brief English: 
*/
bool KuICPLocalizerPr::isAccDeltaMovementForICPOver(double dValue)
{
	if (m_dAccumulatedDeltaMovementForICP > dValue) {
		m_dAccumulatedDeltaMovementForICP = 0.0;
		return true;
	}
	else return false;
}
