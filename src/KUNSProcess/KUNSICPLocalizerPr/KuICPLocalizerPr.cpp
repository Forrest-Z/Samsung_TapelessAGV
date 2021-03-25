#include "stdafx.h"
#include "KuICPLocalizerPr.h"


KuICPLocalizerPr::KuICPLocalizerPr()
{
	initialize(); //���� ���� �ʱ�ȭ �Լ� ����.
	cout<<"[KuICPLocalizerPr]: Instance is created!!!"<<endl;

}
KuICPLocalizerPr::~KuICPLocalizerPr()
{
	cout<<"[KuICPLocalizerPr]: Instance is destroyed!!!"<<endl;
}


/**
@brief Korean: ���� �������� �ʱ�ȭ �ϴ� �Լ�
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
	m_dLaserSensorOffsetmm = KuRobotParameter::getInstance()->getFrontLaserXOffset(); //������ ������ �κ� �߽ɰ��� �Ÿ�, unit mm
	m_dMaxSensorRangeMM = 1/3.0*KuRobotParameter::getInstance()->getURG04LXLaserMaxDist(); //����� ������������ �ִ� �Ÿ�, unit mm
	m_dMinSensorRangeMM= KuRobotParameter::getInstance()->getURG04LXLaserMinDist(); //����� ������������ �ִ� �Ÿ�, unit mm

	// ICP�� ����� ����. �� �Ÿ����� ICP�� ����. 
	// �ʹ� ������ ���� �߻�. �ּ� 0.3~0.5m�� ����
	// �ʹ� ũ�� ���� �߻�. Ư�� ���� �������� ������ ��Ī�Ǵ� ����Ʈ�� ��� ������ Ȯ~ Ʋ����.
	m_dDistanceForICPExecution = 0.35;
}

/**
@brief Korean: ������ǥ���� x(mm)��ǥ�� �ٸ� Ŭ�������� ������
@brief English : 
*/
double KuICPLocalizerPr::getRobotPosX()
{
	return m_RobotPos.getX();	
}

/**
@brief Korean: ������ǥ���� y(mm)��ǥ�� �ٸ� Ŭ�������� ������
@brief English : 
*/
double KuICPLocalizerPr::getRobotPosY()
{
	return m_RobotPos.getY();	
}

/**
@brief Korean: ������ǥ���� ����(deg)�� �ٸ� Ŭ�������� ������
@brief English : 
*/
double KuICPLocalizerPr::getRobotPosDeg()
{
	return m_RobotPos.getThetaDeg();	

}

/**
@brief Korean: ���� ��ǥ���� X(mm)��ǥ�� ������ 
@brief English : 
*/
void KuICPLocalizerPr::setRobotPosX(double dRobotPosX)
{
	m_RobotPos.setX(dRobotPosX);	
	m_LastRobotPosForICP.setX(dRobotPosX); 
}

/**
@brief Korean: ���� ��ǥ���� Y(mm)��ǥ�� ������ 
@brief English : 
*/
void KuICPLocalizerPr::setRobotPosY(double dRobotPosY)
{
	m_RobotPos.setY( dRobotPosY ); 	
	m_LastRobotPosForICP.setY( dRobotPosY );	
}

/**
@brief Korean: ���� ��ǥ���� ������ ������ 
@brief English : 
*/
void KuICPLocalizerPr::setRobotPosDeg(double dRobotPosThetaDeg)
{
	m_RobotPos.setThetaDeg( dRobotPosThetaDeg );
	m_LastRobotPosForICP.setThetaDeg(dRobotPosThetaDeg);	
}

/**
@brief Korean: ���� ��ǥ���� �κ� ��ġ�� ������. 
@brief English : 
*/
void KuICPLocalizerPr::setRobotPos(KuPose RobotPos)
{
	m_RobotPos = RobotPos;
	m_LastRobotPosForICP = RobotPos;	
	m_dAccumulatedDeltaMovementForICP = 0.0;
}

/**
@brief Korean: ���� ��ǥ���� �κ� ��ġ�� �Ѱ��ش�.
@brief English : 
*/
KuPose KuICPLocalizerPr::getRobotPos()
{
	return m_RobotPos;
}


/**
@brief Korean: ���� ��ǥ���� �ʱ� �κ� ��ġ�� ������. 
@brief English : 
*/
void KuICPLocalizerPr::setInitRobotPos(KuPose RobotPos)
{
	m_RobotPos = RobotPos;
	m_LastRobotPosForICP = RobotPos;
}


/**
@brief Korean: �Ÿ����� ���� ������. 
			   poloar coordinate������ �Ÿ� ���� ����, 
			   cartesian coordinate������ ���� �����Ѵ�.
@brief English : 
*/
void KuICPLocalizerPr::copyRangeData(int_1DArray  nData)
{
	KuCartesianCoordinate2D CartesianCoordiData;
	KuCartesianCoordinate2D PreCartesianCoordiData;

	for(int i=0; i<LASER_SCAN_IDX; i++){

		if (nData[i]< m_dMaxSensorRangeMM &&
			nData[i]>m_dMinSensorRangeMM)
		{	// ��������ĳ���� ��ȿ�Ÿ��� ����� ó�� ����.

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
@brief Korean:ICP �˰����� �̿��Ͽ� �κ��� ��ġ�� �����ϴ� �Լ�
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
	computeAccumulatedDeltaMovementForICP(DeltaEncoderData); //ICP������ ���� ���ڴ����� ���� delta x,y,theta. ��ȭ���� �����Ѵ�.

	bool bICPExecution = isAccDeltaMovementForICPOver(m_dDistanceForICPExecution); //ICP�� ���������� ���ϴ� �Լ�.

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
	// yaw ��꿡 �ݺ��Ǵ� ����,�ڽ��ΰ� �̸� ���
	s1 = sin(PreRobotPos.getThetaRad() );	// yaw
	c1 = cos(PreRobotPos.getThetaRad() );


	// localizer�� �����Ͽ� �����ϴ� ��ġ
	dXm = (c1 * DeltaPos.getXm() -s1 * DeltaPos.getYm());
	dYm = (s1 * DeltaPos.getXm() + c1 * DeltaPos.getYm());

	PreRobotPos.setXm( PreRobotPos.getXm() + dXm);
	PreRobotPos.setYm( PreRobotPos.getYm() + dYm);
	PreRobotPos.setThetaRad( PreRobotPos.getThetaRad() + DeltaPos.getThetaRad() );


	// ------------------------- motion correction by ICP result ------------------------- //
	// �ݺ��Ǵ� ����,�ڽ��ΰ� �̸� ���
	s1 = sin(PreRobotPos.getThetaRad() );	// yaw
	c1 = cos(PreRobotPos.getThetaRad());


	// localizer�� �����Ͽ� �����ϴ� ��ġ
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
	double PreData[362], NewData[362]; //--> 181*2 181�� ������ ������ ����, 2�� x,y
	double Tmatrix[3], Rmatrix[3][3];
	double RMSError=-1;

	// ���Ŀ� ���缭 ������ �Է�
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

	// ICP ����
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
		
	m_DeltaPos = DeltaPos; // ���ڴ����� ���� delta x,y,theta. ��ȭ��. ���밪 �ƴ�.
	bool bICPExecution = isAccDeltaMovementForICPOver(m_dDistanceForICPExecution); //ICP�� ���������� ���ϴ� �Լ�.
	if (bICPExecution) {
		// -------------------------------------------------------- ICP ---------------------------------------------------------------- //			
		double PreData[362], NewData[362]; //--> 181*2 181�� ������ ������ ����, 2�� x,y
		double Tmatrix[3], Rmatrix[3][3];
		double RMSError;

		// ���Ŀ� ���缭 ������ �Է�
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

		// ICP ����
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
	else{ //ICP�� �������� �ʴ� ��Ȳ������ ���ڴ� ������ ������ �κ��� ��ġ�� ����Ѵ�.
		calculateRobotPosByEncoderData();
	}
		
	computeAccumulatedDeltaMovementForICP(DeltaPos); //ICP������ ���� ���ڴ����� ���� delta x,y,theta. ��ȭ���� �����Ѵ�.
	return bReturn;
}

void KuICPLocalizerPr::computeRobotPoseByICPFor2D(double dX, double dY, double dTheta)
{
	double s1=0.,c1=0.;
	m_RobotPos = m_LastRobotPosForICP;


	// ------------------------- movement by encoder data ------------------------- //
	// yaw ��꿡 �ݺ��Ǵ� ����,�ڽ��ΰ� �̸� ���
	s1 = sin(m_RobotPos.getThetaRad() );	// yaw
	c1 = cos(m_RobotPos.getThetaRad() );


	// localizer�� �����Ͽ� �����ϴ� ��ġ
	double dXm = (c1 * m_DeltaPosForICP.getXm()  -s1 * m_DeltaPosForICP.getYm());
	double dYm = (s1 * m_DeltaPosForICP.getXm() + c1 * m_DeltaPosForICP.getYm());

	m_RobotPos.setXm( m_RobotPos.getXm() + dXm);
	m_RobotPos.setYm( m_RobotPos.getYm() + dYm);
	m_RobotPos.setThetaRad( m_RobotPos.getThetaRad() + m_DeltaPosForICP.getThetaRad() );

	m_DeltaPosForICP.init();
	

	// ------------------------- motion correction by ICP result ------------------------- //
	// �ݺ��Ǵ� ����,�ڽ��ΰ� �̸� ���
	s1 = sin(m_RobotPos.getThetaRad());	// yaw
	c1 = cos(m_RobotPos.getThetaRad());

	
	// localizer�� �����Ͽ� �����ϴ� ��ġ
	dXm = (c1*dX + -s1*dY);
	dYm = (s1*dX + c1*dY);

	m_RobotPos.setXm( m_RobotPos.getXm() + dXm);
	m_RobotPos.setYm( m_RobotPos.getYm() + dYm);
	m_RobotPos.setThetaRad( m_RobotPos.getThetaRad() + dTheta);

	m_LastRobotPosForICP = m_RobotPos;
}
/**
@brief Korean: ���ڴ� �����͸��� �̿��� �κ���ġ ��� �Լ�
@brief English: 
*/
void KuICPLocalizerPr::calculateRobotPosByEncoderData(KuPose DeltaPos)
{
	double s1=0.,c1=0.;		

	// roll-pitch-yaw ��꿡 �ݺ��Ǵ� ����,�ڽ��ΰ� �̸� ���
	s1 = sin(m_RobotPos.getThetaRad());	// yaw
	c1 = cos(m_RobotPos.getThetaRad() );


	// localizer�� �����Ͽ� �����ϴ� ��ġ
	double dXm = ( c1* DeltaPos.getXm() + -s1 * DeltaPos.getYm() );
	double dYm = (s1 * DeltaPos.getXm() + c1*DeltaPos.getYm() );

	m_RobotPos.setXm( m_RobotPos.getXm() + dXm );
	m_RobotPos.setYm( m_RobotPos.getYm() + dYm );
	m_RobotPos.setThetaRad( m_RobotPos.getThetaRad() + DeltaPos.getThetaRad() );
}
	
/**
@brief Korean: ���ڴ� �����͸��� �̿��� �κ���ġ ��� �Լ�
@brief English: 
*/
void KuICPLocalizerPr::calculateRobotPosByEncoderData()
{
	double s1=0.,c1=0.;
	
	s1 = sin(m_RobotPos.getThetaRad());	// yaw
	c1 = cos(m_RobotPos.getThetaRad() );


	// localizer�� �����Ͽ� �����ϴ� ��ġ
	double dXm = (c1* m_DeltaPos.getXm()  -s1 * m_DeltaPos.getYm() );
	double dYm = (s1 * m_DeltaPos.getXm() + c1*m_DeltaPos.getYm() );
	m_RobotPos.setXm( m_RobotPos.getXm() + dXm );
	m_RobotPos.setYm( m_RobotPos.getYm() + dYm );
	m_RobotPos.setThetaRad( m_RobotPos.getThetaRad() + m_DeltaPos.getThetaRad() );
}

/**
@brief Korean: //ICP������ ���� ���ڴ����� ���� delta x,y,theta. ��ȭ���� �����ϴ� �Լ�.
@brief English: 
*/
void KuICPLocalizerPr::computeAccumulatedDeltaMovementForICP(KuPose DeltaPos)
{
	m_dAccumulatedDeltaMovementForICP += fabs(DeltaPos.getXm()) + fabs(DeltaPos.getYm()) + fabs(DeltaPos.getThetaRad())*20.0/m_math.getPI();	// ������ 180�� �̵��� 20m �̵��� ������ ���
	
	double dXm = DeltaPos.getXm() * cos(m_DeltaPosForICP.getThetaRad() ) - DeltaPos.getYm() * sin(m_DeltaPosForICP.getThetaRad() );
	double dYm = DeltaPos.getXm() *sin(m_DeltaPosForICP.getThetaRad() ) + DeltaPos.getYm() * cos(m_DeltaPosForICP.getThetaRad() );

	m_DeltaPosForICP.setXm( m_DeltaPosForICP.getXm() + dXm );
	m_DeltaPosForICP.setYm( m_DeltaPosForICP.getYm() + dYm );
	m_DeltaPosForICP.setThetaRad( m_DeltaPosForICP.getThetaRad() + DeltaPos.getThetaRad() );
	
		
}

/**
@brief Korean: ICP�� ���������� ���ϴ� �Լ�. 				
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
