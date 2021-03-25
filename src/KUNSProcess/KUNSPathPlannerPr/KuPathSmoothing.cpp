#include "stdafx.h"
#include "KuPathSmoothing.h"

KuPathSmoothing::KuPathSmoothing()
{

}
KuPathSmoothing::~KuPathSmoothing()
{

}
/**
 @brief Korean:  �־��� ��ο��� �������� �н��� �ٲپ��ִ� �Լ�
 @brief English: 
*/
list<KuPose> KuPathSmoothing::smoothingPath(list<KuPose> originalPathList)
{//Cubic�� Least�� �Ѱ��̴�.
	list<KuPose> WayPointList = generateWaypointFromPath(originalPathList, WAY_POINT_INTERVAL);//WAY_POINT_INTERVAL ���ݸ��� �н����� �������� �����Ѵ�.
	list<KuPose> WayPointListAfterLeastSquare = doLeastSquare(WayPointList);//������ �������鿡�� Least�� ������ ���ο� �������� �����Ѵ�.
	list<KuPose> PathListAfterSmoothing = doCubicSplineInterpolation(0.05,WayPointListAfterLeastSquare);//0.02(m)�������� �������鰣�� �н��� �����Ѵ�.
// 	m_InflectionWayPointList=extractWayPoint(PathListAfterSmoothing,PathListAfterSmoothing.size());
// 	m_WayPointList=generateWaypointFromPath(PathListAfterSmoothing,BIG_WAY_POINT_INTERVAL);

	return PathListAfterSmoothing;
}
/**
@brief Korean:  �־��� ��ο��� Cubic Spline �� ��η� �ٲپ��ִ� �Լ�
@brief English: 
*/
list<KuPose> KuPathSmoothing::smoothingCubicSplinePath(list<KuPose> originalPathList)
{//Cubic�� �Ѱ��̴�.
	list<KuPose> WayPointList = generateWaypointFromPath(originalPathList,WAY_POINT_INTERVAL);
	list<KuPose> PathListAfterSmoothing = doCubicSplineInterpolation(0.05,WayPointList);

	return PathListAfterSmoothing;
}
/**
 @brief Korean:  �־��� ��ο��� ������ �������� �������� �̴� �Լ�
 @brief English: 
*/
list<KuPose> KuPathSmoothing::generateWaypointFromPath(list<KuPose> originalPathList,int ninterval)
{//�н��κ��� �������� �̴´�.
	list<KuPose> WayPointList;
	if(originalPathList.size()>0)
	{
		WayPointList.push_back( originalPathList.front());
		list<KuPose>::iterator it;
		int i = 0;
		for (it = originalPathList.begin(); it != originalPathList.end(); it++){
			if(i%ninterval==0) WayPointList.push_back(*it);
			i++;
		}
		WayPointList.push_back( originalPathList.back());
	}

	return WayPointList;
}
/**
@brief Korean:  ������ list �� �Ѱ��ִ� �Լ�
@brief English: 
*/
list<KuPose> KuPathSmoothing::getWayPointList()
{
	return m_WayPointList;
}
/**
@brief Korean:  ������ list �� �Ѱ��ִ� �Լ�
@brief English: 
*/
list<KuPose> KuPathSmoothing::getInflectionWayPointList()
{
	return m_InflectionWayPointList;
}
/**
@brief Korean:  ���������� ���� Least Square�� �� �������� �����ϴ� �Լ�
@brief English: 
*/
list<KuPose> KuPathSmoothing::doLeastSquare(list<KuPose> WayPointList)
{	
	//������ ������ Least square�� �Ѵ�.
	list<KuPose>  newWayPointList;//���Ӱ� ������ ������ ����Ʈ�̴�.
	list<KuPose>::iterator itFirstWayPoint;//ù��° �������̴�.
	list<KuPose>::iterator itSecondWayPoint;//�ι�° �������̴�.
	list<KuPose>::iterator itThirdWayPoint;//����° �������̴�.
	const int ntotalwaypointnum=3;

	KuPose WayPoint;
	KuPose updatedWayPoint;//���������� ���Ӱ� ������ waypoint

	itFirstWayPoint =itSecondWayPoint=itThirdWayPoint= WayPointList.begin();
	itSecondWayPoint++;itThirdWayPoint++;itThirdWayPoint++;
	double dWayPointX[ntotalwaypointnum];//�������� X��ǥ
	double dWayPointY[ntotalwaypointnum];//�������� Y��ǥ
	double da=0;//����
	double db=0;//y����
	double dSigmaX2=0;	//X^2�� ��
	double dSigmaX=0;//X�� ��
	double dSigmaY=0;//Y�� ��
	double dSigmaXY=0;//X*Y�� ��
	double dmidPointX=0;//�߰� �������� X��ǥ
	double dmidPointY=0;//�߰� �������� Y��ǥ
	int i=0;
	bool binitposflag=true;
	newWayPointList.push_back(WayPointList.front());

	for (; itThirdWayPoint != WayPointList.end(); itFirstWayPoint++,itSecondWayPoint++,itThirdWayPoint++)
	{
		dSigmaX2=0;		dSigmaX=0;		 dSigmaY=0;		dSigmaXY=0;//������ �յ��� �ʱ�ȭ���ش�.
		//�������� ����� ���Ǹ� ���� �迭�� �ٲ۴�.
		if(binitposflag){dWayPointX[0]=itFirstWayPoint->getX();	dWayPointY[0]=itFirstWayPoint->getY();}
		else{
			binitposflag=false;
			updatedWayPoint=newWayPointList.back();
			dWayPointX[0]=updatedWayPoint.getX();	dWayPointY[0]=updatedWayPoint.getY();
		}
		dWayPointX[1]=itSecondWayPoint->getX();	dWayPointY[1]=itSecondWayPoint->getY(); 
		dWayPointX[2]=itThirdWayPoint->getX();	dWayPointY[2]=itThirdWayPoint->getY(); 
		//sigma x^2�� ���
		for(i=0;i<ntotalwaypointnum;i++){	dSigmaX2+=pow(dWayPointX[i],2);}
		//sigma x�� ���
		for(i=0;i<ntotalwaypointnum;i++){	dSigmaX+=dWayPointX[i];	}
		//sigma y�� ���
		for(i=0;i<ntotalwaypointnum;i++){	dSigmaY+=dWayPointY[i];	}
		//sigma xy�� ���
		for(i=0;i<ntotalwaypointnum;i++){	dSigmaXY+=(dWayPointX[i]*dWayPointY[i]);	}
		
		if((3*dSigmaX2-dSigmaX*dSigmaX)==0||dSigmaX*dSigmaX-3*dSigmaX==0
			/*||dWayPointX[1]==dWayPointX[2]||dWayPointY[1]==dWayPointY[2*]*/){	
			WayPoint.setX(dWayPointX[1]);
			WayPoint.setY(dWayPointY[1]);
			newWayPointList.push_back(WayPoint);
			continue;
		}//���Ⱑ 0�̰ų� �и� 0�ΰ�� ����.

		da=(ntotalwaypointnum*dSigmaXY-dSigmaX*dSigmaY)/(ntotalwaypointnum*dSigmaX2-dSigmaX*dSigmaX);
		db=(dSigmaXY*dSigmaX-dSigmaX2*dSigmaY)/(dSigmaX*dSigmaX-3*dSigmaX2);
		

		dmidPointX=(-db*da+(dWayPointX[1]+da*dWayPointY[1]))/(da*da+1);//���ο� ������ X�� ����Ѵ�.
		dmidPointY=((da*dWayPointX[1]+da*da*dWayPointY[1])+db)/(da*da+1);//���ο� ������ Y�� ����Ѵ�.

		if(Obstacleflag((int)dmidPointX,(int)dmidPointY)==true)//���� ������ ���Ͽ� ����� ��ǥ�� ��ֹ��� �ִ��� �ƴ��� �˻��Ѵ�.
		{//��ֹ��̸� ������ �������� ��������.
			WayPoint.setX(dWayPointX[1]);
			WayPoint.setY(dWayPointY[1]);
		}
		else{//��ֹ��� ���ٸ� ���ο� �������� ��������.
			WayPoint.setX((int)dmidPointX);
			WayPoint.setY((int)dmidPointY);
		}
	
		newWayPointList.push_back(WayPoint);
	}
	newWayPointList.push_back(WayPointList.back());

	return newWayPointList;
}
/**
 @brief Korean:  ������ ����� ��ġ�� ��ֹ��� �ִ��� ������ ���θ� Ȯ���ϴ� �Լ�
 @brief English: 
*/
bool  KuPathSmoothing::Obstacleflag(int nmidPointX,int nmidPointY)
{//�������� OCCUPIED_AREA���� �ƴ��� �˻��Ѵ�.
	KuMap *pmap=kuMapRepository::getInstance()->getMap();
	int** nmap=pmap->getMap();	
	int ninterval=4;
	int i=0,j=0;
	for(i=-ninterval;i<ninterval+1;i++){
		for(j=-ninterval;j<ninterval+1;j++){
			if(nmap[(int)(nmidPointX/100)+i][(int)(nmidPointY/100)+j]==KuMap::OCCUPIED_AREA){
				return true;
			}
		}
	}
	return false;
}
/**
@brief Korean:  ���������� ���� ���ϴ� ���ݸ�ŭ Cubic Spline�� ��θ� �����ϴ� �Լ�
@brief English: 
*/
list<KuPose> KuPathSmoothing::doCubicSplineInterpolation(  double dinterval, list<KuPose> stlWayPoint)
{

	list<KuPose> WayPointlist=stlWayPoint;//������ ����Ʈ

	if (dinterval < 0.001) dinterval = 0.001;//path�� ������ �ʹ� ���� ��� 0.001�� �����Ѵ�.
	if (WayPointlist.size() < 2) return stlWayPoint;//�������� ������ 1���� ��� �������� ��������.

	unsigned int nWayPonitVectosize = WayPointlist.size () - 1;

	list<KuPose>::iterator itPastWayPoint;//���� �������� ����Ű�� iterator�̴�.
	list<KuPose>::iterator itCurrentWayPoint;//���� �������� ����Ű�� iterator�̴�.
	list<KuPose>::iterator itAftertWayPoint;//���� �������� ����Ű�� iterator�̴�.


	//�������� �Ÿ��� ����Ѵ�.-----------------------------------------------------------------------------------------------------
	double dTemp[3] = {0.0, 0.0, 0.0};
	list<KuPose>::iterator itwaypoint;
	for (itwaypoint = WayPointlist.begin(); itwaypoint != WayPointlist.end(); itwaypoint++)
	{
		itwaypoint->setDist(  sqrt( (dTemp[0] -   itwaypoint->getXm())*(dTemp[0] - itwaypoint->getXm()) + (dTemp[1] -   itwaypoint->getYm())*(dTemp[1] -   itwaypoint->getYm())   )   + dTemp[2]    );
		dTemp[0] = itwaypoint->getXm(); dTemp[1] = itwaypoint->getYm(); dTemp[2] = itwaypoint->getDist();
	}
	//========================================================================================================


	itCurrentWayPoint=itAftertWayPoint=WayPointlist.begin();
	itAftertWayPoint++;

	vector<double> WayPonitDistanceGap(nWayPonitVectosize);

	for(int i=0;itAftertWayPoint!=WayPointlist.end();itCurrentWayPoint++,itAftertWayPoint++,i++)
	{
		WayPonitDistanceGap[i] =itAftertWayPoint->getDist() - itCurrentWayPoint->getDist();
		if (WayPonitDistanceGap[i] < dinterval) WayPonitDistanceGap[i] = dinterval;
	}//WayPonitDistanceGap�� ���������� �Ÿ��� �����̴�.

	//dMatrix A ���------------------------------------------------------------------------------------------------------------------------------------------
	dMatrix dMatrixA(nWayPonitVectosize+1, nWayPonitVectosize+1);
	dMatrixA.null();
	{
		unsigned int i = 0;
		dMatrixA(0, 0) = 2*WayPonitDistanceGap[i];
		dMatrixA(0, 1) = WayPonitDistanceGap[i];
		for (i=1; i<nWayPonitVectosize; ++i) {
			dMatrixA(i, i-1) = WayPonitDistanceGap[i-1];
			dMatrixA(i, i+0) = 2*(WayPonitDistanceGap[i-1] + WayPonitDistanceGap[i]);
			dMatrixA(i, i+1) = WayPonitDistanceGap[i];
		}
		dMatrixA(i, i-1) = WayPonitDistanceGap[i-1];
		dMatrixA(i, i)   = 2.*WayPonitDistanceGap[i-1];
	}
	//======================================================================================================================

	//dMatrix b ���-------------------------------------------------------------------------------------------------------------------------------------------------
	int mcolum=3;
	itPastWayPoint=itCurrentWayPoint=itAftertWayPoint=WayPointlist.begin();
	itCurrentWayPoint++;itAftertWayPoint++;itAftertWayPoint++;
	dMatrix dMatrixb(nWayPonitVectosize+1, mcolum);
	{
		unsigned int i = 0;

		dMatrixb(i, 0)=6.*( itCurrentWayPoint->getDist()-itPastWayPoint->getDist())/WayPonitDistanceGap[i];
		dMatrixb(i, 1)=6.*( itCurrentWayPoint->getXm()-itPastWayPoint->getXm())/WayPonitDistanceGap[i];
		dMatrixb(i, 2)=6.*( itCurrentWayPoint->getYm()-itPastWayPoint->getYm())/WayPonitDistanceGap[i];

		for (i=1; itAftertWayPoint!=WayPointlist.end();itPastWayPoint++,itCurrentWayPoint++,itAftertWayPoint++,i++) {
			dMatrixb(i, 0)=6.*(( itAftertWayPoint->getDist()-itCurrentWayPoint->getDist())/WayPonitDistanceGap[i]-( itCurrentWayPoint->getDist()-itPastWayPoint->getDist())/WayPonitDistanceGap[i-1]);
			dMatrixb(i, 1)=6.*( (itAftertWayPoint->getXm()-itCurrentWayPoint->getXm())/WayPonitDistanceGap[i]-( itCurrentWayPoint->getXm()-itPastWayPoint->getXm())/WayPonitDistanceGap[i-1]);
			dMatrixb(i, 2)=6.*( (itAftertWayPoint->getYm()-itCurrentWayPoint->getYm())/WayPonitDistanceGap[i]-( itCurrentWayPoint->getYm()-itPastWayPoint->getYm())/WayPonitDistanceGap[i-1]);

		}
		dMatrixb(i, 0)=6.*(-( itCurrentWayPoint->getDist()-itPastWayPoint->getDist()))/WayPonitDistanceGap[i-1];
		dMatrixb(i, 1)=6.*(-( itCurrentWayPoint->getXm()-itPastWayPoint->getXm()))/WayPonitDistanceGap[i-1];
		dMatrixb(i, 2)=6.*(-( itCurrentWayPoint->getYm()-itPastWayPoint->getYm()))/WayPonitDistanceGap[i-1];
	}
	//======================================================================================================================

	dMatrix dMatrixz = !dMatrixA*dMatrixb;//dMatrix z ���

	//PathList ���-------------------------------------------------------------------------------------------------------------------------------------------------
	list <KuPose> PathList;

	itCurrentWayPoint=itAftertWayPoint=WayPointlist.begin();
	itAftertWayPoint++;

	double realPathPoint = itCurrentWayPoint->getDist();
	for(int i=0;itAftertWayPoint!=WayPointlist.end();itCurrentWayPoint++,itAftertWayPoint++,i++)
	{
		for (; realPathPoint< itAftertWayPoint->getDist(); realPathPoint+=dinterval) {

			double dDistancePathnFeuturePoint = realPathPoint - itCurrentWayPoint->getDist();
			double tdDistancePathnCurrentPointp = itAftertWayPoint->getDist() - realPathPoint;

			double dDistancePathnFeuturePoint3 = pow(dDistancePathnFeuturePoint,3);
			double tdDistancePathnCurrentPointp3 =pow(tdDistancePathnCurrentPointp,3);


			KuPose  WayPoint;
			WayPoint.setDist(realPathPoint);

			WayPoint.setXm((dMatrixz(i+1,1)*dDistancePathnFeuturePoint3 + dMatrixz(i,1)*tdDistancePathnCurrentPointp3)/(6.*WayPonitDistanceGap[i]) 
				+ ( itAftertWayPoint->getXm()/WayPonitDistanceGap[i] - WayPonitDistanceGap[i]*dMatrixz(i+1,1)/6.)*dDistancePathnFeuturePoint
				+ (itCurrentWayPoint->getXm()/WayPonitDistanceGap[i] - WayPonitDistanceGap[i]*dMatrixz(i,1)/6.)*tdDistancePathnCurrentPointp);
			WayPoint.setYm((dMatrixz(i+1,2)*dDistancePathnFeuturePoint3 + dMatrixz(i,2)*tdDistancePathnCurrentPointp3)/(6.*WayPonitDistanceGap[i])
				+ ( itAftertWayPoint->getYm()/WayPonitDistanceGap[i] - WayPonitDistanceGap[i]*dMatrixz(i+1,2)/6.)*dDistancePathnFeuturePoint	
				+ (itCurrentWayPoint->getYm()/WayPonitDistanceGap[i] - WayPonitDistanceGap[i]*dMatrixz(i,2)/6.)*tdDistancePathnCurrentPointp);

			PathList.push_back (WayPoint);
		}
	
	}
	if(PathList.size()>1){PathList.pop_front();}
	else{PathList.push_back(stlWayPoint.front());}
	//======================================================================================================================
	printf("the end maybe......\n");
	return PathList;
}
/**
 @brief Korean:  ��κ��� �������� �����ϴ� �Լ�( ��� ���� �������� ������ ������ �ٸ��� ���´�)
 @brief English: 
*/
list<KuPose> KuPathSmoothing::extractWayPoint(list<KuPose>PathList, int nPathCnt)
{
	int nCnt=0;
	
	KuPose WayPoint;
	list<KuPose>  WayPointList;
	list<KuPose>::iterator iteratorPath;
	
	int *nPathX = (int*)calloc(PathList.size(), sizeof(int));
	int *nPathY = (int*)calloc(PathList.size(), sizeof(int));

	for(iteratorPath = PathList.begin();  iteratorPath != PathList.end(); iteratorPath++){
		nPathX[nCnt] = (int)m_math.MM2AI(iteratorPath->getX());
		nPathY[nCnt] = (int)m_math.MM2AI(iteratorPath->getY());
		nCnt++;
	}    

	bool bCheckCluster = true;
	int *anStartPoint = (int*)calloc(PathList.size(), sizeof(int));
	int *anFinishPoint = (int*)calloc(PathList.size(), sizeof(int));

	anStartPoint[0] = 0;
	anFinishPoint[0] = nPathCnt-1;

	int iIndex =0;
	int nNum =0;
	double nMaxDist =0;
	double dLinearEqA, dLinearEqB, dLinearEqC ; 
	double dVerticalDist;	

	int nPoint1;	
	int nPoint2=0;	

	while(bCheckCluster){
		nMaxDist  =0;
		for (nPoint1 = anStartPoint[nPoint2]+1; nPoint1<anFinishPoint[nPoint2]-1; nPoint1++){	//i

			dLinearEqA = nPathY[anFinishPoint[nPoint2]] - nPathY[anStartPoint[nPoint2]];
			dLinearEqB = -(nPathX[anFinishPoint[nPoint2]] - nPathX[anStartPoint[nPoint2]]);
			dLinearEqC = nPathX[anFinishPoint[nPoint2]]*nPathY[anStartPoint[nPoint2]] - nPathY[anFinishPoint[nPoint2]]*nPathX[anStartPoint[nPoint2]];
			dVerticalDist = fabs(dLinearEqA*nPathX[nPoint1]+dLinearEqB*nPathY[nPoint1]+dLinearEqC)/sqrt(dLinearEqA*dLinearEqA+dLinearEqB*dLinearEqB);

			if (dVerticalDist>=nMaxDist){
				nMaxDist = dVerticalDist;
				iIndex = nPoint1;
			}
		}

		if (nMaxDist>=WAY_POINT_VALUE){
			anFinishPoint[nPoint2] = iIndex;
		}
		else{
			nPoint2++;
			anStartPoint[nPoint2] = anFinishPoint[nPoint2-1];
			anFinishPoint[nPoint2] = nPathCnt-1;
		}

		if(anStartPoint[nPoint2]==anFinishPoint[nPoint2]) bCheckCluster=false;

		nNum = nPoint2;
	}

	for(int nOrderCluster=0; nOrderCluster<nNum; nOrderCluster++){

		int nX = (int)( m_math.AI2MM( nPathX[anFinishPoint[nOrderCluster]] ) );
		int nY = (int)( m_math.AI2MM( nPathY[anFinishPoint[nOrderCluster]] ) );
		WayPoint.setX(nX);
		WayPoint.setY(nY);
		WayPointList.push_back(WayPoint);

	}

	free(nPathX);
	free(nPathY);
	free(anStartPoint);
	free(anFinishPoint);

	return WayPointList;
}