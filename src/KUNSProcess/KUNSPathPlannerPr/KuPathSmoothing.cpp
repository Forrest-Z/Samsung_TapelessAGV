#include "stdafx.h"
#include "KuPathSmoothing.h"

KuPathSmoothing::KuPathSmoothing()
{

}
KuPathSmoothing::~KuPathSmoothing()
{

}
/**
 @brief Korean:  주어진 경로에서 스무스한 패스로 바꾸어주는 함수
 @brief English: 
*/
list<KuPose> KuPathSmoothing::smoothingPath(list<KuPose> originalPathList)
{//Cubic과 Least를 한것이다.
	list<KuPose> WayPointList = generateWaypointFromPath(originalPathList, WAY_POINT_INTERVAL);//WAY_POINT_INTERVAL 간격마다 패스에서 경유점을 선택한다.
	list<KuPose> WayPointListAfterLeastSquare = doLeastSquare(WayPointList);//선정된 경유점들에서 Least를 적용한 새로운 경유점을 생성한다.
	list<KuPose> PathListAfterSmoothing = doCubicSplineInterpolation(0.05,WayPointListAfterLeastSquare);//0.02(m)간격으로 경유점들간의 패스를 선정한다.
// 	m_InflectionWayPointList=extractWayPoint(PathListAfterSmoothing,PathListAfterSmoothing.size());
// 	m_WayPointList=generateWaypointFromPath(PathListAfterSmoothing,BIG_WAY_POINT_INTERVAL);

	return PathListAfterSmoothing;
}
/**
@brief Korean:  주어진 경로에서 Cubic Spline 된 경로로 바꾸어주는 함수
@brief English: 
*/
list<KuPose> KuPathSmoothing::smoothingCubicSplinePath(list<KuPose> originalPathList)
{//Cubic만 한것이다.
	list<KuPose> WayPointList = generateWaypointFromPath(originalPathList,WAY_POINT_INTERVAL);
	list<KuPose> PathListAfterSmoothing = doCubicSplineInterpolation(0.05,WayPointList);

	return PathListAfterSmoothing;
}
/**
 @brief Korean:  주어진 경로에서 일정한 간격으로 경유점을 뽑는 함수
 @brief English: 
*/
list<KuPose> KuPathSmoothing::generateWaypointFromPath(list<KuPose> originalPathList,int ninterval)
{//패스로부터 경유점을 뽑는다.
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
@brief Korean:  경유점 list 를 넘겨주는 함수
@brief English: 
*/
list<KuPose> KuPathSmoothing::getWayPointList()
{
	return m_WayPointList;
}
/**
@brief Korean:  경유점 list 를 넘겨주는 함수
@brief English: 
*/
list<KuPose> KuPathSmoothing::getInflectionWayPointList()
{
	return m_InflectionWayPointList;
}
/**
@brief Korean:  경유점으로 부터 Least Square를 한 경유점을 생성하는 함수
@brief English: 
*/
list<KuPose> KuPathSmoothing::doLeastSquare(list<KuPose> WayPointList)
{	
	//세점을 가지고 Least square를 한다.
	list<KuPose>  newWayPointList;//새롭게 생성된 경유점 리스트이다.
	list<KuPose>::iterator itFirstWayPoint;//첫번째 경유점이다.
	list<KuPose>::iterator itSecondWayPoint;//두번째 경유점이다.
	list<KuPose>::iterator itThirdWayPoint;//세번째 경유점이다.
	const int ntotalwaypointnum=3;

	KuPose WayPoint;
	KuPose updatedWayPoint;//계산과정에서 새롭게 선정된 waypoint

	itFirstWayPoint =itSecondWayPoint=itThirdWayPoint= WayPointList.begin();
	itSecondWayPoint++;itThirdWayPoint++;itThirdWayPoint++;
	double dWayPointX[ntotalwaypointnum];//경유점의 X좌표
	double dWayPointY[ntotalwaypointnum];//경유점의 Y좌표
	double da=0;//기울기
	double db=0;//y절편
	double dSigmaX2=0;	//X^2의 합
	double dSigmaX=0;//X의 합
	double dSigmaY=0;//Y의 합
	double dSigmaXY=0;//X*Y의 합
	double dmidPointX=0;//중간 경유점의 X좌표
	double dmidPointY=0;//중간 경유점의 Y좌표
	int i=0;
	bool binitposflag=true;
	newWayPointList.push_back(WayPointList.front());

	for (; itThirdWayPoint != WayPointList.end(); itFirstWayPoint++,itSecondWayPoint++,itThirdWayPoint++)
	{
		dSigmaX2=0;		dSigmaX=0;		 dSigmaY=0;		dSigmaXY=0;//각각의 합들을 초기화해준다.
		//경유점을 계산의 편의를 위해 배열로 바꾼다.
		if(binitposflag){dWayPointX[0]=itFirstWayPoint->getX();	dWayPointY[0]=itFirstWayPoint->getY();}
		else{
			binitposflag=false;
			updatedWayPoint=newWayPointList.back();
			dWayPointX[0]=updatedWayPoint.getX();	dWayPointY[0]=updatedWayPoint.getY();
		}
		dWayPointX[1]=itSecondWayPoint->getX();	dWayPointY[1]=itSecondWayPoint->getY(); 
		dWayPointX[2]=itThirdWayPoint->getX();	dWayPointY[2]=itThirdWayPoint->getY(); 
		//sigma x^2를 계산
		for(i=0;i<ntotalwaypointnum;i++){	dSigmaX2+=pow(dWayPointX[i],2);}
		//sigma x를 계산
		for(i=0;i<ntotalwaypointnum;i++){	dSigmaX+=dWayPointX[i];	}
		//sigma y를 계산
		for(i=0;i<ntotalwaypointnum;i++){	dSigmaY+=dWayPointY[i];	}
		//sigma xy를 계산
		for(i=0;i<ntotalwaypointnum;i++){	dSigmaXY+=(dWayPointX[i]*dWayPointY[i]);	}
		
		if((3*dSigmaX2-dSigmaX*dSigmaX)==0||dSigmaX*dSigmaX-3*dSigmaX==0
			/*||dWayPointX[1]==dWayPointX[2]||dWayPointY[1]==dWayPointY[2*]*/){	
			WayPoint.setX(dWayPointX[1]);
			WayPoint.setY(dWayPointY[1]);
			newWayPointList.push_back(WayPoint);
			continue;
		}//기울기가 0이거나 분모가 0인경우 제외.

		da=(ntotalwaypointnum*dSigmaXY-dSigmaX*dSigmaY)/(ntotalwaypointnum*dSigmaX2-dSigmaX*dSigmaX);
		db=(dSigmaXY*dSigmaX-dSigmaX2*dSigmaY)/(dSigmaX*dSigmaX-3*dSigmaX2);
		

		dmidPointX=(-db*da+(dWayPointX[1]+da*dWayPointY[1]))/(da*da+1);//새로운 경유점 X를 계산한다.
		dmidPointY=((da*dWayPointX[1]+da*da*dWayPointY[1])+db)/(da*da+1);//새로운 경유점 Y를 계산한다.

		if(Obstacleflag((int)dmidPointX,(int)dmidPointY)==true)//맵의 정보와 비교하여 추출된 좌표가 장애물에 있는지 아닌지 검사한다.
		{//장애물이면 기존의 경유점을 대입힌다.
			WayPoint.setX(dWayPointX[1]);
			WayPoint.setY(dWayPointY[1]);
		}
		else{//장애물이 없다면 새로운 경유점을 대입힌다.
			WayPoint.setX((int)dmidPointX);
			WayPoint.setY((int)dmidPointY);
		}
	
		newWayPointList.push_back(WayPoint);
	}
	newWayPointList.push_back(WayPointList.back());

	return newWayPointList;
}
/**
 @brief Korean:  생성된 경로의 위치에 장애물이 있는지 없는지 여부를 확인하는 함수
 @brief English: 
*/
bool  KuPathSmoothing::Obstacleflag(int nmidPointX,int nmidPointY)
{//경유점이 OCCUPIED_AREA인지 아닌지 검사한다.
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
@brief Korean:  경유점으로 부터 원하는 간격만큼 Cubic Spline한 경로를 생성하는 함수
@brief English: 
*/
list<KuPose> KuPathSmoothing::doCubicSplineInterpolation(  double dinterval, list<KuPose> stlWayPoint)
{

	list<KuPose> WayPointlist=stlWayPoint;//경유점 리스트

	if (dinterval < 0.001) dinterval = 0.001;//path간 간격이 너무 적을 경우 0.001로 설정한다.
	if (WayPointlist.size() < 2) return stlWayPoint;//경유점의 개수가 1개일 경우 경유점을 내보낸다.

	unsigned int nWayPonitVectosize = WayPointlist.size () - 1;

	list<KuPose>::iterator itPastWayPoint;//이전 경유점을 가리키는 iterator이다.
	list<KuPose>::iterator itCurrentWayPoint;//현재 경유점을 가리키는 iterator이다.
	list<KuPose>::iterator itAftertWayPoint;//이후 경유점을 가리키는 iterator이다.


	//경유점간 거리를 계산한다.-----------------------------------------------------------------------------------------------------
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
	}//WayPonitDistanceGap은 경유점간의 거리의 간격이다.

	//dMatrix A 계산------------------------------------------------------------------------------------------------------------------------------------------
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

	//dMatrix b 계산-------------------------------------------------------------------------------------------------------------------------------------------------
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

	dMatrix dMatrixz = !dMatrixA*dMatrixb;//dMatrix z 계산

	//PathList 계산-------------------------------------------------------------------------------------------------------------------------------------------------
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
 @brief Korean:  경로부터 경유점을 추출하는 함수( 곡선에 따라 경유점이 뽑히는 정도가 다르게 나온다)
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