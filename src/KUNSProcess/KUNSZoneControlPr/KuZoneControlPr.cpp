#include "stdafx.h"
#include "KuZoneControlPr.h"

KuZoneControlPr::KuZoneControlPr()
{
	clear();
	m_smtpMap=NULL;
	m_dMap=NULL;
}
KuZoneControlPr::~KuZoneControlPr()
{ 

}

/**
@brief Korean: Path를 load하기위한 함수
*/
list<KuPose> KuZoneControlPr::loadPath(string  strDataPath)
{
	ifstream DataLog;
	m_PathList.clear();
	DataLog.open(strDataPath);

	double dPathX,dPathY,dPathThetaDeg;
	KuPose PathPos;
	while(!DataLog.eof()){
		DataLog >> dPathX >> dPathY>>dPathThetaDeg;
		PathPos.setX(dPathX);
		PathPos.setY(dPathY);
		PathPos.setThetaDeg(dPathThetaDeg);
		m_PathList.push_back(PathPos);
	}	
	DataLog.close();

	return m_PathList;
}
/**
@brief Korean: Zone이 나눠져 있는 지도 부르는 함수
*/
void KuZoneControlPr::saveZoneMap(int nPathIdx)
{
	string strNewPath;
	char cFilePathName[150];
	memset(cFilePathName,0,sizeof(cFilePathName));
	sprintf_s(cFilePathName,"./data/path/path_%d.txt",nPathIdx);
	strNewPath=cFilePathName;//char 포인터값을 string에 대입//path를 순서대로 string에 저장
	list <KuPose> Pathlist=loadPath(strNewPath);//path1 값을 불러와서 pathlist_1에 대입
	m_vecTotalPath.push_back(Pathlist);

	if(nPathIdx<2) return;

	list<KuPose> ZoenPoseList = ZonePoint(nPathIdx);

	generateZoneMap(ZoenPoseList);

	strNewPath= KuRobotParameter::getInstance()->getZoneMapNameNPath();
	saveZoneMap(strNewPath, m_smtpMap);
}


void KuZoneControlPr::saveZoneMap(string strMapFilePath, KuMap* pMap)
{

	int ** nMap = pMap->getMap();	
	int nMapSizeX = pMap->getX();
	int nMapSizeY = pMap->getY();
	IplImage* IplMapImage = cvCreateImage(cvSize (nMapSizeX, nMapSizeY),IPL_DEPTH_8U, 3);

	for(int LoopX=0 ; LoopX<pMap->getX() ; LoopX++){
		for(int LoopY=0 ; LoopY<pMap->getY(); LoopY++){

			if(	nMap[LoopX][pMap->getY()-1-LoopY] == 0 ){ //흰색
				IplMapImage->imageData[(LoopX+LoopY*pMap->getX())*3]=(unsigned char)255; //b
				IplMapImage->imageData[(LoopX+LoopY*pMap->getX())*3+1]=(unsigned char)255; //g
				IplMapImage->imageData[(LoopX+LoopY*pMap->getX())*3+2]=(unsigned char)255; //r
				continue;
			}

			for(int l=0; l<10;l++)
			{
				for(int m=0; m<10;m++)
				{
					for(int n=0; n<10;n++)
					{
						if(	nMap[LoopX][pMap->getY()-1-LoopY] == n+10*m+100*l ){ 
							IplMapImage->imageData[(LoopX+LoopY*pMap->getX())*3]=(unsigned char)25*n; //b
							IplMapImage->imageData[(LoopX+LoopY*pMap->getX())*3+1]=(unsigned char)25*m; //g
							IplMapImage->imageData[(LoopX+LoopY*pMap->getX())*3+2]=(unsigned char)25*l; //r
							n=100;	m=100;	l=100;
						}

					}
				}
			}	
		}
	}
	cvSaveImage(strMapFilePath.c_str(),IplMapImage);
	cvReleaseImage(&IplMapImage);
}
/**
 @brief Korean: bmp파일의 지도파일을 읽어드려 지도정보를 저장하는 변수에 저장한다. 
 @brief English: write in English
*/
bool KuZoneControlPr::loadZoneMap(string strMapFilePath)
{
	FILE	*infile;
 	infile = fopen(strMapFilePath.c_str() ,"rb");
 	if(infile==NULL) {
 		cout<< "[CMapRepository] : Error - Could not open map from "<<strMapFilePath<<endl;
		return false;
 	}
	fclose(infile);

	IplImage *IplMapImage = cvLoadImage(strMapFilePath.c_str(),1);

	int nMapSizeX = IplMapImage->width;
	int nMapSizeY = IplMapImage->height;


	if(NULL==m_smtpMap){
		m_smtpMap = new KuMap(nMapSizeX, nMapSizeY); //map이 존재하지 않을 때 map 형성.
	}
	else{//map의 사이즈가 다를 경우. 
		if(nMapSizeX != m_smtpMap->getX() || nMapSizeY !=m_smtpMap->getY())
		{
			delete m_smtpMap;
			m_smtpMap = new KuMap(nMapSizeX, nMapSizeY); 			
		}
	}	


	int **nMap=m_smtpMap->getMap();

	for(int nX=0; nX< IplMapImage->width; nX++){
		for(int nY=0; nY<IplMapImage->height; nY++){

			if(IplMapImage->imageData[(nX+nY*IplMapImage->width)*3] == ( char)0 && //Blue
				IplMapImage->imageData[(nX+nY*IplMapImage->width)*3+1] == ( char)0 && //Green
				IplMapImage->imageData[(nX+nY*IplMapImage->width)*3+2] == ( char)0){ //Red 

					nMap[nX][IplMapImage->height-1-nY] = 0;				
					continue;
			}

			for(int l=0; l<10;l++)
			{
				for(int m=0; m<10;m++)
				{
					for(int n=0; n<10;n++)
					{
						int nB=25*n; int nR=25*l; int nG=25*m;

						if(IplMapImage->imageData[(nX+nY*IplMapImage->width)*3] == (char)nB && //Blue
							IplMapImage->imageData[(nX+nY*IplMapImage->width)*3+1] == (char)nG && //Green
							IplMapImage->imageData[(nX+nY*IplMapImage->width)*3+2] == (char)nR){ //Red 

								nMap[nX][IplMapImage->height-1-nY] = n+10*m+100*l;				
								n=100;	m=100;	l=100;
						}
					}
				}
			}	

		}
	} 

 	cout<< "[CMapRepository] : Map loading success!!!" << endl;	
 	cvReleaseImage(&IplMapImage);
 	

	KuDrawingInfo::getInstance()->setZoneMap(m_smtpMap); //지도 정보 설정

	return true;
}
/**
@brief Korean: Zone이 나눠져 있는 지도 부르는 함수
*/
void KuZoneControlPr::LoadZoneMap()
{
	string strNewPath;
	int pathIdx=1;
	char cFilePathName[150];

	//------------------------------------path1---------------------------------------------	
	memset(cFilePathName,0,sizeof(cFilePathName));
	sprintf_s(cFilePathName,"./data/path/path_%d.txt",pathIdx);
	strNewPath=cFilePathName;//char 포인터값을 string에 대입//path를 순서대로 string에 저장
	pathIdx++;//실행될 때마다 카운트(ex: path1, path2, path3)

	//string strDataPath_1 =KuRobotParameter::getInstance()->getTeachingPathNameNPath();
	list <KuPose> Pathlist_1=loadPath(strNewPath);//path1 값을 불러와서 pathlist_1에 대입
	KuDrawingInfo::getInstance()->setPath(Pathlist_1);//path1 그리기
	//------------------------------------path2---------------------------------------------	
	memset(cFilePathName,0,sizeof(cFilePathName));
	sprintf_s(cFilePathName,"./data/path/path_%d.txt",pathIdx);
	strNewPath=cFilePathName;//char 포인터값을 string에 대입
	pathIdx++;//실행될 때마다 카운트

	//string strDataPath_2 =KuRobotParameter::getInstance()->getTeachingPathNameNPath();
	list <KuPose> Pathlist_2=loadPath(strNewPath);
	//KuDrawingInfo::getInstance()->setPath2(Pathlist_2);
}

/** 
@brief Korean: 추출된 point를 확장하여 Zone을 형성시키는 함수
*/
void  KuZoneControlPr::generateZoneMap(list<KuPose> ZonePointlist)
{ 
	ZonePointlist = m_PathList;

	int nMapSizeX=1000;
	int nMapSizeY=1000;

	if(NULL==m_smtpMap){
		m_smtpMap = new KuMap(nMapSizeX, nMapSizeY); //map이 존재하지 않을 때 map 형성.
	}
	else{//map의 사이즈가 다를 경우. 
		if(nMapSizeX != m_smtpMap->getX() || nMapSizeY !=m_smtpMap->getY())
		{
			delete m_smtpMap;
			m_smtpMap = new KuMap(nMapSizeX, nMapSizeY); 			
		}
	}	

	int **nMap=m_smtpMap->getMap();

	for(int i=0;i<nMapSizeX;i++)
	{
		for(int j=0;j<nMapSizeY;j++)
		{
			nMap[i][j]=0;
		}
	}

	list<KuPose>::iterator it;
	int nZone=1;
	//int nBoundary = 10;//1m
	int nZoneNum=0;

	for (it = ZonePointlist.begin(); it != ZonePointlist.end(); it++,nZone++) {
		int ntempX = it->getXm()*10;
		int ntempY = it->getYm()*10;
		int nDist = it->getDist()/100.0;
		nZoneNum++;
		for(int nX=-nDist; nX<nDist; nX++)
		{
			for(int nY=-nDist; nY<nDist; nY++)
			{
				if(ntempX+nX<1||ntempY+nY<1||ntempX+nX>nMapSizeX-1||ntempY+nY>nMapSizeY-1) continue;

				nMap[ntempX+nX][ntempY+nY]=nZone;
			}
		}
	}

	for(int j=0; j<25;j++)
		for(int i=0; i<nZoneNum;i++)
		{
			compensateMap( nMapSizeX,  nMapSizeY,  nMap, i);
		}

		//KuDrawingInfo::getInstance()->setZoneMap(m_smtpMap);

		//return  m_smtpMap;

}
int KuZoneControlPr::getZonedata(KuPose RobotPos)
{
	RobotPos =KuDrawingInfo::getInstance()->getRobotPos();

	int **nMap=m_smtpMap->getMap();
	int ZoneName=0;
	double temp_X= RobotPos.getX()/100;
	double temp_Y= RobotPos.getY()/100;

	int X = (int)temp_X;
	int Y = (int)temp_Y;

	ZoneName=nMap[X][Y];

	return ZoneName;

}
int KuZoneControlPr::getNextZonedata(KuPose RobotPos, KuPose TargetPos)
{
// 	RobotPos =KuDrawingInfo::getInstance()->getRobotPos();
// 	TargetPos =KuDrawingInfo::getInstance()->getTargetPos();

	int **nMap=m_smtpMap->getMap();
	int NextZoneName=0;
	double temp_X= RobotPos.getXm()*10;
	double temp_Y= RobotPos.getYm()*10;
	double target_X = TargetPos.getXm()*10;
	double target_Y = TargetPos.getYm()*10;

	int X_R = (int)temp_X;
	int Y_R = (int)temp_Y;

	double delta_X =(target_X-temp_X)/1000.0;
	double delta_Y = (target_Y-temp_Y)/1000.0;
	int n=0;

	if(m_smtpMap->getX()-1<X_R||m_smtpMap->getY()-1<Y_R
		||1>Y_R||1>X_R){
		return -1;
	}
	int nCurZone= nMap[X_R][Y_R];

	while( true)
	{
		if(abs(X_R+(int)(delta_X*n))>m_smtpMap->getX()) break;
		if(abs(Y_R+(int)(delta_Y*n))>m_smtpMap->getY()) break;

		if(m_smtpMap->getX()-1<X_R+(int)(delta_X*n)||m_smtpMap->getY()-1<Y_R+(int)(delta_Y*n)
			||1>Y_R+(int)(delta_Y*n)||1>X_R+(int)(delta_X*n)){
				n++;
				continue;
		}
	
		NextZoneName=nMap[X_R+(int)(delta_X*n)][Y_R+(int)(delta_Y*n)];

		if(nCurZone!=NextZoneName)
		{

			return NextZoneName;
		}
		n++;
	}

}
/**
@brief Korean: 추출된 point를 확장하여 Zone을 형성시키는 함수
*/
void KuZoneControlPr::intersectionZoneMap(list<KuPose> ZonePointlist)
{
	ZonePointlist = m_PathList;

	int dMapSizeX=1000;
	int dMapSizeY=1000;

	double **dMap=m_dMap->getProbMap();

	for(int i=0;i<dMapSizeX;i++)
	{
		for(int j=0;j<dMapSizeY;j++)
		{
			dMap[i][j]=0;
		}
	}
	vector<KuPose> vintersectionZone;

	vintersectionZone.clear();

	for(int i=0; i<vintersectionZone.size(); i++){
		KuPose Zone;
		Zone.setX(vectorWayPoint_temp[i].getX());
		Zone.setY(vectorWayPoint_temp[i].getY());
		vintersectionZone.push_back(Zone);
	}

}

void KuZoneControlPr::compensateMap(int nSizeX, int nSizeY, int** nMap,int nZone)//Mask로 Zone확장하는 함수
{
	int ntemp=1000000;

	//빈공간을 메꾸는 루틴 시작..
	for(int i=1;i<nSizeX-1;i++){
		for(int j=1;j<nSizeY-1;j++){
			if(nMap[i][j] == nZone) { //빈공간 중에서 상하좌우에 unknown:(0.5) 지역이 있으면
				//1로 메우기 위한 과정..
				if(nMap[i-1][j-1] == 0 ){ //상 검사...
					nMap[i-1][j-1] = ntemp;
				}
				if(nMap[i-1][j] == 0 ){ //상 검사...
					nMap[i-1][j] = ntemp;
				}
				if(nMap[i-1][j+1] == 0 ){ //상 검사...
					nMap[i-1][j+1] = ntemp;
				}
				if(nMap[i][j-1]== 0){ //좌 검사.
					nMap[i][j-1] = ntemp;
				}
				if(nMap[i][j+1]== 0){ //좌 검사.
					nMap[i][j+1] = ntemp;
				}
				if(nMap[i+1][j-1] == 0){ //하 검사.
					nMap[i+1][j-1]  = ntemp;
				}
				if(nMap[i+1][j] == 0 ){ //상 검사...
					nMap[i+1][j] = ntemp;
				}
				if(nMap[i+1][j+1]== 0){ //좌 검사.
					nMap[i+1][j+1] = ntemp;
				}
				if(nMap[i][j+1]== 0){  //우 검사.
					nMap[i][j+1] = ntemp;
				}
			}
		}
	}
	for(int i=1;i<nSizeX-1;i++){
		for(int j=1;j<nSizeY-1;j++){
			if(nMap[i][j] == ntemp) { //빈공간 중에서 상하좌우 에 unknown:(0.5) 지역이 있으면
				nMap[i][j] =nZone;
			}
		}
	}
}
/**
@brief Korean: Zone Point를 list로 return하는 함수
*/
list<KuPose> KuZoneControlPr::ZonePoint(int nPathIdx)
{
	string strNewPath;
	int pathIdx=1;
	char cFilePathName[150];


	list <KuPose> Pathlist_1=m_vecTotalPath[0];//path1 값을 불러와서 pathlist_1에 대입
	//------------------------------------path2---------------------------------------------	

	list <KuPose> Pathlist_2=m_vecTotalPath[1];

	//-----------------------------------corner와 시작점 추출----------------------------------------
	vector<KuPose> vectorPointtoCorner;

	vectorPointtoCorner.clear();

	m_vectorWayPoint.clear();
	m_vectorWayPoint=extractWayPoint(Pathlist_1);//path1의 corner 추출 => m_vectorWayPoint에 대입

	vectorWayPoint_temp.clear();
	vectorWayPoint_temp=extractWayPoint(Pathlist_2);//paht2의 corner 추출 => vectorWayPoint_temp에 대입

	for(int i=0; i<vectorWayPoint_temp.size(); i++){
		KuPose WayPoint;
		WayPoint.setX(vectorWayPoint_temp[i].getX());
		WayPoint.setY(vectorWayPoint_temp[i].getY());
		m_vectorWayPoint.push_back(WayPoint);
	}//m_vectorWayPoint에 vectorWayPoint_temp값 추가   //	path1_corner+시작점 + path2_corner+시작점

	m_PathList.clear();
	for(int i=0; i<m_vectorWayPoint.size(); i++){
		KuPose WayPoint;
		WayPoint.setX(m_vectorWayPoint[i].getX());
		WayPoint.setY(m_vectorWayPoint[i].getY());
		vectorPointtoCorner.push_back(WayPoint);
	}//path1,2의 corner정보 벡터 => List에 대입 == m_PathList

	//-----------------------------------끝점 추출----------------------------------------
	list<KuPose>::iterator iteratorPath;
	KuPose startWayPoint_1;
	iteratorPath = Pathlist_1.begin();
	startWayPoint_1.setX(iteratorPath->getX());
	startWayPoint_1.setY(iteratorPath->getY());
	m_PathList.push_back(startWayPoint_1);

	iteratorPath = Pathlist_1.end();
	iteratorPath--;
	startWayPoint_1.setX(iteratorPath->getX());
	startWayPoint_1.setY(iteratorPath->getY());
	m_PathList.push_back(startWayPoint_1);


	KuPose startWayPoint_2;
	iteratorPath = Pathlist_2.begin();
	startWayPoint_2.setX(iteratorPath->getX());
	startWayPoint_2.setY(iteratorPath->getY());
	m_PathList.push_back(startWayPoint_2);


	iteratorPath = Pathlist_2.end();
	iteratorPath--;
	startWayPoint_2.setX(iteratorPath->getX());
	startWayPoint_2.setY(iteratorPath->getY());
	m_PathList.push_back(startWayPoint_2);

	//-----------------------------------교차점 추출----------------------------------------
	vector<KuPose> vectorCrossPoint;
	vector<KuPose> vectorintersectionPoint;

	vectorWayPoint_temp.clear();
	vectorWayPoint_temp=extractCrossPoint(Pathlist_1,Pathlist_2);
	for(int i=0; i<vectorWayPoint_temp.size(); i++){
		KuPose WayPoint;
		WayPoint.setX(vectorWayPoint_temp[i].getX());
		WayPoint.setY(vectorWayPoint_temp[i].getY());
		vectorCrossPoint.push_back(WayPoint);
		vectorPointtoCorner.push_back(WayPoint);
	}

	vector<KuPose> vectempextraRegion;
	list<KuPose>::iterator it;
	for (it = m_PathList.begin(); it != m_PathList.end(); it++) {
		KuPose LastPoint;
		KuPose SelPoint;

		LastPoint.setX(it->getX());
		LastPoint.setY(it->getY());
		double Mindist=1000000000000000;
		for(int i=0; i<vectorPointtoCorner.size(); i++){
			double dist=hypot(LastPoint.getX()- vectorPointtoCorner[i].getX(),LastPoint.getY()- vectorPointtoCorner[i].getY());
			if(Mindist>dist)
			{
				Mindist=dist;
				SelPoint.setX( vectorPointtoCorner[i].getX());
				SelPoint.setY( vectorPointtoCorner[i].getY());

			}
		}
		KuPose MidPoint;

		MidPoint.setX((LastPoint.getX()+SelPoint.getX())/2.0);
		MidPoint.setY((LastPoint.getY()+SelPoint.getY())/2.0);
		vectempextraRegion.push_back(MidPoint);
	}

	for(int i=0; i<vectempextraRegion.size(); i++){
		KuPose WayPoint;
		WayPoint.setX(vectempextraRegion[i].getX());
		WayPoint.setY(vectempextraRegion[i].getY());
		vectorPointtoCorner.push_back(WayPoint);		
	}

	for(int i=0; i<vectorPointtoCorner.size(); i++){
		KuPose WayPoint;
		WayPoint.setX(vectorPointtoCorner[i].getX());
		WayPoint.setY(vectorPointtoCorner[i].getY());
		m_PathList.push_back(WayPoint);
	}
	// m_PathList에는 corner점 + 끝점 + 시작점 + 교차점 정보가 들어있음 

	list<KuPose>::iterator jt;
	for (it = m_PathList.begin(); it != m_PathList.end(); it++) {

		KuPose LastPoint;
		KuPose SelPoint;
		LastPoint.setX(it->getX());
		LastPoint.setY(it->getY());
		double Mindist=1000000000000000;

		for (jt = m_PathList.begin(); jt != m_PathList.end(); jt++) {
			SelPoint.setX(jt->getX());
			SelPoint.setY(jt->getY());
			double dist=hypot(LastPoint.getX()- SelPoint.getX(),LastPoint.getY()- SelPoint.getY());

			if(Mindist>dist&&dist!=0)
			{
				Mindist=dist;
			}

		}
		it->setDist(Mindist/2.0);

	}


	return m_PathList;
}

/**
@brief Korean:3개의 경로에서 코너점과 시작점 추출하여 벡터로 리턴하는 함수.-민국이형이 준 code
*/
vector<KuPose> KuZoneControlPr::extractWayPoint(list<KuPose> Pathlist)
{
	int nCnt=0;
	int nWayPointValue = 1; 
	int nPathCnt=Pathlist.size();
	KuMath m_math;//멤버변수로 설정
	vector<KuPose> vectorWayPoint;// 결과 값

	KuPose WayPoint;

	//list<Path>::iterator iPath;
	list<KuPose>::iterator iteratorPath;
	int *nPathX = (int*)calloc(nPathCnt, sizeof(int));
	int *nPathY = (int*)calloc(nPathCnt, sizeof(int));

	for(iteratorPath = Pathlist.begin();  iteratorPath != Pathlist.end(); iteratorPath++){
		nPathX[nCnt] = (int)m_math.MM2AI(iteratorPath->getX());
		nPathY[nCnt] = (int)m_math.MM2AI(iteratorPath->getY());
		nCnt++;
	}    

	bool bCheckCluster = true;
	int *anStartPoint = (int*)calloc(nPathCnt, sizeof(int));
	int *anFinishPoint = (int*)calloc(nPathCnt, sizeof(int));

	anStartPoint[0] = 0;
	anFinishPoint[0] = nPathCnt-1;

	int iIndex =0;
	int nNum =0;
	double nMaxDist =0;
	double dLinearEqA, dLinearEqB, dLinearEqC ; //ax+by+c=0... .... ..
	double dVerticalDist;	//..... ....

	int nPoint1;	//i 
	int nPoint2=0;	//j

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

		if (nMaxDist>=nWayPointValue){
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

	for(int nOrderCluster=0; nOrderCluster<nNum-1; nOrderCluster++){

		int nX = (int)( m_math.AI2MM( nPathX[anFinishPoint[nOrderCluster]] ) );
		int nY = (int)( m_math.AI2MM( nPathY[anFinishPoint[nOrderCluster]] ) );
		WayPoint.setX(nX);
		WayPoint.setY(nY);
		vectorWayPoint.push_back(WayPoint);
	}

	free(nPathX);
	free(nPathY);
	free(anStartPoint);
	free(anFinishPoint);

	return vectorWayPoint;
}

/**
@brief Korean: 교차점을 벡터(vectorWayPoint)로 추출해서 리턴하는 함수
*/
vector<KuPose> KuZoneControlPr::extractCrossPoint(list<KuPose> Pathlist_a,list<KuPose> Pathlist_b)
{
	KuMath m_math;//멤버변수로 설정
	double dDist;
	double dDistance;
	list<KuPose>::iterator iteratorPath_a;
	list<KuPose>::iterator iteratorPath_b;
	vector<KuPose> vectorWayPoint;
	vectorWayPoint.clear();

	for(iteratorPath_a = Pathlist_a.begin();  iteratorPath_a != Pathlist_a.end(); iteratorPath_a++)
	{
		for(iteratorPath_b = Pathlist_b.begin();  iteratorPath_b != Pathlist_b.end(); iteratorPath_b++)//for문 2개 돌림으로 
		{
			dDist=hypot(iteratorPath_a->getX()-iteratorPath_b->getX(), iteratorPath_a->getY()-iteratorPath_b->getY());

			if(dDist<300)
			{
				for(int i=0; i<vectorWayPoint.size(); i++)
				{
					dDistance=hypot(vectorWayPoint[i].getX()-iteratorPath_a->getX(), vectorWayPoint[i].getY()-iteratorPath_a->getY());
					if(dDistance>500)//300루트2제외시켜야함
						vectorWayPoint.push_back(*iteratorPath_a);

					dDistance=hypot(vectorWayPoint[i].getX()-iteratorPath_b->getX(), vectorWayPoint[i].getY()-iteratorPath_b->getY());
					if(dDistance>500)
						vectorWayPoint.push_back(*iteratorPath_b);
				}

				if(vectorWayPoint.size()<1)
				{
					vectorWayPoint.push_back(*iteratorPath_a);
				}
			}
		}  
	}    

	return vectorWayPoint;
}

KuPose KuZoneControlPr::distZone(KuPose RobotPos, list<KuPose> m_PathList)
{
	KuMath m_math;//멤버변수로 설정
	double dDist;
	double dmin_Dist;
	list<KuPose>::iterator iteratorPath;
	m_ZonePosition.clear();
	KuPose ZonePoint;

	dmin_Dist=10000000000000.0;
	for(iteratorPath = m_PathList.begin();  iteratorPath != m_PathList.end(); iteratorPath++)
	{
		dDist=hypot(RobotPos.getX()-iteratorPath->getX(), RobotPos.getY()- iteratorPath->getY());

		if(dDist<dmin_Dist)
		{
			dmin_Dist=dDist;
			ZonePoint.setX(iteratorPath->getX());
			ZonePoint.setY(iteratorPath->getY());
		}
	}

	return ZonePoint;
}

/**
@brief Korean: List 초기화
*/
void KuZoneControlPr::clear()
{
	//기존의 경로 및 경유점 정보를 삭제한다.
	m_PathList.clear();
	m_WayPointList.clear();
}

/**
@brief Korean: 
*/
KuMap* KuZoneControlPr::getZoneMap()
{
	return m_smtpMap;
}
