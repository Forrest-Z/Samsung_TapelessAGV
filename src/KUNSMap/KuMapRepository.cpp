#include "stdafx.h"
#include "kuMapRepository.h"

/**
 @brief Korean: 주행에 필요한 지도정보를 저장하고 있는 클래스이다.
 @brief         하나의 객체만이 존재해야 하기 때문에 싱글톤 타입으로 생성된다. 
 @brief English: write in English
*/
kuMapRepository::kuMapRepository()
{
	cout<<"[kuMapRepository]: Singletone type instance is created!!!"<<endl;

	m_smtpMap = NULL;
	m_velocitypMap=NULL;

	// Initialize
	m_poseRobotTemp.setXm(KuRobotParameter::getInstance()->getMapSizeXm() / 2.);
	m_poseRobotTemp.setYm(KuRobotParameter::getInstance()->getMapSizeYm() / 2.);
}

kuMapRepository::~kuMapRepository()
{
	if(m_smtpMap!=NULL)
	delete m_smtpMap;
	if(m_velocitypMap!=NULL)
	delete m_velocitypMap;

	cout<<"[kuMapRepository]: Singletone type instance is destroyed!!!"<<endl;	
}


/**
 @brief Korean: bmp파일의 지도파일을 로드하여 지도정보를 저장하는 변수에 저장한다. 
 @brief English: write in English
*/
bool kuMapRepository::loadMap(string strMapFilePath)
{
	FILE	*infile;
 	infile = fopen(strMapFilePath.c_str() ,"rb");
 	if(infile==NULL) {
 		cout<< "[CMapRepository] : Error - Could not open map from "<<strMapFilePath<<endl;
		return false;
 	}
	fclose(infile);
	m_strMapFilePath =strMapFilePath;
	m_IplMapImage = cvLoadImage(strMapFilePath.c_str(),1);

	int nImageSizeX = m_IplMapImage->width;
	int nImageSizeY = m_IplMapImage->height;
	int nMapSizeX = KuRobotParameter::getInstance()->getMapSizeXm()*10;
	int nMapSizeY = KuRobotParameter::getInstance()->getMapSizeYm()*10;
	if(nMapSizeX<nImageSizeX)
	{
		nImageSizeX = nMapSizeX;
	}
	if(nMapSizeY<nImageSizeY)
	{
		nImageSizeY = nMapSizeY;
	}
	
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

	for(int nX=0; nX< nImageSizeX; nX++){
		for(int nY=0; nY< nImageSizeY; nY++){
			if(m_IplMapImage->imageData[(nX+nY*m_IplMapImage->width)*3] == (char)255 && //Blue
				m_IplMapImage->imageData[(nX+nY*m_IplMapImage->width)*3+1] == (char)255 && //Green
				m_IplMapImage->imageData[(nX+nY*m_IplMapImage->width)*3+2] == (char)255	){ //Red 

				nMap[nX][nImageSizeY-1-nY] = KuMap::EMPTY_AREA;				

			}
			else if(m_IplMapImage->imageData[(nX+nY*m_IplMapImage->width)*3] == (char)0 && 
				m_IplMapImage->imageData[(nX+nY*m_IplMapImage->width)*3+1] == (char)0 &&
				m_IplMapImage->imageData[(nX+nY*m_IplMapImage->width)*3+2] == (char)0){

				nMap[nX][nImageSizeY-1-nY] =  KuMap::OCCUPIED_AREA;
			}
			else if(m_IplMapImage->imageData[(nX+nY*m_IplMapImage->width)*3] == (char)0 && 
				m_IplMapImage->imageData[(nX+nY*m_IplMapImage->width)*3+1] == (char)255 &&
				m_IplMapImage->imageData[(nX+nY*m_IplMapImage->width)*3+2] == (char)0){

				nMap[nX][nImageSizeY-1-nY] =  KuMap::GIVEN_PATH_AREA;					
			}
			else if(m_IplMapImage->imageData[(nX+nY*m_IplMapImage->width)*3] == (char)0 && 
				m_IplMapImage->imageData[(nX+nY*m_IplMapImage->width)*3+1] == (char)0 &&
				m_IplMapImage->imageData[(nX+nY*m_IplMapImage->width)*3+2] == (char)255){

				nMap[nX][nImageSizeY-1-nY] =  KuMap::VIRTUAL_WALL_AREA;

			}
			else{
				nMap[nX][nImageSizeY-1-nY] =  KuMap::UNKNOWN_AREA;				

			}
		}
	} 

 	cout<< "[CMapRepository] : Map loading success!!!" << endl;	
 	cvReleaseImage(&m_IplMapImage);
 	
	KuPose RobotPos;
	RobotPos.setXm( (double)nMapSizeX/10 /2. );
	RobotPos.setYm( (double)nMapSizeY/10 /2. );
	KuDrawingInfo::getInstance()->setRobotPos(RobotPos); //지도를 기준 중앙위치를 로봇의 초기위치로 설정
	KuDrawingInfo::getInstance()->setMap(m_smtpMap); //지도 정보 설정

	return true;
}

/**
 @brief Korean: bmp파일의 지도파일을 읽어드려 지도정보를 저장하는 변수에 저장한다. 
 @brief English: write in English
*/
KuMap* kuMapRepository::getMap()
{
	return m_smtpMap;
}
 void kuMapRepository::saveMap(KuMap*pMap, bool bFiltering, bool bDrawingInfoSetMap)
{
	m_smtpMap=pMap;
	if(bFiltering)
	{
		filterMap(m_smtpMap);
		filterMapForOccupiedGrid(m_smtpMap);
		compensateMap(m_smtpMap->getX(), m_smtpMap->getY(),m_smtpMap->getMap());
	}
	generateMap(KuRobotParameter::getInstance()->getMapNameNPath(),m_smtpMap );
	generateMap(KuRobotParameter::getInstance()->getVelocityMapNameNPath(),m_smtpMap );

	if(bDrawingInfoSetMap)
	{
		KuDrawingInfo::getInstance()->setMap(m_smtpMap);
	}

}
 
 void kuMapRepository::saveMapNavi(KuMap*pMap)
{
	m_smtpMap=pMap;
	generateMap(KuRobotParameter::getInstance()->getMapNameNPath(),m_smtpMap );
	KuDrawingInfo::getInstance()->setMap(m_smtpMap);

}

void kuMapRepository::compensateMap(int nSizeX, int nSizeY, int** nMap)
{

	//빈공간을 메꾸는 루틴 시작..
	for(int i=1;i<nSizeX-1;i++){
		for(int j=1;j<nSizeY-1;j++){
			if(nMap[i][j] ==KuMap::EMPTY_AREA) { //빈공간 중에서 상하좌우 에 unknown:(0.5) 지역이 있으면
				//1로 메우기 위한 과정..
				if(nMap[i-1][j] == KuMap::UNKNOWN_AREA ){ //상 검사...
					nMap[i][j] = KuMap::OCCUPIED_AREA;
				}
				else if(nMap[i+1][j] == KuMap::UNKNOWN_AREA){ //하 검사.
					nMap[i][j] = KuMap::OCCUPIED_AREA;
				}
				else if(nMap[i][j-1]== KuMap::UNKNOWN_AREA){ //좌 검사.
					nMap[i][j] = KuMap::OCCUPIED_AREA;
				}
				else if(nMap[i][j+1]== KuMap::UNKNOWN_AREA){  //우 검사.
					nMap[i][j] = KuMap::OCCUPIED_AREA;
				}
			}
		}
	}

}

void kuMapRepository::filterMap(KuMap* pMap)
{
	//모폴로지 연산을 통해 지도를 보정해주는 작업을 하는 함수
	for(int i=0;i<4;i++){
		doMorphologyClose(pMap);
		}

	for(int i=0;i<4;i++){
		doMorphologyOpen(pMap);
	}
}


void kuMapRepository::filterMapForOccupiedGrid(KuMap* pMap)
{
	//모폴로지 연산을 통해 지도를 보정해주는 작업을 하는 함수
	for(int i=0;i<3;i++){
		doMorphologyCloseForOccupiedGrid(pMap);
	}

	for(int i=0;i<2;i++){
		doMorphologyOpenForOccupiedGrid(pMap);
	}
}


void kuMapRepository::doMorphologyClose(KuMap* pMap)
{

	int nTmpVal=12345678;
	int **  nMap = pMap->getMap();
	for(int i=0;i<pMap->getX();i++){
		for(int j=0;j<pMap->getY();j++){
			if(i<1 || i > pMap->getX()-2 || j<1 || j> pMap->getY()-2){
				continue;
			}

			if(nMap[i][j] ==KuMap::EMPTY_AREA) {
				if(nMap[i-1][j] == KuMap::UNKNOWN_AREA ){ //상 검사...
					nMap[i-1][j] = nTmpVal;
				}
				else if(nMap[i+1][j] == KuMap::UNKNOWN_AREA){ //하 검사.
					nMap[i+1][j] = nTmpVal;
				}
				else if(nMap[i][j-1]== KuMap::UNKNOWN_AREA){ //좌 검사.
					nMap[i][j-1] = nTmpVal;
				}
				else if(nMap[i][j+1]== KuMap::UNKNOWN_AREA){  //우 검사.
					nMap[i][j+1] = nTmpVal;
				}
			}
		}
	}

	for(int i=0;i<pMap->getX();i++){
		for(int j=0;j<pMap->getY();j++){
			if(nMap[i][j] == nTmpVal ){
				nMap[i][j] = KuMap::EMPTY_AREA;
			}
		}
	}
	
}


void kuMapRepository::doMorphologyOpen(KuMap* pMap)
{
	int nTmpVal=12345678;
	int ** nMap = pMap->getMap();
	for(int i=0;i<pMap->getX();i++){
		for(int j=0;j<pMap->getY();j++){
			if(i<1 || i > pMap->getX()-2 || j<1 || j> pMap->getY()-2){
				continue;
			}

			if(nMap[i][j] ==KuMap::EMPTY_AREA) {
				if(nMap[i-1][j] == KuMap::UNKNOWN_AREA ){ //상 검사...
					nMap[i][j] = nTmpVal;
				}
				else if(nMap[i+1][j] == KuMap::UNKNOWN_AREA){ //하 검사.
					nMap[i][j] = nTmpVal;
				}
				else if(nMap[i][j-1]== KuMap::UNKNOWN_AREA){ //좌 검사.
					nMap[i][j] = nTmpVal;
				}
				else if(nMap[i][j+1]== KuMap::UNKNOWN_AREA){  //우 검사.
					nMap[i][j] = nTmpVal;
				}
			}
		}
	}
	for(int i=0;i<pMap->getX();i++){
		for(int j=0;j<pMap->getY();j++){
			if(nMap[i][j] == nTmpVal ){
				nMap[i][j] = KuMap::UNKNOWN_AREA;
			}
		}
	}

}


void kuMapRepository::doMorphologyCloseForOccupiedGrid(KuMap* pMap)
{

	int nTmpVal=12345678;
	int ** nMap = pMap->getMap();
	for(int i=0;i<pMap->getX();i++){
		for(int j=0;j<pMap->getY();j++){
			if(i<1 || i > pMap->getX()-2 || j<1 || j> pMap->getY()-2){
				continue;
			}

			if(nMap[i][j] ==KuMap::OCCUPIED_AREA) {
				if( nMap[i-1][j] == KuMap::EMPTY_AREA ){ //상 검사...
					nMap[i-1][j] = nTmpVal;
				}
				else if( nMap[i+1][j] == KuMap::EMPTY_AREA){ //하 검사.
					nMap[i+1][j] = nTmpVal;
				}
				else if( nMap[i][j-1]== KuMap::EMPTY_AREA){ //좌 검사.
					nMap[i][j-1] = nTmpVal;
				}
				else if( nMap[i][j+1]== KuMap::EMPTY_AREA ){  //우 검사.
					nMap[i][j+1] = nTmpVal;
				}
			}
		}
	}

	for(int i=0;i<pMap->getX();i++){
		for(int j=0;j<pMap->getY();j++){
			if(nMap[i][j] == nTmpVal ){
				nMap[i][j] = KuMap::OCCUPIED_AREA;
			}
		}
	}


}


void kuMapRepository::doMorphologyOpenForOccupiedGrid(KuMap* pMap)
{
	int nTmpVal=12345678;
	int ** nMap = pMap->getMap();
	for(int i=0;i<pMap->getX();i++){
		for(int j=0;j<pMap->getY();j++){
			if(i<1 || i > pMap->getX()-2 || j<1 || j> pMap->getY()-2){
				continue;
			}

			if(nMap[i][j] ==KuMap::OCCUPIED_AREA) {
				if( nMap[i-1][j] == KuMap::EMPTY_AREA){ //상 검사...
					nMap[i][j] = nTmpVal;
				}
				else if( nMap[i+1][j] == KuMap::EMPTY_AREA){ //하 검사.
					nMap[i][j] = nTmpVal;
				}
				else if( nMap[i][j-1]== KuMap::EMPTY_AREA){ //좌 검사.
					nMap[i][j] = nTmpVal;
				}
				else if( nMap[i][j+1]== KuMap::EMPTY_AREA ){  //우 검사.
					nMap[i][j] = nTmpVal;
				}
			}
		}
	}
	for(int i=0;i<pMap->getX();i++){
		for(int j=0;j<pMap->getY();j++){
			if(nMap[i][j] == nTmpVal ){
				nMap[i][j] = KuMap::EMPTY_AREA;
			}
		}
	}


}

void kuMapRepository::generateMap(string strMapFilePath, KuMap* pMap)
{

	int ** nMap = pMap->getMap();	
	int nMapSizeX = pMap->getX();
	int nMapSizeY = pMap->getY();
	IplImage* IplMapImage = cvCreateImage(cvSize (nMapSizeX, nMapSizeY),IPL_DEPTH_8U, 3);

	for(int LoopX=0 ; LoopX<pMap->getX() ; LoopX++){
		for(int LoopY=0 ; LoopY<pMap->getY(); LoopY++){
			if(	nMap[LoopX][pMap->getY()-1-LoopY] == KuMap::EMPTY_AREA ){ //흰색
				IplMapImage->imageData[(LoopX+LoopY*pMap->getX())*3]=(char)255; //b
				IplMapImage->imageData[(LoopX+LoopY*pMap->getX())*3+1]=(char)255; //g
				IplMapImage->imageData[(LoopX+LoopY*pMap->getX())*3+2]=(char)255; //r
			}
			else if(nMap[LoopX][pMap->getY()-1-LoopY]== KuMap::UNKNOWN_AREA ){ //회색
				IplMapImage->imageData[(LoopX+LoopY*pMap->getX())*3]=(char)150; //b
				IplMapImage->imageData[(LoopX+LoopY*pMap->getX())*3+1]=(char)150; //g
				IplMapImage->imageData[(LoopX+LoopY*pMap->getX())*3+2]=(char)150; //r
			}
			else if(nMap[LoopX][pMap->getY()-1-LoopY] == KuMap::OCCUPIED_AREA ){ //검은색
				IplMapImage->imageData[(LoopX+LoopY*pMap->getX())*3]=(char)0; //b
				IplMapImage->imageData[(LoopX+LoopY*pMap->getX())*3+1]=(char)0; //g
				IplMapImage->imageData[(LoopX+LoopY*pMap->getX())*3+2]=(char)0; //r
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
bool kuMapRepository::loadVelocityMap(string strMapFilePath)
{
	FILE	*infile;
 	infile = fopen(strMapFilePath.c_str() ,"rb");
 	if(infile==NULL) {
 		cout<< "[CMapRepository] : Error - Could not open map from "<<strMapFilePath<<endl;
		return false;
 	}
	fclose(infile);

	m_IplMapImage = cvLoadImage(strMapFilePath.c_str(),1);

	int nMapSizeX = m_IplMapImage->width;
	int nMapSizeY = m_IplMapImage->height;


	if(NULL==m_velocitypMap){
		m_velocitypMap = new KuMap(nMapSizeX, nMapSizeY); //map이 존재하지 않을 때 map 형성.
	}
	else{//map의 사이즈가 다를 경우. 
		if(nMapSizeX != m_velocitypMap->getX() || nMapSizeY !=m_velocitypMap->getY())
		{
			delete m_velocitypMap;
			m_velocitypMap = new KuMap(nMapSizeX, nMapSizeY); 			
		}
	}	
	

	int **nMap=m_velocitypMap->getMap();

	for(int nX=0; nX< m_IplMapImage->width; nX++){
		for(int nY=0; nY<m_IplMapImage->height; nY++){//흰색
			if(m_IplMapImage->imageData[(nX+nY*m_IplMapImage->width)*3] == (char)255 && //Blue
			   m_IplMapImage->imageData[(nX+nY*m_IplMapImage->width)*3+1] == (char)255 && //Green
			   m_IplMapImage->imageData[(nX+nY*m_IplMapImage->width)*3+2] == (char)255	){ //Red 
			   	
			   	nMap[nX][m_IplMapImage->height-1-nY] = KuMap::NOMALVELOCITY;				
			   		
			}
			else if(m_IplMapImage->imageData[(nX+nY*m_IplMapImage->width)*3] == (char)0 && 
			   		m_IplMapImage->imageData[(nX+nY*m_IplMapImage->width)*3+1] == (char)0 &&
			   		m_IplMapImage->imageData[(nX+nY*m_IplMapImage->width)*3+2] == (char)255){//빨강
			   		
					nMap[nX][m_IplMapImage->height-1-nY] =  KuMap::FIRDECEL;
			}
			else if(m_IplMapImage->imageData[(nX+nY*m_IplMapImage->width)*3] == (char)0 && 
				    m_IplMapImage->imageData[(nX+nY*m_IplMapImage->width)*3+1] == (char)255 &&
				    m_IplMapImage->imageData[(nX+nY*m_IplMapImage->width)*3+2] == (char)255){//노랑

					nMap[nX][m_IplMapImage->height-1-nY] =  KuMap::SECDECEL ;
					
			}
			else if(m_IplMapImage->imageData[(nX+nY*m_IplMapImage->width)*3] == (char)0 && 
				    m_IplMapImage->imageData[(nX+nY*m_IplMapImage->width)*3+1] == (char)255 &&
				    m_IplMapImage->imageData[(nX+nY*m_IplMapImage->width)*3+2] == (char)0){//초록

					nMap[nX][m_IplMapImage->height-1-nY] =  KuMap::THIDECEL ;					
			}
			else if(m_IplMapImage->imageData[(nX+nY*m_IplMapImage->width)*3] == (char)255 && 
				    m_IplMapImage->imageData[(nX+nY*m_IplMapImage->width)*3+1] == (char)0 &&
				    m_IplMapImage->imageData[(nX+nY*m_IplMapImage->width)*3+2] == (char)0){//파랑

					nMap[nX][m_IplMapImage->height-1-nY] =  KuMap::FOUDECEL ;					
			}
			else if(m_IplMapImage->imageData[(nX+nY*m_IplMapImage->width)*3] == (char)255 && 
				    m_IplMapImage->imageData[(nX+nY*m_IplMapImage->width)*3+1] == (char)0 &&
				    m_IplMapImage->imageData[(nX+nY*m_IplMapImage->width)*3+2] == (char)255){//보라

					nMap[nX][m_IplMapImage->height-1-nY] =  KuMap::FIFDECEL ;					
			}
			else{
				nMap[nX][m_IplMapImage->height-1-nY] =  KuMap::UNKNOWN_AREA;				

			}
		}
	} 

 	cout<< "[CMapRepository] : Map loading success!!!" << endl;	
 	cvReleaseImage(&m_IplMapImage);
 
	return true;
}

int **kuMapRepository::getVelocityMap()
{
	return m_velocitypMap->getMap();
}

/**
 * @brief 확률 지도를 파일로 저장한다.
 * @date 2014/05/07
 * @param pMap
 * @return void
 */
void kuMapRepository::saveProbMap(KuMap*pMap, KuPose& poseRobot)
{
	double** pdMap = pMap->getProbMap();
	int nMapSizeX = pMap->getX();
	int nMapSizeY = pMap->getY();
	IplImage* IplMapImage = cvCreateImage(cvSize(nMapSizeX, nMapSizeY), IPL_DEPTH_8U, 3);

	for(int LoopX=0 ; LoopX < pMap->getX(); LoopX++)
	{
		for(int LoopY=0 ; LoopY < pMap->getY(); LoopY++)
		{
			IplMapImage->imageData[(LoopX+LoopY*pMap->getX())*3]=(char)((1 - pdMap[LoopX][pMap->getY()-1-LoopY]) * 255); //b
			IplMapImage->imageData[(LoopX+LoopY*pMap->getX())*3+1]=(char)((1 - pdMap[LoopX][pMap->getY()-1-LoopY]) * 255); //g
			IplMapImage->imageData[(LoopX+LoopY*pMap->getX())*3+2]=(char)((1 - pdMap[LoopX][pMap->getY()-1-LoopY]) * 255); //r
		}
	}

	cvSaveImage(KuRobotParameter::getInstance()->getTempMapPath().c_str(), IplMapImage);
	cvReleaseImage(&IplMapImage);

	m_poseRobotTemp = poseRobot;
}

/**
 @brief Korean: bmp파일의 확률 지도파일을 로드하여 지도정보를 저장하는 변수에 저장한다. 
 @brief English: write in English
*/
bool kuMapRepository::loadProbMap(string strMapFilePath, double** pdMap, KuPose& poseRobot)
{
	FILE	*infile;
 	infile = fopen(strMapFilePath.c_str() ,"rb");
 	if(infile==NULL) {
 		cout<< "[CMapRepository] : Error - Could not open map from "<<strMapFilePath<<endl;
		return false;
 	}
	fclose(infile);
	m_strMapFilePath =strMapFilePath;
	m_IplMapImage = cvLoadImage(strMapFilePath.c_str(),1);

	int nImageSizeX = m_IplMapImage->width;
	int nImageSizeY = m_IplMapImage->height;
	int nMapSizeX = KuRobotParameter::getInstance()->getMapSizeXm()*10;
	int nMapSizeY = KuRobotParameter::getInstance()->getMapSizeYm()*10;

	if(nMapSizeX < nImageSizeX)
	{
		nImageSizeX = nMapSizeX;
	}
	if(nMapSizeY < nImageSizeY)
	{
		nImageSizeY = nMapSizeY;
	}
	
/*
	if(NULL==m_smtpMap)
	{
		m_smtpMap = new KuMap(nMapSizeX, nMapSizeY); //map이 존재하지 않을 때 map 형성.
	}
	else{//map의 사이즈가 다를 경우. 
		if(nMapSizeX != m_smtpMap->getX() || nMapSizeY != m_smtpMap->getY())
		{
			delete m_smtpMap;
			m_smtpMap = new KuMap(nMapSizeX, nMapSizeY);
		}
	}	

	double** pdMap = m_smtpMap->getProbMap();
*/
	
	for(int nX = 0; nX < nImageSizeX; nX++)
	{
		for(int nY = 0; nY < nImageSizeY; nY++)
		{
			pdMap[nX][nImageSizeY - 1 - nY] = (double)((unsigned char)255 - (unsigned char)(m_IplMapImage->imageData[(nX + nY * m_IplMapImage->width) * 3])) / 255.;
		}
	} 

 	cout<< "[CMapRepository] : Probability grid map has been loaded successfully." << endl;
 	cvReleaseImage(&m_IplMapImage);
 	
/*
	KuPose RobotPos;
	RobotPos.setXm( (double)nMapSizeX/10 /2. );
	RobotPos.setYm( (double)nMapSizeY/10 /2. );
	KuDrawingInfo::getInstance()->setRobotPos(RobotPos); //지도를 기준 중앙위치를 로봇의 초기위치로 설정
*/
// 	KuDrawingInfo::getInstance()->setMap(pMap); //지도 정보 설정

	KuDrawingInfo::getInstance()->setRobotPos(m_poseRobotTemp); // 임시지도 저장 시점의 로봇 위치를 입력
	poseRobot = m_poseRobotTemp;

	return true;
}
