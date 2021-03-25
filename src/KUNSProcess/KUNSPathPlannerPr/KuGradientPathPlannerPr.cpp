#include "stdafx.h"
#include "KuGradientPathPlannerPr.h"
/**
@brief Korean: 생성자 함수
@brief English:
*/
KuGradientPathPlannerPr::KuGradientPathPlannerPr()
{
	m_listPath.clear(); //경로를 저장하는 STL 리스트 초기화.
	m_nMap = NULL;
	m_fIntCost = NULL;
	m_fAdjCost = NULL;
	m_fNavCost = NULL;
	m_finitIntCost=NULL;
		
 	initIntCost(CSPACE_OBSTACLE);
	m_nCSpaceObstacle=CSPACE_OBSTACLE;
	m_nCSpaceInfinity=CSPACE_OBSTACLE_INFINITY;
	m_dRadiusofRobot=300;
}
/**
@brief Korean: 소멸자 함수
@brief English:
*/
KuGradientPathPlannerPr::~KuGradientPathPlannerPr()
{
	
	if(m_nMap!=NULL)
	{
		for(int i = 0 ; i < m_nMapSizeX ; i++){
			delete[] m_nMap[i];
			delete[] m_fIntCost[i];
			delete[] m_fAdjCost[i];
			delete[] m_fNavCost[i];


			m_nMap[i] = NULL;
			m_fIntCost[i] = NULL;
			m_fAdjCost[i] = NULL;
			m_fNavCost[i] = NULL;

		}
		delete[] m_nMap;
		delete[] m_fIntCost;
		delete[] m_fAdjCost;
		delete[] m_fNavCost;

	}


	for(int i = 0 ; i < (m_nCSpaceObstacle*2)+1 ; i++){
		delete[] m_finitIntCost[i];
	}
	delete[] m_finitIntCost;

	cout << "[KuGradientPathPlannerPr] : Instance is destroyed." <<endl;
}
/**
@brief Korean: 로봇 반지름 정보를 받아오는 함수
@brief English:
*/
void KuGradientPathPlannerPr::setRadiusofRobotp(int nRadiusofRobot)
{
		m_dRadiusofRobot = (double) nRadiusofRobot;
}
/**
@brief Korean: 지도 정보를 받아오는 함수
@brief English:
*/
void KuGradientPathPlannerPr::setMap(KuMap* pMap)
{
		m_smtpMap = pMap;
}
/**
@brief Korean: 지도의 크기를 받아와서 지도를 초기화 함수
@brief English:
*/
void KuGradientPathPlannerPr::initializeMapSize(int nMapSizeX, int nMapSizeY)
{
	m_nMapSizeX=nMapSizeX;
	m_nMapSizeY=nMapSizeY;
	initialize();

}
/**
@brief Korean: 지도를 초기화하는 함수 
@brief English:
*/
void KuGradientPathPlannerPr::initialize()
{

	if(m_nMap==NULL){
		m_nMap = new int*[m_nMapSizeX];
		m_fIntCost = new float*[m_nMapSizeX];
		m_fAdjCost = new float*[m_nMapSizeX];
		m_fNavCost = new float*[m_nMapSizeX];

		if(m_nMap){
			for(int i = 0 ; i < m_nMapSizeX ; i++){
				m_nMap[i] = new int[m_nMapSizeY];
				m_fIntCost[i]  = new float[m_nMapSizeY];
				m_fAdjCost [i] = new float[m_nMapSizeY];
				m_fNavCost [i] = new float[m_nMapSizeY];
			}
		}
	}

}
/**
@brief Korean: 지도 초기화 함수
@brief English:
*/
void KuGradientPathPlannerPr::initMap()
{
	int** nMap = m_smtpMap->getMap();

	for(int i=0; i<m_smtpMap->getX(); i++){
		for(int j=0; j<m_smtpMap->getY(); j++){

			if(m_nMap[i][j] ==KuMap::WARNING_AREA)
				m_nMap[i][j] = KuMap::OCCUPIED_AREA;
			else 
				m_nMap[i][j] = nMap[i][j];

			m_fIntCost[i][j] = 0.;
			m_fAdjCost[i][j] = 0.;
			m_fNavCost[i][j] = INFINITY_VALUE;  
		}
	}

}
/**
@brief Korean: 지도와 시작지점 골지점을 이용해 경로를 생성하는 함수
@brief English:
*/
int KuGradientPathPlannerPr::generatePath(KuPose GoalPos, KuPose RobotPos)
{
	
	initMap();
	m_listPath.clear();

	int nStartPosition[2];
    int nGoalPosition[2];

	nStartPosition[0] = (int)( m_math.MM2AI(RobotPos.getX()) ); //mm형태를 배열 형태로 변화해준다
	nStartPosition[1] = (int)( m_math.MM2AI(RobotPos.getY()) );  //mm형태를 배열 형태로 변화해준다
	nGoalPosition[0] = (int)( m_math.MM2AI(GoalPos.getX()) ); //mm형태를 배열 형태로 변화해준다.;
	nGoalPosition[1] = (int)( m_math.MM2AI(GoalPos.getY()) ); //mm형태를 배열 형태로 변화해준다.; 

	if (nStartPosition[0]<1 || nStartPosition[0]> m_smtpMap->getX() || nStartPosition[1]< 1 || nStartPosition[1]> m_smtpMap->getY() 
		||nGoalPosition[0]<1 || nGoalPosition[0]> m_smtpMap->getX() || nGoalPosition[1]< 1 || nGoalPosition[1]> m_smtpMap->getY() )
	{
		return ALL_PATH_BLOCKED;
	}

	for (int i=-CSPACE_OBSTACLE_INFINITY; i<=CSPACE_OBSTACLE_INFINITY; i++) {
		for (int j=-CSPACE_OBSTACLE_INFINITY; j<=CSPACE_OBSTACLE_INFINITY; j++) {
			if (nStartPosition[0]+i<0 || nStartPosition[0]+i>= m_smtpMap->getX() ||nStartPosition[1]+ j< 0 || nStartPosition[1]+j>= m_smtpMap->getY()) continue;
			if (nGoalPosition[0]+i<0 || nGoalPosition[0]+i>= m_smtpMap->getX() ||nGoalPosition[1]+ j< 0 || nGoalPosition[1]+j>= m_smtpMap->getY()) continue;
			m_nMap[nStartPosition[0]+i][nStartPosition[1]+j] = 0;
			m_nMap[nGoalPosition[0]+i][nGoalPosition[1]+j] = 0;
		}
	}

	if (nStartPosition[0]<0 || nStartPosition[0]>= m_smtpMap->getX() ||nStartPosition[1]< 0 || nStartPosition[1]>= m_smtpMap->getY()) return false;
	if (nGoalPosition[0]<0 || nGoalPosition[0]>= m_smtpMap->getX() ||nGoalPosition[1]< 0 || nGoalPosition[1]>= m_smtpMap->getY()) return false;
	if (m_nMap[nStartPosition[0]][nStartPosition[1]]>0)
		return ROBOT_NEAR_OBSTACLE;
	if (m_nMap[nGoalPosition[0]][nGoalPosition[1]]>0)
		return GOAL_NEAR_OBSTACLE;

	// calculate cost--------------------------------------------------------------------------------------------------------------
	 calCastIntrinCost(m_nCSpaceObstacle);// set intrinsic cost
 	calAdjCost(nGoalPosition);// set adjacent cost
	calNavCost();// set navigation cost
	// calculate cost===============================================================================



	if( extractGradientPath(nStartPosition,nGoalPosition ) ){ //path 추출
		return CORRECT_PATH_PLANNED;
    }
    else{
		m_listPath.clear();
		return ALL_PATH_BLOCKED;
    }
}

/**
@brief Korean:  주행 비용함수를  계산하는  함수
@brief English:
*/
void KuGradientPathPlannerPr::calNavCost()
{
    for (int i=0; i<m_smtpMap->getX(); i++) {
        for (int j=0; j<m_smtpMap->getY(); j++) {
				m_fNavCost[i][j] = m_fIntCost[i][j] + m_fAdjCost[i][j];

        }
    }
}

/**
@brief Korean: 인접비용을 계산하는 함수
@brief English:
*/
void KuGradientPathPlannerPr::calAdjCost(int nGoalPosition[2])
{
           
        int **nActiveList;
        int **nNewActiveList;
        int nActiveListNo=0;
        int nNewActiveListNo = 0;

        // create sufficient number of variable for active list
		nActiveList = new int*[2];
		nNewActiveList= new int*[2];
		if(nActiveList){
			for(int i = 0 ; i < 2 ; i++){
				nActiveList[i] = new int[m_nMapSizeY*m_nMapSizeX];
				nNewActiveList[i]= new int[m_nMapSizeY*m_nMapSizeX];
			}
		}

        // first active list
        nActiveList[0][0] = nGoalPosition[0];
        nActiveList[1][0] = nGoalPosition[1];
        nActiveListNo = 1;

        // ------------------------------------------------------------------------------ //
        // repeat calculation until adjacent cost of all cell is calculated.
        bool bResult, bComplete;
        int nCurrent_i, nCurrent_j;
        float fCurrentValue;
        bComplete = false;

        while ( (nActiveListNo > 0) && (bComplete==false) ) {
                nNewActiveListNo = 0;
                for (int i=0; i<nActiveListNo; i++) {
                        nCurrent_i = nActiveList[0][i];
                        nCurrent_j = nActiveList[1][i];
                        fCurrentValue = m_fAdjCost[nCurrent_i][nCurrent_j];

                        // finish the operation when the current cell meets start point
                       //if ( abs(nCurrent_i-nStartPosition[0])<=1 && abs(nCurrent_j-nStartPosition[1])<=1 ) bComplete = true;
                        // need to test..... ex. Is this the optimal path when moving obstacle exists....

                        bResult = calAdjCostUp(nCurrent_i,nCurrent_j,fCurrentValue);
                        if (bResult) {
                                // register new active list if upper cell is updated.
                                nNewActiveList[0][nNewActiveListNo] = nCurrent_i-1;
                                nNewActiveList[1][nNewActiveListNo] = nCurrent_j;
                                nNewActiveListNo++;
                        }

                        bResult = calAdjCostDown(nCurrent_i,nCurrent_j,fCurrentValue);
                        if (bResult) {
                                // register new active list if lower cell is updated.
                                nNewActiveList[0][nNewActiveListNo] = nCurrent_i+1;
                                nNewActiveList[1][nNewActiveListNo] = nCurrent_j;
                                nNewActiveListNo++;
                        }

                        bResult = calAdjCostLeft(nCurrent_i,nCurrent_j,fCurrentValue);
                        if (bResult) {
                                // register new active list if left cell is updated.
                                nNewActiveList[0][nNewActiveListNo] = nCurrent_i;
                                nNewActiveList[1][nNewActiveListNo] = nCurrent_j-1;
                                nNewActiveListNo++;
                        }

                        bResult = calAdjCostRight(nCurrent_i,nCurrent_j,fCurrentValue);
                        if (bResult) {
                                // register new active list if right cell is updated.
                                nNewActiveList[0][nNewActiveListNo] = nCurrent_i;
                                nNewActiveList[1][nNewActiveListNo] = nCurrent_j+1;
                                nNewActiveListNo++;
                        }

                        bResult = calAdjCostUpRight(nCurrent_i,nCurrent_j,fCurrentValue);
                        if (bResult) {
                                // register new active list if upper cell is updated.
                                nNewActiveList[0][nNewActiveListNo] = nCurrent_i-1;
                                nNewActiveList[1][nNewActiveListNo] = nCurrent_j+1;
                                nNewActiveListNo++;
                        }

                        bResult = calAdjCostDownLeft(nCurrent_i,nCurrent_j,fCurrentValue);
                        if (bResult) {
                                // register new active list if lower cell is updated.
                                nNewActiveList[0][nNewActiveListNo] = nCurrent_i+1;
                                nNewActiveList[1][nNewActiveListNo] = nCurrent_j-1;
                                nNewActiveListNo++;
                        }

                        bResult = calAdjCostLeftUp(nCurrent_i,nCurrent_j,fCurrentValue);
                        if (bResult) {
                                // register new active list if left cell is updated.
                                nNewActiveList[0][nNewActiveListNo] = nCurrent_i-1;
                                nNewActiveList[1][nNewActiveListNo] = nCurrent_j-1;
                                nNewActiveListNo++;
                        }

                        bResult = calAdjCostRightDown(nCurrent_i,nCurrent_j,fCurrentValue);
                        if (bResult) {
                                // register new active list if right cell is updated.
                                nNewActiveList[0][nNewActiveListNo] = nCurrent_i+1;
                                nNewActiveList[1][nNewActiveListNo] = nCurrent_j+1;
                                nNewActiveListNo++;
                        }
                        
                }

                // copy new active list to current active list for next operation
                nActiveListNo = nNewActiveListNo;
                for(int j=0; j<nActiveListNo; j++) {
                        nActiveList[0][j] = nNewActiveList[0][j];
                        nActiveList[1][j] = nNewActiveList[1][j];
                }

                if (nActiveListNo > hypot(m_smtpMap->getX(),m_smtpMap->getY()) * 100 ) {
                        printf("Error!! Error!! Active list no. overflow error!!\n");
                        printf("Active list no = %d\n", nActiveListNo);

						for(int i = 0 ; i < 2 ; i++){
							delete[] nActiveList[i];
							delete[] nNewActiveList[i];
						}
						delete[] nActiveList;
						delete[] nNewActiveList;

						return;
                }
		
        }
        // ------------------------------------------------------------------------------ //
	
        // delete dynamic variable
      
		for(int i = 0 ; i < 2 ; i++){
			delete[] nActiveList[i];
			delete[] nNewActiveList[i];
		}
		delete[] nActiveList;
		delete[] nNewActiveList;


}
/**
@brief Korean: 현 위치로부터 위쪽영역의 인접비용을 계산하는 함수
@brief English:
*/
bool KuGradientPathPlannerPr::calAdjCostUp(int nI, int nJ, float fValue)
{
        // boundary condition
        if (nI<1 || nI>m_smtpMap->getX()-2 || nJ<1 || nJ>m_smtpMap->getY()-2) return false;
	 	if (m_fIntCost[nI-1][nJ] == INFINITY_VALUE){m_fAdjCost[nI-1][nJ] = INFINITY_VALUE; return false;}	
		if (m_smtpMap->getMap()[nI-1][nJ] == KuMap::UNKNOWN_AREA) return false;		
		if (m_smtpMap->getMap()[nI-1][nJ] == KuMap::OCCUPIED_AREA) return false;		
        // calculate adjcent cost of upper cell
        if ( (m_fAdjCost[nI-1][nJ] > (fValue+1.0)) || (m_fAdjCost[nI-1][nJ]==0) ) {
                m_fAdjCost[nI-1][nJ] = (fValue+1.0);
                return true;
        } else return false;
}
/**
@brief Korean: 현 위치로부터 아래쪽 영역의 인접비용을 계산하는 함수
@brief English:
*/
bool KuGradientPathPlannerPr::calAdjCostDown(int nI, int nJ, float fValue)
{
        // boundary condition
        if (nI<1 || nI>m_smtpMap->getX()-2 || nJ<1 || nJ>m_smtpMap->getY()-2) return false;
        if (m_fIntCost[nI+1][nJ] == INFINITY_VALUE){m_fAdjCost[nI+1][nJ] = INFINITY_VALUE; return false;}	
		if (m_smtpMap->getMap()[nI+1][nJ] == KuMap::UNKNOWN_AREA) return false;		
		if (m_smtpMap->getMap()[nI+1][nJ] == KuMap::OCCUPIED_AREA) return false;		
        // calculate adjcent cost of lower cell
        if ( (m_fAdjCost[nI+1][nJ] > (fValue+1.0)) || (m_fAdjCost[nI+1][nJ]==0) ) {
                m_fAdjCost[nI+1][nJ] = (fValue+1.0);
                return true;
        } else return false;
}
/**
@brief Korean: 현 위치로부터 왼쪽 영역의 인접비용을 계산하는 함수
@brief English:
*/
bool KuGradientPathPlannerPr::calAdjCostLeft(int nI, int nJ, float fValue)
{
        // boundary condition
        if (nI<1 || nI>m_smtpMap->getX()-2 || nJ<1 || nJ>m_smtpMap->getY()-2) return false;
        if (m_fIntCost[nI][nJ-1] == INFINITY_VALUE){m_fAdjCost[nI][nJ-1] = INFINITY_VALUE; return false;}	
		if (m_smtpMap->getMap()[nI][nJ-1] == KuMap::UNKNOWN_AREA) return false;	
		if (m_smtpMap->getMap()[nI][nJ-1] == KuMap::OCCUPIED_AREA) return false;		
        // calculate adjcent cost of left cell
        if ( (m_fAdjCost[nI][nJ-1] > (fValue+1.0)) || (m_fAdjCost[nI][nJ-1]==0) ) {
                m_fAdjCost[nI][nJ-1] = (fValue+1.0);
                return true;
        } else return false;
}
/**
@brief Korean: 현 위치로부터 오른쪽 영역의 인접비용을 계산하는 함수
@brief English:
*/
bool KuGradientPathPlannerPr::calAdjCostRight(int nI, int nJ, float fValue)
{
        // boundary condition
        if (nI<1 || nI>m_smtpMap->getX()-2 || nJ<1 || nJ>m_smtpMap->getY()-2) return false;
        if (m_fIntCost[nI][nJ+1] == INFINITY_VALUE) {m_fAdjCost[nI][nJ+1] = INFINITY_VALUE; return false;}	
		if (m_smtpMap->getMap()[nI][nJ+1] == KuMap::UNKNOWN_AREA) return false;	
		if (m_smtpMap->getMap()[nI][nJ+1] == KuMap::OCCUPIED_AREA) return false;		

        // calculate adjcent cost of right cell
        if ( (m_fAdjCost[nI][nJ+1] > (fValue+1.0)) || (m_fAdjCost[nI][nJ+1]==0) ) {
                m_fAdjCost[nI][nJ+1] = (fValue+1.0);
                return true;
        } else return false;
}
/**
@brief Korean: 현 위치로부터 위,오른쪽 영역의 인접비용을 계산하는 함수
@brief English:
*/
bool KuGradientPathPlannerPr::calAdjCostUpRight(int nI, int nJ, float fValue)
{
        // boundary condition
        if (nI<1 || nI>m_smtpMap->getX()-2 || nJ<1 || nJ>m_smtpMap->getY()-2) return false;
        if (m_fIntCost[nI-1][nJ+1] == INFINITY_VALUE){m_fAdjCost[nI-1][nJ+1] = INFINITY_VALUE; return false;}	
		if (m_smtpMap->getMap()[nI-1][nJ+1] == KuMap::UNKNOWN_AREA) return false;	
		if (m_smtpMap->getMap()[nI-1][nJ+1] == KuMap::OCCUPIED_AREA) return false;		
        // calculate adjcent cost of upper cell
        if ( (m_fAdjCost[nI-1][nJ+1] > (fValue+1.4)) || (m_fAdjCost[nI-1][nJ+1]==0) ) {
                m_fAdjCost[nI-1][nJ+1] = (fValue+1.4);
                return true;
        } else return false;
}
/**
@brief Korean: 현 위치로부터 아래,왼쪽 영역의 인접비용을 계산하는 함수
@brief English:
*/
bool KuGradientPathPlannerPr::calAdjCostDownLeft(int nI, int nJ, float fValue)
{
        // boundary condition
        if (nI<1 || nI>m_smtpMap->getX()-2 || nJ<1 || nJ>m_smtpMap->getY()-2) return false;
        if (m_fIntCost[nI+1][nJ-1] == INFINITY_VALUE){m_fAdjCost[nI+1][nJ-1] = INFINITY_VALUE; return false;}	
		if (m_smtpMap->getMap()[nI+1][nJ-1] == KuMap::UNKNOWN_AREA) return false;		
		if (m_smtpMap->getMap()[nI+1][nJ-1] == KuMap::OCCUPIED_AREA) return false;		
        // calculate adjcent cost of lower cell
        if ( (m_fAdjCost[nI+1][nJ-1] > (fValue+1.4)) || (m_fAdjCost[nI+1][nJ-1]==0) ) {
                m_fAdjCost[nI+1][nJ-1] = (fValue+1.4);
                return true;
        } else return false;
}
/**
@brief Korean: 현 위치로부터 위,왼쪽 영역의 인접비용을 계산하는 함수
@brief English:
*/
bool KuGradientPathPlannerPr::calAdjCostLeftUp(int nI, int nJ, float fValue)
{
        // boundary condition
        if (nI<1 || nI>m_smtpMap->getX()-2 || nJ<1 || nJ>m_smtpMap->getY()-2) return false;
        if (m_fIntCost[nI-1][nJ-1] == INFINITY_VALUE) {m_fAdjCost[nI-1][nJ-1] = INFINITY_VALUE; return false;}	
		if (m_smtpMap->getMap()[nI-1][nJ-1] == KuMap::UNKNOWN_AREA) return false;		
		if (m_smtpMap->getMap()[nI-1][nJ-1] == KuMap::OCCUPIED_AREA) return false;		
        // calculate adjcent cost of left cell
        if ( (m_fAdjCost[nI-1][nJ-1] > (fValue+1.4)) || (m_fAdjCost[nI-1][nJ-1]==0) ) {
                m_fAdjCost[nI-1][nJ-1] = (fValue+1.4);
                return true;
        } else return false;
}
/**
@brief Korean: 현 위치로부터 아래,오른쪽 영역의 인접비용을 계산하는 함수
@brief English:
*/
bool KuGradientPathPlannerPr::calAdjCostRightDown(int nI, int nJ, float fValue)
{
        // boundary condition
        if (nI<1 || nI>m_smtpMap->getX()-2 || nJ<1 || nJ>m_smtpMap->getY()-2) return false;
          if (m_fIntCost[nI+1][nJ+1] == INFINITY_VALUE){m_fAdjCost[nI+1][nJ+1]  = INFINITY_VALUE; return false;}	
		if (m_smtpMap->getMap()[nI+1][nJ+1] == KuMap::UNKNOWN_AREA) return false;		
		if (m_smtpMap->getMap()[nI+1][nJ+1] == KuMap::OCCUPIED_AREA) return false;		
        // calculate adjcent cost of right cell
        if ( (m_fAdjCost[nI+1][nJ+1] > (fValue+1.4)) || (m_fAdjCost[nI+1][nJ+1]==0) ) {
                m_fAdjCost[nI+1][nJ+1] = (fValue+1.4);
                return true;
        } else return false;
}

/**
@brief Korean: 경로를 넘겨주는 함수
@brief English:
*/
list<KuPose> KuGradientPathPlannerPr::getPath()
{
	return m_listPath;
	
}
/**
@brief Korean: 고유비용의 계산속도의 향상을 위하여 look-up table를 만들어주는 함수
@brief English:
*/
void KuGradientPathPlannerPr::initIntCost(int nRange )
{
	//장애물 비용을 포함시킨 초기계산값을 저장해 놓은 것이다.

	if(m_finitIntCost!=NULL)
	{
		for(int i = 0 ; i < (m_nCSpaceObstacle*2)+1 ; i++){
			delete[] m_finitIntCost[i];
		}
		delete[] m_finitIntCost;
	}
	m_nCSpaceInfinity=(int)(m_dRadiusofRobot/100.0+0.5);

	//초기 고유 비용의 메모리 할당----------------------------------------------------
	m_finitIntCost = new float*[(nRange*2)+1];
	if(m_finitIntCost){
		for(int i = 0 ; i < (nRange*2)+1 ; i++){
			m_finitIntCost[i] = new float[(nRange*2)+1];
		}
	}

	//==========================================================================
	
	//마스크 고유 비용--------------------------------------------------------------------------
	for(int i=0 ;i<2*nRange+1;i++) {	
		for(int j=0 ;j<2*nRange+1;j++) {	
			double ddistance=sqrt((double)(i-nRange)*(i-nRange)+(j-nRange)*(j-nRange));

			if(ddistance<=m_nCSpaceInfinity){
			m_finitIntCost[i][j] =INFINITY_VALUE;
			}
			else if(ddistance>nRange-1)
			{
				m_finitIntCost[i][j] =0;
			}
			else{
				m_finitIntCost[i][j] =CSPACE_OBSTACLE_ASSISTANCE*sqrt(1-pow(((ddistance-(double)m_nCSpaceInfinity)/(double)nRange),2));//타원 모델링	
			}
		}
	}
	//============================================================================
	m_nCSpaceObstacle=nRange;

}
/**
@brief Korean: 고유비용을 계산하는 함수
@brief English:
*/
void KuGradientPathPlannerPr::calCastIntrinCost(int nRange)
{  // intrinsic cost을 계산하는 함수이다. 
	for (int i=nRange ; i<m_smtpMap->getX()-nRange ; i++) {
	for (int j=nRange ; j<m_smtpMap->getY()-nRange ; j++){
		if ( m_nMap[i][j]== KuMap::OCCUPIED_AREA ){ 
				m_fIntCost[i][j] = INFINITY_VALUE;	
				for (int n=-nRange ; n<nRange+1 ; n++){
					for (int m=-nRange ; m<nRange+1 ; m++) 	{
						if(n+nRange>m_nMapSizeX||m+nRange>m_nMapSizeY||i+n>m_nMapSizeX||j+m>m_nMapSizeY||
							n+nRange<0||m+nRange<0||i+n<0||j+m<0){
								continue;
						}
						
					if(m_fIntCost[i+n][j+m]!= INFINITY_VALUE&& m_fIntCost[i+n][j+m] <m_finitIntCost[n+nRange][m+nRange]){m_fIntCost[i+n][j+m] =	m_finitIntCost[n+nRange][m+nRange] ;}
						
				}}} 
	}}
}
/**
@brief Korean: 주행비용으로부터 경로를 생성하는 함수
@brief English:
*/
bool KuGradientPathPlannerPr::extractGradientPath(int nStartPosition[2],int nGoalPosition[2])
{
	KuPose Path;
	list<KuPose>::iterator iteratorPath;
	int nGradientnum=-1;
	int nCount=0; 
	bool nFinish = false;
	bool bExtract;

	//로봇의 위치를 경로에 넣어준다.,------------------------------------------------------------------------
	Path.setX( m_math.AI2MM(nStartPosition[0]) ); 
	Path.setY( m_math.AI2MM(nStartPosition[1]) );
	m_listPath.push_back(Path); 
	iteratorPath = m_listPath.begin(); 
	int nX = (int)m_math.MM2AI(iteratorPath->getX());
	int nY = (int)m_math.MM2AI(iteratorPath->getY());
	//==========================================================================================

	//골주위에  주행비용을 낮춘다.-----------------------------------------------------------------------------
	for(int i=-CSPACE_OBSTACLE_INFINITY;i<CSPACE_OBSTACLE_INFINITY+1;i++){
		for(int j=-CSPACE_OBSTACLE_INFINITY;j<CSPACE_OBSTACLE_INFINITY+1;j++){
		 if(nGoalPosition[0]+i>m_nMapSizeX-3||nGoalPosition[0]+i<3||nGoalPosition[1]+j>m_nMapSizeY-3||nGoalPosition[1]+j<3){return false;}
			m_fNavCost[nGoalPosition[0]+i][nGoalPosition[1]+j] = -1;
		} 
	}
	//=========================================================================================

while ( (!nFinish) ) {
		//path의 마지막 지점을 받아 온다----------------------------------------------------------------------
		 nX = (int)m_math.MM2AI(iteratorPath->getX());
		 nY = (int)m_math.MM2AI(iteratorPath->getY());
		 //======================================================================================

		bExtract = false;

		if(nX>m_nMapSizeX-3||nX<3||nY>m_nMapSizeY-3||nY<3){return false;	}
		else{	nGradientnum=calGradient(nX,nY,nGradientnum);	}

	//select Gradient path ---------------------------------------------------------------------------------------------------------------------------------------------
		if(nGradientnum==0){
			Path.setX( m_math.AI2MM((nX- 1))  );
			Path.setY( m_math.AI2MM(nY-1));
			m_listPath.push_back(Path);
			iteratorPath++;
			bExtract = true;
		}
		else if(nGradientnum==1){
			Path.setX( m_math.AI2MM((nX- 1))  );
			Path.setY( m_math.AI2MM(nY-2));
			m_listPath.push_back(Path);
			iteratorPath++;
			bExtract = true;
		}
		else if(nGradientnum==2){
			Path.setX( m_math.AI2MM((nX))  );
			Path.setY( m_math.AI2MM(nY-1));
			m_listPath.push_back(Path);
			iteratorPath++;
			bExtract = true;
		}
		else if(nGradientnum==3){
			Path.setX( m_math.AI2MM((nX+1))  );
			Path.setY( m_math.AI2MM(nY-2));
			m_listPath.push_back(Path);
			iteratorPath++;
			bExtract = true;
		}
		else if(nGradientnum==4){
			Path.setX( m_math.AI2MM((nX+1))  );
			Path.setY( m_math.AI2MM(nY-1));
			m_listPath.push_back(Path);
			iteratorPath++;
			bExtract = true;
		}
		else if(nGradientnum==15){
			Path.setX( m_math.AI2MM((nX-2 ))  );
			Path.setY( m_math.AI2MM(nY-1));
			m_listPath.push_back(Path);
			iteratorPath++;
			bExtract = true;
		}
		else if(nGradientnum==5){
			Path.setX( m_math.AI2MM((nX+2))  );
			Path.setY( m_math.AI2MM(nY-1));
			m_listPath.push_back(Path);
			iteratorPath++;
			bExtract = true;
		}
		else if(nGradientnum==14){
			Path.setX( m_math.AI2MM((nX-1))  );
			Path.setY( m_math.AI2MM(nY));
			m_listPath.push_back(Path);
			iteratorPath++;
			bExtract = true;
		}
		else if(nGradientnum==6){
			Path.setX( m_math.AI2MM((nX+1 ))  );
			Path.setY( m_math.AI2MM(nY));
			m_listPath.push_back(Path);
			iteratorPath++;
			bExtract = true;
		}
		else if(nGradientnum==13){
			Path.setX( m_math.AI2MM((nX-2))  );
			Path.setY( m_math.AI2MM(nY+1));
			m_listPath.push_back(Path);
			iteratorPath++;
			bExtract = true;
		}
		else if(nGradientnum==7){
			Path.setX( m_math.AI2MM((nX+2))  );
			Path.setY( m_math.AI2MM(nY+1));
			m_listPath.push_back(Path);
			iteratorPath++;
			bExtract = true;
		}
		else if(nGradientnum==12){
			Path.setX( m_math.AI2MM((nX- 1))  );
			Path.setY( m_math.AI2MM(nY+1));
			m_listPath.push_back(Path);
			iteratorPath++;
			bExtract = true;
		}else if(nGradientnum==11){
			Path.setX( m_math.AI2MM((nX- 1))  );
			Path.setY( m_math.AI2MM(nY+2));
			m_listPath.push_back(Path);
			iteratorPath++;
			bExtract = true;
		}
		else if(nGradientnum==10){
			Path.setX( m_math.AI2MM((nX))  );
			Path.setY( m_math.AI2MM(nY+1));
			m_listPath.push_back(Path);
			iteratorPath++;
			bExtract = true;
		}
		else if(nGradientnum==9){
			Path.setX( m_math.AI2MM((nX+1))  );
			Path.setY( m_math.AI2MM(nY+2));
			m_listPath.push_back(Path);
			iteratorPath++;
			bExtract = true;
		}
		else if(nGradientnum==8){
			Path.setX( m_math.AI2MM((nX+1))  );
			Path.setY( m_math.AI2MM(nY+1));
			m_listPath.push_back(Path);
			iteratorPath++;
			bExtract = true;
		}
		//select Gradient path =======================================================================================

		if (bExtract==false) {	return false; } //fail extract path

		if ( abs(nX- nGoalPosition[0] )+	abs(nY - nGoalPosition[1] )<=10 ) {
			Path.setX( m_math.AI2MM((nGoalPosition[0] )) );
			Path.setY( m_math.AI2MM(nGoalPosition[1]));
			m_listPath.push_back(Path);
			return true; //success extract path
		}
			if(m_listPath.size()>PATH_MAXSIZE) {return false; }
	}

	return false; //fail extract path
}
/**
@brief Korean: 기울기 값을 이용하여 다음 좌표의 방향을 추출하는 함수
@brief English:
*/
int KuGradientPathPlannerPr::calGradient(int nX,int nY,int nGradientnum)
{

	double dCost[16]={0};
	double dMagf=INFINITY_VALUE;
	double dCMagf=INFINITY_VALUE;

	int nCriGradientnum=nGradientnum;
	int nCnum=-1;
	
	//일반적인 Gradient path----------------------------------------------------------------------------------

		dCost[0]=	(m_fNavCost[nX-1][nY-1]-m_fNavCost[nX][nY])/sqrt(2.);
		dCost[1]=	(m_fNavCost[nX-1][nY-2]-m_fNavCost[nX][nY])/sqrt(5.);
		dCost[2]=	(m_fNavCost[nX][nY-1]-m_fNavCost[nX][nY])/sqrt(1.);
		dCost[3]=	(m_fNavCost[nX+1][nY-2]-m_fNavCost[nX][nY])/sqrt(5.);
		dCost[4]=	(m_fNavCost[nX+1][nY-1]-m_fNavCost[nX][nY])/sqrt(2.);
		dCost[15]=(m_fNavCost[nX-2][nY-1]-m_fNavCost[nX][nY])/sqrt(5.);
		dCost[5]=	(m_fNavCost[nX+2][nY-1]-m_fNavCost[nX][nY])/sqrt(5.);
		dCost[14]=(m_fNavCost[nX-1][nY]-m_fNavCost[nX][nY])/sqrt(1.);
		dCost[6]=	(m_fNavCost[nX+1][nY]-m_fNavCost[nX][nY])/sqrt(1.);
		dCost[13]=(m_fNavCost[nX-2][nY+1]-m_fNavCost[nX][nY])/sqrt(5.);
		dCost[7]=	(m_fNavCost[nX+2][nY+1]-m_fNavCost[nX][nY])/sqrt(5.);
		dCost[12]=(m_fNavCost[nX-1][nY+1]-m_fNavCost[nX][nY])/sqrt(2.);
		dCost[11]=	(m_fNavCost[nX-1][nY+2]-m_fNavCost[nX][nY])/sqrt(5.);
		dCost[10]=(m_fNavCost[nX][nY+1]-m_fNavCost[nX][nY])/sqrt(1.);
		dCost[9]=	(m_fNavCost[nX+1][nY+2]-m_fNavCost[nX][nY])/sqrt(5.);
		dCost[8]=	(m_fNavCost[nX+1][nY+1]-m_fNavCost[nX][nY])/sqrt(2.);

		if(nGradientnum!=-1){
			int nCheckBack=(nCriGradientnum+nCriGradientnum+8)/2;
			if(nCheckBack>15){nCheckBack=nCheckBack-16+1;	}
			else{nCheckBack=nCheckBack+1;}
			for(int j=nCheckBack;j<nCheckBack+8;j++){
				if(j>15){ dCost[j-16]=1000;
				}
				else{dCost[j]=1000;
				}			
			}
		}

		//기울기가 최소가 되는 곳을 선택--------------------------------------------------------------------
		for(int i=0;i<16;i++){
			if(dMagf>dCost[i]){
				dMagf=dCost[i];
				nGradientnum=i;
			}
		}
		//=========================================================================================

	return  nGradientnum;
}