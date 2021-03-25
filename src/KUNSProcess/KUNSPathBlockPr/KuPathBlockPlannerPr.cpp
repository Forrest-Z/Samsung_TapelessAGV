#include "stdafx.h"
#include "../../MultiRobotSupervisor/XmldataSetting.h"
#include "../../MultiRobotSupervisor/Serverpart.h"
#include "../../MultiRobotSupervisor/Clientpart.h"
#include "KuPathBlockPlannerPr.h"
#include "../../ANSCommon/ANSCommon.h"
#include "../../MultiRobotSupervisor/TotalTcpipCommunication.h"

KuPathBlockPlannerPr::KuPathBlockPlannerPr()
{
	m_vecPathBlockPos.reserve(1000);
	m_vecPathBlockPos.clear();
	m_vecTotalPathBlockPos.reserve(1000);
	m_vecTotalPathBlockPos.clear();
	m_WayPointPathBlock.TaskList.reserve(100);
	m_WayPointPathBlock.TaskList.clear();
	bTaskThreadflag=false;
	m_nCurrentIDX=-1;
	m_nNextIDX=-1;
	m_ncheckStep=2;
	m_bTerminateflag=false;
	m_bReversePathflag=false;
	m_bOtherAGVPathflag=false;
	m_bOnWaypoint = false;
	m_bOpeningDoor = false;
	m_nReturningBlockID = 0;
	m_nPreviousIDX = 0;
	m_nWayPointIDX = -1;
	m_nWayPointIDXforTasks = -1;
	m_bTaskRunning = false;
	m_bRunTaskThread = false;
	m_nJobNum = 0; // default

	m_Thread.start(doTaskThread, this, 100, "KuPathBlockPlannerPr::KuPathBlockPlannerPr()");
}

KuPathBlockPlannerPr::~KuPathBlockPlannerPr()
{
	m_vecGlobalPath.clear();
	m_Thread.terminate();
}

void KuPathBlockPlannerPr::initialize(vector<PBlock> vecPathBlockPos)
{
	bTaskThreadflag=false;
	m_bTerminateflag=false;
	m_bReversePathflag=false;
	m_nCurrentIDX=-1;
	m_nNextIDX=-1;
	m_vecTotalPathBlockPos.clear();
	for (int i=0; i<vecPathBlockPos.size();i++)
	{
		m_vecTotalPathBlockPos.push_back(vecPathBlockPos[i]);
	}
	m_bOtherAGVPathflag=false;

	// ISSAC
	m_nPreviousIDX = 0;
}
void KuPathBlockPlannerPr::setSequencePathblock(vector<PBlock> vecPathBlockPos, bool bFirstPath /*=false*/)
{
	bTaskThreadflag=false;
	m_bTerminateflag=false;
	m_nCurrentIDX=-1;
	PBlock last_block_in_previous_path;
/*
	if(!bFirstPath)
	{
		last_block_in_previous_path = m_vecPathBlockPos[m_vecPathBlockPos.size() - 1];

		m_vecPathBlockPos.clear();

		m_vecPathBlockPos.push_back(last_block_in_previous_path);
	}
	else
	{
		m_vecPathBlockPos.clear();
	}
*/
	m_vecPathBlockPos.clear();


	for (int i=0; i<vecPathBlockPos.size();i++)
	{
		m_vecPathBlockPos.push_back(vecPathBlockPos[i]);
	}
	m_bOtherAGVPathflag=false;
}

bool KuPathBlockPlannerPr::generatePathBlock(KuPose RobotPose)
{
	double dMinStartDist=DBL_MAX;
	int nStartPBlockID=-1;
	vector<PBlock> vectempTotalPathBlockPos;
	vector<PBlock> vectempPathBlockPos;
	KuPose PathPose;

	// ISSAC ///////////////////////////////////////////////////////////////////////////////////////////////////
	if(KuRobotParameter::getInstance()->getUsingISSAC() == 1) // ISSAC을 사용하는 경우
	{
		nStartPBlockID = 0; // 첫 번째 block이 항상 start

		//m_vecTotalPathBlockPos[m_vecTotalPathBlockPos.size() - 1].waypoint = PathBlock::GOAL; // 마지막 block이 항상 goal
	}
	// ISSAC ///////////////////////////////////////////////////////////////////////////////////////////////////

	for (int i=0; i<m_vecTotalPathBlockPos.size();i++)
	{
		double dStartDist=_hypot(m_vecTotalPathBlockPos[i].x-RobotPose.getXm(),m_vecTotalPathBlockPos[i].y-RobotPose.getYm());

		vectempTotalPathBlockPos.push_back(m_vecTotalPathBlockPos[i]);

	//	if(KuRobotParameter::getInstance()->getUsingISSAC() != 1) // ISSAC을 사용하지 않을 경우
	//	{
			if(vectempTotalPathBlockPos[i].waypoint==PathBlock::START)
			{
				nStartPBlockID = i;

				if(dMinStartDist > dStartDist) // 가장 가까운 start block을 start block으로 선택
				{
					dMinStartDist = dStartDist;
					nStartPBlockID = i;
				}
			}
	//	}
	}

	if(nStartPBlockID == -1) return false;

	PathPose.setXm(vectempTotalPathBlockPos[nStartPBlockID].x+vectempTotalPathBlockPos[nStartPBlockID].dist1.end_x);
	PathPose.setYm(vectempTotalPathBlockPos[nStartPBlockID].y+vectempTotalPathBlockPos[nStartPBlockID].dist1.end_y);

	if(KPBPr.generatePathBlockConnection(vectempTotalPathBlockPos,nStartPBlockID,&vectempPathBlockPos)) // 결과 값이 vectempPathBlockPos에 저장됨. nStartPBlockID부터 저장됨.
	{
		m_vecGlobalPath.clear();
		m_vecPathBlockPos.clear();
		m_vecPathBlockPos.push_back(vectempTotalPathBlockPos[nStartPBlockID]);

		vector<PBlock> vecTempBlock;
		for (int i=0; i<vectempPathBlockPos.size();i++)
		{
			m_vecPathBlockPos.push_back(vectempPathBlockPos[i]);
			vecTempBlock.push_back(vectempPathBlockPos[i]);
			if(vectempPathBlockPos[i].waypoint==1) // 1: waypoint
			{
				m_vecGlobalPath.push_back(vecTempBlock);
				vecTempBlock.clear();
				//vecTempBlock.push_back(vectempPathBlockPos[i]);
			}
		}

		m_vecGlobalPath.push_back(vecTempBlock);

		return true;
	}

	return false;
}

void  KuPathBlockPlannerPr::getPathBlockPos(vector<PBlock>*vecPathBlockPos)
{

	for(int i=0; i<m_vecPathBlockPos.size();i++)
	{
		(*vecPathBlockPos).push_back(m_vecPathBlockPos[i]);
	}

}

bool KuPathBlockPlannerPr::isOtherAGVPath()
{
	return m_bOtherAGVPathflag;
}

bool KuPathBlockPlannerPr::isEntryOtherAGVPath()
{
	if(m_nCurrentIDX + 1 < m_vecPathBlockPos.size())
	{
		if(m_vecPathBlockPos[m_nCurrentIDX].check && m_vecPathBlockPos[m_nCurrentIDX + 1].check)
		{
			return true;
		}
	}

	return false;
/*
	for(int i=m_nCurrentIDX; i<m_vecPathBlockPos.size();i++)
	{
		if(i==m_nCurrentIDX) continue;
		if(m_vecPathBlockPos[i].check)
		{
			return true;
		}
	}
	return false;
*/
}

bool KuPathBlockPlannerPr::getPassedStartingPoint(void)
{
	return m_bRobotPassedStartingPoint;
}

void KuPathBlockPlannerPr::setPassedStartingPoint(bool bPassed)
{
	m_bRobotPassedStartingPoint = bPassed;
}

KuPose KuPathBlockPlannerPr::getTargetPoseISSAC(KuPose RobotPose)
{
	KuPose TargetPos;
	m_RobotPose = RobotPose;

	// Update current block index (current block to last block)
	int nStartIDX;

	if(m_nCurrentIDX < 0)
	{
		nStartIDX = 0;
	}
	else
	{
		nStartIDX = m_nCurrentIDX;
	}

	// Waypoint index
	for(int i = nStartIDX; i < m_vecPathBlockPos.size(); i++)
	{
//		if((m_vecPathBlockPos[i].waypoint==PathBlock::WAYPOINT
//			||m_vecPathBlockPos[i].waypoint==PathBlock::START
//			||m_vecPathBlockPos[i].waypoint==PathBlock::GOAL)
//			&&m_vecPathBlockPos[i].task==-1)
		if(((m_vecPathBlockPos[i].waypoint > 0) || (i == m_vecPathBlockPos.size() - 1)) &&
			(m_vecPathBlockPos[i].task == -1)) // 아직 task가 완료되지 않은 경우
		{
			m_nWayPointIDX = i;
			break;
		}
		else
		{
			m_nWayPointIDX = -1;
		}
	}

	m_nCurrentIDX = get_nearest_block_idx(RobotPose,
											nStartIDX, // start index
											m_nWayPointIDX);//m_vecPathBlockPos.size() - 1); // end index

//	printf("Current block index = %d, m_nWayPointIDX = %d\n", m_nCurrentIDX, m_nWayPointIDX);

	if(m_nCurrentIDX >= 0)
	{
		if(m_vecPathBlockPos[m_nCurrentIDX].check)
		{
			m_bOtherAGVPathflag = true;
		}
		else
		{
			m_bOtherAGVPathflag = false;
		}
	}

	// Task
//	if(m_vecPathBlockPos[m_nCurrentIDX].waypoint > 0 && m_nCurrentIDX != -1)
	if(m_nCurrentIDX == m_nWayPointIDX && m_nCurrentIDX != -1)
	{
		m_WayPointPathBlock = m_vecPathBlockPos[m_nCurrentIDX];

		if(bTaskThreadflag == false)
		{
			m_CrtRobotPose = m_RobotPose;
			bTaskThreadflag = true;
			m_nWayPointIDXforTasks = m_nWayPointIDX;
			m_bTaskRunning = true;
			m_bRunTaskThread = true;
		}

		TargetPos.setXm(m_WayPointPathBlock.x);
		TargetPos.setYm(m_WayPointPathBlock.y);
		TargetPos.setID(1); // 정지?
	}
	else
	{
		int nTargetIdx = get_target_block_idx(m_nCurrentIDX, 2);

		TargetPos.setXm(m_vecPathBlockPos[nTargetIdx].x);
		TargetPos.setYm(m_vecPathBlockPos[nTargetIdx].y);
		TargetPos.setID(-1); // 이동?

		if(abs(m_nWayPointIDX - m_nCurrentIDX) <= 2 && m_nWayPointIDX != -1 && m_nCurrentIDX != -1)//waypoint에 거의(2칸) 도착했을 시
		{
			TargetPos.setXm(m_vecPathBlockPos[m_nWayPointIDX].x);
			TargetPos.setYm(m_vecPathBlockPos[m_nWayPointIDX].y);
			TargetPos.setID(-1);
		}

		if(m_nCurrentIDX == m_vecPathBlockPos.size() - 1)
		{
			TargetPos.setID(1); // 정지?
		}
	}
	
	// 문 개폐를 위해 현재 로봇이 waypoint 위에 있는지를 확인
	if(m_vecPathBlockPos[m_nCurrentIDX].waypoint > 0)
	{
		m_bOnWaypoint = true;
	}
	else
	{
		m_bOnWaypoint = false;
		setRobotOpeningDoor(false);
	}

	if(m_nCurrentIDX == m_vecPathBlockPos.size() - 1) // Path의 마지막 block에 도달할 경우
	{
		terminate();
	}

	return TargetPos;
}

bool KuPathBlockPlannerPr::isTaskRunning(void)
{
	return m_bTaskRunning;
}

int KuPathBlockPlannerPr::get_target_block_idx(int nCurrentBlockIdx, int nTargetOffset)
{
	int i;
	int nTargetIdx(0);
	int nTargetOffsetFinal(nTargetOffset);

	// Find returning point
	if(nTargetOffset > 1)
	{
		for(i = 0; i <= nTargetOffset; i++)
		{
			if(nCurrentBlockIdx + i + 2 < m_vecPathBlockPos.size())
			{
				if(m_vecPathBlockPos[nCurrentBlockIdx + i].block_id_for_ISSAC ==
					m_vecPathBlockPos[nCurrentBlockIdx + i + 2].block_id_for_ISSAC) // Same block id
				{
					m_nReturningBlockID = nCurrentBlockIdx + i + 1;
					nTargetOffsetFinal = i + 1;
					break;
				}
			}
		}
	}

	nTargetIdx = nCurrentBlockIdx + nTargetOffsetFinal;

	if(nTargetIdx >= m_vecPathBlockPos.size())
	{
		nTargetIdx = m_vecPathBlockPos.size() - 1; // last block
	}

	return nTargetIdx;
}

int KuPathBlockPlannerPr::get_nearest_block_idx(KuPose poseRobot, int nStartIdx, int nEndIdx)
{
	int i;
	int nNearestBlockNum(0);
	float fDist(0);
	float fMinDist(999999);

	if(nStartIdx < 0) nStartIdx = 0;

	for(i = nStartIdx; i <= nEndIdx; i++)
	{
		fDist = (float)hypot((double)(poseRobot.getXm() - m_vecPathBlockPos[i].x),
										(double)(poseRobot.getYm() - m_vecPathBlockPos[i].y));

		if(fDist < fMinDist - 0.0001)
		{
			fMinDist = fDist;
			nNearestBlockNum = i;
		}
	}

	return nNearestBlockNum;
}

KuPose KuPathBlockPlannerPr::getTargetPose(KuPose RobotPose)
{
	KuPose TargetPos;
	int CurrentIDX=-1;
	m_RobotPose=RobotPose;
	getCurrentPathBlock(m_vecPathBlockPos, RobotPose, &CurrentIDX);
	m_nCurrentIDX=CurrentIDX;

	// ISSAC
	if(CurrentIDX != m_vecPathBlockPos.size() - 1) // 마지막 block이 아닌 경우
	{
		m_bRobotPassedStartingPoint = true;
	}

	if(m_nCurrentIDX >= 0)
	{
		if(m_vecPathBlockPos[CurrentIDX].check)
		{
			m_bOtherAGVPathflag = true;
		}
		else
		{
			m_bOtherAGVPathflag = false;
		}
	}

	int nPathIDX=CurrentIDX+m_ncheckStep;
	
	if(nPathIDX > m_vecPathBlockPos.size() - 1) //pathblock의 마지막 검사
	{
		nPathIDX = m_vecPathBlockPos.size() - 1;
	}

	TargetPos.setXm(m_vecPathBlockPos[nPathIDX].x);
	TargetPos.setYm(m_vecPathBlockPos[nPathIDX].y);
	TargetPos.setID(-1);//

//	printf("TargetPos: %f %f %f\n", TargetPos.getXm(), TargetPos.getYm());

	for(int i = CurrentIDX; i < m_vecPathBlockPos.size();i++)
	{		
		if((m_vecPathBlockPos[i].waypoint==PathBlock::WAYPOINT
			||m_vecPathBlockPos[i].waypoint==PathBlock::START
			||m_vecPathBlockPos[i].waypoint==PathBlock::GOAL)
			&&m_vecPathBlockPos[i].task==-1)
		{
			m_nWayPointIDX=i;
			break;
		}
		else
		{
			m_nWayPointIDX=-1;
		}
	}

	if(m_nWayPointIDX == CurrentIDX && CurrentIDX != -1)//waypoint일때
	{
		m_WayPointPathBlock = m_vecPathBlockPos[m_nWayPointIDX];

		if(bTaskThreadflag == false)
		{
			m_CrtRobotPose = m_RobotPose;
			bTaskThreadflag = true;
			m_Thread.start(doTaskThread, this, 100, "KuPathBlockPlannerPr::getTargetPose()");
		}

		TargetPos.setXm(m_WayPointPathBlock.x);
		TargetPos.setYm(m_WayPointPathBlock.y);
		TargetPos.setID(1);

		if(m_vecPathBlockPos[m_nWayPointIDX].waypoint==PathBlock::GOAL)
		{
			//TargetPos.setID(-1);//0304
		}
	}
	else if(abs(m_nWayPointIDX-CurrentIDX)<2&&m_nWayPointIDX!=-1&&CurrentIDX!=-1)//waypoint에 거의(2칸) 도착했을 시
	{
		TargetPos.setXm(m_vecPathBlockPos[m_nWayPointIDX].x);
		TargetPos.setYm(m_vecPathBlockPos[m_nWayPointIDX].y);
		TargetPos.setID(-1);
	}
	
	// ISSAC ///////////////////////////////////////////////////////////
/*
	if(KuRobotParameter::getInstance()->getUsingISSAC() == 1) // 0: ISSAC 미사용, 1: ISSAC 사용
	{
		if(CurrentIDX >= 0)
		{
			if(m_vecPathBlockPos[CurrentIDX].waypoint==PathBlock::GOAL)
			{
				TargetPos.setXm(m_vecPathBlockPos[0].x);
				TargetPos.setYm(m_vecPathBlockPos[0].y);
				TargetPos.setID(1);
			}
		}
	}
*/
	// ISSAC ///////////////////////////////////////////////////////////

//	printf("TargetPos: %f %f %f\n\n", TargetPos.getXm(), TargetPos.getYm());

	// 문 개폐를 위해 현재 로봇이 waypoint 위에 있는지를 확인
	if(m_nWayPointIDX==CurrentIDX)
	{
		m_bOnWaypoint = true;
	}
	else
	{
		m_bOnWaypoint = false;
		setRobotOpeningDoor(false);
	}


	//TargetPos.setDist(_hypot(m_GoalPos.getX()-m_RobotPose.getX(),m_GoalPos.getY()-m_RobotPose.getY()));

	return TargetPos;
}

bool KuPathBlockPlannerPr::doRotation(int nState)
{
	double dAngleDiff = m_CrtRobotPose.getThetaRad() - m_RobotPose.getThetaRad();
	if (dAngleDiff >= M_PI){ dAngleDiff -= 2.0*M_PI; }			
	else if (dAngleDiff <= -M_PI){ dAngleDiff += 2.0*M_PI; }

	if(nState == PathBlock::MOTION_ROTATION_LEFT90)//nState/2==1&&nState%2==0)
	{
		if (dAngleDiff > -M_PI/2.0+M_PI*1/36) { 
			m_nTransVel = 0;
			m_nRotateVel = 21;
			return false;
		}	
	}
	else if(nState == PathBlock::MOTION_ROTATION_LEFT180)//nState/2==1&&nState%2==1)
	{
		if (dAngleDiff > -M_PI+M_PI*1/36) { 
			m_nTransVel = 0;
			m_nRotateVel = 21;
			return false;
		}			
	}
	else if(nState == PathBlock::MOTION_ROTATION_RIGHT90)//nState/2==2&&nState%2==0)
	{
		if (dAngleDiff < M_PI/2.0-M_PI*1/36) {
			m_nTransVel = 0;
			m_nRotateVel = -21;
			return false;
		}
	}	
	else if(nState == PathBlock::MOTION_ROTATION_RIGHT180)//nState/2==2&&nState%2==1)
	{
		if (dAngleDiff < M_PI-M_PI*1/36) {
			m_nTransVel = 0;
			m_nRotateVel = -21;
			return false;
		}
	}
	return true;
}
//123
bool KuPathBlockPlannerPr::doFinemove()
{
	for(int i=0; i<50; i++)
	{
		m_nTransVel = 100;
		m_nRotateVel = 0;
	}
	return true;
}
//123
void KuPathBlockPlannerPr::getMotionData(int *nTransVel,int *nRotateVel)
{
	(*nTransVel)=m_nTransVel;
	(*nRotateVel)=m_nRotateVel;

}
bool KuPathBlockPlannerPr::doMotion(int nMotion)
{
	switch(nMotion)
	{
	case PathBlock::MOTION_PAUSE:
		m_nTransVel = 0;
		m_nRotateVel = 0;
		return true;
		break;
	case PathBlock::MOTION_RESUME:
		m_nTransVel = 0;
		m_nRotateVel = 0;
		return true;
		break;
	case PathBlock::MOTION_ROTATION_LEFT90:
		if(doRotation(PathBlock::MOTION_ROTATION_LEFT90))
		{
			m_CrtRobotPose.setThetaRad(m_RobotPose.getThetaRad());
			m_nTransVel = 0;	m_nRotateVel = 0;
			return true;
		}
		else return false;
		break;
	case PathBlock::MOTION_ROTATION_LEFT180:
		if(doRotation(PathBlock::MOTION_ROTATION_LEFT180))
		{
			m_CrtRobotPose.setThetaRad(m_RobotPose.getThetaRad());
			m_nTransVel = 0;	m_nRotateVel = 0;
			return true;
		}
		else return false;
		break;
	case PathBlock::MOTION_ROTATION_RIGHT90:
		if(doRotation(PathBlock::MOTION_ROTATION_RIGHT90))
		{
			m_CrtRobotPose.setThetaRad(m_RobotPose.getThetaRad());
			m_nTransVel = 0;	m_nRotateVel = 0;
			return true;
		}
		else return false;
		break;
	case PathBlock::MOTION_ROTATION_RIGHT180:
		if(doRotation(PathBlock::MOTION_ROTATION_RIGHT180))
		{
			m_CrtRobotPose.setThetaRad(m_RobotPose.getThetaRad());
			m_nTransVel = 0;	m_nRotateVel = 0;
			return true;
		}
		else return false;
		break;
	default:
		return false;
		break;
	}

	return false;
}

bool KuPathBlockPlannerPr::doplayingSound(string strNameNPath,int  nIdx)
{
	char cFilePathName[100];	
	WCHAR wcharFilePathName[100];

	sprintf_s(cFilePathName,"%s/sound_%d.wav",strNameNPath.c_str(), nIdx-PathBlock::SOUND1+1);

	MultiByteToWideChar(0,0,cFilePathName,100,wcharFilePathName,100);
	LPCWSTR lpcwstrFilePathName=wcharFilePathName;
	PlaySound(lpcwstrFilePathName,NULL, SND_SYNC);

	return true;
}
bool KuPathBlockPlannerPr::doSound(int nSound)
{
	string strNameNPath = KuRobotParameter::getInstance()->getMovieNameNPath();	
	if((PathBlock::SOUND1 <= nSound) && (PathBlock::SOUND4 >= nSound))
	{
		doplayingSound(strNameNPath,nSound);		
		return true;
	}
	return false;
}
//123
bool KuPathBlockPlannerPr::doDevice(int nDevice)
{
	int nCount=0;

	switch(nDevice)
	{
	case PathBlock::DEVICE1: // 대차 분리
		m_nTransVel=0;m_nRotateVel=0; // 황서연(140409): 대차가 없을 경우 waypoint에서 정지하지 않고 앞으로 정지하는 것을 방지하기 위하여 추가.
		if(KuPRIMUSCommSupervisor::getInstance()->checkConnection())
		{
			m_nTransVel=0;m_nRotateVel=0;
			Sleep(1000);
			KuPRIMUSCommSupervisor::getInstance()->sendDisconnecTime();//이게 대차 커넥션 오프시키는거(푸는거)
			m_nTransVel=100;m_nRotateVel=0;		
		}
		for(int i=0; i<8; i++)
		{
			if(KuPRIMUSCommSupervisor::getInstance()->checkConnection())
			{
				Sleep(500);
			}
		}
		Sleep(2000);
		m_nTransVel=0;m_nRotateVel=0;	
		if(KuPRIMUSCommSupervisor::getInstance()->checkConnection()!=false)
		{
			return false;
		}
		Sleep(3000);
		return true;
		break;
	case PathBlock::DEVICE2: // 자동문 열기(SEVT)
// 		doFinemove();
// 		m_nTransVel=0;m_nRotateVel=0;
// 		Sleep(5000);
		m_nTransVel=0;m_nRotateVel=0;
		//Sleep(1000);
		for(int i=0; i<3; i++)
		{
			KuPRIMUSCommSupervisor::getInstance()->sendDoorOpen1();//door open 1
			Sleep(330);
		}
		return true;
		break;
	case PathBlock::DEVICE3: // 자동문 열기2(SEVT)
		m_nTransVel=0;m_nRotateVel=0;
		//Sleep(1000);
		for(int i=0; i<3; i++)
		{
			KuPRIMUSCommSupervisor::getInstance()->sendDoorOpen2();//door open 2
			Sleep(330);
		}
		return true;
		break;
	case PathBlock::DEVICE4:
		m_nTransVel=0;m_nRotateVel=0;	
		while(KuPRIMUSCommSupervisor::getInstance()->checkConnection()==false)
		{
			nCount++;
			if(KuPRIMUSCommSupervisor::getInstance()->checkConnection())
			{
				m_bReversePathflag = false;
				Sleep(1500);
				break;
			}
			else if(nCount>300)
			{
				m_bReversePathflag = true;
				break;
			}
			Sleep(100);
		}
		return true;
		break;
	case PathBlock::DEVICE5:
		return true;
		break;
	default:
		return false;
		break;
	}

	return false;
}
//123
//123
bool KuPathBlockPlannerPr::doReadytoSignal(int nReadytoSignal)
{
	SwitchInterface::getInstance()->setState(0);
	Sleep(20);
	if(PathBlock::READYTOSIGNAL == nReadytoSignal)
	{
		m_nTransVel=0;m_nRotateVel=0;

		int nSwitch(0);
		int nPath(0);

		for(int i = 0; i < XmldataSetting::getInstance()->getJobList().size(); i++)
		{
			if(i < 3)
			{
				if(XmldataSetting::getInstance()->getJobList()[i].job_id == 
					XmldataSetting::getInstance()->getCurrentJobID())
				{
					m_nJobNum = i;
					break;
				}
			}
		}

		SwitchInterface::getInstance()->initLampflag();

		while(nSwitch != 1)
		{
			m_nTransVel=0;m_nRotateVel=0;
/*
			if(nSwitch == 2)//call signal이 input
			{
				SwitchInterface::getInstance()->offLampflag();
				break;
			}// 마지막 지점 lamp
*/
			Sleep(100);

			nSwitch = SwitchInterface::getInstance()->getState();
		}

		if(nSwitch == 1)
		{
			bool bApproval(true);
/*
			if(TotalTcpipCommunication::getInstance()->isISSACConnected() == true) // ISSAC과 통신 중일 경우
			{
				if(SwitchInterface::getInstance()->getJobNum() >= XmldataSetting::getInstance()->getJobList().size())
				{
					return false;
				}

				// ISSAC에 경로 변경 가능한 지 확인 요청
				Clientpart::getInstance()->setJobEnableReq(true, SwitchInterface::getInstance()->getJobNum());

				// ISSAC에서 응답이 올 때까지 기다림
				int nJobChangeApproval(-1);

				while((nJobChangeApproval = Serverpart::getInstance()->getJobChangeApproval()) == -1)
				{
					Sleep(100);

					printf("[KuPathBlockPlannerPr] Waiting for job change approval.\n");
				}

				if(nJobChangeApproval == 0)
				{
					bApproval = false;
				}
				else if(nJobChangeApproval == 1)
				{
					bApproval = true;
				}
			}
*/
			// Path selection
			if(bApproval)
			{
				if(m_nJobNum != SwitchInterface::getInstance()->getJobNum() && m_nCurrentIDX == 0) // Job 0 ~ 2, 첫 번째 block에서만 path 변경
				{
					if(XmldataSetting::getInstance()->change_job_num(SwitchInterface::getInstance()->getJobNum()))
					{
						m_nJobNum = SwitchInterface::getInstance()->getJobNum();
						m_bTerminateflag = true; // 현재 gotogoal thread를 종료하고 새로 경로를 로드하여 실행
					
						// Sound
						stringstream ss;
						ss << "User pressed the switch (path " << SwitchInterface::getInstance()->getJobNum() << ")";
						ANS_LOG_WRITE(ss.str());

						SwitchInterface::getInstance()->offLampflag();

						playSound(SwitchInterface::getInstance()->getJobNum());

						return false;
					}
					else
					{
						playSoundFailure();

						return false;
					}
				}

				// ------ If job start approval is not used -------
				string sJobID = XmldataSetting::getInstance()->getCurrentJobID();
				string sJobIDPrev = XmldataSetting::getInstance()->getPrevJobID();
				if(sJobIDPrev != "")
				{
					Clientpart::getInstance()->RspStop(sJobIDPrev);
				}

				if(Clientpart::getInstance()->RspStart(sJobID))
				{
					if(XmldataSetting::getInstance()->selectJob(sJobID))
					{
						XmldataSetting::getInstance()->setJobStatus(XmldataSetting::JOB_RUN);

						printf("[AGV -> ISSAC] RSP_JOB_START\n");
					}
				}
				// ------
			}
			else
			{
				playSoundFailure();

				return false;
			}

			return true;
		}
		else if(nSwitch == 2)
		{
			SwitchInterface::getInstance()->offLampflag();	
			return true;
		}
	}

	return false;
}
//123
bool KuPathBlockPlannerPr::doTowerLamp(int nTowerLamp)
{
	if(PathBlock::TOWERLAMP==nTowerLamp)
	{
		Sleep(10000);
		return true;
	}

	return false;
}

void KuPathBlockPlannerPr::terminate()
{
	bTaskThreadflag = false;
	//m_Thread.terminate();
	m_bRunTaskThread = false;

	if(m_nCurrentIDX == m_vecPathBlockPos.size() - 1)
	{
		m_bTerminateflag = true;
	}
}

bool KuPathBlockPlannerPr::getTerminateState()
{
	return m_bTerminateflag;
}

void KuPathBlockPlannerPr::doTaskThread(void* arg)
{
	KuPathBlockPlannerPr* pKPBPr = (KuPathBlockPlannerPr*)arg;

	if(pKPBPr->m_bRunTaskThread == true)
	{
		bool bTerminate(true); // Thread 종료 여부를 결정

		for(int nTskListCnt = 0; nTskListCnt < pKPBPr->m_WayPointPathBlock.TaskList.size(); )
		{
			int nTask = pKPBPr->m_WayPointPathBlock.TaskList[nTskListCnt];

			if(pKPBPr->doReadytoSignal(nTask))
			{
				printf("Task complete: ReadytoSignal\n");
				nTskListCnt++;
			}
			if(pKPBPr->doMotion(nTask))
			{
				printf("Task complete: Motion\n");
				nTskListCnt++;
			}
			if(pKPBPr->doDevice(nTask))
			{
				printf("Task complete: Device\n");
				nTskListCnt++;

				// 문 개폐 task일 경우
				/**/
				if(nTask == PathBlock::DEVICE2 || nTask == PathBlock::DEVICE3)
				{
					if(pKPBPr->isRobotOnWaypoint()) // 로봇이 waypoint 위에 있는 경우
					{
						bTerminate = false;
						pKPBPr->setRobotOpeningDoor(true);
					}
					else
					{
						bTerminate = true;
						pKPBPr->setRobotOpeningDoor(false);
					}
				}
				/**/
			}
			if(pKPBPr->doTowerLamp(nTask))
			{
				printf("Task complete: TowerLamp\n");
				nTskListCnt++;
			}
			if(pKPBPr->doSound(nTask))
			{
				printf("Task complete: Sound\n");
				nTskListCnt++;
			}

			Sleep(50); // to prevent 100% cpu usage
		}

 		if(pKPBPr->m_nWayPointIDXforTasks!=-1)
 			pKPBPr->m_vecPathBlockPos[pKPBPr->m_nWayPointIDXforTasks].task=1;

		if(bTerminate)
		{
			pKPBPr->m_bTaskRunning = false;
			pKPBPr->terminate();
		}
	}
}

PBlock KuPathBlockPlannerPr::getCurrentPathBlock(vector<PBlock> vecPathBlockPos, KuPose RobotPose,int* IDX )
{
	double dRX=RobotPose.getXm();
	double dRY=RobotPose.getYm();
	double dMinDist=1000000000.0;
	PBlock PB;

	for(int i=0; i<vecPathBlockPos.size();i++)
	{
		double dPathX=vecPathBlockPos[i].x;
		double dPathY=vecPathBlockPos[i].y;

		double dDist=_hypot(dPathX-dRX,dPathY-dRY);

		if(dDist<dMinDist)
		{
			(*IDX)=i;
			dMinDist=dDist;
			PB=vecPathBlockPos[i];
		}
	}

	// ISSAC //////////////////////////////////////////////////////////////////////
	if(KuRobotParameter::getInstance()->getUsingISSAC() == 1) // 0: ISSAC 미사용, 1: ISSAC 사용
	{
		if((m_nPreviousIDX == vecPathBlockPos.size() - 2 &&
			*IDX == 0) ||
			m_nPreviousIDX == vecPathBlockPos.size() - 1)
		{
			*IDX = vecPathBlockPos.size() - 1; // 마지막 block
			PB = vecPathBlockPos[*IDX];
		}

		m_nPreviousIDX = *IDX;
	}
	// ISSAC //////////////////////////////////////////////////////////////////////

	return PB;
}

void KuPathBlockPlannerPr::initPreviousBlockIDX(void)
{
	m_nPreviousIDX = 0;
}

PBlock KuPathBlockPlannerPr::getCurrentISSACPathBlock(vector<PBlock> vecPathBlockPos, KuPose RobotPose,int* IDX)
{
	double dRX=RobotPose.getXm();
	double dRY=RobotPose.getYm();
	double dMinDist=1000000000.0;
	PBlock PB;

	for(int i=0; i<vecPathBlockPos.size();i++)
	{
		double dPathX=vecPathBlockPos[i].x;
		double dPathY=vecPathBlockPos[i].y;

		double dDist=_hypot(dPathX-dRX,dPathY-dRY);

		if(dDist<dMinDist)
		{
			(*IDX)=i;
			dMinDist=dDist;
			PB=vecPathBlockPos[i];
		}
	}

	return PB;
}


double KuPathBlockPlannerPr::getVelocityWeight()
{
	double dWeight=1.0;

	dWeight=dWeight*(6-m_vecPathBlockPos[m_nCurrentIDX].velocity)/6.0;

	return dWeight;
}

double KuPathBlockPlannerPr::detectObstacles( KuPose RobotPos,int_1DArray nLaserData181, int_1DArray nKinectLaserData,int_1DArray nSonarObsState,double dRotVel )
{
	double dWeight=1.0;
	double dKOffset= (double)KuRobotParameter::getInstance()->getKinectXOffset();
	double dLOffset= (double)KuRobotParameter::getInstance()->getFrontLaserXOffset();
	double dKinectMinDist=(double)KuRobotParameter::getInstance()->getKinectMinDist();
	double dLaserMinDist=(double)KuRobotParameter::getInstance()->getURG04LXLaserMinDist();

	//printf("obstacle detection in current block [%d]: %d\n", m_nCurrentIDX, m_vecPathBlockPos[m_nCurrentIDX].detectObstacle);
	if(m_vecPathBlockPos[m_nCurrentIDX].detectObstacle) // Block에 장애물 감지 옵션이 켜져있는 경우
	{
		for(int i=0; i<Sensor::URG04LX_DATA_NUM181;i++)//레이저 근접시 강제 정지
		{
			if(nLaserData181[i]>dLaserMinDist && nLaserData181[i]<350)
			{
				dWeight=0.0;
				return dWeight;	 
			}
		}
		/*
		for(int i=75; i<Sensor::URG04LX_DATA_NUM181 - 75;i++)
		{
			if(nLaserData181[i]>dLaserMinDist && nLaserData181[i]<700)
			{
				dWeight=0.0;
				return dWeight;	 
			}
		}
		*/

		for(int i=0; i<Sensor::URG04LX_DATA_NUM181;i++)//레이저 근접시 강제 정지
		{
			if(nLaserData181[i]>dLaserMinDist && nLaserData181[i]<1200)
			{
				if(abs(nLaserData181[i] * cos(i*M_PI/180)) < 300 && (nLaserData181[i] * sin(i*M_PI/180)) < 600)
				{
				dWeight=0.0;
				return dWeight;
				}
				if(abs(nLaserData181[i] * cos(i*M_PI/180)) < 150 && (nLaserData181[i] * sin(i*M_PI/180)) < 1000)
				{
				dWeight=0.0;
				return dWeight;
				}
				if(abs(nLaserData181[i] * cos(i*M_PI/180)) < 350 && (nLaserData181[i] * sin(i*M_PI/180)) < 450)
				{
				dWeight=0.0;
				return dWeight;
				}
				if(abs(nLaserData181[i] * cos(i*M_PI/180)) < 280 && (nLaserData181[i] * sin(i*M_PI/180)) < 800)
				{
				//printf("%s %d>\n", __FILE__, __LINE__);
				dWeight=0.4;
				return dWeight;
				}
				if(abs(nLaserData181[i] * cos(i*M_PI/180)) < 350 && (nLaserData181[i] * sin(i*M_PI/180)) < 600)
				{
					//printf("%s %d>\n", __FILE__, __LINE__);
				dWeight=0.4;
				return dWeight;
				}
				if(abs(nLaserData181[i] * cos(i*M_PI/180)) < 340 && (nLaserData181[i] * sin(i*M_PI/180)) < 1200)
				{
					//printf("%s %d>\n", __FILE__, __LINE__);
				dWeight=0.5;
				return dWeight;
				}
			}
		}

		for(int i=m_nCurrentIDX; i<m_nCurrentIDX+m_ncheckStep+1 && (m_nCurrentIDX + m_ncheckStep+1 < m_vecPathBlockPos.size()) ;i++)
		{
			if(m_vecPathBlockPos[i].detectObstacle)
			{
				double dBoundaryX1=m_vecPathBlockPos[i].x-m_vecPathBlockPos[i].sizex;
				double dBoundaryX2=m_vecPathBlockPos[i].x+m_vecPathBlockPos[i].sizex;
				double dBoundaryY1=m_vecPathBlockPos[i].y-m_vecPathBlockPos[i].sizey;
				double dBoundaryY2=m_vecPathBlockPos[i].y+m_vecPathBlockPos[i].sizey;
		
		/*		for(int nLaser=0; nLaser<Sensor::URG04LX_DATA_NUM181;nLaser++)
				{
					if(nLaserData181[nLaser]<dLaserMinDist) continue;
			
					double dAngleRad = (double)(nLaser -  Sensor::URG04LX_DATA_NUM181/2) * D2R;
					double dX = (RobotPos.getX() + ((double)nLaserData181[nLaser] * cos(dAngleRad) + dLOffset ) * cos(RobotPos.getThetaRad()) + 
						((double)nLaserData181[nLaser] * sin(dAngleRad) ) * sin(-RobotPos.getThetaRad()))/1000.0;
					double dY = (RobotPos.getY() + ((double)nLaserData181[nLaser] * cos(dAngleRad) + dLOffset ) * sin(RobotPos.getThetaRad()) + 
						((double)nLaserData181[nLaser] * sin(dAngleRad) ) * cos(RobotPos.getThetaRad()))/1000.0;

					if(dBoundaryX1<dX&&dBoundaryX2>dX&&dBoundaryY1<dY&&dBoundaryY2>dY) 
					{
						dWeight=0.0;
					}
					else if(dBoundaryX1>dX&&dBoundaryX2<dX&&dBoundaryY1>dY&&dBoundaryY2<dY)
					{
						dWeight=0.0;
					}
				}
		*/		
				for(int nKinect=0; nKinect<Sensor::KINECT_SENSOR_FOV;nKinect++)
				{
					if(nKinectLaserData[nKinect]<dKinectMinDist) continue;

					double dAngleRad = (double)(nKinect -  Sensor::KINECT_SENSOR_FOV/2) * D2R;
					double dX = (RobotPos.getX() + ((double)nKinectLaserData[nKinect] * cos(dAngleRad) + dKOffset ) * cos(RobotPos.getThetaRad()) + 
						((double)nKinectLaserData[nKinect] * sin(dAngleRad) ) * sin(-RobotPos.getThetaRad()))/1000.0;
					double dY = (RobotPos.getY() + ((double)nKinectLaserData[nKinect] * cos(dAngleRad) + dKOffset ) * sin(RobotPos.getThetaRad()) + 
						((double)nKinectLaserData[nKinect] * sin(dAngleRad) ) * cos(RobotPos.getThetaRad()))/1000.0;

					if(dBoundaryX1<dX&&dBoundaryX2>dX&&dBoundaryY1<dY&&dBoundaryY2>dY)
					{
						//dWeight=0.0;
						dWeight=0.35;//0.2;
						//printf("%s %d>\n", __FILE__, __LINE__);
					}
					else if(dBoundaryX1<dX&&dBoundaryX2>dX&&dBoundaryY1<dY&&dBoundaryY2>dY)
					{
						//dWeight=0.0;
						dWeight=0.35;//0.2;
						//printf("%s %d>\n", __FILE__, __LINE__);
					}
				}
			}
		}
	}
	else // Block에 장애물 감지 옵션이 꺼져있는 경우
	{
		//printf("do not detect obstacle@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
		for(int i=0; i<Sensor::URG04LX_DATA_NUM181;i++)//레이저 근접시 강제 정지
		{
			if(nLaserData181[i]>dLaserMinDist && nLaserData181[i]<300)
			{
				dWeight=0.0;
				return dWeight;
			}
		}
	}
	
/*
	string strData= KuRobotParameter::getInstance()->getdoSonar();

	int nState=0;

	for(int nSonar=0; nSonar<Sensor::SONAR_NUM;nSonar++)
	{
		if(nSonarObsState[nSonar]==Sensor::WARNING)
		{
			dWeight=0.5;
		}
	}


	if(dRotVel<0)
	{
		if(nSonarObsState[0]==Sensor::DANGER||nSonarObsState[1]==Sensor::DANGER)
		{
			dWeight=0.0;
		}
	}
	else if(dRotVel>0)
	{
		if(nSonarObsState[2]==Sensor::DANGER||nSonarObsState[3]==Sensor::DANGER)
		{
			dWeight=0.0;
		}
	}
*/

	return dWeight;
}

void KuPathBlockPlannerPr::setGlobalPath(vector<vector<PBlock>> vecGlobalPath)
{
	m_vecGlobalPath.clear();
	for(int i=0; i<vecGlobalPath.size(); i++)
	{
		m_vecGlobalPath.push_back(vecGlobalPath[i]);
	}	
}

void KuPathBlockPlannerPr::getGlobalPath(vector<vector<PBlock>>& vecGlobalPath)
{
	vecGlobalPath.clear();
	for(int i=0; i<m_vecGlobalPath.size(); i++)
	{
		vecGlobalPath.push_back(m_vecGlobalPath[i]);
	}
}

void KuPathBlockPlannerPr::setReversePathflag(bool bReversePathflag)
{
	m_bReversePathflag = bReversePathflag;
}

bool KuPathBlockPlannerPr::getReversePathflag()
{
	return m_bReversePathflag;
}

void KuPathBlockPlannerPr::setCurrentBlockIdx(int nIdx)
{
	m_nCurrentIDX=nIdx;
}

int KuPathBlockPlannerPr::getCurrentBlockIdx()
{
	return m_nCurrentIDX;
}


void KuPathBlockPlannerPr::setNextBlockIdx(int nIdx)//0516
{
	m_nNextIDX=nIdx;
}

int KuPathBlockPlannerPr::getNextBlockIdx()
{
	int nNextBlockID(m_nCurrentIDX + 1);	

	if(nNextBlockID >= m_vecPathBlockPos.size())
	{
		nNextBlockID = m_nCurrentIDX;
	}

	return nNextBlockID;
}

vector<PBlock>* KuPathBlockPlannerPr::getPathBlock(void)
{
	return &m_vecPathBlockPos;
}

bool KuPathBlockPlannerPr::isRobotOnWaypoint()
{
	return m_bOnWaypoint;
}

bool KuPathBlockPlannerPr::isRobotOpeningDoor()
{
	return m_bOpeningDoor;
}

void KuPathBlockPlannerPr::setRobotOpeningDoor(bool bOpening)
{
	m_bOpeningDoor = bOpening;
}

void KuPathBlockPlannerPr::playSound(int nPath)
{
	string strNameNPath;
	strNameNPath=KuRobotParameter::getInstance()->getMovieNameNPath();		 

	WCHAR wcharFilePathName[100];
	stringstream ss;
	ss << strNameNPath << "/path_selection_" << nPath << ".wav";

	MultiByteToWideChar(0,0,ss.str().c_str(),100,wcharFilePathName,100);
	LPCWSTR lpcwstrFilePathName=wcharFilePathName;
	PlaySound(lpcwstrFilePathName,NULL, SND_SYNC);
}

void KuPathBlockPlannerPr::playSoundFailure(void)
{
	string strNameNPath;
	strNameNPath=KuRobotParameter::getInstance()->getMovieNameNPath();		 

	WCHAR wcharFilePathName[100];
	stringstream ss;
	ss << strNameNPath << "/path_selection_failure.wav";

	MultiByteToWideChar(0,0,ss.str().c_str(),100,wcharFilePathName,100);
	LPCWSTR lpcwstrFilePathName=wcharFilePathName;
	PlaySound(lpcwstrFilePathName,NULL, SND_SYNC);
}