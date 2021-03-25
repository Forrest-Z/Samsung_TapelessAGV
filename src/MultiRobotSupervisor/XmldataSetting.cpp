#include "stdafx.h"
#include "XmldataSetting.h"
#include "../ANSCommon/ANSCommon.h"

XmldataSetting::XmldataSetting()
{
	string sPathID = "";
	m_vecJobList.clear();
	m_vecJobList.reserve(100);

	// Load path ID from a file
	ifstream path_list_file;
	path_list_file.open("./data/path/path_list.txt");
	if(!path_list_file.is_open())
	{
		AfxMessageBox(_T("Could not find data/path/path_list.txt file."));
		exit(1);
	}
	else
	{
		while(!path_list_file.eof())
		{
			Job job;
			path_list_file >> job.job_id >> job.path_id >> job.job_status_id >> job.repeat_cnt >> job.complete_cnt;
			if(job.job_id != "")
			{
				m_vecJobList.push_back(job);
			}
		}

		// Log
/*
		stringstream ss;
		ss << "Path ID: " << sPathID;
		ANS_LOG_WRITE(ss.str());
*/
	}
	path_list_file.close();

	// Last job id
	m_nCurrentJobNum = 0;
	ifstream file_last_job("./data/ISSAC/last_job.txt");
	string sJobID = "";
	file_last_job >> sJobID;
	if(!selectJob(sJobID))
	{
		ANS_LOG_WRITE("[XmldataSetting] Unable to load an initial job id.");
	}

	if(file_last_job.is_open())
	{
		file_last_job.close();
	}

	// Etc
	m_strsendXml="";
	m_nJobStatus=JOB_DEFAULT;
	m_nMode=AGV_MODE;
	m_strJobID.clear();
	m_sPathID = "";
	m_nSendRepeatCounter=0;
// 	Job job_temp;
// 	job_temp.job_id = "0";
// 	job_temp.path_id = nPathID;
// 	m_vecJobList.push_back(job_temp);
	m_nPrevJobNum = -1;
	m_nNextJobNum = -1; // no next job
	m_nCompleteCount = 0;
}

XmldataSetting::~XmldataSetting()
{

}

bool XmldataSetting::selectJob(string sJobID)
{
	bool bRes(false);

	for(int i = 0; i < m_vecJobList.size(); i++)
	{
		if(m_vecJobList[i].job_id == sJobID)
		{
			m_nCurrentJobNum = i;
			bRes = true;

			setJobStatus(m_vecJobList[i].job_status_id);

			ofstream file_last_job("./data/ISSAC/last_job.txt", ios::out);

			file_last_job << m_vecJobList[i].job_id;

			if(file_last_job.is_open())
			{
				file_last_job.close();
			}

			break;
		}
	}

	return bRes;
}

void XmldataSetting::setPathBlock(vector<PBlock> vecPathBlcok)
{

}

/*
vector<PBlock> XmldataSetting::getPathBlock()
{

}*/

void XmldataSetting::setJobStatus(int job_status)
{
/*
	// 시스템이 예기치 못하게 종료되는 경우를 대비하여 job 정보를 파일로 저장
	ofstream jobfile;
	jobfile.open("./data/ISSAC/JobInfo.txt", ios::out);
	jobfile << job_status << endl;
//	jobfile << m_nCurrentJobNum << endl;
//	jobfile << m_vecJobList.size() - m_nCurrentJobNum << endl;
	jobfile << 1 << endl;

//	for(int i = m_nCurrentJobNum; i < m_vecJobList.size(); i++)
//	{
//		jobfile << m_vecJobList[i].job_id << " ";
//		jobfile << m_vecJobList[i].path_id << endl;
//	}


	jobfile << getCurrentJobID() << " ";
	jobfile << getCurrentPathNum() << endl;

	jobfile.close();

	printf("Saved job information to JobInfo.txt file.\n");
*/

	m_nJobStatus = job_status;
}

void XmldataSetting::setCompleteCount(int nCompCnt)
{
	// 수행한 cycle 횟수를 저장
/*
	ofstream cyclefile;
	cyclefile.open("./data/ISSAC/CycleInfo.txt", ios::out);
	cyclefile << nCompCnt << endl;
	cyclefile.close();

	printf("Saved cycle number to CycleInfo.txt file.\n");
*/
	
	if(m_nCurrentJobNum < m_vecJobList.size())
	{
		m_vecJobList[m_nCurrentJobNum].complete_cnt = nCompCnt;
	}
}

int XmldataSetting::getJobStatus(void)
{
	return m_nJobStatus;
}

int XmldataSetting::getCompleteCount(void)
{
	if(m_vecJobList.size() > 0)
	{
		return m_vecJobList[m_nCurrentJobNum].complete_cnt;
	}

	return 0;
}

bool XmldataSetting::loadISSACPath(int job_status, vector<PBlock>& vecPathBlock)
{
	if(job_status == JOB_RUN)
	{
		char cFilePathName[300];
		string strDataPath;int nIdx =1;
		sprintf(cFilePathName,"./data/path/LoadPathBlock%d.txt",nIdx);
		strDataPath=cFilePathName;//char 포인터값을 string에 대입
		KuPathBlockPr PB;
		vecPathBlock=PB.loadPathBlock(strDataPath);
		
		return true;
	}

	return false;
}

void XmldataSetting::setMode(int nMode)
{
	m_nMode=nMode;
}

int XmldataSetting::getMode()
{
	return m_nMode;
}

int XmldataSetting::getRepeatCount()
{
	if(m_vecJobList.size() > 0)
	{
		return m_vecJobList[m_nCurrentJobNum].repeat_cnt;
	}

	return 0;
}

void XmldataSetting::setSendRepeatCounter(int nCount)
{
	m_nSendRepeatCounter=nCount;
}

int XmldataSetting::getSendRepeatCounter()
{
	return m_nSendRepeatCounter;
}

void XmldataSetting::add_job_and_path(string sJobID, string sPathID, int nRepeatCnt, int nJobStatusID /*= JOB_DEFAULT*/, int nCompleteCnt /*= 0*/)
{
// 	m_strJobID=strJobID;
// 	m_nPathID=nPathID;

	// temporary ----------------
/*
	if(m_vecJobList.size() > 0)
	{
		m_vecJobList[0].job_id = sJobID;
		return;
	}
*/
	// temporary ----------------

	bool bDuplicated(false);

	for(int i = 0; i < m_vecJobList.size(); i++)
	{
		if(m_vecJobList[i].job_id == sJobID)
		{
			// Select a previously added job from the list
			bDuplicated = true;
			m_nNextJobNum = i;
			break;
		}
	}

	if(!bDuplicated)
	{
		// Add a new job to the list
		Job job_temp;

		job_temp.job_id = sJobID;
		job_temp.path_id = sPathID;
		job_temp.job_status_id = nJobStatusID;
		job_temp.repeat_cnt = nRepeatCnt;
		job_temp.complete_cnt = 0;

		m_vecJobList.push_back(job_temp);

		m_nNextJobNum = m_vecJobList.size() - 1;
	}
}

void XmldataSetting::clear_job_list(void)
{
	m_vecJobList.clear();
}

bool XmldataSetting::change_job_num(int nNum)
{
	if(nNum < m_vecJobList.size())
	{
		m_nPrevJobNum = m_nCurrentJobNum;
		m_nCurrentJobNum = nNum;

		ofstream file_last_job("./data/ISSAC/last_job.txt", ios::out);

		file_last_job << m_vecJobList[m_nCurrentJobNum].job_id;

		if(file_last_job.is_open())
		{
			file_last_job.close();
		}

		return true;
	}

	return false;
}

string XmldataSetting::getJobID_PathID(string& sJobID)
{
	if(m_vecJobList.size() > 0)
	{
		sJobID = m_vecJobList[m_nCurrentJobNum].job_id;

		return m_vecJobList[m_nCurrentJobNum].path_id;
	}

	sJobID = "";

	return "";
}

string XmldataSetting::getCurrentJobID(void)
{
	string sJobID = "";

	if(m_vecJobList.size() > m_nCurrentJobNum)
	{
		sJobID = m_vecJobList[m_nCurrentJobNum].job_id;
	}

	return sJobID;
}

string XmldataSetting::getCurrentPathID(void)
{
	if(m_vecJobList.size() > 0)
	{
		return m_vecJobList[m_nCurrentJobNum].path_id;
	}

	return "";
}

void XmldataSetting::selectNextJob(void)
{
	if(m_nNextJobNum != -1) // 다음 job이 있을 경우
	{
		m_nCurrentJobNum = m_nNextJobNum; // Job 변경

		m_nNextJobNum = -1; // 초기화(no next job)

		// Log
		if(m_nCurrentJobNum < m_vecJobList.size())
		{
			stringstream ss;
			ss << "Switched to a next job:  " << m_vecJobList[m_nCurrentJobNum].job_id << " (path: " << m_vecJobList[m_nCurrentJobNum].path_id << ")";
			ANS_LOG_WRITE(ss.str());
		}
	}
	else
	{
		printf("[XmldataSetting] No next job\n");
	}
/*
	if(m_nCurrentJobNum < m_vecJobList.size() - 1)
	{
		m_nCurrentJobNum++;
	}
*/
}

vector<XmldataSetting::Job>& XmldataSetting::getJobList(void)
{
	return m_vecJobList;
}

string XmldataSetting::getNextJobID(void)
{
	int nNextJobNum(m_nCurrentJobNum + 1);

	if(nNextJobNum > m_vecJobList.size() - 1)
	{
		nNextJobNum = m_vecJobList.size() - 1;
	}

	return m_vecJobList[nNextJobNum].job_id;
}

string XmldataSetting::getPrevJobID(void)
{
	if(m_nPrevJobNum != -1)
	{
		if(m_nPrevJobNum < m_vecJobList.size())
		{
			return m_vecJobList[m_nPrevJobNum].job_id;
		}
	}
/*
	if(m_vecJobList.size() > 1)
	{
		int nPrevJobNum(m_nCurrentJobNum - 1);

		if(nPrevJobNum < 0)
		{
			nPrevJobNum = 0;
		}

		return m_vecJobList[nPrevJobNum].job_id;
	}
*/
	return "";
}