#include "stdafx.h"
#include "ANSLog.h"
#include <time.h>
#include <sstream>
#include "ANSCommon.h"

CANSLog::CANSLog(void)
{
}


CANSLog::~CANSLog(void)
{
}


/**
 * @brief Write messages to a log file.
 * @date 2014/06/03
 * @param sContent
 * @return bool
 */
bool CANSLog::write(const string sContent)
{
	bool bRes(false);
	char chFileName[200];

	// Current time
	time_t now;
	time(&now); // Seconds since 1970

	struct tm timeCurrent;
	localtime_s(&timeCurrent, &now);

	// Create directory
#ifdef OS_WIN
	CreateDirectory(_T("d:/agv_log"), NULL);
#endif

	// Open
	ofstream log;

	strftime(chFileName, sizeof(chFileName), "d:/agv_log/log_%Y_%m_%d.txt", &timeCurrent); // %Y, %m, %d, %H, %M, %S

 	log.open(chFileName, ios::app);

	// Write
	if(log.is_open())
	{
		log << "[";

		if(timeCurrent.tm_hour < 10) // hour
		{
			log << "0" << timeCurrent.tm_hour;
		}
		else
		{
			log << timeCurrent.tm_hour;
		}

		log << ":";

		if(timeCurrent.tm_min < 10) // minute
		{
			log << "0" << timeCurrent.tm_min;
		}
		else
		{
			log << timeCurrent.tm_min;
		}

		log << ":";

		if(timeCurrent.tm_sec < 10) // second
		{
			log << "0" << timeCurrent.tm_sec;
		}
		else
		{
			log << timeCurrent.tm_sec;
		}

		log << "] ";
		log << sContent.c_str() << endl;

		log.close();
	}

	return bRes;
}


/**
 * @brief Write error messages to a log file.
 * @date 2014/06/03
 * @param sContent
 * @param bExit
 * @param sFile
 * @param nLine
 * @param sFunc
 * @return bool
 */
bool CANSLog::error(const string sContent, const bool bExit /*= false*/, const string sFile, const int nLine, const string sFunc)
{
	bool bRes(false);
	string strError;
	stringstream ss;

	ss << "Error: " << sContent << " (" << sFile << ", line " << nLine << ", function " << sFunc << ")";

	strError =  ss.str();

	write(strError);

	// Exit
	if(bExit)
	{
		write("Abnormal exit of the program.");
		exit(EXIT_FAILURE);
	}

	return bRes;
}
