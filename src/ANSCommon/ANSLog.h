#pragma once

#include <fstream>
#include <string>
#include <sstream>

using namespace std;

class CANSLog
{
public:
	// Constructor and destructor
	CANSLog(void);
	~CANSLog(void);

	// Variables

	// Functions
	static bool write(const string sContent);
	static bool error(const string sContent, const bool bExit /*= false*/, const string sFile, const int nLine, const string sFunc);

private:
	// Variables

	// Functions
};

