#pragma once

#define OS_WIN
//#define OS_LINUX

#define ANS_LOG_WRITE(msg) CANSLog::write(msg)
#define ANS_LOG_ERROR(msg, exit) CANSLog::error(msg, exit, __FILE__, __LINE__, __FUNCTION__) // __FUNCTION__ is not a standard C++ expression
#define ANS_LOG_ERROR(msg) CANSLog::error(msg, false, __FILE__, __LINE__, __FUNCTION__)

#define PI 3.141592
#define PI2 6.283184
#define R2D 57.295791
#define D2R 0.017453

#include "ANSLog.h"
#include "ANSPose.h"
//#include "ANSThread.h"
//#include "ANSImage.h"
//#include "ANSXML.h"
//#include "ANSTime.h"
#include "ANSPoint2D.h"
#include "ANSPoint3D.h"
#include "ANSArray2D.hpp"

class CANSCommon
{
public:
	// Constructor and destructor
	CANSCommon(void);
	~CANSCommon(void);

	// Variables

	// Functions
};

