#pragma once

#include "../../ANSCommon/ANSCommon.h"
#include <windows.h>
// #include <ole2.h>
#include <NuiApi.h>
#include "../../KUNSUtil/KUNSSingletone/KuSingletone.h"
#include "../../KUNSUtil/KUNSThread/KuThread.h"

class CKinect: public KuSingletone <CKinect>
{
public:
	// Structures
	typedef struct _tagKinectParameter
	{
		bool essential;
		bool enable;
		int depth_width;
		int depth_height;
		float pos_x;
		float pos_y;
		float pos_z;
		float pos_roll;
		float pos_pitch;
		float pos_yaw;
	} KinectParameter;

	// Constructor and destructor
	CKinect(void);
	~CKinect(void);

	// Variables

	// Functions
	bool init(void);
	bool receive_depth_frames(void);
	const CKinect::KinectParameter& get_parameters(void) const;
	void close_connection(void);
	CANSArray2D<CANSPoint3D>* get_depth_data(void);

private:
	// Variables
	bool m_bInitialized;
	KinectParameter m_param;
	INuiSensor* m_pNuiSensor;
	HANDLE m_hNextDepthFrameEvent;
	HANDLE m_pDepthStreamHandle;
	KuThread m_thread_kinect_depth; // Thread for receiving Kinect depth frames
	CANSArray2D<CANSPoint3D> m_depth_in_meters; // Depth information in meters
	NUI_IMAGE_RESOLUTION m_depth_resolution;

	// Functions
	void load_parameters(const std::string sFileName);
	static void thread_kinect_depth(void* arg); // Thread function for receiving Kinect depth frames
};

