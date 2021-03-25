#include <stdafx.h>
#include "Kinect.h"


CKinect::CKinect(void)
	: m_bInitialized(false)
	, m_hNextDepthFrameEvent(INVALID_HANDLE_VALUE)
	, m_pDepthStreamHandle(INVALID_HANDLE_VALUE)
{
	// Load parameters
	load_parameters("./Parameters/sensor.xml");

	m_depth_in_meters.create(m_param.depth_width, m_param.depth_height);
}


CKinect::~CKinect(void)
{
	close_connection();
}


/**
 * @brief Read parameters from a XML file.
 * @date 2014/11/12
 * @param sFileName
 * @return void
 */
void CKinect::load_parameters(const std::string sFileName)
{
	m_param.essential = true;
	m_param.enable = true;
	m_param.depth_width = 320;
	m_param.depth_height = 240;
	m_param.pos_x = 0;
	m_param.pos_y = 0;
	m_param.pos_z = 0;
	m_param.pos_roll = 0;
	m_param.pos_pitch = 0;
	m_param.pos_yaw = 0;

/*
	CANSXML xml_reader(sFileName);

	std::string sParam;

	xml_reader.read("sensor.kinect.essential", sParam);
	m_param.essential = (sParam == "yes" || sParam == "Yes" || sParam == "YES") ? true : false;
	xml_reader.read("sensor.kinect.enable", sParam);
	m_param.enable = (sParam == "yes" || sParam == "Yes" || sParam == "YES") ? true : false;
	xml_reader.read("sensor.kinect.resolution.width", m_param.depth_width);
	xml_reader.read("sensor.kinect.resolution.height", m_param.depth_height);
	xml_reader.read("sensor.kinect.position.x", m_param.pos_x);
	xml_reader.read("sensor.kinect.position.y", m_param.pos_y);
	xml_reader.read("sensor.kinect.position.z", m_param.pos_z);
	xml_reader.read("sensor.kinect.position.roll", m_param.pos_roll);
	xml_reader.read("sensor.kinect.position.pitch", m_param.pos_pitch);
	xml_reader.read("sensor.kinect.position.yaw", m_param.pos_yaw);

	if(m_param.enable == false) m_param.essential = false;

	ANS_LOG_WRITE("[Kinect] Successfully loaded kinect parameters.");
*/
}


/**
 * @brief Initialize a Kinect sensor.
 * @date 2014/11/12
 * @return bool
 */
bool CKinect::init(void)
{
	INuiSensor * pNuiSensor;
	int nSensorCount(0);
	HRESULT hr;

	hr = NuiGetSensorCount(&nSensorCount);

	if(FAILED(hr) ) { return false; }

	if(nSensorCount > 0)
	{
		// Create the sensor so we can check status, if we can't create it, return false
		hr = NuiCreateSensorByIndex(0, &pNuiSensor);

		if (FAILED(hr))
		{
			return false;
		}

		// Get the status of the sensor, and if connected, then we can initialize it
		hr = pNuiSensor->NuiStatus();

		if (hr == S_OK)
		{
			m_pNuiSensor = pNuiSensor;

			// Initialize the Kinect and specify that we'll be using depth
			hr = m_pNuiSensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH); //NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX); 
			if(FAILED(hr)) { return false; }

			// Create an event that will be signaled when depth data is available
			m_hNextDepthFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);

			// Open a depth image stream to receive depth frames
			if(m_param.depth_width == 80 && m_param.depth_height == 60)
			{
				m_depth_resolution = NUI_IMAGE_RESOLUTION_80x60;
			}
			else if(m_param.depth_width == 320 && m_param.depth_height == 240)
			{
				m_depth_resolution = NUI_IMAGE_RESOLUTION_320x240;
			}
			else if(m_param.depth_width == 640 && m_param.depth_height == 480)
			{
				m_depth_resolution = NUI_IMAGE_RESOLUTION_640x480;
			}
			else
			{
				ANS_LOG_ERROR("Invalid depth data resolution. Please check sensor.xml file.", true);
			}

			hr = m_pNuiSensor->NuiImageStreamOpen(
				NUI_IMAGE_TYPE_DEPTH, //NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX, // Depth camera or rgb camera?
				m_depth_resolution, // Image resolution
				0, // Image stream flags, e.g. near mode
				2, // Number of frames to buffer
				m_hNextDepthFrameEvent, // Event handle
				&m_pDepthStreamHandle);

			if(FAILED(hr)) { return false; }

			// Create a thread
			m_thread_kinect_depth.start(thread_kinect_depth, this, 30, "CKinect::init()");

			m_bInitialized = true;

			return true;
		}
		else
		{
			pNuiSensor->Release(); // This sensor wasn't OK, so release it since we're not using it
			return false;
		}
	}

	return false;
}


/**
 * @brief Thread function for receiving Kinect depth frames
 * @date 2014/11/12
 * @param arg
 * @return void
 */
void CKinect::thread_kinect_depth(void* arg)
{
	CKinect* pKinect = (CKinect*)arg;

	pKinect->receive_depth_frames();
}


/**
 * @brief Receive Kinect depth frames
 * @date 2014/11/12
 * @return void
 */
bool CKinect::receive_depth_frames(void)
{
	if(m_bInitialized)
	{
		if(WaitForSingleObject(m_hNextDepthFrameEvent, 0) == WAIT_OBJECT_0)
		{
			NUI_IMAGE_FRAME imageFrame;

			HRESULT hr = m_pNuiSensor->NuiImageStreamGetNextFrame(m_pDepthStreamHandle, 0, &imageFrame);
			if(FAILED(hr)) { return false; }

			INuiFrameTexture* pFrameTexture;
			hr = m_pNuiSensor->NuiImageFrameGetDepthImagePixelFrameTexture(m_pDepthStreamHandle, &imageFrame, nullptr, &pFrameTexture);
			if(FAILED(hr)) { return false; }

			NUI_LOCKED_RECT LockedRect;
			hr = pFrameTexture->LockRect(0, &LockedRect, NULL, 0);
			if(FAILED(hr)) { return false; }

			NUI_DEPTH_IMAGE_PIXEL* pBufferRun = reinterpret_cast<NUI_DEPTH_IMAGE_PIXEL *>(LockedRect.pBits);
			NUI_DEPTH_IMAGE_PIXEL* pBufferRunEnd = pBufferRun + LockedRect.size/sizeof(NUI_DEPTH_IMAGE_PIXEL);

			int nu(0), nv(0);
			const USHORT minDepth(1);
			const USHORT maxDepth(5000);
			const float fMultiplier(10);

			while(pBufferRun != pBufferRunEnd)
			{
				NUI_DEPTH_IMAGE_PIXEL depthPixel = *pBufferRun++;
				Vector4 depth_in_homogeneous_coord;
				CANSPoint3D depth_in_meters;

				if(depthPixel.depth <= maxDepth && depthPixel.depth >= minDepth)
				{
					USHORT depth = depthPixel.depth;
				//	USHORT player = depthPixel.playerIndex;

					depth_in_homogeneous_coord = NuiTransformDepthImageToSkeleton((LONG)nu, (LONG)nv, depth, m_depth_resolution);

					if(depth_in_homogeneous_coord.w != 1)
					{
						depth_in_meters.x = depth_in_homogeneous_coord.x / depth_in_homogeneous_coord.w * fMultiplier;
						depth_in_meters.y = depth_in_homogeneous_coord.y / depth_in_homogeneous_coord.w * fMultiplier;
						depth_in_meters.z = depth_in_homogeneous_coord.z / depth_in_homogeneous_coord.w * fMultiplier;
					}
					else
					{
						depth_in_meters.x = depth_in_homogeneous_coord.x * fMultiplier;
						depth_in_meters.y = depth_in_homogeneous_coord.y * fMultiplier;
						depth_in_meters.z = depth_in_homogeneous_coord.z * fMultiplier;
					}

					m_depth_in_meters.set(nu, nv, depth_in_meters);
				}
				else // inaccurate data
				{
					depth_in_meters.x = 0;
					depth_in_meters.y = 0;
					depth_in_meters.z = 0;

					m_depth_in_meters.set(nu, nv, depth_in_meters);
				}

				if(nu < m_param.depth_width - 1)
				{
					nu++;
				}
				else
				{
					nu = 0;
					nv++;
				}
			}

			hr = pFrameTexture->UnlockRect(0);
			if(FAILED(hr)) { return false; };
				
			hr = m_pNuiSensor->NuiImageStreamReleaseFrame(m_pDepthStreamHandle, &imageFrame);

			return true;
		}
	}

	return false;
}


/**
 * @brief Return Kinect parameters.
 * @date 2014/11/12
 * @return CKinect::KinectParameter
 */
const CKinect::KinectParameter& CKinect::get_parameters(void) const
{
	return m_param;
}


/**
 * @brief Close connection.
 * @date 2014/11/12
 * @return void
 */
void CKinect::close_connection(void)
{
	m_thread_kinect_depth.terminate();

	if(m_pNuiSensor)
	{
		m_pNuiSensor->NuiShutdown();
		m_pNuiSensor->Release();
		m_pNuiSensor = 0;
	}

	if(m_bInitialized)
	{
		CloseHandle(m_hNextDepthFrameEvent);
	}
	
	m_bInitialized = false;
}


/**
 * @brief Return depth data.
 * @date 2014/11/12
 * @return CANSArray2D<CANSPoint3D>*
 */
CANSArray2D<CANSPoint3D>* CKinect::get_depth_data(void)
{
	return &m_depth_in_meters;
}