#pragma once

#include "../../ANSCommon/ANSCommon.h"
#include <vector>

using namespace std;

class CANSMotionController
{
public:
	// Structures
	typedef struct _tagMotionControlParameter
	{
		float obstacle_avoidance_range;
	} MotionControlParameter;

	typedef struct _tagControlCommand
	{
		float linear_velocity; // m/s
		float angular_velocity; // rad/s
	} ControlCommand;

	typedef struct _tagLaserScannerParameter
	{
		bool essential;
		bool enable;
		std::string model_name;
		std::string ip_address;
		unsigned int port;
		float max_distance;
		float pos_x;
		float pos_y;
		float pos_z;
		float pos_roll;
		float pos_pitch;
		float pos_yaw;
	} LaserScannerParameter;

	typedef struct _tagScanData
	{
		float* data;
		int amount;
		int frequency;
		float angle_step;
		float angle_start;
		float angle_stop;
	} ScanData;

	// Variables

	// Functcions
	CANSMotionController(void);
	~CANSMotionController(void);

	const CANSMotionController::ControlCommand control(CANSPose& robot_pose,
		CANSPoint2D& target_point,
		CANSMotionController::ScanData& laser_scan_front,
		const CANSMotionController::LaserScannerParameter& laser_param_front,
		const float linear_vel_desired,
		const float linear_vel_max,
		const float angular_vel_max,
		const float safety_region_length,
		const float safety_region_width,
		const float safety_region_offset_x);
	vector<CANSPose>* get_control_curve(void);

private:
	// Variables
	MotionControlParameter m_param;
	ControlCommand m_control_command;
	vector<CANSPose> m_control_curve; // control curve in robot coordinate system

	// Functions
	void load_parameters(const std::string sFileName);
	inline bool check_collision(vector<CANSPose>& control_curve,
		const CANSMotionController::ScanData& laser_scan_front,
		const CANSMotionController::LaserScannerParameter& laser_param_front,
		const float safety_region_length,
		const float safety_region_width,
		const float safety_region_offset);
	inline void generate_curve(vector<CANSPose>& control_curve,
		CANSPose& robot_pose,
		CANSPoint2D& target_point,
		const float linear_velocity,
		const float angular_velocity,
		const float curve_point_step);
	inline bool check_inside_rectangle(const float x, const float y,
		const float x1, const float y1,
		const float x2, const float y2,
		const float x3, const float y3,
		const float x4, const float y4, const float area_rect);
};

