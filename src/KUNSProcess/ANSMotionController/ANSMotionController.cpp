#include "stdafx.h"
#include "ANSMotionController.h"

#define CURVE_POINT_STEP 0.2 // 20 cm
#define ANGULAR_VELOCITY_STEP 0.017 // 1 deg

CANSMotionController::CANSMotionController(void)
{
	load_parameters("./Parameters/ans.xml");

	// control curve
	const int num_points = (int)(m_param.obstacle_avoidance_range / CURVE_POINT_STEP + 1);

	m_control_curve.reserve(100);
	m_control_curve.clear();

	for(int i = 0; i < num_points; i++)
	{
		CANSPose curve_point;

		curve_point.set(CURVE_POINT_STEP * i, 0, 0);

		m_control_curve.push_back(curve_point);
	}
}


CANSMotionController::~CANSMotionController(void)
{
	m_control_curve.clear();
}


/**
 * @brief Load parameters from an XML file.
 * @date 2014/11/16
 * @param sFileName
 * @return void
 */
void CANSMotionController::load_parameters(const std::string sFileName)
{
	m_param.obstacle_avoidance_range = 4; // m

/*
	CANSXML xml_reader(sFileName);

	xml_reader.read("ans.motion_control.obstacle_avoidance.range", m_param.obstacle_avoidance_range);

	ANS_LOG_WRITE("[CANSMotionController] Successfully loaded motion control parameters.");
*/
}


/**
 * @brief Path tracking and obstacle avoidance.
 * @date 2014/11/20
 * @param robot_pose
 * @param target_point
 * @param laser_scan_front
 * @param laser_param_front
 * @param linear_velocity_desired
 * @param linear_velocity_max
 * @param angular_velocity_max
 * @param safety_region_length
 * @param safety_region_width
 * @param safety_region_offset_x
 * @return const CANSMotionController::ControlCommand
 */
const CANSMotionController::ControlCommand CANSMotionController::control(CANSPose& robot_pose,
																		 CANSPoint2D& target_point,
																		 CANSMotionController::ScanData& laser_scan_front,
																		 const CANSMotionController::LaserScannerParameter& laser_param_front,
																		 const float linear_vel_desired,
																		 const float linear_vel_max,
																		 const float angular_vel_max,
																		 const float safety_region_length,
																		 const float safety_region_width,
																		 const float safety_region_offset_x)
{
	vector<CANSPose> control_curve = m_control_curve;
	float angular_vel(0);
	float angular_vel_desired(0);
	const float angular_vel_coeff(angular_vel_max / (45 * D2R));
	float angle_to_target;
	bool controllable(false);

	// desired angular velocity
	angle_to_target = atan2(target_point.y - robot_pose.getYm(), target_point.x - robot_pose.getXm());

	angular_vel_desired = 0;//angular_vel_coeff * ANS_GET_ANGLE_DIFF(angle_to_target, robot_pose.getThRad());

	if(angular_vel_desired > fabs(angular_vel_max))
	{ angular_vel_desired = fabs(angular_vel_max); }
	if(angular_vel_desired < -fabs(angular_vel_max))
	{ angular_vel_desired = -fabs(angular_vel_max); }

	// generate curve
	for(float angular_velocity_increment = 0; angular_velocity_increment <= 2 * fabs(angular_vel_max); angular_velocity_increment += ANGULAR_VELOCITY_STEP)
	{
		// check CCW
		angular_vel = angular_vel_desired + angular_velocity_increment;

		if(angular_vel < fabs(angular_vel_max))
		{
			generate_curve(control_curve, robot_pose, target_point, linear_vel_desired, angular_vel, CURVE_POINT_STEP);
			if(!check_collision(control_curve,
								laser_scan_front, laser_param_front,
								safety_region_length, safety_region_width, safety_region_offset_x))
			{
				controllable = true;

				break;
			}
		}

		// check CW
		angular_vel = angular_vel_desired - angular_velocity_increment;

		if(angular_velocity_increment != 0)
		{
			if(angular_vel > -fabs(angular_vel_max))
			{
				generate_curve(control_curve, robot_pose, target_point, linear_vel_desired, angular_vel, CURVE_POINT_STEP);
				if(!check_collision(control_curve,
									laser_scan_front, laser_param_front,
									safety_region_length, safety_region_width, safety_region_offset_x))
				{
					controllable = true;

					break;
				}
			}
		}
	}

	if(controllable)
	{
		m_control_command.linear_velocity = linear_vel_desired;
		m_control_command.angular_velocity = angular_vel;
	}
	else // no available path
	{
		generate_curve(control_curve, robot_pose, target_point, linear_vel_desired, 0, 0);

		m_control_command.linear_velocity = 0;
		m_control_command.angular_velocity = 0;
	}

	m_control_curve = control_curve;

	return m_control_command;
}


/**
 * @brief Return a pointer of control curve.
 * @date 2014/11/16
 * @return vector<CANSPose>*
 */
vector<CANSPose>* CANSMotionController::get_control_curve(void)
{
	return &m_control_curve;
}


/**
 * @brief Calculate a collision-free path.
 * @date 2014/11/20
 * @param control_curve
 * @param robot_pose
 * @param target_point
 * @param linear_velocity
 * @param angular_velocity
 * @param curve_point_step
 * @return void
 */
inline void CANSMotionController::generate_curve(vector<CANSPose>& control_curve,
												 CANSPose& robot_pose,
												 CANSPoint2D& target_point,
												 const float linear_velocity,
												 const float angular_velocity,
												 const float curve_point_step)
{
	int i;
	const float delta_t(curve_point_step / linear_velocity);
	const float delta_th(angular_velocity * delta_t);
	float delta_x, delta_y;

	if(control_curve.size() > 0)
	{
		control_curve[0].set(0, 0, 0); // m, m, rad

		for(i = 1; i < control_curve.size(); i++)
		{
			delta_x = curve_point_step * cos(control_curve[i - 1].getThRad() + delta_th / 2);
			delta_y = curve_point_step * sin(control_curve[i - 1].getThRad() + delta_th / 2);

			control_curve[i].set(control_curve[i - 1].getXm() + delta_x,
									control_curve[i - 1].getYm() + delta_y,
									control_curve[i - 1].getThRad() + delta_th);
		}
	}
}


/**
 * @brief Check collision of the corresponding curve.
 * @date 2014/11/17
 * @param control_curve
 * @param front_laser_scan_data
 * @return void
 */
bool CANSMotionController::check_collision(vector<CANSPose>& control_curve,
										   const CANSMotionController::ScanData& laser_scan_front,
										   const CANSMotionController::LaserScannerParameter& laser_param_front,
										   const float safety_region_length,
										   const float safety_region_width,
										   const float safety_region_offset)
{
	if(laser_scan_front.data != 0)
	{
		// (x1, y1) -------- (x4, y4)
		//         |  Robot  |   Front
		// (x2, y2) -------- (x3, y3)
		const float safety_region_x1 = -safety_region_length / 2 + safety_region_offset;
		const float safety_region_y1 = safety_region_width / 2;
		const float safety_region_x2 = safety_region_x1;
		const float safety_region_y2 = -safety_region_y1;
		const float safety_region_x3 = safety_region_length / 2 + safety_region_offset;
		const float safety_region_y3 = safety_region_y2;
		const float safety_region_x4 = safety_region_x3;
		const float safety_region_y4 = safety_region_y1;

		// search all curve points
		for(int i = 0; i < control_curve.size(); i++)
		{
			// transformation of safety region corner points
			// based on the coordinate system of the first point of a curve
			const float cos_th = cos(control_curve[i].getThRad());
			const float sin_th = sin(control_curve[i].getThRad());
			const float x1 = safety_region_x1 * cos_th - safety_region_y1 * sin_th + control_curve[i].getXm();
			const float y1 = safety_region_x1 * sin_th + safety_region_y1 * cos_th + control_curve[i].getYm();
			const float x2 = safety_region_x2 * cos_th - safety_region_y2 * sin_th + control_curve[i].getXm();
			const float y2 = safety_region_x2 * sin_th + safety_region_y2 * cos_th + control_curve[i].getYm();
			const float x3 = safety_region_x3 * cos_th - safety_region_y3 * sin_th + control_curve[i].getXm();
			const float y3 = safety_region_x3 * sin_th + safety_region_y3 * cos_th + control_curve[i].getYm();
			const float x4 = safety_region_x4 * cos_th - safety_region_y4 * sin_th + control_curve[i].getXm();
			const float y4 = safety_region_x4 * sin_th + safety_region_y4 * cos_th + control_curve[i].getYm();

			// laser scan data
			for(int j = 0; j < laser_scan_front.amount; j++)
			{
				if(laser_scan_front.data[j] < m_param.obstacle_avoidance_range &&
					laser_scan_front.data[j] > 0.01)
				{
					if(laser_scan_front.angle_start + laser_scan_front.angle_step * j >= -PI / 2 &&
						laser_scan_front.angle_start + laser_scan_front.angle_step * j <= PI / 2) // -90 deg ~ 90 deg
					{
						float scan_th = laser_scan_front.angle_start + laser_scan_front.angle_step * j
										+ laser_param_front.pos_yaw;

						float scan_x = laser_scan_front.data[j] * (float)cos(scan_th) + laser_param_front.pos_x;
						float scan_y = laser_scan_front.data[j] * (float)sin(scan_th) + laser_param_front.pos_y;

						if(check_inside_rectangle(scan_x, scan_y, x1, y1, x2, y2, x3, y3, x4, y4, safety_region_length * safety_region_width))
						{
							return true; // collision exists
						}
					}
				}
			}
		}
	}

	return false; // no collision
}


/**
 * @brief Check whether a point lies inside a rectangle or not.
 * @date 2014/11/17
 * @param x1
 * @param y1
 * @param x2
 * @param y2
 * @param x3
 * @param y3
 * @param x4
 * @param y4
 * @return bool
 */
inline bool CANSMotionController::check_inside_rectangle(const float x, const float y,
														 const float x1, const float y1,
														 const float x2, const float y2,
														 const float x3, const float y3,
														 const float x4, const float y4,
														 const float area_rect)
{
	// triangle 1
	if(((x1 - x) * (y2 - y) - (y1 - y) * (x2 - x)) > area_rect) return false; // outside
	// triangle 2
	if(((x2 - x) * (y3 - y) - (y2 - y) * (x3 - x)) > area_rect) return false; // outside
	// triangle 3
	if(((x3 - x) * (y4 - y) - (y3 - y) * (x4 - x)) > area_rect) return false; // outside
	// triangle 4
	if(((x4 - x) * (y1 - y) - (y4 - y) * (x1 - x)) > area_rect) return false; // outside

	return true; // inside
}