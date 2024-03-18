#pragma once
#include <stdint.h>

namespace krabi_msgs
{
	struct motors_cmd
	{
		typedef bool _enable_motors_type;
		_enable_motors_type enable_motors;
		typedef bool _override_PWM_type;
		_override_PWM_type override_PWM;
		typedef int32_t _PWM_override_left_type;
		_PWM_override_left_type PWM_override_left;
		typedef int32_t _PWM_override_right_type;
		_PWM_override_right_type PWM_override_right;
		typedef bool _reset_encoders_type;
		_reset_encoders_type reset_encoders;
	};

	struct motors_parameters
	{
		typedef float _max_current_left_type;
		_max_current_left_type max_current_left;
		typedef float _max_current_right_type;
		_max_current_right_type max_current_right;
		typedef float _max_current_type;
		_max_current_type max_current;
	};

	struct encoders
	{
	    typedef int32_t _encoder_right_type;
	    _encoder_right_type encoder_right;
	    typedef int32_t _encoder_left_type;
	    _encoder_left_type encoder_left;
	};

	struct motors
	{
		typedef krabi_msgs::encoders _encoders_type;
		_encoders_type encoders;
		typedef uint16_t _current_left_type;
		_current_left_type current_left;
		typedef uint16_t _current_right_type;
		_current_right_type current_right;
		typedef uint32_t _current_left_accumulated_type;
		_current_left_accumulated_type current_left_accumulated;
		typedef uint32_t _current_right_accumulated_type;
		_current_right_accumulated_type current_right_accumulated;
	};

	struct odom_lighter
	{
		typedef float _poseX_type;
		_poseX_type poseX;
		typedef float _poseY_type;
		_poseY_type poseY;
		typedef float _angleRz_type;
		_angleRz_type angleRz;
		typedef float _speedVx_type;
		_speedVx_type speedVx;
		typedef float _speedWz_type;
		_speedWz_type speedWz;
	};
}
namespace geometry_msgs
{
	struct Vector3
	{
		typedef double _x_type;
		_x_type x;
		typedef double _y_type;
		_y_type y;
		typedef double _z_type;
		_z_type z;
	};
	struct Twist
	{
		typedef geometry_msgs::Vector3 _linear_type;
		_linear_type linear;
		typedef geometry_msgs::Vector3 _angular_type;
		_angular_type angular;
	};
}

namespace std_msgs
{
	struct Bool
	{
	      typedef bool _data_type;
	      _data_type data;
	};
}
