#include "mini_ros/teleop.hpp"

namespace tele
{
	Teleop::Teleop(const int & linear, const int & angular, const double & l_scale, const double & a_scale, const int & sw, const int & es)
	{
		linear_ = linear;
		angular_ = angular;
		l_scale_ = l_scale;
		a_scale_ = a_scale;
		sw_ = sw;
		es_ = es;
		switch_trigger = false;
		ESTOP = false;
	}

	void Teleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
	{
		geometry_msgs::Twist twist;
		twist.linear.x = l_scale_*joy->axes[linear_];
		twist.angular.z = a_scale_*joy->axes[angular_];
		

		// Switch Trigger: Button X
		switch_trigger = joy->buttons[sw_];

		// ESTOP: Button Y
		ESTOP = joy->buttons[es_];
	}

	geometry_msgs::Twist Teleop::return_twist()
	{
		return twist;
	}

	bool Teleop::return_trigger()
	{
		return switch_trigger;
	}

	bool Teleop::return_estop()
	{
		return ESTOP;
	}
}