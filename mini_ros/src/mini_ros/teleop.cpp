#include "mini_ros/teleop.hpp"

namespace tele
{
	Teleop::Teleop(const int & linear, const int & angular, const double & l_scale, const double & a_scale)
	{
		linear_ = linear;
		angular_ = angular;
		l_scale_ = l_scale;
		a_scale_ = a_scale;
	}

	void Teleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
	{
		geometry_msgs::Twist twist;
		twist.angular.z = a_scale_*joy->axes[angular_];
		twist.linear.x = l_scale_*joy->axes[linear_];
		// TODO:
		switch_trigger = false;
	}

	geometry_msgs::Twist Teleop::return_twist()
	{
		return twist;
	}

	bool Teleop::return_trigger()
	{
		return switch_trigger;
	}
}