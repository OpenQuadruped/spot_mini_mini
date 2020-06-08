#include "mini_ros/teleop.hpp"

namespace tele
{
	Teleop::Teleop(const int & linear_x, const int & linear_y, const int & linear_z, const int & angular, const double & l_scale, const double & a_scale, const int & sw, const int & es)
	{
		linear_x_ = linear_x;
		linear_y_ = linear_y;
		linear_z_ = linear_z;
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
		twist.linear.x = l_scale_*joy->axes[linear_x_];
		twist.linear.y = l_scale_*joy->axes[linear_y_];
		// NOTE: used to control robot height
		twist.linear.z = -l_scale_*joy->axes[linear_z_];
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