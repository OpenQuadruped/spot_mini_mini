#include "mini_ros/teleop.hpp"

namespace tele
{
	Teleop::Teleop(const int & linear_x, const int & linear_y, const int & linear_z,
	               const int & angular, const double & l_scale, const double & a_scale,
	               const int & LB, const int & RB, const int & B_scale, const int & LT,
	               const int & RT, const int & UD, const int & LR,
	               const int & sw, const int & es)
	{
		// step length or pitch
		linear_x_ = linear_x;
		// lateral fraction or roll
		linear_y_ = linear_y;
		// height
		linear_z_ = linear_z;
		// yaw rate or yaw
		angular_ = angular;
		// scale linear axis (usually just 1)
		l_scale_ = l_scale;
		// scale angular axis (usually just 1)
		a_scale_ = a_scale;
		// for incrementing and decrementing step velocity
		// Bottom Bumpers
		RB_ = RB;
		LB_ = LB;
		// scale Bottom Trigger axis (usually just 1)
		B_scale_ = B_scale;
		// Top Bumpers
		RT_ = RT;
		LT_ = LT;
		// Switch between Stepping and Viewing
		sw_ = sw;
		// E-STOP
		es_ = es;

		// Arrow PAd
		UD_ = UD;
		LR_ = LR;

		switch_trigger = false;
		ESTOP = false;
		updown = 0;
        leftright = 0;

		left_bump = false;
		right_bump = false;
	}

	void Teleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
	{
		twist.linear.x = l_scale_*joy->axes[linear_x_];
		twist.linear.y = l_scale_*joy->axes[linear_y_];
		// NOTE: used to control robot height
		twist.linear.z = -l_scale_*joy->axes[linear_z_];
		twist.angular.z = a_scale_*joy->axes[angular_];
		// NOTE: bottom bumpers used for changing step velocity
		twist.angular.x = B_scale_*joy->axes[RB_];
		twist.angular.y = B_scale_*joy->axes[LB_];
		

		// Switch Trigger: Button A
		switch_trigger = joy->buttons[sw_];

		// ESTOP: Button B
		ESTOP = joy->buttons[es_];

		// Arrow Pad
		updown = joy->axes[UD_];
		leftright = -joy->axes[LR_];

		// Top Bumpers
		left_bump = joy->buttons[LT_];
		right_bump = joy->buttons[RT_];
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

	mini_ros::JoyButtons Teleop::return_buttons()
	{
		mini_ros::JoyButtons jb;
		jb.updown = updown;
		jb.leftright = leftright;
		jb.left_bump = left_bump;
		jb.right_bump = right_bump;

		return jb;
	}	
}