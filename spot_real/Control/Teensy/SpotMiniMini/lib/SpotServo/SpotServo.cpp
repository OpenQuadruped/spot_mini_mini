#include "SpotServo.hpp"

using namespace std;

// Spot Full Constructor
void SpotServo::Initialize(const int & servo_pin, const double & stand_angle_, const double & home_angle_, const double & offset_, const LegType & leg_type_, const JointType & joint_type_)
{
	conv_slope = (max_pwm - min_pwm) / (control_range - 0.0);
	conv_intcpt = max_pwm - conv_slope * control_range;
	servo.attach(servo_pin, min_pwm, max_pwm);
	offset = offset_;
	home_angle = home_angle_;
	current_pose = home_angle + offset;
	leg_type = leg_type_;
	joint_type = joint_type;
	stand_angle = stand_angle_;
	int pwm = (stand_angle + offset) * conv_slope + conv_intcpt;
	servo.writeMicroseconds(pwm);
	delay(1000);
	last_actuated = millis();
}

void SpotServo::SetGoal(const double & goal_pose_, const double & desired_speed_)
{
	goal_pose = goal_pose_;
	goal_pose += offset;
	
	// cpp would be std::clamp() with include cmath
	constrain(goal_pose, 0.0, control_range);

	desired_speed = desired_speed_;
}

double SpotServo::return_home()
{
	return home_angle;
}

double SpotServo::GetPoseEstimate()
{
	return current_pose - offset;
}

void SpotServo::update_clk()
{
	// Only perform update if loop rate is met
	if(millis() - last_actuated > wait_time)
	{
		// Only update position if not within threshold
		if(abs(current_pose - goal_pose) > error_threshold)
		{
			// returns 1.0 * sign of goal_pose - current_pose
			double direction = copysign(1.0, goal_pose - current_pose);
			current_pose += direction * (wait_time / 1000.0) * desired_speed;
			int pwm = (current_pose) * conv_slope + conv_intcpt;
			servo.writeMicroseconds(pwm);
			last_actuated = millis();
		}
	}

}

void SpotServo::detach()
{
	servo.detach();
}