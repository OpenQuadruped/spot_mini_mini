#include "SpotServo.hpp"

using namespace std;

// Spot Constructor
Spot::Spot()
{
}

// Spot Full Constructor
void Spot::Initialize(int & servo_pin, double & home_angle_, double & offset_, LegType & leg_type_, JointType & joint_type_)
{
	servo.attach(servo_pin, 500, 2500);
	offset = offset_;
	home_angle = home_angle_;
	current_pose = home_angle + offset;
	leg_type = leg_type_;
	joint_type = joint_type;
	servo.write((home_angle + offset) * 180.0 / control_range);
	delay(1000);
	last_actuated = millis();
}

void Spot::SetGoal(double & goal_pose_, double & desired_speed_)
{
	goal_pose_ += offset;
	
	// cpp would be std::clamp() with include cmath
	constrain(goal_pose_, 0.0, control_range);

	goal_pose = goal_pose_;
	desired_speed = desired_speed_;
}

double Spot::GetPoseEstimate()
{
	return current_pose - offset;
}

void Spot::update_clk()
{
	// Only perform update if loop rate is met
	if(millis() - last_actuated > wait_time)
	{
		// Only update position if not within threshold
		if(abs(current_pose - goal_pose) > error_threshold)
		{
			// returns 1.0 * sign of goal_pose - current_pose
			double direction = copysign(1.0, goal_pose - current_pose);
			current_pose += direction * (wait_time / 1000.0) * spd;
			servo.write(current_pose * 180.0 / control_range);
			last_actuated = millis();
		}
	}

}

void SpotServo::detach()
{
	servo.detach();
}