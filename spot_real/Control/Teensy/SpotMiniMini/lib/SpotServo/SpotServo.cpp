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
	leg_type = leg_type_;
	joint_type = joint_type;
	stand_angle = stand_angle_;
	goal_pose = stand_angle + offset;
	current_pose = stand_angle + offset;
	int pwm = (goal_pose) * conv_slope + conv_intcpt;
	servo.writeMicroseconds(pwm);
	last_actuated = millis();
}

void SpotServo::SetGoal(const double & goal_pose_, const double & desired_speed_, const bool & step_or_view_)
{
	// Update Move Type
	step_or_view = step_or_view_;

	// Catch for invalid command (used by calibration node to single out motors)
	// Only update if valid command
	if (goal_pose_ > -998)
	{
		goal_pose = goal_pose_;
		// Add Offset
		goal_pose += offset;
		
		// cpp would be std::clamp() with include cmath
		if (goal_pose < 0.0)
		{
			goal_pose = 0.0;
		} else if (goal_pose > control_range)
		{
			goal_pose = control_range;
		}

		desired_speed = desired_speed_;
	}
}

JointType SpotServo::return_joint_type()
{
	return joint_type;
}

LegType SpotServo::return_legtype()
{
	return leg_type;
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
	// Viewing Mode
	if (step_or_view)
	{
		// Only perform update if loop rate is met
		if(millis() - last_actuated > wait_time)
		{
			// Only update position if not within threshold
			if(!GoalReached())
			{
				// returns 1.0 * sign of goal_pose - current_pose
				double direction = 1.0;
				if (goal_pose - current_pose < 0.0)
				{
					direction = -1.0;
				}
				current_pose += direction * (wait_time / 1000.0) * desired_speed;
				int pwm = (current_pose) * conv_slope + conv_intcpt;
				servo.writeMicroseconds(pwm);
				last_actuated = millis();
			} else
			// if we are at small error thresh, actuate directly
			{
				int pwm = (goal_pose) * conv_slope + conv_intcpt;
				current_pose = goal_pose;
				servo.writeMicroseconds(pwm);
				last_actuated = millis();
			}
		}
	// Stepping Mode
	} else
	{
		// Instant move
		int pwm = (goal_pose) * conv_slope + conv_intcpt;
		current_pose = goal_pose;
		servo.writeMicroseconds(pwm);
	}

}

bool SpotServo::GoalReached()
{
	return (abs(current_pose - goal_pose) < error_threshold);
}