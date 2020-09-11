#include "SpotServo.hpp"

using namespace std;

// Spot Full Constructor
void SpotServo::Initialize(const int & servo_pin, const double & stand_angle_, const double & home_angle_, const double & offset_, const LegType & leg_type_, const JointType & joint_type_,
						   const int & min_pwm_, const int & max_pwm_, const double & ang_min_pwm, const double & ang_max_pwm)
{
	// use defaults for servo attach
	AssemblyInit(servo_pin, min_pwm, max_pwm);
	// these are not really min and max, just used for interpolation
	min_pwm = min_pwm_;
	max_pwm = max_pwm_;
	conv_slope = (double)(max_pwm - min_pwm) / (ang_max_pwm - ang_min_pwm);
	conv_intcpt = max_pwm - conv_slope * ang_max_pwm;
	offset = offset_;
	home_angle = home_angle_;
	leg_type = leg_type_;
	joint_type = joint_type;
	stand_angle = stand_angle_;
	goal_pose = stand_angle + offset;
	current_pose = stand_angle + offset;
	int pwm = round((goal_pose) * conv_slope + conv_intcpt);
	servo.writeMicroseconds(pwm);
	last_actuated = millis();
}

void SpotServo::SetGoal(const double & goal_pose_, const double & desired_speed_, const bool & step_or_view_)
{
	// remove calibrating flag
	calibrating = false;
	// Update Move Type
	step_or_view = step_or_view_;

	// Catch for invalid command (used by calibration node to single out motors)
	// Only update if valid command
	if (goal_pose_ > -998)
	{
		goal_pose = goal_pose_;
		// Add Offset
		goal_pose += offset;
		
		// TODO: ADD JOINT LIM 

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

void SpotServo::actuate()
{
	if (goal_pose > -998)
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
			int pwm = round((current_pose) * conv_slope + conv_intcpt);
			servo.writeMicroseconds(pwm);
			last_actuated = millis();
		} else
		// if we are at small error thresh, actuate directly
		{
			int pwm = round((goal_pose) * conv_slope + conv_intcpt);
			current_pose = goal_pose;
			servo.writeMicroseconds(pwm);
			last_actuated = millis();
		}
	}

}

void SpotServo::update_clk()
{
	if (!calibrating)
	{
		// Only perform update if loop rate is met - or instantly if walking
		if ((millis() - last_actuated > wait_time ) or !step_or_view)
		{
			actuate();
		}
	}

}

bool SpotServo::GoalReached()
{
	return (abs(current_pose - goal_pose) < error_threshold);
}

void SpotServo::writePulse(const int & pulse)
{
	servo.writeMicroseconds(pulse);
	calibrating = true;
}

void SpotServo::AssemblyInit(const int & servo_pin, const int & min_pwm_, const int & max_pwm_)
{
	// use defaults for servo attach
	servo.attach(servo_pin, min_pwm, max_pwm);
}