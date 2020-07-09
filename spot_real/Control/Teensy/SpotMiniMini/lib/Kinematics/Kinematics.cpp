#include "Kinematics.hpp"

void Kinematics::Initialize(double & shoulder_length_, double & elbow_length_, double & wrist_length_)
{
	shoulder_length = shoulder_length_;
    elbow_length = elbow_length_;
    wrist_length = wrist_length_;
}

double Kinematics::GetDomain(double & x, double & y, double & z)
{
	double D = (pow(y, 2) + pow(-z, 2) - pow(shoulder_length, 2) +
             	pow(-x, 2) - pow(elbow_length, 2) - pow(wrist_length, 2)) / (
                2.0 * wrist_length * elbow_length);
    D = constrain(D, -0.99, 0.99);

    return D;
}

double * Kinematics::RightIK(double & x, double & y, double & z, double & D)
{
	double wrist_angle = atan2(-sqrt(1.0 - pow(D, 2)), D);
	double sqrt_component = pow(y, 2) + pow(-z, 2) - pow(shoulder_length, 2);
	if (sqrt_component < 0.0)
	{
		sqrt_component = 0.0;
	}
	double shoulder_angle = -atan2(z, y) - atan2(
				            sqrt(sqrt_component), -shoulder_length);
	double elbow_angle = atan2(-x, sqrt(sqrt_component)) - atan2(
            			 wrist_length * sin(wrist_angle),
            			 elbow_length + wrist_length * cos(wrist_angle));

	static double joint_angles[3];
	joint_angles[0] = shoulder_angle;
	joint_angles[1] = -elbow_angle;
	joint_angles[2] = -wrist_angle;

	return joint_angles;
}

double * Kinematics::LeftIK(double & x, double & y, double & z, double & D)
{
	double wrist_angle = atan2(-sqrt(1.0 - pow(D, 2)), D);
	double sqrt_component = pow(y, 2) + pow(-z, 2) - pow(shoulder_length, 2);
	if (sqrt_component < 0.0)
	{
		sqrt_component = 0.0;
	}
	double shoulder_angle = -atan2(z, y) - atan2(
            				sqrt(sqrt_component), shoulder_length);
	double elbow_angle = atan2(-x, sqrt(sqrt_component)) - atan2(
            			 wrist_length * sin(wrist_angle),
            			 elbow_length + wrist_length * cos(wrist_angle));

	static double joint_angles[3];
	joint_angles[0] = shoulder_angle;
	joint_angles[1] = -elbow_angle;
	joint_angles[2] = -wrist_angle;

	return joint_angles;
}

double * Kinematics::GetJointAngles(double & x, double & y, double & z, LegType & legtype)
{
	double D = GetDomain(x, y, z);
	if (legtype == Right)
	{
		return RightIK(x, y, z, D);
	} else
	{
		return LeftIK(x, y, z, D);
	}
}

