#include "Kinematics.hpp"

void Kinematics::Initialize(const double & shoulder_length_, const double & elbow_length_, const double & wrist_length_)
{
	shoulder_length = shoulder_length_;
    elbow_length = elbow_length_;
    wrist_length = wrist_length_;
}

double Kinematics::GetDomain(const double & x, const double & y, const double & z)
{
	double D = (pow(y, 2) + pow(-z, 2) - pow(shoulder_length, 2) +
             	pow(-x, 2) - pow(elbow_length, 2) - pow(wrist_length, 2)) / (
                2.0 * wrist_length * elbow_length);
    if (D > 1.0)
    {
        D = 1.0;
    } if (D < -1.0)
    {
        D = -1.0;
    }

    return D;
}

void Kinematics::RightIK(const double & x, const double & y, const double & z, const double & D, double (& angles) [3])
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

	angles[0] = shoulder_angle;
	angles[1] = -elbow_angle;
	angles[2] = -wrist_angle;
}

void Kinematics::LeftIK(const double & x, const double & y, const double & z, const double & D, double (& angles) [3])
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

	angles[0] = shoulder_angle;
	angles[1] = -elbow_angle;
	angles[2] = -wrist_angle;
}

void Kinematics::GetJointAngles(const double & x, const double & y, const double & z, const LegQuadrant & legquad, double (& angles) [3])
{
	double D = GetDomain(x, y, z);
	if (legquad == Right)
	{
		RightIK(x, y, z, D, angles);
	} else
	{
		LeftIK(x, y, z, D, angles);
	}
}

