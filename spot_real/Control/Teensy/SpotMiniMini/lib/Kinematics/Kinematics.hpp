#ifndef KINEMATICS_INCLUDE_GUARD_HPP
#define KINEMATICS_INCLUDE_GUARD_HPP
/// \file
/// \brief Leg Kinematics Library.
#include <Arduino.h>

enum LegQuadrant {Right, Left};

class Kinematics {
	private:		
        double shoulder_length = 0.0;
        double elbow_length = 0.0;
        double wrist_length = 0.0;
	public:
		// using default constructor

		/// \brief Initialize parameters
		/// \param shoulder_length_: length of shoulder link
		/// \param elbow_length_: length of elbow link
		/// \param wrist_length_: length of wrist link
		/// \param leg_type_: right or left legs
		void Initialize(const double & shoulder_length_, const double & elbow_length_, const double & wrist_length_);

		/// \brief Calculates the leg's Domain and caps it in case of a breach
		/// \param x: x coordinate of Hip To Foot Vector 
		/// \param y: y coordinate of Hip To Foot Vector 
		/// \param z: z coordinate of Hip To Foot Vector 
		/// \returns: Leg Domain D
		double GetDomain(const double & x, const double & y, const double & z);

		/// \brief Right Leg Inverse Kinematics Solver
		/// \param x: x coordinate of Hip To Foot Vector 
		/// \param y: y coordinate of Hip To Foot Vector 
		/// \param z: z coordinate of Hip To Foot Vector
		/// \param D: the leg domain
		/// \param angles: array to populate with IK angles
		/// \returns: pointer to beginning of array containing joint angles for this leg
		void RightIK(const double & x, const double & y, const double & z, const double & D, double (& angles) [3]);

		/// \brief Left Leg Inverse Kinematics Solver
		/// \param x: x coordinate of Hip To Foot Vector 
		/// \param y: y coordinate of Hip To Foot Vector 
		/// \param z: z coordinate of Hip To Foot Vector
		/// \param D: the leg domain
		/// \param angles: array to populate with IK angles
		/// \returns: pointer to beginning of array containing joint angles for this leg
		void LeftIK(const double & x, const double & y, const double & z, const double & D, double (& angles) [3]);

		/// \brief Retrives Joint Angles using a Hip To Foot Vector (x, y, z)
		/// \param x: x coordinate of Hip To Foot Vector 
		/// \param y: y coordinate of Hip To Foot Vector 
		/// \param z: z coordinate of Hip To Foot Vector
		/// \param legquad: Leg quadrant (left or right)
		/// \param angles: array to populate with IK angles
		/// \returns: pointer to beginning of array containing joint angles for this leg
		void GetJointAngles(const double & x, const double & y, const double & z, const LegQuadrant & legquad, double (& angles) [3]);


};
#endif