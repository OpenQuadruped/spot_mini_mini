#ifndef UTILITIES_INCLUDE_GUARD_HPP
#define UTILITIES_INCLUDE_GUARD_HPP
#include "SpotServo.hpp"
/// \file
/// \brief Utilities Library.
class Utilities {
  public:
  double angleConversion(double angle, double home_angle, LegType legtype, JointType joint_type);
  double toDegrees(double radianVal);
  double max(double a0, double a1, double a2);
};

#endif
