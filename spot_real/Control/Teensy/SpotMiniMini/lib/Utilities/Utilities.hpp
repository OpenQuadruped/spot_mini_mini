#ifndef UTILITIES_INCLUDE_GUARD_HPP
#define UTILITIES_INCLUDE_GUARD_HPP
#include "Kinematics.hpp"
/// \file
/// \brief Utilities Library. Adapted from https://github.com/adham-elarabawy/OpenQuadruped
class Utilities {
  public:
  void upper(char* s);
  double angleConversion(double angle, double home_angle, LegQuadrant legquad);
  double toDegrees(double radianVal);
  double max(double a0, double a1, double a2);
};

#endif
