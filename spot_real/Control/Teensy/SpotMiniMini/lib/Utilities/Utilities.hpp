#ifndef UTILITIES_INCLUDE_GUARD_HPP
#define UTILITIES_INCLUDE_GUARD_HPP
/// \file
/// \brief Utilities Library. Adapted from https://github.com/adham-elarabawy/OpenQuadruped
class Util {
  public:
  void Util::upper(char* s);
  double angleConversion(int leg, int joint, double angle);
  int inverse_angleConversion(int leg, int joint, double angle);
  double toDegrees(double radianVal);
  double max(double a0, double a1, double a2);
};

#endif
