#include "Utilities.hpp"
#include <Arduino.h>

double Utilities::toDegrees(double radianVal) {
  return radianVal * 57296 / 1000.0;
}

double Utilities::max(double a0, double a1, double a2) {
  if(a0 >= a1 && a0 >= a2) {
    return a0;
  }
  if(a1 >= a0 && a1 >= a2) {
    return a1;
  }
  if(a2 >= a1 && a2 >= a0) {
    return a2;
  }
}


double Utilities::angleConversion(double angle, double home_angle, LegType legtype, JointType joint_type) {
  double mod_angle = home_angle;

  if (joint_type == Shoulder)
  {
    mod_angle = home_angle - angle;
  } else if (joint_type == Elbow)
  {
    if (legtype == FR or legtype == BR)
    {
      mod_angle = home_angle - angle;
    } else
    // FL or BL
    {
      mod_angle = home_angle + angle;
    }
  } else if (joint_type == Wrist)
  {
    if (legtype == FR or legtype == BR)
    {
      mod_angle = home_angle + angle;
    } else
    // FL or BL
    {
      mod_angle = home_angle - angle;
    }
  }

  return mod_angle;
}
