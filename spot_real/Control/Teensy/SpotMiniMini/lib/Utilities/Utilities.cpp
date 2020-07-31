#include "Utilities.hpp"
#include <Arduino.h>

double Utilities::toDegrees(double radianVal) {
  return radianVal * 57296 / 1000.0;
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
