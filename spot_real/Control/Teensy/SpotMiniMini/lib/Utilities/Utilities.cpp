#include "Utilities.hpp"
#include <Arduino.h>

void Utilities::upper(char* s) {
  for(int i = 0; i < strlen(s); i++){
    s[i] = toupper(s[i]);
  }
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

double Utilities::toDegrees(double radianVal) {
  return radianVal * 57296 / 1000.0;
}

double Utilities::angleConversion(double angle, double home_angle, LegQuadrant legquad) {
  double mod_angle = 0.0;

  if (legquad == Left)
  {
    mod_angle = home_angle - angle;
  } else
  {
    mod_angle = angle + home_angle;
  }

  return mod_angle;
}
