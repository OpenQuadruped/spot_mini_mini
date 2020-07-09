#include "Util.h"
#include <Arduino.h>

void Util::upper(char* s) {
  for(int i = 0; i < strlen(s); i++){
    s[i] = toupper(s[i]);
  }
}

double Util::max(double a0, double a1, double a2) {
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

double Util::toDegrees(double radianVal) {
  return radianVal * 57296 / 1000;
}

double Util::angleConversion(int leg, int joint, double angle) {
  if(joint == 0){
    if(leg == 0 || leg == 1) {
      angle = -angle;
    }
    angle = angle + 135;
  }

  if(joint == 1) {
    if(leg == 0 || leg == 2) {
      angle = 90 + angle;
    }
    if(leg == 1 || leg == 3) {
      angle = 180 - angle;
    }
  }

  if(joint == 2) {
    double weird_offset = 50;
    if(leg == 0 || leg == 2) {
      angle = angle - weird_offset;
    }
    if(leg == 1 || leg == 3) {
      angle = (270 + weird_offset) - angle;
    }
  }
  return angle;
}

int Util::inverse_angleConversion(int leg, int joint, double angle) {
  if(joint == 0){
    if (leg == 0 || leg == 1) {
      angle = 135 - angle;
    }
    if (leg == 2 || leg == 3) {
      angle = angle - 135;
    }
  }

  if(joint == 1) {
    if(leg == 0 || leg == 2) {
      angle = angle - 90;
    }
    if(leg == 1 || leg == 3) {
      angle = 180 - angle;
    }
  }

  if(joint == 2) {
    double weird_offset = 50;
    if(leg == 0 || leg == 2) {
      angle = weird_offset + angle;
    }
    if(leg == 1 || leg == 3) {
      angle = (270 + weird_offset) - angle;
    }
  }
  return angle;
}
