/// \file
/// \brief Teensy Main. Adapted from https://github.com/adham-elarabawy/OpenQuadruped

#include <Arduino.h>
#include "SpotServo.hpp"
#include "ContactSensor.hpp"
#include "Kinematics.hpp"
#include "Utilities.hpp"
#include "IMU.hpp"
#include <Servo.h>

using namespace std;

String serialResponse = "";
// LegNum, FootX, FootY, FootZ
char msg0[] = "0,-999.9,-999.9,-999.9"; // structure for position command
// LegNum, FootX, FootY, FootZ, FootSpeedX, FootSpeedY, FootSpeedZ
// char msg1[] = "0,-999.9,-999.9,-999.9,-999.9,-999.9,-999.9"; // structure for position/speed command
bool ESTOPPED = false;
int max_speed = 500; // deg/sec
double last_estop = millis();


SpotServo FL_Shoulder, FL_Elbow, FL_Wrist, FR_Shoulder, FR_Elbow, FR_Wrist, BL_Shoulder, BL_Elbow, BL_Wrist, BR_Shoulder, BR_Elbow, BR_Wrist;
ContactSensor FL_sensor, FR_sensor, BL_sensor, BR_sensor;
SpotServo * Shoulders[4] = {&FL_Shoulder, &FR_Shoulder, &BL_Shoulder, &BR_Shoulder};
SpotServo * Elbows[4] = {&FL_Elbow, &FR_Elbow, &BL_Elbow, &BR_Elbow};
SpotServo * Wrists[4] = {&FL_Wrist, &FR_Wrist, &BL_Wrist, &BR_Wrist};
Utilities util;
Kinematics ik;
IMU imu;

void detach_servos()
{
    // Shoulders
    FL_Shoulder.detach();
    FR_Shoulder.detach();
    BL_Shoulder.detach();
    BR_Shoulder.detach();

    // Elbows
    FL_Elbow.detach();
    FR_Elbow.detach();
    BL_Elbow.detach();
    BR_Elbow.detach();

    // Wrists
    FL_Wrist.detach();
    FR_Wrist.detach();
    BL_Wrist.detach();
    BR_Wrist.detach();
}

void update_servos()
{
    // ShoulderS
    FL_Shoulder.update_clk();
    FR_Shoulder.update_clk();
    BL_Shoulder.update_clk();
    BR_Shoulder.update_clk();

    // Elbow
    FL_Elbow.update_clk();
    FR_Elbow.update_clk();
    BL_Elbow.update_clk();
    BR_Elbow.update_clk();

    // WRIST
    FL_Wrist.update_clk();
    FR_Wrist.update_clk();
    BL_Wrist.update_clk();
    BR_Wrist.update_clk();
}

void update_sensors()
{
    FL_sensor.update_clk();
    FR_sensor.update_clk();
    BL_sensor.update_clk();
    BR_sensor.update_clk();
}

// THIS ONLY RUNS ONCE
void setup() {
  Serial1.begin(500000);

  ik.Initialize(0.04, 0.1, 0.1);

  // Shoulders
  FL_Shoulder.Initialize(4, 135, 0, FL, Shoulder);
  FR_Shoulder.Initialize(11, 135, 0, FR, Shoulder);
  BL_Shoulder.Initialize(7, 135, 0, BL, Shoulder);
  BR_Shoulder.Initialize(8, 135, 0, BR, Shoulder);

  //Elbows
  FL_Elbow.Initialize(2, 135, 0, FL, Elbow);
  FR_Elbow.Initialize(13, 135, 0, FR, Elbow);
  BL_Elbow.Initialize(5, 135, 0, BL, Elbow);
  BR_Elbow.Initialize(10, 135, 0, BR, Elbow);

  //Wrists
  FL_Wrist.Initialize(3, 135, 0, FL, Wrist);
  FR_Wrist.Initialize(12, 135, 0, FR, Wrist);
  BL_Wrist.Initialize(6, 135, 0, BL, Wrist);
  BR_Wrist.Initialize(9, 135, 0, BR, Wrist);

  // Contact Sensors
  FL_sensor.Initialize(A9, 17);
  FR_sensor.Initialize(A8, 16);
  BL_sensor.Initialize(A7, 15);
  BR_sensor.Initialize(A6, 14);

  // IMU
  imu.Initialize();

  last_estop = millis();

  delay(1000);
}

// THIS LOOPS FOREVER
void loop() {
  if(!ESTOPPED){
    update_servos();
  } else {
    detach_servos();
  }
  update_sensors();
  if (Serial1.available()) {
    serialResponse = Serial1.readStringUntil('\r\n');
    // Convert from String Object to String.
    char buf[sizeof(msg0)];
    serialResponse.toCharArray(buf, sizeof(buf));
    char *ptr = buf;
    char *str;
    int message_string_index = 0;
    int leg = -1; //0=FL, 1=FR, 2=BL, 3=BR
    double x = -9999;
    double y = -9999;
    double z = -9999;
    while ((str = strtok_r(ptr, ",", &ptr)) != NULL) { // delimiter is the dash
      if((strcmp(str, "e") == 0 or strcmp(str, "E") == 0) and last_estop > 200) {
        // TRIGGER
        ESTOPPED = !ESTOPPED;
        last_estop = millis();
      }

      // Read Desired Leg
      if(message_string_index == 0) {
        util.upper(str);
        if(strcmp(str, "0") == 0){
          leg = 0;
        }
        if(strcmp(str, "1") == 0){
          leg = 1;
        }
        if(strcmp(str, "2") == 0){
          leg = 2;
        }
        if(strcmp(str, "3") == 0){
          leg = 3;
        }
      }

      // Read Desired Foot x pos
      if(message_string_index == 1) {
        x = atof(str);
      }
      // Read Desired Foot y pos
      if(message_string_index == 2) {
        y = atof(str);
      }
      // Read Desired Foot z pos
      if(message_string_index == 3) {
        z = atof(str);
      }

      // Increment message message_string_index
      message_string_index++;
    }

    //COMPLETE MESSAGE CHECK
    if(leg != -9999 || x != -9999 || y != -9999 || z != -9999){
      Serial.println("complete message");
      double *angles;

      LegType legtype;
      if (leg == 0 or leg == 2)
      {
        legtype = Left;
      } else
      {
        legtype = Right;
      }

      angles = ik.run(x, y, z, legtype);

      double Shoulder_angle = util.angleConversion(leg, 0, util.toDegrees(*angles));
      double Elbow_angle = util.angleConversion(leg, 1, util.toDegrees(*(angles+1)));
      double wrist_angle = util.angleConversion(leg, 2, util.toDegrees(*(angles+2)));

      double h_dist = abs(Shoulder_angle - (*Shoulders[leg]).getPosition());
      double s_dist = abs(Elbow_angle - (*Elbows[leg]).getPosition());
      double w_dist = abs(wrist_angle - (*Wrists[leg]).getPosition());

      double scaling_factor = util.max(h_dist, s_dist, w_dist);

      h_dist /= scaling_factor;
      s_dist /= scaling_factor;
      w_dist /= scaling_factor;

      (*Shoulders[leg]).setPosition(Shoulder_angle, max_speed * h_dist);
      (*Elbows[leg]).setPosition(Elbow_angle, max_speed * s_dist);
      (*Wrists[leg]).setPosition(wrist_angle, max_speed * w_dist);
    }
  }
}
