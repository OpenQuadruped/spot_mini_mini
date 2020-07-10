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
SpotServo * AllServos[12] = {&FL_Shoulder, &FR_Shoulder, &BL_Shoulder, &BR_Shoulder, &FL_Elbow, &FR_Elbow, &BL_Elbow, &BR_Elbow, &FL_Wrist, &FR_Wrist, &BL_Wrist, &BR_Wrist};
Utilities util;
Kinematics ik;
IMU imu_sensor;

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
  // HARDWARE - PI COMM
  Serial1.begin(500000);
  // DEBUG - USB
  Serial.begin(9600);
  Serial.print("INITIALIZING!\n");

  ik.Initialize(0.04, 0.1, 0.1);

  // Shoulders
  FL_Shoulder.Initialize(4, 135, 0, FL, Shoulder); // 0
  FR_Shoulder.Initialize(11, 135, 0, FR, Shoulder); // 1
  BL_Shoulder.Initialize(7, 135, 0, BL, Shoulder); // 2
  BR_Shoulder.Initialize(8, 135, 0, BR, Shoulder); // 3

  //Elbows
  FL_Elbow.Initialize(2, 135, 0, FL, Elbow); // 4
  FR_Elbow.Initialize(13, 135, 0, FR, Elbow); // 5
  BL_Elbow.Initialize(5, 135, 0, BL, Elbow); // 6
  BR_Elbow.Initialize(10, 135, 0, BR, Elbow); // 7

  //Wrists
  FL_Wrist.Initialize(3, 135, 0, FL, Wrist); // 8
  FR_Wrist.Initialize(12, 135, 0, FR, Wrist); // 9
  BL_Wrist.Initialize(6, 135, 0, BL, Wrist); // 10
  BR_Wrist.Initialize(9, 135, 0, BR, Wrist); // 11

  // Contact Sensors
  FL_sensor.Initialize(A9, 17);
  FR_sensor.Initialize(A8, 16);
  BL_sensor.Initialize(A7, 15);
  BR_sensor.Initialize(A6, 14);

  // IMU
  imu_sensor.Initialize();

  last_estop = millis();

  Serial.print("READY!\n");

  delay(1000);
}

// THIS LOOPS FOREVER
void loop()
{
  // Serial.print("Looping!\n");
  // Serial.print("----------------------------------\n");

  if(!ESTOPPED){
    update_servos();
  } else {
    detach_servos();
  }
  update_sensors();
  if (Serial1.available())
  {
    Serial.println("SERIAL1 OK\n");
    serialResponse = Serial1.readStringUntil('\r\n');
    Serial.println(serialResponse);
    // Convert from String Object to String.
    // NOTE: Must have size of msg0
    char buf[sizeof(msg0)];
    serialResponse.toCharArray(buf, sizeof(buf));
    char *ptr = buf;
    char *str;
    int message_string_index = 0;
    int leg = -1; //0=FL, 1=FR, 2=BL, 3=BR
    double x = -9999;
    double y = -9999;
    double z = -9999;
    // For Servo Calibration Request
    int servo_num = -1;
    double servo_calib_angle = 135.0;
    while ((str = strtok_r(ptr, ",", &ptr)) != NULL) // delimiter is the comma
    {
      if((strcmp(str, "e") == 0 or strcmp(str, "E") == 0) and last_estop > 200)
      {
        // TRIGGER
        ESTOPPED = !ESTOPPED;
        last_estop = millis();
      }

      // Read Desired Leg
      if(message_string_index == 0)
      {
        // between 0 and 4
        leg = atoi(str);
        constrain(leg, 0, 4);
      }

      if (leg >= 0 and leg <=3)
      {
        // Read Desired Foot x pos
        if (message_string_index == 1)
        {
          x = atof(str);
        } else if (message_string_index == 2)
        {
          // Read Desired Foot y pos
          y = atof(str);
        } else if (message_string_index == 3)
        {
          // Read Desired Foot z pos
          z = atof(str);
        }
      } else if (leg == 4)
      {
        // CALIBRATION REQUEST
        if(message_string_index == 1)
        {
          servo_num = atoi(str);
          // only one of 12 servos
          x = 0;
          y = 0;
          z = 0;
        }  else if (message_string_index == 2)
        {
          // Read Desired Foot y pos
          servo_calib_angle = atof(str);
        }
      }

      Serial.println("------------");
      Serial.println(atoi(str));

      // Increment message message_string_index
      message_string_index++;
    }

    Serial.println("------------");

    //COMPLETE MESSAGE CHECK
    if(leg != -9999 || x != -9999 || y != -9999 || z != -9999){
      // Serial.println("complete message");

      if (servo_num == -1 and leg != -1)
      // NORMAL OPERATION
      {
        Serial.println("SERVO ACT\n");
        double *angles;

        LegQuadrant legquad;
        if (leg == 0 or leg == 2)
        {
          legquad = Left;
        } else
        {
          legquad = Right;
        }

        angles = ik.GetJointAngles(x, y, z, legquad);

        double Shoulder_angle = util.angleConversion(leg, 0, util.toDegrees(*angles));
        double Elbow_angle = util.angleConversion(leg, 1, util.toDegrees(*(angles+1)));
        double wrist_angle = util.angleConversion(leg, 2, util.toDegrees(*(angles+2)));

        double h_dist = abs(Shoulder_angle - (*Shoulders[leg]).GetPoseEstimate());
        double s_dist = abs(Elbow_angle - (*Elbows[leg]).GetPoseEstimate());
        double w_dist = abs(wrist_angle - (*Wrists[leg]).GetPoseEstimate());

        double scaling_factor = util.max(h_dist, s_dist, w_dist);

        h_dist /= scaling_factor;
        s_dist /= scaling_factor;
        w_dist /= scaling_factor;

        (*Shoulders[leg]).SetGoal(Shoulder_angle, max_speed * h_dist);
        (*Elbows[leg]).SetGoal(Elbow_angle, max_speed * s_dist);
        (*Wrists[leg]).SetGoal(wrist_angle, max_speed * w_dist);
      } else if (servo_num != -1)
      {
        // SERVO CALIBRATION - SEND ANGLE DIRECTLY
        Serial.println("SERVO CALIB\n");
        (*AllServos[servo_num]).SetGoal(servo_calib_angle, max_speed);
      }
    }
  }
  //  else
  // {
  //   Serial.println("SERIAL1 NOT AVAILABLE\n");
  // }
}
