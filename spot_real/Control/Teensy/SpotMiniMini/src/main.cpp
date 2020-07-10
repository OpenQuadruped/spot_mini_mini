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


// Example 5 - Receive with start- and end-markers combined with parsing

const byte numChars = 64;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing

      // variables to hold the parsed data
char messageFromPC[numChars] = {0};

boolean newData = false;

//============

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (Serial1.available() > 0 && newData == false) {
        rc = Serial1.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

//============

void parseData(int & leg, double & x, double & y, double & z, int & servo_num, double & servo_calib_angle) {      // split the data into its parts

    char * strtokIndx; // this is used by strtok() as an index

    strtokIndx = strtok(tempChars,",");      // get the first part - the string
    leg = atoi(strtokIndx);     // convert this part to an integer

    if (leg == 4)
    {
      strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
      servo_num = atoi(strtokIndx);     // convert this part to an integer
      strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
      servo_calib_angle = atof(strtokIndx);     // convert this part to an integer

      Serial1.println(servo_num);
      Serial1.println(servo_calib_angle);
    } else
    {
      strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
      x = atof(strtokIndx);     // convert this part to an integer
      strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
      y = atof(strtokIndx);     // convert this part to an integer
      strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
      z = atof(strtokIndx);     // convert this part to an integer
    }
}

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

  Serial1.print("READY!\n");

  delay(1000);
}

// THIS LOOPS FOREVER
void loop()
{

  if(!ESTOPPED){
    update_servos();
  } else {
    detach_servos();
  }
  update_sensors();
  if (Serial1.available())
  {
    Serial1.println("Serial OK\n");
    // serialResponse = Serial.readStringUntil('\n');
    // serialResponse = Serial.readStringUntil('\r\n');
    // Convert from String Object to String.
    // NOTE: Must have size of msg0
    // char buf[sizeof(msg0)];
    // serialResponse.toCharArray(buf, sizeof(buf));
    // char *ptr = buf;
    // char *str;
    // int message_string_index = 0;
    int leg = -1; //0=FL, 1=FR, 2=BL, 3=BR
    double x = -9999;
    double y = -9999;
    double z = -9999;
    // For Servo Calibration Request
    int servo_num = -1;
    double servo_calib_angle = 135.0;

    // NOTE: PARSE DATA!
    recvWithStartEndMarkers();
    if (newData == true) {
        strcpy(tempChars, receivedChars);
            // this temporary copy is necessary to protect the original data
            //   because strtok() used in parseData() replaces the commas with \0
        parseData(leg, x, y, z, servo_num, servo_calib_angle);
        newData = false;
    }

    //COMPLETE MESSAGE CHECK
    if(leg != -9999 || x != -9999 || y != -9999 || z != -9999){
      // Serial.println("complete message");

      if (servo_num == -1 and leg != -1)
      // NORMAL OPERATION
      {
        Serial1.println("SERVO ACT\n");
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
        Serial1.println("SERVO CALIB\n");
        (*AllServos[servo_num]).SetGoal(servo_calib_angle, max_speed);
      }
    }
  }
  //  else
  // {
  //   Serial.println("Serial NOT AVAILABLE\n");
  // }
}
