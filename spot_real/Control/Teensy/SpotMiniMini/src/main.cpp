/// \file
/// \brief Teensy Main. Adapted from https://github.com/adham-elarabawy/OpenQuadruped

#include <Arduino.h>
#include "SpotServo.hpp"
#include "ContactSensor.hpp"
#include "Kinematics.hpp"
#include "Utilities.hpp"
#include "IMU.hpp"
#include <Servo.h>

#define DEBUGSERIAL Serial

using namespace std;

String serialResponse = "";
// LegNum, FootX, FootY, FootZ
char msg0[] = "0,-999.9,-999.9,-999.9"; // structure for position command
// LegNum, FootX, FootY, FootZ, FootSpeedX, FootSpeedY, FootSpeedZ
// char msg1[] = "0,-999.9,-999.9,-999.9,-999.9,-999.9,-999.9"; // structure for position/speed command
bool ESTOPPED = false;
int max_speed = 500; // deg/sec
double last_estop = millis();
const int ledPin = 13;


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
  while (!Serial1);
  // DEBUG - USB
  // Serial.begin(9600);
  // while (!Serial);

  // above code waits until serial1 and serial ready

  pinMode(ledPin, OUTPUT);

  ik.Initialize(0.04, 0.1, 0.1);

  // Shoulders
  FL_Shoulder.Initialize(4, 134, 135, -7, FL, Shoulder);  // 0 | FLS start: 134
  FL_Shoulder.SetGoal(134, max_speed / 5.0);
  FR_Shoulder.Initialize(11, 136, 135, -5, FR, Shoulder); // 1 | FRS start: 136
  FR_Shoulder.SetGoal(136, max_speed / 5.0);
  BL_Shoulder.Initialize(7, 136, 135, 5, BL, Shoulder);  // 2 | BLS start: 136
  BL_Shoulder.SetGoal(136, max_speed / 5.0);
  BR_Shoulder.Initialize(8, 134, 135, -7, BR, Shoulder);  // 3 | BRS start: 134
  BR_Shoulder.SetGoal(134, max_speed / 5.0);

  //Elbows
  FL_Elbow.Initialize(2, 182, 135, 2, FL, Elbow);  // 4 | FLE start: 182
  FL_Elbow.SetGoal(182, max_speed / 5.0);
  FR_Elbow.Initialize(13, 88, 135, -6, FR, Elbow); // 5 | FRE start: 88
  FR_Elbow.SetGoal(88, max_speed / 5.0);
  BL_Elbow.Initialize(5, 182, 135, 7, BL, Elbow);  // 6 | BLE start: 182
  BL_Elbow.SetGoal(182, max_speed / 5.0);
  BR_Elbow.Initialize(10, 88, 135, -7, BR, Elbow); // 7 | BRE start: 88
  BR_Elbow.SetGoal(88, max_speed / 5.0);

  //Wrists
  FL_Wrist.Initialize(3, 76, 170, 1, FL, Wrist);  // 8 | FLW start: 76
  FL_Wrist.SetGoal(76, max_speed / 5.0);
  FR_Wrist.Initialize(12, 194, 100, -13, FR, Wrist); // 9 | FRW start: 194
  FR_Wrist.SetGoal(194, max_speed / 5.0);
  BL_Wrist.Initialize(6, 76, 170, 13, BL, Wrist); // 10 | BLW start: 76
  BL_Wrist.SetGoal(76, max_speed / 5.0);
  BR_Wrist.Initialize(9, 194, 100, -8, BR, Wrist); // 11 | BRW start: 194
  BR_Wrist.SetGoal(194, max_speed / 5.0);

  // Contact Sensors
  FL_sensor.Initialize(A9, 17);
  FR_sensor.Initialize(A8, 16);
  BL_sensor.Initialize(A7, 15);
  BR_sensor.Initialize(A6, 14);

  // IMU
  imu_sensor.Initialize();

  last_estop = millis();

  Serial1.print("READY!\n");
}

// THIS LOOPS FOREVER
void loop()
{
  // CHECK SENSORS AND SEND INFO
  // CONTACT
  char FL_sensor_buf[2];
  char FR_sensor_buf[2];
  char BL_sensor_buf[2];
  char BR_sensor_buf[2];
  char contact_sensor_buf[9];

  if (FL_sensor.isTriggered())
  {
    // integer, buffer, base
    itoa(1, FL_sensor_buf, 10);
  } else
  {
    itoa(0, FL_sensor_buf, 10);
  }

  if (FR_sensor.isTriggered())
  {
    itoa(1, FR_sensor_buf, 10);
  } else
  {
    itoa(0, FR_sensor_buf, 10);
  }
  if (BL_sensor.isTriggered())
  {
    itoa(1, BL_sensor_buf, 10);
  } else
  {
    itoa(0, BL_sensor_buf, 10);
  }
  if (BR_sensor.isTriggered())
  {
    itoa(1, BR_sensor_buf, 10);
  } else
  {
    itoa(0, BR_sensor_buf, 10);
  }

  // convert all to string
  sprintf_P(contact_sensor_buf, PSTR("CONTACT,%s,%s,%s,%s\n"), FL_sensor_buf, FR_sensor_buf, BL_sensor_buf, BR_sensor_buf);

  // Send to RPI
  // Serial1.println(contact_sensor_buf);

  // DEBUG
  // DEBUGSERIAL.println(contact_sensor_buf);

  //IMU
  if (imu_sensor.available())
  {
    char roll_buf[20];
    char pitch_buf[20];
    char yaw_buf[20];
    char acc_x_buf[20];
    char acc_y_buf[20];
    char acc_z_buf[20];
    char gyro_x_buf[20];
    char gyro_y_buf[20];
    char gyro_z_buf[20];
    // just in case we get some big values
    char imu_buf[512];

    imu::Vector<3> eul = imu_sensor.GetEuler();
    // val, width, precision, buff
    // NOTE: switching eul.x and eul.x because Bosch is weird..
    dtostrf(eul.z(), 0, 4, roll_buf);
    dtostrf(eul.y(), 0, 4, pitch_buf);
    dtostrf(eul.x(), 0, 4, yaw_buf);
    imu::Vector<3> acc = imu_sensor.GetAcc();
    dtostrf(acc.x(), 0, 4, acc_x_buf);
    dtostrf(acc.y(), 0, 4, acc_y_buf);
    dtostrf(acc.z(), 0, 4, acc_z_buf);
    imu::Vector<3> gyro = imu_sensor.GetGyro();
    dtostrf(gyro.x(), 0, 4, gyro_x_buf);
    dtostrf(gyro.y(), 0, 4, gyro_y_buf);
    dtostrf(gyro.z(), 0, 4, gyro_z_buf);

    // convert all to string
    sprintf_P(imu_buf, PSTR("IMU,%s,%s,%s,%s,%s,%s,%s,%s\n"), roll_buf, pitch_buf, acc_x_buf, acc_y_buf, acc_z_buf, gyro_x_buf, gyro_y_buf, gyro_z_buf);

    // sprintf_P(imu_buf, PSTR("Roll: %s,\tPitch: %s,\tYaw: %s\n"), roll_buf, pitch_buf, yaw_buf);


    // Send to RPI
    // Serial1.println(imu_buf);

    // DEBUG
    // DEBUGSERIAL.println(imu_buf);

  }

  if(!ESTOPPED){
    update_servos();
  } else {
    detach_servos();
  }
  update_sensors();
  if (Serial1.available() > 0)
  {
    serialResponse = Serial1.readStringUntil('\n');
    Serial1.println(serialResponse);
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
        // Serial1.println(leg);
        if (leg < 0 or leg > 4)
        {
          leg = -1;
        }
      }

      if (leg >= 0 and leg <=3)
      {
        // Read Desired Foot x pos
        if (message_string_index == 1)
        {
          x = atof(str);
          // Serial1.println(x);
        } else if (message_string_index == 2)
        {
          // Read Desired Foot y pos
          y = atof(str);
          // Serial1.println(y);
        } else if (message_string_index == 3)
        {
          // Read Desired Foot z pos
          z = atof(str);
          // Serial1.println(z);
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

      // Increment message message_string_index
      message_string_index++;
    }

    //COMPLETE MESSAGE CHECK
    if(leg != -9999 and x != -9999 and y != -9999 and z != -9999){
      // Serial.println("complete message");

      if (servo_num == -1 and leg != -1)
      // NORMAL OPERATION
      {
        // DEBUGSERIAL.println("SERVO ACT\n");

        LegType leg_type;
        LegQuadrant legquad;
        if (leg == 0 or leg == 2)
        {
          legquad = Left;
          if (leg == 0)
          {
            leg_type = FL;
          } else
          {
            leg_type = BL;
          }
        } else
        {
          legquad = Right;
          if (leg == 1)
          {
            leg_type = FR;
          } else
          {
            leg_type = BR;
          }
        }

        double shoulder_home = (*Shoulders[leg]).return_home();
        double elbow_home = (*Elbows[leg]).return_home();
        double wrist_home = (*Wrists[leg]).return_home();

        // NOTE: x,y,z maps to shoulder, elbow wrist... trying smth new


        double Shoulder_angle = util.angleConversion(x, shoulder_home, leg_type, Shoulder);
        double Elbow_angle = util.angleConversion(y, elbow_home, leg_type, Elbow);
        double Wrist_angle = util.angleConversion(z, wrist_home, leg_type, Wrist);

        double h_dist = abs(Shoulder_angle - (*Shoulders[leg]).GetPoseEstimate());
        double s_dist = abs(Elbow_angle - (*Elbows[leg]).GetPoseEstimate());
        double w_dist = abs(Wrist_angle - (*Wrists[leg]).GetPoseEstimate());

        double scaling_factor = util.max(h_dist, s_dist, w_dist);

        h_dist /= scaling_factor;
        s_dist /= scaling_factor;
        w_dist /= scaling_factor;

        (*Shoulders[leg]).SetGoal(Shoulder_angle, max_speed * h_dist);
        (*Elbows[leg]).SetGoal(Elbow_angle, max_speed * s_dist);
        (*Wrists[leg]).SetGoal(Wrist_angle, max_speed * w_dist);

        // SEND DEBUG DATA
        char Shoulder_buf[10];
        char Elbow_buf[10];
        char Wrist_buf[10];
        char Leg_buf[10];
        char Debug_buf[512];

        itoa(leg, Leg_buf, 10);
        dtostrf(Shoulder_angle, 0, 4, Shoulder_buf);
        dtostrf(Elbow_angle, 0, 4, Elbow_buf);
        dtostrf(Wrist_angle, 0, 4, Wrist_buf);

        // convert all to string
        sprintf_P(Debug_buf, PSTR("Leg: %s \t Shoulder: %s \t Elbow: %s \t Wirst:%s\n"), Leg_buf, Shoulder_buf, Elbow_buf, Wrist_buf);

        // Send to RPI
        Serial1.println(Debug_buf);

      } else if (servo_num != -1)
      {
        // SERVO CALIBRATION - SEND ANGLE DIRECTLY
        // DEBUGSERIAL.println("SERVO CALIB\n");
        (*AllServos[servo_num]).SetGoal(servo_calib_angle, max_speed);
      }
    }
  }
}
