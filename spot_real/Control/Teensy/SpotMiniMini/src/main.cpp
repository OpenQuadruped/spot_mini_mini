/// \file
/// \brief Teensy Main. Adapted from https://github.com/adham-elarabawy/OpenQuadruped

#include <Arduino.h>
#include <SpotServo.hpp>
#include <ContactSensor.hpp>
#include <Kinematics.hpp>
#include <Utilities.hpp>
#include <IMU.hpp>
#include <Servo.h>
#include <ROSSerial.hpp>

#define DEBUGSERIAL Serial

bool ESTOPPED = false;
int max_speed = 1000; // doesn't really mean anything, theoretically deg/sec
double last_estop = millis();
static unsigned long prev_publish_time;
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
LegType fl_leg = FL;
LegType fr_leg = FR;
LegType bl_leg = BL;
LegType br_leg = BR;
LegJoints FL_ = LegJoints(fl_leg);
LegJoints FR_ = LegJoints(fr_leg);
LegJoints BL_ = LegJoints(bl_leg);
LegJoints BR_ = LegJoints(br_leg);
ros_srl::ROSSerial ros_serial;

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

void command_servos(const LegJoints & legjoint)
{
  int leg = -1;

  if (legjoint.legtype == FL)
  {
    leg = 0;
  } else if (legjoint.legtype == FR)
  {
    leg = 1;
  } else if (legjoint.legtype == BL)
  {
    leg = 2;
  } else if (legjoint.legtype == BR)
  {
    leg = 3;
  }


  double shoulder_home = (*Shoulders[leg]).return_home();
  double elbow_home = (*Elbows[leg]).return_home();
  double wrist_home = (*Wrists[leg]).return_home();

  double Shoulder_angle = util.angleConversion(legjoint.shoulder, shoulder_home, legjoint.legtype, Shoulder);
  double Elbow_angle = util.angleConversion(legjoint.elbow, elbow_home, legjoint.legtype, Elbow);
  double Wrist_angle = util.angleConversion(legjoint.wrist, wrist_home, legjoint.legtype, Wrist);

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

  pinMode(ledPin, OUTPUT);

  ik.Initialize(0.04, 0.1, 0.1);

  // Shoulders
  FL_Shoulder.Initialize(2, 135, 135, 0.0, FL, Shoulder);  // 0 | 
  FL_Shoulder.SetGoal(135, max_speed / 5.0);
  FR_Shoulder.Initialize(3, 135, 135, 0.0, FR, Shoulder); // 1 | 
  FR_Shoulder.SetGoal(135, max_speed / 5.0);
  BL_Shoulder.Initialize(4, 135, 135, 0.0, BL, Shoulder);  // 2 | 
  BL_Shoulder.SetGoal(135, max_speed / 5.0);
  BR_Shoulder.Initialize(5, 135, 135, 0.0, BR, Shoulder);  // 3 | 
  BR_Shoulder.SetGoal(135, max_speed / 5.0);

  //Elbows
  FL_Elbow.Initialize(6, 135, 135, 0.0, FL, Elbow);  // 4 | 135  mid - 0 in front - 270 behind
  FL_Elbow.SetGoal(135, max_speed / 5.0);
  FR_Elbow.Initialize(7, 135, 135, 0.0, FR, Elbow); // 5 | 135  mid - 0 in behind - 270 in front
  FR_Elbow.SetGoal(135, max_speed / 5.0);
  BL_Elbow.Initialize(8, 135, 135, 0.0, BL, Elbow);  // 6 | 135  mid - 0 in front - 270 behind
  BL_Elbow.SetGoal(135, max_speed / 5.0);
  BR_Elbow.Initialize(9, 135, 135, 0.0, BR, Elbow); // 7 | 135  mid - 0 in behind - 270 in front
  BR_Elbow.SetGoal(135, max_speed / 5.0);

  //Wrists
  FL_Wrist.Initialize(10, 90, 90, 0.0, FL, Wrist);  // 8 | 90 straight - 270 bent in
  FL_Wrist.SetGoal(180, max_speed / 5.0);
  FR_Wrist.Initialize(11, 90, 90, 0.0, FR, Wrist); // 9 | 180 straight - 0 bent in
  FR_Wrist.SetGoal(90, max_speed / 5.0);
  BL_Wrist.Initialize(12, 180, 180, 0.0, BL, Wrist); // 10 | 90 straight - 270 bent in
  BL_Wrist.SetGoal(180, max_speed / 5.0);
  BR_Wrist.Initialize(13, 90, 90, 0.0, BR, Wrist); // 11 | 180 straight - 0 bent in
  BR_Wrist.SetGoal(90, max_speed / 5.0);

  // Contact Sensors
  FL_sensor.Initialize(A9, 17);
  FR_sensor.Initialize(A8, 16);
  BL_sensor.Initialize(A7, 15);
  BR_sensor.Initialize(A6, 14);

  // IMU
  imu_sensor.Initialize();

  last_estop = millis();

  prev_publish_time = micros();
}

// THIS LOOPS FOREVER
void loop()
{
  // CHECK SENSORS AND SEND INFO
  // CONTACT
  if ((micros() - prev_publish_time) >= 20000)
  {
    ros_serial.publishContacts(FL_sensor.isTriggered(), FR_sensor.isTriggered(), BL_sensor.isTriggered(), BR_sensor.isTriggered());

    //IMU
    if (imu_sensor.available())
    {
      imu::Vector<3> eul = imu_sensor.GetEuler();
      imu::Vector<3> acc = imu_sensor.GetAcc();
      imu::Vector<3> gyro = imu_sensor.GetGyro();

      ros_serial.publishIMU(eul, acc, gyro);
    }

    prev_publish_time = micros();
  }

  if(!ESTOPPED){
    update_servos();
  } else {
    detach_servos();
  }
  update_sensors();
  
  // Command Servos
  ros_serial.returnJoints(FL_, FR_, BL_, BR_);
  command_servos(FL_);
  command_servos(FR_);
  command_servos(BL_);
  command_servos(BR_);

  // Update ROS Node (spinOnce etc...)
  ros_serial.run();
}
