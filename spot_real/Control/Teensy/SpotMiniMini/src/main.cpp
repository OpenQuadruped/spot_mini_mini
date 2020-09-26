/// \file
/// \brief Teensy Main.

#include <Arduino.h>
#include <SpotServo.hpp>
#include <ContactSensor.hpp>
#include <Kinematics.hpp>
#include <Utilities.hpp>
#include <IMU.hpp>
#include <Servo.h>
#include <ROSSerial.hpp>

#define DEBUGSERIAL Serial

// NOTE: IMPORTANT - CALIBRATION VS RUN PARAMS
/* Instructions:
        - When assembling your servos, use NOMINAL_PWM mode, and be sure to enter the
          appropriate degree and pwm range too.
        - Use servo_calibration.launch on your Ubuntu machine running ROS Melodic and
          hence the servo_calibrator service to send PWM values to each joint, and then
          enter the resultant values in the Initialize() method of the SpotServo class.
        - STRAIGHT_LEGS, LIEDOWN, and PERPENDICULAR_LEGS are used to validate your calibraton.
        - RUN is used when you are finished calibrating, and are ready to run normal operations.
*/
enum MODE {NOMINAL_PWM, STRAIGHT_LEGS, LIEDOWN, PERPENDICULAR_LEGS, RUN};
MODE spot_mode = RUN;

bool ESTOPPED = false;
int viewing_speed = 400; // doesn't really mean anything, theoretically deg/sec
int walking_speed = 1500; // doesn't really mean anything, theoretically deg/sec
double last_estop = millis();
static unsigned long prev_publish_time;
const int ledPin = 13;


SpotServo FL_Shoulder, FL_Elbow, FL_Wrist, FR_Shoulder, FR_Elbow, FR_Wrist, BL_Shoulder, BL_Elbow, BL_Wrist, BR_Shoulder, BR_Elbow, BR_Wrist;
ContactSensor FL_sensor, FR_sensor, BL_sensor, BR_sensor;
SpotServo * Shoulders[4] = {&FL_Shoulder, &FR_Shoulder, &BL_Shoulder, &BR_Shoulder};
SpotServo * Elbows[4] = {&FL_Elbow, &FR_Elbow, &BL_Elbow, &BR_Elbow};
SpotServo * Wrists[4] = {&FL_Wrist, &FR_Wrist, &BL_Wrist, &BR_Wrist};
SpotServo * AllServos[12] = {&FL_Shoulder, &FL_Elbow, &FL_Wrist, &FR_Shoulder, &FR_Elbow, &FR_Wrist, &BL_Shoulder, &BL_Elbow, &BL_Wrist, &BR_Shoulder, &BR_Elbow, &BR_Wrist};
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

void command_servos(const LegJoints & legjoint, const bool & step_or_view = true)
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
  double Elbow_angle = legjoint.elbow;
  double Wrist_angle = legjoint.wrist;

  double s_dist = abs(Shoulder_angle - (*Shoulders[leg]).GetPoseEstimate());
  double e_dist = abs(Elbow_angle - (*Elbows[leg]).GetPoseEstimate());
  double w_dist = abs(Wrist_angle - (*Wrists[leg]).GetPoseEstimate());

  double scaling_factor = util.max(s_dist, e_dist, w_dist);

  s_dist /= scaling_factor;
  e_dist /= scaling_factor;
  w_dist /= scaling_factor;

  double s_speed = 0.0;
  double e_speed = 0.0;
  double w_speed = 0.0;

  if (step_or_view)
  {
    s_speed = viewing_speed * s_dist;
    s_speed = max(s_speed, viewing_speed / 10.0);
    e_speed = viewing_speed * e_dist;
    e_speed = max(e_speed, viewing_speed / 10.0);
    w_speed = viewing_speed * w_dist;
    w_speed = max(w_speed, viewing_speed / 10.0);
  } else
  {
    s_speed = walking_speed * s_dist;
    e_speed = walking_speed * e_dist;
    w_speed = walking_speed * w_dist;
  }

  (*Shoulders[leg]).SetGoal(Shoulder_angle, s_speed, step_or_view);
  (*Elbows[leg]).SetGoal(Elbow_angle, e_speed, step_or_view);
  (*Wrists[leg]).SetGoal(Wrist_angle, w_speed, step_or_view);
  
}

void update_sensors()
{
    FL_sensor.update_clk();
    FR_sensor.update_clk();
    BL_sensor.update_clk();
    BR_sensor.update_clk();
}

void set_stance(const double & f_shoulder_stance = 0.0, const double & f_elbow_stance = 0.0, const double & f_wrist_stance = 0.0,
                const double & r_shoulder_stance = 0.0, const double & r_elbow_stance = 0.0, const double & r_wrist_stance = 0.0)
{
  // Legs
  FL_.FillLegJoint(f_shoulder_stance, f_elbow_stance, f_wrist_stance);
  FR_.FillLegJoint(f_shoulder_stance, f_elbow_stance, f_wrist_stance);
  BL_.FillLegJoint(r_shoulder_stance, r_elbow_stance, r_wrist_stance);
  BR_.FillLegJoint(r_shoulder_stance, r_elbow_stance, r_wrist_stance);

  command_servos(FL_);
  command_servos(FR_);
  command_servos(BL_);
  command_servos(BR_);

  // Loop until goal reached - check BR Wrist (last one)
  while (!BR_Wrist.GoalReached())
  {
    update_servos();
  }
}


// Set servo pwm values to nominal assuming 500~2500 range and 270 degree servos.
void nominal_servo_pwm(const double & servo_range = 270, const int & min_pwm = 500, const int & max_pwm = 2500)
{
  // Attach motors for assembly
  // Shoulders
  FL_Shoulder.AssemblyInit(2, min_pwm, max_pwm);
  FR_Shoulder.AssemblyInit(5, min_pwm, max_pwm);
  BL_Shoulder.AssemblyInit(8, min_pwm, max_pwm);
  BR_Shoulder.AssemblyInit(11, min_pwm, max_pwm);
  
  //Elbows
  FL_Elbow.AssemblyInit(3, min_pwm, max_pwm);
  FR_Elbow.AssemblyInit(6, min_pwm, max_pwm);
  BL_Elbow.AssemblyInit(9, min_pwm, max_pwm);
  BR_Elbow.AssemblyInit(12, min_pwm, max_pwm);

  //Wrists
  FL_Wrist.AssemblyInit(4, min_pwm, max_pwm);
  FR_Wrist.AssemblyInit(7, min_pwm, max_pwm);
  BL_Wrist.AssemblyInit(10, min_pwm, max_pwm);
  BR_Wrist.AssemblyInit(13, min_pwm, max_pwm);

  // halfway for shoulder and elbow
  int halfway_pulse = round(0.5 * (max_pwm - min_pwm) + min_pwm); // 1500
  int shoulder_pulse = halfway_pulse; // 1500
  int elbow_pulse = halfway_pulse; // 1500

  // wrists need roughly 180
  int remaining_range = round(((180.0 - (servo_range / 2.0)) / double(servo_range)) * (max_pwm - min_pwm));
  int left_wrist_pulse = halfway_pulse - remaining_range; // 1167
  int right_wrist_pulse = halfway_pulse + remaining_range; // 1833
  //FL
  AllServos[0]->writePulse(shoulder_pulse);
  AllServos[1]->writePulse(elbow_pulse);
  AllServos[2]->writePulse(left_wrist_pulse);
  //FR
  AllServos[3]->writePulse(shoulder_pulse);
  AllServos[4]->writePulse(elbow_pulse);
  AllServos[5]->writePulse(right_wrist_pulse);
  //BL
  AllServos[6]->writePulse(shoulder_pulse);
  AllServos[7]->writePulse(elbow_pulse);
  AllServos[8]->writePulse(left_wrist_pulse);
  //BR
  AllServos[9]->writePulse(shoulder_pulse);
  AllServos[10]->writePulse(elbow_pulse);
  AllServos[11]->writePulse(right_wrist_pulse);
}

void run_sequence()
{
  // Move to Crouching Stance
  delay(2000);
  double f_shoulder_stance = 0.0;
  double f_elbow_stance =  36.13;
  double f_wrist_stance = -75.84;
  double r_shoulder_stance = 0.0;
  double r_elbow_stance =  36.13;
  double r_wrist_stance = -75.84;
  set_stance(f_shoulder_stance, f_elbow_stance, f_wrist_stance, r_shoulder_stance, r_elbow_stance, r_wrist_stance);
}

void straight_calibration_sequence()
{
  set_stance();
}

void lie_calibration_sequence()
{

  // Move to Extended stance
  delay(2000);
  double shoulder_stance = 0.0;
  double elbow_stance =  90.0;
  double wrist_stance = -170.3;
  set_stance(shoulder_stance, elbow_stance, wrist_stance, shoulder_stance, elbow_stance, wrist_stance);

}

void perpendicular_calibration_sequence()
{

  // Move to Extended stance
  delay(2000);
  set_stance();
  // Move to Extended stance
  delay(10000);
  double shoulder_stance = 0.0;
  double elbow_stance =  90.0;
  double wrist_stance = -90.0;
  set_stance(shoulder_stance, elbow_stance, wrist_stance, shoulder_stance, elbow_stance, wrist_stance);

}

// THIS ONLY RUNS ONCE
void setup()
{

  Serial.begin(9600);

  // NOTE: See top of file for spot_mode explanation:
  // ONLY USE THIS MODE DURING ASSEMBLY
  if (spot_mode == NOMINAL_PWM)
  {
    // Neutral Setting
    nominal_servo_pwm();
    // Prevent Servo Updates
    ESTOPPED = true;
  } else 
  {

    // IK - unused
    ik.Initialize(0.04, 0.1, 0.1);

    // SERVOS: Pin, StandAngle, HomeAngle, Offset, LegType, JointType, min_pwm, max_pwm, min_pwm_angle, max_pwm_angle
    // Shoulders
    double shoulder_liedown = 0.0;
    FL_Shoulder.Initialize(2, 135 + shoulder_liedown, 135, -7.25, FL, Shoulder, 500, 2400); // 0 | 0: STRAIGHT | 90: OUT | -90 IN
    FR_Shoulder.Initialize(5, 135 - shoulder_liedown, 135, -5.5, FR, Shoulder, 500, 2400);  // 1 | 0: STRAIGHT | 90: IN  | -90 OUT
    BL_Shoulder.Initialize(8, 135 + shoulder_liedown, 135, 5.75, BL, Shoulder, 500, 2400);  // 2 | 0: STRAIGHT | 90: OUT | -90 IN
    BR_Shoulder.Initialize(11, 135 - shoulder_liedown, 135, -4.0, BR, Shoulder, 500, 2400); // 3 | 0: STRAIGHT | 90: IN  | -90 OUT
    
    //Elbows
    double elbow_liedown = 90.0;
    FL_Elbow.Initialize(3, elbow_liedown, 0, 0.0, FL, Elbow, 1410, 2062, 0.0, 90.0);        // 4 | 0: STRAIGHT | 90: BACK
    FR_Elbow.Initialize(6, elbow_liedown, 0, 0.0, FR, Elbow, 1408, 733, 0.0, 90.0);         // 5 | 0: STRAIGHT | 90: BACK
    BL_Elbow.Initialize(9, elbow_liedown, 0, 0.0, BL, Elbow, 1460, 2095, 0.0, 90.0);        // 6 | 0: STRAIGHT | 90: BACK
    BR_Elbow.Initialize(12, elbow_liedown, 0, 0.0, BR, Elbow, 1505, 850, 0.0, 90.0);        // 7 | 0: STRAIGHT | 90: BACK

    //Wrists
    double wrist_liedown = -160.0;
    FL_Wrist.Initialize(4, wrist_liedown, 0, 0.0, FL, Wrist, 1755, 2320, -90.0, -165.0);    // 8 | 0: STRAIGHT | -90: FORWARD
    FR_Wrist.Initialize(7, wrist_liedown, 0, 0.0, FR, Wrist, 1805, 1150, 0.0, -90.0);       // 9 | 0: STRAIGHT | -90: FORWARD
    BL_Wrist.Initialize(10, wrist_liedown, 0, 0.0, BL, Wrist, 1100, 1733, 0.0, -90.0);     // 10 | 0: STRAIGHT | -90: FORWARD
    BR_Wrist.Initialize(13, wrist_liedown, 0, 0.0, BR, Wrist, 1788, 1153, 0.0, -90.0);     // 11 | 0: STRAIGHT | -90: FORWARD

    // Contact Sensors
    FL_sensor.Initialize(A9, 17);
    FR_sensor.Initialize(A8, 16);
    BL_sensor.Initialize(A7, 15);
    BR_sensor.Initialize(A6, 14);

    // IMU
    imu_sensor.Initialize();

    // NOTE: See top of file for spot_mode explanation:
    if (spot_mode == STRAIGHT_LEGS)
    {
      straight_calibration_sequence();
    } else if (spot_mode == LIEDOWN)
    {
      lie_calibration_sequence();
    } else if (spot_mode == PERPENDICULAR_LEGS)
    {
      perpendicular_calibration_sequence();
    } else
    {
      run_sequence();
    }
  }

  last_estop = millis();

  prev_publish_time = micros();
}

// THIS LOOPS FOREVER
void loop()
{
  // CHECK SENSORS AND SEND INFO
  // CONTACT
  // 1'000'000 / 20'000 = 50hz
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

  // Only allow controller commands if not E-STOPPED
  // Direct pulse commands from calibration still allowed.
  if(!ESTOPPED)
  {
    update_servos();
  }

  // Update Sensors
  update_sensors();
  
  // Command Servos
  if (ros_serial.jointsInputIsActive())
  {
    bool step_or_view = false;
    ros_serial.returnJoints(FL_, FR_, BL_, BR_, step_or_view);
    command_servos(FL_, step_or_view);
    command_servos(FR_, step_or_view);
    command_servos(BL_, step_or_view);
    command_servos(BR_, step_or_view);
  } else if (ros_serial.jointsPulseIsActive())
  {
    int servo_num = ros_serial.returnServoNum();
    int pulse = ros_serial.returnPulse();

    if (servo_num > -1 and servo_num < 12)
    {
      AllServos[servo_num]->writePulse(pulse);
    }

    ros_serial.resetPulseTopic();
  }

  // Update ROS Node (spinOnce etc...)
  ros_serial.run();
}
