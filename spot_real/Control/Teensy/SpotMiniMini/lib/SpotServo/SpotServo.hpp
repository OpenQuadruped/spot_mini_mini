#ifndef SPOTSERVO_INCLUDE_GUARD_HPP
#define SPOTSERVO_INCLUDE_GUARD_HPP
/// \file
/// \brief Servo Speed Control Library. Adapted from https://github.com/adham-elarabawy/OpenQuadruped
#include <Servo.h>
#include <Arduino.h>
// #include <cmath>

enum LegType {FL, FR, BL, BR};
enum JointType {Shoulder, Elbow, Wrist};

/// \brief SpotServo class responsible servo control
class SpotServo
{

public:
    // using default constructor

    /// \brief Initialize parameters
    /// \param servo_pin: pin number on Teensy 4.0 (see dataseet)
    /// \param home_angle_: default joint angle
    /// \param offset_: motor position offset (due to mechanical fit issues)
    /// \param leg_type_: Front Left, Front Right, Back Left, or Back Right leg (see enum)
    /// \param joint_type_: Shoulder, Elbow or Wrist (see enum)
    void Initialize(int & servo_pin, double & home_angle_, double & offset_, LegType & leg_type_, JointType & joint_type_);

    /// \brief Commands a motor to move to a certain goal at a certain speed
    /// \param goal_pose_: the desired motor position in degrees
    /// \param desired_speed_: the desired motor speed (deg/sec) while reaching this goal
    void SetGoal(double & goal_pose_, double & desired_speed_);

    /// \brief Return the interpolated position (time-step is small enough that this is mostly accurate)
    /// \returns: motor_pose_est
    double GetPoseEstimate();

    /// \brief Perform one motor update, potentially sending a new motor command, and updating the time-step.
    void update_clk();

    /// \brief Detach the motor
    void detach();

private:
    // Intrinsic Parameters
    Servo servo;
    // time elapsed since last servo update
    double last_actuated = 0.0;
    // error threshold for servo position
    double error_threshold = 0.5;
    // maximum servo angle (minimum is 0)
    double control_range = 270.0;
    // loop period (milisec)
    double wait_time = 1.0;
    // motor position offset (due to mechanical fit issues)
    int offset = 0;
    // Leg Type (see enum)
    LegType leg_type;
    // Joint Type (see enum)
    JointType joint_type;
    // default angle
    double home_angle = 0.0;

    // Changeable Parameters
    double goal_pose = 0.0; // deg
    double current_pose = 0.0; // deg
    double desired_speed = 0.0; // deg/sec
};
    

#endif