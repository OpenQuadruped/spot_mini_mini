#ifndef SPOTSERVO_INCLUDE_GUARD_HPP
#define SPOTSERVO_INCLUDE_GUARD_HPP
/// \file
/// \brief Servo Speed Control Library
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
    void Initialize(const int & servo_pin, const double & stand_angle_, const double & home_angle_, const double & offset_, const LegType & leg_type_, const JointType & joint_type_);

    /// \brief Commands a motor to move to a certain goal instantly when stepping, and slowly when viewing
    /// \param goal_pose_: the desired motor position in degrees
    /// \param desired_speed_: the desired motor speed (deg/sec) while reaching this goal
    /// \param step_or_view_: operation mode, stepping or viewing.
    void SetGoal(const double & goal_pose_, const double & desired_speed_, const bool & step_or_view_ = true);

    /// \brief returns joint_type
    /// \returns: joint_type
    JointType return_joint_type();

    /// \brief returns leg_type
    /// \returns: leg_type
    LegType return_legtype();

    /// \brief returns this servo's home angle
    /// \returns: home_angle
    double return_home();

    /// \brief Return the interpolated position (time-step is small enough that this is mostly accurate)
    /// \returns: motor_pose_est
    double GetPoseEstimate();

    /// \brief Perform one motor update, potentially sending a new motor command, and updating the time-step.
    void update_clk();

    /// \brief Check if goal position reached
    bool GoalReached();

    /// \biref Actuate motors
    void actuate();



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
    double stand_angle = 0.0;

    // Changeable Parameters
    double goal_pose = 0.0; // deg
    double current_pose = 0.0; // deg
    double desired_speed = 0.0; // deg/sec

    // Servo's PWM range (usec)
    // NOTE: This is unique to your servos, VERY IMPORTANT
    int min_pwm = 500;
    int max_pwm = 2500;

    // Interpolation to convert from deg to usec
    double conv_slope = 0.0;
    double conv_intcpt = 0.0;

    // False is step, True is view
    bool step_or_view = false;
};
    

#endif