#ifndef TELEOP_INCLUDE_GUARD_HPP
#define TELEOP_INCLUDE_GUARD_HPP
/// \file
/// \brief Teleoperation Library that converts Joystick commands to motion
#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include "mini_ros/JoyButtons.h"

namespace tele
{

    // \brief Teleop class responsible for convertick Joystick commands into linear and angular velocity
    class Teleop
    {
    public:
        // \brief Teleop constructor that defines the axes used for control and sets their scaling factor
        // \param linear_x: joystick axis assigned to linear velocity (x)
        // \param linear_y: joystick axis assigned to linear velocity (y)
        // \param linear_z: joystick axis assigned to robot height [overloading]
        // \param angular: joystick axis assigned to angular velocity
        // \param l_scale: scaling factor for linear velocity
        // \param a_scale: scaling factor for angular velocity
        // \param LB: left bottom bumper axis
        // \param RB: right bottom bumper axis
        // \param B_scale: scaling factor for bottom bumpers
        // \param LT: left top bumper button
        // \param RT: right top bumper button
        // \param UD: up/down key on arrow pad
        // \param LR: left/right key on arrow pad
        // \param sw: button for switch_trigger
        // \param es: button for ESTOP
        Teleop(const int & linear_x, const int & linear_y, const int & linear_z,
               const int & angular, const double & l_scale, const double & a_scale,
               const int & LB, const int & RB, const int & B_scale, const int & LT,
               const int & RT, const int & UD, const int & LR,
               const int & sw, const int & es);

        // \brief Takes a Joy messages and converts it to linear and angular velocity (Twist)
        // \param joy: sensor_msgs describing Joystick inputs
        void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

        // \brief returns the  most recently commanded Twist
        // \returns: Twist
        geometry_msgs::Twist return_twist();

        // \brief returns a boolean indicating whether the movement switch trigger has been pressed
        // \returns: switch_trigger(bool)
        bool return_trigger();

        // \brief returns whether the E-STOP has been pressed
        // \returns: ESTOP(bool)
        bool return_estop();

        /// \brief returns other joystick buttons triggers, arrow pad etc)
        mini_ros::JoyButtons return_buttons();

    private:
        // AXES ON JOYSTICK
        int linear_x_ = 0;
        int linear_y_ = 0;
        int linear_z_ = 0;
        int angular_= 0;
        int RB_ = 0;
        int LB_ = 0;
        // BUTTONS ON JOYSTICK
        int sw_ = 0;
        int es_ = 0;
        int RT_ = 0;
        int LT_ = 0;
        int UD_ = 0;
        int LR_ = 0;
        // AXIS SCALES
        double l_scale_, a_scale_, B_scale_;
        // TWIST
        geometry_msgs::Twist twist;
        // TRIGGERS
        bool switch_trigger = false;
        bool ESTOP = false;
        int updown = 0;
        int leftright = 0;
        bool left_bump = false;
        bool right_bump = false;
    };
    
}

#endif
