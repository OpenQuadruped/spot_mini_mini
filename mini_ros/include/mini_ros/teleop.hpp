#ifndef TELEOP_INCLUDE_GUARD_HPP
#define TELEOP_INCLUDE_GUARD_HPP
/// \file
/// \brief Teleoperation Library that converts Joystick commands to motion
#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

namespace tele
{

    // \brief Teleop class responsible for convertick Joystick commands into linear and angular velocity
    class Teleop
    {
    public:
        // \brief Teleop constructor that defines the axes used for control and sets their scaling factor
        // \param linear: joystick axis assigned to linear velocity
        // \param angular: joystick axis assigned to angular velocity
        // \param l_scale: scaling factor for linear velocity
        // \param a_scale: scaling factor for angular velocity
        // \param sw: button for switch_trigger
        // \param es: button for ESTOP
        Teleop(const int & linear, const int & angular, const double & l_scale, const double & a_scale, const int & sw, const int & es);

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

    private:

        int linear_ = 1;
        int angular_= 2;
        int sw_ = 0;
        int es_ = 1;
        double l_scale_, a_scale_;
        geometry_msgs::Twist twist;
        bool switch_trigger = false;
        bool ESTOP = false;
    };
    
}

#endif
