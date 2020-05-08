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
        // \param
        Teleop(const int & linear, const int & angular, const double & l_scale, const double & a_scale);

    private:

        // \brief Takes a Joy messages and converts it to linear and angular velocity (Twist)
        // \param joy: sensor_msgs describing Joystick inputs
        // \returns: resultant Twist due to joy command
        geometry_msgs::Twist joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

        int linear_, angular_;
        double l_scale_, a_scale_;
    };
    
}

#endif
