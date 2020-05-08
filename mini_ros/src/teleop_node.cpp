/// \file
/// \brief Draws Each Obstacle in RViz using MarkerArrays
///
/// PARAMETERS:
/// PUBLISHES:
/// SUBSCRIBES:
/// FUNCTIONS:

#include <ros/ros.h>

#include <math.h>
#include <string>
#include <vector>

#include <mini_ros/minitaur.hpp>
#include <mini_ros/teleop.hpp>
#include "mini_ros/MiniCmd.h"

#include <functional>  // To use std::bind


int main(int argc, char** argv)
/// The Main Function ///
{
    ROS_INFO("STARTING NODE: Teleoperation");

    // Vars
    double frequency = 60;
    int linear = 1;
    int angular = 2;
    double l_scale = 2.0;
    double a_scale = 2.0;

    ros::init(argc, argv, "teleop_node"); // register the node on ROS
    ros::NodeHandle nh; // get a handle to ROS
    ros::NodeHandle nh_("~"); // get a handle to ROS
    // Parameters
    nh_.getParam("frequency", frequency);
    nh_.getParam("axis_linear", linear);
    nh_.getParam("axis_angular", angular);
    nh_.getParam("scale_linear", l_scale);
    nh_.getParam("scale_angular", a_scale);

    tele::Teleop teleop = tele::Teleop(linear, angular, l_scale, a_scale);

    // Init Command Publisher
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("teleop", 1);

    // Init Subscriber (also handles pub)
    // TODO: Figure out how to use std::bind properly
    // ros::Subscriber joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 1, std::bind(&tele::Teleop::joyCallback, std::placeholders::_1, vel_pub), &teleop);
    ros::Subscriber joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 1, &tele::Teleop::joyCallback, &teleop);

    ros::Rate rate(frequency);
    // Main While
    while (ros::ok())
    {
        ros::spinOnce();

        if (!teleop.return_trigger())
        {
          vel_pub.publish(teleop.return_twist());
        } else
        {
          // TODO: Switch Service
        }
        
        
        rate.sleep();
    }

    return 0;
}