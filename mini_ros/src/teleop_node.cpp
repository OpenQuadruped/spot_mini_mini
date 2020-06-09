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

#include <mini_ros/spot.hpp>
#include <mini_ros/teleop.hpp>
#include "mini_ros/MiniCmd.h"
#include "std_msgs/Bool.h"
#include "std_srvs/Empty.h"
#include "mini_ros/JoyButtons.h"

#include <functional>  // To use std::bind


int main(int argc, char** argv)
/// The Main Function ///
{
    ROS_INFO("STARTING NODE: Teleoperation");

    // Vars
    double frequency = 60;
    int linear_x = 4;
    int linear_y = 3;
    int linear_z = 1;
    int angular = 0;
    int sw = 0;
    int es = 1;
    int RB = 5;
    int LB = 2;
    int RT = 5;
    int LT = 4;
    int UD = 7;
    int LR = 6;
    double l_scale = 1.0;
    double a_scale = 1.0;
    double B_scale = 1.0;
    double debounce_thresh = 0.15; // sec

    ros::init(argc, argv, "teleop_node"); // register the node on ROS
    ros::NodeHandle nh; // get a handle to ROS
    ros::NodeHandle nh_("~"); // get a handle to ROS
    // Parameters
    nh_.getParam("frequency", frequency);
    nh_.getParam("axis_linear_x", linear_x);
    nh_.getParam("axis_linear_y", linear_y);
    nh_.getParam("axis_linear_z", linear_z);
    nh_.getParam("axis_angular", angular);
    nh_.getParam("scale_linear", l_scale);
    nh_.getParam("scale_angular", a_scale);
    nh_.getParam("scale_bumper", B_scale);
    nh_.getParam("button_switch", sw);
    nh_.getParam("button_estop", es);
    nh_.getParam("rb", RB);
    nh_.getParam("lb", LB);
    nh_.getParam("rt", RT);
    nh_.getParam("lt", LT);
    nh_.getParam("updown", UD);
    nh_.getParam("leftright", LR);
    nh_.getParam("debounce_thresh", debounce_thresh);

    tele::Teleop teleop = tele::Teleop(linear_x, linear_y, linear_z, angular,
                                       l_scale, a_scale, LB, RB, B_scale, LT,
                                       RT, UD, LR, sw, es);

    // Init Switch Movement Server
    ros::ServiceClient switch_movement_client = nh.serviceClient<std_srvs::Empty>("switch_movement");
    ros::service::waitForService("switch_movement", -1);

    // Init ESTOP Publisher
    ros::Publisher estop_pub = nh.advertise<std_msgs::Bool>("estop", 1);
    // Init Command Publisher
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("teleop", 1);
    // Init Joy Button Publisher
    ros::Publisher jb_pub = nh.advertise<mini_ros::JoyButtons>("joybuttons", 1);

    // Init Subscriber (also handles pub)
    // TODO: Figure out how to use std::bind properly
    // ros::Subscriber joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 1, std::bind(&tele::Teleop::joyCallback, std::placeholders::_1, vel_pub), &teleop);
    ros::Subscriber joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 1, &tele::Teleop::joyCallback, &teleop);

    ros::Rate rate(frequency);

    // Record time for debouncing buttons
    ros::Time current_time = ros::Time::now();
    ros::Time last_time = ros::Time::now();


    // Main While
    while (ros::ok())
    {
        ros::spinOnce();
        current_time = ros::Time::now();

        std_msgs::Bool estop;
        estop.data = teleop.return_estop();

        if (estop.data and current_time.toSec() - last_time.toSec() >= debounce_thresh)
        {
            ROS_INFO("SENDING E-STOP COMMAND!");
            last_time = ros::Time::now();
        } else if (!teleop.return_trigger())
        {
          // Send Twist
          vel_pub.publish(teleop.return_twist());
          estop.data = 0;
        } else if (current_time.toSec() - last_time.toSec() >= debounce_thresh)
        {
          // Call Switch Service
          std_srvs::Empty e;
          switch_movement_client.call(e);
          estop.data = 0;
          last_time = ros::Time::now();
        }
        // pub buttons
        jb_pub.publish(teleop.return_buttons());

        estop_pub.publish(estop);
        
        
        rate.sleep();
    }

    return 0;
}