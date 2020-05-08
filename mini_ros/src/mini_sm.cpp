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

// Global Vars
mini::Minitaur minitaur = mini::Minitaur();
bool teleop_flag = false;
bool motion_flag = false;

void teleop_callback(const geometry_msgs::Twist &tw)
{ 
  /// \brief cmd_vel subscriber callback. Records commanded twist
  ///
  /// \param tw (geometry_msgs::Twist): the commanded linear and angular velocity
  /** 
  * This function runs every time we get a geometry_msgs::Twist message on the "cmd_vel" topic.
  * We generally use the const <message>ConstPtr &msg syntax to prevent our node from accidentally
  * changing the message, in the case that another node is also listening to it.
  */
  
  minitaur.update_command(tw.linear.x, tw.angular.z);
}


int main(int argc, char** argv)
/// The Main Function ///
{
    ROS_INFO("STARTING NODE: Minitaur State Machine");

    // Vars
    double frequency = 60;

    ros::init(argc, argv, "mini_sm_node"); // register the node on ROS
    ros::NodeHandle nh; // get a handle to ROS
    ros::NodeHandle nh_("~"); // get a handle to ROS
    // Parameters
    nh_.getParam("frequency", frequency);

    // Init Subscriber
    ros::Subscriber teleop_sub = nh.subscribe("teleop", 1, teleop_callback);
    // Init Command Publisher
    ros::Publisher mini_pub = nh.advertise<mini_ros::MiniCmd>("mini_cmd", 1);

    // Init MiniCmd
    mini_ros::MiniCmd mini_cmd;
    // Placeholder
    mini_cmd.velocity = 0.0;
    mini_cmd.rate = 0.0;
    mini_cmd.motion = "Stop";

    ros::Rate rate(frequency);
    // Main While
    while (ros::ok())
    {
        ros::spinOnce();

        mini::MiniCommand cmd = minitaur.return_command();

        if (!motion_flag)
        {
          mini_cmd.velocity = cmd.velocity;
          mini_cmd.rate = cmd.rate;
          // Now convert enum to string
          if (cmd.motion == mini::Forward)
          {
            mini_cmd.motion = "Forward";
          } else if (cmd.motion == mini::Backward)
          {
            mini_cmd.motion = "Backward";
          } else if (cmd.motion == mini::Left)
          {
            mini_cmd.motion = "Left";
          } else if (cmd.motion == mini::Right)
          {
            mini_cmd.motion = "Right";
          } else if (cmd.motion == mini::CW)
          {
            mini_cmd.motion = "CW";
          } else if (cmd.motion == mini::CCW)
          {
            mini_cmd.motion = "CCW";
          } else if (cmd.motion == mini::Stop)
          {
            mini_cmd.motion = "Stop";
          } else if (cmd.motion == mini::Recover)
          {
            mini_cmd.motion = "Recover";
          } else if (cmd.motion == mini::ForwardLeft)
          {
            mini_cmd.motion = "ForwardLeft";
          } else if (cmd.motion == mini::ForwardRight)
          {
            mini_cmd.motion = "ForwardRight";
          }

        } else
        {
          mini_cmd.velocity = 0.0;
          mini_cmd.rate = 0.0;
          mini_cmd.motion = "Stop";
        }

        // Now publish
        mini_pub.publish(mini_cmd);
        motion_flag = false;
        
        rate.sleep();
    }

    return 0;
}