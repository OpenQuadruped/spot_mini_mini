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
#include "std_srvs/Empty.h"
#include "std_msgs/Bool.h"

// Global Vars
spot::Spot spot_mini = spot::Spot();
bool teleop_flag = false;
bool motion_flag = false;
bool ESTOP = false;
// Init Time
ros::Time current_time;
ros::Time last_time;


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
  
  spot_mini.update_command(tw.linear.x, tw.linear.y, tw.linear.z, tw.angular.z, tw.angular.x, tw.angular.y);
}

void estop_callback(const std_msgs::Bool &estop)
{ 
  if (estop.data)
  {
    spot_mini.update_command(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    motion_flag = true;
    if (!ESTOP)
    {
      ROS_ERROR("ENGAGING MANUAL E-STOP!");
      ESTOP = true;
    } else
    {
      ROS_WARN("DIS-ENGAGING MANUAL E-STOP!");
      ESTOP = false;
    }
  }

  last_time = ros::Time::now();
}


bool swm_callback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
/// Switches the Movement mode from FB (Forward/Backward) to LR (Left/Right)
/// and vice versa
{
    spot_mini.switch_movement();
    motion_flag = true;
    return true;
}


int main(int argc, char** argv)
/// The Main Function ///
{
    ROS_INFO("STARTING NODE: spot_mini State Machine");

    // Vars
    double frequency = 5;
    // Seconds for timeout
    double timeout = 1.0;

    ros::init(argc, argv, "mini_sm_node"); // register the node on ROS
    ros::NodeHandle nh; // get a handle to ROS
    ros::NodeHandle nh_("~"); // get a handle to ROS
    // Parameters
    nh_.getParam("frequency", frequency);

    // Init Subscriber
    ros::Subscriber teleop_sub = nh.subscribe("teleop", 1, teleop_callback);
    ros::Subscriber estop_sub = nh.subscribe("estop", 1, estop_callback);
    // Init Command Publisher
    ros::Publisher mini_pub = nh.advertise<mini_ros::MiniCmd>("mini_cmd", 1);

    // Init Switch Movement Service Server
    ros::ServiceServer switch_movement_server = nh.advertiseService("switch_movement", swm_callback);

    // Init MiniCmd
    mini_ros::MiniCmd mini_cmd;
    // Placeholder
    mini_cmd.x_velocity = 0.0;
    mini_cmd.y_velocity = 0.0;
    mini_cmd.rate = 0.0;
    mini_cmd.roll = 0.0;
    mini_cmd.pitch = 0.0;
    mini_cmd.yaw = 0.0;
    mini_cmd.z = 0.0;
    mini_cmd.faster = 0.0;
    mini_cmd.slower = 0.0;
    mini_cmd.motion = "Stop";
    mini_cmd.movement = "Stepping";

    ros::Rate rate(frequency);
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    // Main While
    while (ros::ok())
    {
        ros::spinOnce();
        current_time = ros::Time::now();

        spot::SpotCommand cmd = spot_mini.return_command();

        // Condition for sending non-stop command
        if (!motion_flag and !(current_time.toSec() - last_time.toSec() > timeout) and !ESTOP)
        {
          mini_cmd.x_velocity = cmd.x_velocity;
          mini_cmd.y_velocity = cmd.y_velocity;
          mini_cmd.rate = cmd.rate;
          mini_cmd.roll = cmd.roll;
          mini_cmd.pitch = cmd.pitch;
          mini_cmd.yaw = cmd.yaw;
          mini_cmd.z = cmd.z;
          mini_cmd.faster = cmd.faster;
          mini_cmd.slower = cmd.slower;
          // Now convert enum to string
          // Motion
          if (cmd.motion == spot::Go)
          {
            mini_cmd.motion = "Go";
          } else
          {
            mini_cmd.motion = "Stop";
          }
          // Movement
          if (cmd.movement == spot::Stepping)
          {
            mini_cmd.movement = "Stepping";
          } else
          {
            mini_cmd.movement = "Viewing";
          }

        } else
        {
          mini_cmd.x_velocity = 0.0;
          mini_cmd.y_velocity = 0.0;
          mini_cmd.rate = 0.0;
          mini_cmd.roll = 0.0;
          mini_cmd.pitch = 0.0;
          mini_cmd.yaw = 0.0;
          mini_cmd.z = 0.0;
          mini_cmd.faster = 0.0;
          mini_cmd.slower = 0.0;
          mini_cmd.motion = "Stop";
        }

        if (current_time.toSec() - last_time.toSec() > timeout)
        {
          ROS_ERROR("TIMEOUT...ENGAGING E-STOP!");
        }

        // Now publish
        mini_pub.publish(mini_cmd);
        motion_flag = false;
        
        rate.sleep();
    }

    return 0;
}