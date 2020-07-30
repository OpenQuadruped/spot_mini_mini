#ifndef _ROS_moveit_msgs_MoveGroupResult_h
#define _ROS_moveit_msgs_MoveGroupResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "moveit_msgs/MoveItErrorCodes.h"
#include "moveit_msgs/RobotState.h"
#include "moveit_msgs/RobotTrajectory.h"

namespace moveit_msgs
{

  class MoveGroupResult : public ros::Msg
  {
    public:
      typedef moveit_msgs::MoveItErrorCodes _error_code_type;
      _error_code_type error_code;
      typedef moveit_msgs::RobotState _trajectory_start_type;
      _trajectory_start_type trajectory_start;
      typedef moveit_msgs::RobotTrajectory _planned_trajectory_type;
      _planned_trajectory_type planned_trajectory;
      typedef moveit_msgs::RobotTrajectory _executed_trajectory_type;
      _executed_trajectory_type executed_trajectory;
      typedef double _planning_time_type;
      _planning_time_type planning_time;

    MoveGroupResult():
      error_code(),
      trajectory_start(),
      planned_trajectory(),
      executed_trajectory(),
      planning_time(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->error_code.serialize(outbuffer + offset);
      offset += this->trajectory_start.serialize(outbuffer + offset);
      offset += this->planned_trajectory.serialize(outbuffer + offset);
      offset += this->executed_trajectory.serialize(outbuffer + offset);
      union {
        double real;
        uint64_t base;
      } u_planning_time;
      u_planning_time.real = this->planning_time;
      *(outbuffer + offset + 0) = (u_planning_time.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_planning_time.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_planning_time.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_planning_time.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_planning_time.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_planning_time.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_planning_time.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_planning_time.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->planning_time);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->error_code.deserialize(inbuffer + offset);
      offset += this->trajectory_start.deserialize(inbuffer + offset);
      offset += this->planned_trajectory.deserialize(inbuffer + offset);
      offset += this->executed_trajectory.deserialize(inbuffer + offset);
      union {
        double real;
        uint64_t base;
      } u_planning_time;
      u_planning_time.base = 0;
      u_planning_time.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_planning_time.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_planning_time.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_planning_time.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_planning_time.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_planning_time.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_planning_time.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_planning_time.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->planning_time = u_planning_time.real;
      offset += sizeof(this->planning_time);
     return offset;
    }

    const char * getType(){ return "moveit_msgs/MoveGroupResult"; };
    const char * getMD5(){ return "34098589d402fee7ae9c3fd413e5a6c6"; };

  };

}
#endif
