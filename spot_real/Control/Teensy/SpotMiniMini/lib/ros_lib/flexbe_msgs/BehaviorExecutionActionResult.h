#ifndef _ROS_flexbe_msgs_BehaviorExecutionActionResult_h
#define _ROS_flexbe_msgs_BehaviorExecutionActionResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalStatus.h"
#include "flexbe_msgs/BehaviorExecutionResult.h"

namespace flexbe_msgs
{

  class BehaviorExecutionActionResult : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalStatus _status_type;
      _status_type status;
      typedef flexbe_msgs::BehaviorExecutionResult _result_type;
      _result_type result;

    BehaviorExecutionActionResult():
      header(),
      status(),
      result()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->status.serialize(outbuffer + offset);
      offset += this->result.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->status.deserialize(inbuffer + offset);
      offset += this->result.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "flexbe_msgs/BehaviorExecutionActionResult"; };
    const char * getMD5(){ return "76e505f84da88685989704705a0d9846"; };

  };

}
#endif
