#ifndef _ROS_flexbe_msgs_BehaviorExecutionActionFeedback_h
#define _ROS_flexbe_msgs_BehaviorExecutionActionFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalStatus.h"
#include "flexbe_msgs/BehaviorExecutionFeedback.h"

namespace flexbe_msgs
{

  class BehaviorExecutionActionFeedback : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalStatus _status_type;
      _status_type status;
      typedef flexbe_msgs::BehaviorExecutionFeedback _feedback_type;
      _feedback_type feedback;

    BehaviorExecutionActionFeedback():
      header(),
      status(),
      feedback()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->status.serialize(outbuffer + offset);
      offset += this->feedback.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->status.deserialize(inbuffer + offset);
      offset += this->feedback.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "flexbe_msgs/BehaviorExecutionActionFeedback"; };
    const char * getMD5(){ return "b767b030e0462dae1168f66f2dd64853"; };

  };

}
#endif
