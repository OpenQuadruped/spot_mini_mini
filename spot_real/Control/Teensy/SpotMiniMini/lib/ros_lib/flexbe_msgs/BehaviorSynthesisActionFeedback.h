#ifndef _ROS_flexbe_msgs_BehaviorSynthesisActionFeedback_h
#define _ROS_flexbe_msgs_BehaviorSynthesisActionFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalStatus.h"
#include "flexbe_msgs/BehaviorSynthesisFeedback.h"

namespace flexbe_msgs
{

  class BehaviorSynthesisActionFeedback : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalStatus _status_type;
      _status_type status;
      typedef flexbe_msgs::BehaviorSynthesisFeedback _feedback_type;
      _feedback_type feedback;

    BehaviorSynthesisActionFeedback():
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

    const char * getType(){ return "flexbe_msgs/BehaviorSynthesisActionFeedback"; };
    const char * getMD5(){ return "74b2c8a9bddfff7e1b57abd9d9fb5b18"; };

  };

}
#endif
