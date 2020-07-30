#ifndef _ROS_flexbe_msgs_BehaviorInputAction_h
#define _ROS_flexbe_msgs_BehaviorInputAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "flexbe_msgs/BehaviorInputActionGoal.h"
#include "flexbe_msgs/BehaviorInputActionResult.h"
#include "flexbe_msgs/BehaviorInputActionFeedback.h"

namespace flexbe_msgs
{

  class BehaviorInputAction : public ros::Msg
  {
    public:
      typedef flexbe_msgs::BehaviorInputActionGoal _action_goal_type;
      _action_goal_type action_goal;
      typedef flexbe_msgs::BehaviorInputActionResult _action_result_type;
      _action_result_type action_result;
      typedef flexbe_msgs::BehaviorInputActionFeedback _action_feedback_type;
      _action_feedback_type action_feedback;

    BehaviorInputAction():
      action_goal(),
      action_result(),
      action_feedback()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->action_goal.serialize(outbuffer + offset);
      offset += this->action_result.serialize(outbuffer + offset);
      offset += this->action_feedback.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->action_goal.deserialize(inbuffer + offset);
      offset += this->action_result.deserialize(inbuffer + offset);
      offset += this->action_feedback.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "flexbe_msgs/BehaviorInputAction"; };
    const char * getMD5(){ return "409aeef36c5fba868c3b1f845c52f119"; };

  };

}
#endif
