#ifndef _ROS_flexbe_msgs_BehaviorExecutionAction_h
#define _ROS_flexbe_msgs_BehaviorExecutionAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "flexbe_msgs/BehaviorExecutionActionGoal.h"
#include "flexbe_msgs/BehaviorExecutionActionResult.h"
#include "flexbe_msgs/BehaviorExecutionActionFeedback.h"

namespace flexbe_msgs
{

  class BehaviorExecutionAction : public ros::Msg
  {
    public:
      typedef flexbe_msgs::BehaviorExecutionActionGoal _action_goal_type;
      _action_goal_type action_goal;
      typedef flexbe_msgs::BehaviorExecutionActionResult _action_result_type;
      _action_result_type action_result;
      typedef flexbe_msgs::BehaviorExecutionActionFeedback _action_feedback_type;
      _action_feedback_type action_feedback;

    BehaviorExecutionAction():
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

    const char * getType(){ return "flexbe_msgs/BehaviorExecutionAction"; };
    const char * getMD5(){ return "5137e5877c4ee9dbe6ca4796ad090f1a"; };

  };

}
#endif
