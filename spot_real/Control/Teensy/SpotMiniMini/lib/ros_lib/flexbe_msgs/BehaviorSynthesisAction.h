#ifndef _ROS_flexbe_msgs_BehaviorSynthesisAction_h
#define _ROS_flexbe_msgs_BehaviorSynthesisAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "flexbe_msgs/BehaviorSynthesisActionGoal.h"
#include "flexbe_msgs/BehaviorSynthesisActionResult.h"
#include "flexbe_msgs/BehaviorSynthesisActionFeedback.h"

namespace flexbe_msgs
{

  class BehaviorSynthesisAction : public ros::Msg
  {
    public:
      typedef flexbe_msgs::BehaviorSynthesisActionGoal _action_goal_type;
      _action_goal_type action_goal;
      typedef flexbe_msgs::BehaviorSynthesisActionResult _action_result_type;
      _action_result_type action_result;
      typedef flexbe_msgs::BehaviorSynthesisActionFeedback _action_feedback_type;
      _action_feedback_type action_feedback;

    BehaviorSynthesisAction():
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

    const char * getType(){ return "flexbe_msgs/BehaviorSynthesisAction"; };
    const char * getMD5(){ return "3f70602e7ef6378d09fa82dd8284fb29"; };

  };

}
#endif
